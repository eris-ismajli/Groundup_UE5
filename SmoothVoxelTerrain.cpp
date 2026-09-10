#include "SmoothVoxelTerrain.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "DynamicMesh/MeshNormals.h"
#include "DynamicMesh/MeshTangents.h"
#include "UDynamicMesh.h"
#include "Engine/World.h"
#include "Engine/Engine.h" 
#include "Components/DynamicMeshComponent.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/Pawn.h"
#include "Async/Async.h" 

using namespace UE::Geometry;

static int32 FloorDiv(int32 Dividend, int32 Divisor)
{
    int32 Quotient = Dividend / Divisor;
    if ((Dividend ^ Divisor) < 0 && Dividend % Divisor != 0) Quotient--;
    return Quotient;
}

// ---------------------------------------------------------------------------------
// HASHING & PERLIN NOISE IMPLEMENTATION
// ---------------------------------------------------------------------------------

FORCEINLINE float Hash2D(int32 x, int32 y)
{
    uint32 h = (uint32)x * 374761393U + (uint32)y * 668265263U;
    h = (h ^ (h >> 13)) * 1274126177U;
    return (float)(h & 0x7FFFFFFF) * 4.656612873077392578125e-10f;
}

FORCEINLINE float Hash3D(int32 x, int32 y, int32 z)
{
    uint32 h = (uint32)x * 73856093U ^ (uint32)y * 19349663U ^ (uint32)z * 83492791U;
    h = (h ^ (h >> 13)) * 1274126177U;
    return (float)(h & 0x7FFFFFFF) * 4.656612873077392578125e-10f;
}

FORCEINLINE FVector2f GetGradient2D(int32 X, int32 Y)
{
    float Angle = Hash2D(X, Y) * 2.0f * PI;
    return FVector2f(FMath::Cos(Angle), FMath::Sin(Angle));
}

// Quintic curve for smoother interpolation
FORCEINLINE float Fade(float t) { return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); }

FORCEINLINE float FastPerlinNoise2D(float x, float y)
{
    int32 ix = FMath::FloorToInt(x), iy = FMath::FloorToInt(y);
    float fx = x - ix, fy = y - iy;

    FVector2f g00 = GetGradient2D(ix, iy), g10 = GetGradient2D(ix + 1, iy);
    FVector2f g01 = GetGradient2D(ix, iy + 1), g11 = GetGradient2D(ix + 1, iy + 1);

    float n00 = g00.X * fx + g00.Y * fy;
    float n10 = g10.X * (fx - 1.0f) + g10.Y * fy;
    float n01 = g01.X * fx + g01.Y * (fy - 1.0f);
    float n11 = g11.X * (fx - 1.0f) + g11.Y * (fy - 1.0f);

    float ux = Fade(fx), uy = Fade(fy);
    return FMath::Lerp(FMath::Lerp(n00, n10, ux), FMath::Lerp(n01, n11, ux), uy) * 1.414f;
}

FORCEINLINE FVector3f GetGradient3D(int32 X, int32 Y, int32 Z)
{
    float h = Hash3D(X, Y, Z);
    float theta = h * 2.0f * PI;
    float phi = FMath::Acos((Hash3D(X + 1, Y, Z) * 2.0f) - 1.0f);
    return FVector3f(FMath::Sin(phi) * FMath::Cos(theta), FMath::Sin(phi) * FMath::Sin(theta), FMath::Cos(phi));
}

FORCEINLINE float FastPerlinNoise3D(float x, float y, float z)
{
    int32 ix = FMath::FloorToInt(x), iy = FMath::FloorToInt(y), iz = FMath::FloorToInt(z);
    float fx = x - ix, fy = y - iy, fz = z - iz;

    float ux = Fade(fx), uy = Fade(fy), uz = Fade(fz);

    auto DotGrad = [&](int32 cx, int32 cy, int32 cz, float vx, float vy, float vz) {
        FVector3f g = GetGradient3D(cx, cy, cz);
        return g.X * vx + g.Y * vy + g.Z * vz;
        };

    float c000 = DotGrad(ix, iy, iz, fx, fy, fz);
    float c100 = DotGrad(ix + 1, iy, iz, fx - 1.0f, fy, fz);
    float c010 = DotGrad(ix, iy + 1, iz, fx, fy - 1.0f, fz);
    float c110 = DotGrad(ix + 1, iy + 1, iz, fx - 1.0f, fy - 1.0f, fz);
    float c001 = DotGrad(ix, iy, iz + 1, fx, fy, fz - 1.0f);
    float c101 = DotGrad(ix + 1, iy, iz + 1, fx - 1.0f, fy, fz - 1.0f);
    float c011 = DotGrad(ix, iy + 1, iz + 1, fx, fy - 1.0f, fz - 1.0f);
    float c111 = DotGrad(ix + 1, iy + 1, iz + 1, fx - 1.0f, fy - 1.0f, fz - 1.0f);

    return FMath::Lerp(FMath::Lerp(FMath::Lerp(c000, c100, ux), FMath::Lerp(c010, c110, ux), uy),
        FMath::Lerp(FMath::Lerp(c001, c101, ux), FMath::Lerp(c011, c111, ux), uy), uz) * 1.414f;
}

// ---------------------------------------------------------------------------------
// NOISE HELPERS
// ---------------------------------------------------------------------------------

float CalculateFBM2D(float x, float y, int32 octaves, float freq, float amp, int32 layerSeed)
{
    float total = 0.0f; float maxAmp = 0.0f;
    for (int32 i = 0; i < octaves; ++i) {
        float offsetX = Hash2D(layerSeed, i) * 5000.0f;
        float offsetY = Hash2D(layerSeed + 1, i) * 5000.0f;

        float rx = (x * freq) + offsetX;
        float ry = (y * freq) + offsetY;
        float rotX = rx * 0.707f - ry * 0.707f;
        float rotY = rx * 0.707f + ry * 0.707f;

        total += FastPerlinNoise2D(rotX, rotY) * amp;
        maxAmp += amp;
        freq *= 2.13f;
        amp *= 0.48f;
    }
    return maxAmp > 0.0f ? total / maxAmp : 0.0f;
}

float CalculateRidgedFBM2D(float x, float y, int32 octaves, float freq, float amp, int32 layerSeed)
{
    float total = 0.0f; float maxAmp = 0.0f;
    for (int32 i = 0; i < octaves; ++i) {
        float offsetX = Hash2D(layerSeed, i) * 5000.0f;
        float offsetY = Hash2D(layerSeed + 1, i) * 5000.0f;

        float rx = (x * freq) + offsetX;
        float ry = (y * freq) + offsetY;
        float rotX = rx * 0.707f - ry * 0.707f;
        float rotY = rx * 0.707f + ry * 0.707f;

        float noiseVal = FastPerlinNoise2D(rotX, rotY);
        float ridge = 1.0f - FMath::Abs(noiseVal);
        ridge = (ridge * 2.0f) - 1.0f;

        total += ridge * amp;
        maxAmp += amp;
        freq *= 2.13f;
        amp *= 0.48f;
    }
    return maxAmp > 0.0f ? total / maxAmp : 0.0f;
}

float CalculateFBM3D(float x, float y, float z, int32 octaves, float freq, float amp, int32 layerSeed)
{
    float total = 0.0f;
    float maxAmp = 0.0f;
    for (int32 i = 0; i < octaves; ++i)
    {
        float offsetX = Hash3D(layerSeed, i, 11) * 3000.0f;
        float offsetY = Hash3D(layerSeed + 1, i, 22) * 3000.0f;
        float offsetZ = Hash3D(layerSeed + 2, i, 33) * 3000.0f;

        float rx = (x * freq) + offsetX;
        float ry = (y * freq) + offsetY;
        float rz = (z * freq) + offsetZ;

        total += FastPerlinNoise3D(rx, ry, rz) * amp;
        maxAmp += amp;
        freq *= 2.02f;
        amp *= 0.5f;
    }
    return maxAmp > 0.0f ? total / maxAmp : 0.0f;
}

FORCEINLINE FLinearColor FastColorLerp(const FLinearColor& A, const FLinearColor& B, float Alpha)
{
    return FLinearColor(A.R + (B.R - A.R) * Alpha, A.G + (B.G - A.G) * Alpha, A.B + (B.B - A.B) * Alpha, A.A + (B.A - A.A) * Alpha);
}

struct FFastRandom
{
    uint32 State;
    FORCEINLINE FFastRandom(uint32 Seed) : State(Seed) {}
    FORCEINLINE float NextFloat() { State = State * 1664525U + 1013904223U; return (float)(State & 0x7FFFFFFF) * 4.656612873077392578125e-10f; }
};

struct FChunkNeighborhood
{
    FIntVector SelfCoord;
    const EVoxelType* SelfData = nullptr;
    const EVoxelType* WestData = nullptr;
    const EVoxelType* EastData = nullptr;
    const EVoxelType* SouthData = nullptr;
    const EVoxelType* NorthData = nullptr;
    const EVoxelType* SouthWestData = nullptr;
    const EVoxelType* SouthEastData = nullptr;
    const EVoxelType* NorthWestData = nullptr;
    const EVoxelType* NorthEastData = nullptr;

    int32 ChunkSize = 32;
    int32 MaxHeight = 256;
    int32 StepY = 32;
    int32 StepZ = 32 * 32;

    FORCEINLINE EVoxelType GetVoxel(int32 LocalX, int32 LocalY, int32 LocalZ) const
    {
        if (LocalZ < 0) return EVoxelType::Air;
        if (LocalZ >= MaxHeight) return EVoxelType::Air;

        if (uint32(LocalX) < uint32(ChunkSize) && uint32(LocalY) < uint32(ChunkSize))
            return SelfData[LocalX + LocalY * StepY + LocalZ * StepZ];

        int32 LX = LocalX, LY = LocalY;
        int32 SX = 0, SY = 0;

        if (LX < 0) { SX = -1; LX += ChunkSize; }
        else if (LX >= ChunkSize) { SX = 1; LX -= ChunkSize; }

        if (LY < 0) { SY = -1; LY += ChunkSize; }
        else if (LY >= ChunkSize) { SY = 1; LY -= ChunkSize; }

        const EVoxelType* TargetData = nullptr;
        if (SX == 0)      TargetData = (SY < 0) ? SouthData : NorthData;
        else if (SX < 0)  TargetData = (SY == 0) ? WestData : (SY < 0 ? SouthWestData : NorthWestData);
        else              TargetData = (SY == 0) ? EastData : (SY < 0 ? SouthEastData : NorthEastData);

        if (!TargetData) return EVoxelType::Stone;

        return TargetData[LX + LY * ChunkSize + LocalZ * StepZ];
    }
};

ASmoothVoxelTerrain::ASmoothVoxelTerrain()
{
    PrimaryActorTick.bCanEverTick = true;
    RootSceneComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    RootSceneComponent->SetMobility(EComponentMobility::Static);
    RootComponent = RootSceneComponent;
}

ASmoothVoxelTerrain::~ASmoothVoxelTerrain() { bIsDestroyed = true; }

void ASmoothVoxelTerrain::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    if (bIsDestroyed || HasAnyFlags(RF_ClassDefaultObject | RF_ArchetypeObject) || !GetWorld() || GetWorld()->bIsTearingDown || IsActorBeingDestroyed()) return;
    RebuildTerrain();
}

void ASmoothVoxelTerrain::BeginPlay()
{
    Super::BeginPlay();
    if (Chunks.Num() == 0) RebuildTerrain();

    if (GetWorld())
    {
        if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
        {
            if (APawn* Pawn = PC->GetPawnOrSpectator())
            {
                RegisterPlayer(Pawn);
            }
        }
    }
}

void ASmoothVoxelTerrain::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (bIsDestroyed) return;

    if (TrackedPlayerComponent.IsValid())
    {
        FIntVector CurrentChunk = WorldToChunkCoord(TrackedPlayerComponent->GetComponentLocation());
        if (CurrentChunk != LastPlayerChunkCoord)
        {
            HandleBoundaryCrossing(CurrentChunk);
        }
    }

    ProcessTasks();
    UpdateCollisionIfNeeded();
    UpdateChunkVisibilityAndShadows();

    if (TrackedPlayerComponent.IsValid() && GEngine)
    {
        FVector PlayerLoc = TrackedPlayerComponent->GetComponentLocation();
        PlayerLoc.Z -= 90.0f;
        FVector LocalPos = GetActorTransform().InverseTransformPosition(PlayerLoc);
        int32 AltitudeVoxels = FMath::FloorToInt(LocalPos.Z / CubeSize);
        GEngine->AddOnScreenDebugMessage(1337, 0.0f, FColor::Cyan, FString::Printf(TEXT("Altitude: %d Voxels"), AltitudeVoxels));
    }
}

void ASmoothVoxelTerrain::RegisterPlayer(APawn* PlayerPawn)
{
    if (PlayerPawn && PlayerPawn->GetRootComponent())
    {
        TrackedPlayerComponent = PlayerPawn->GetRootComponent();
        FIntVector InitialChunk = WorldToChunkCoord(TrackedPlayerComponent->GetComponentLocation());
        HandleBoundaryCrossing(InitialChunk);
    }
}

void ASmoothVoxelTerrain::OnPlayerMoved(USceneComponent* UpdatedComponent, EUpdateTransformFlags UpdateTransformFlags, ETeleportType Teleport)
{
    if (!UpdatedComponent) return;
    FIntVector CurrentChunk = WorldToChunkCoord(UpdatedComponent->GetComponentLocation());
    if (CurrentChunk != LastPlayerChunkCoord) HandleBoundaryCrossing(CurrentChunk);
}

void ASmoothVoxelTerrain::HandleBoundaryCrossing(const FIntVector& NewChunkCoord)
{
    LastPlayerChunkCoord = NewChunkCoord;

    DataGenerationQueue.Empty();
    MeshGenerationQueue.Empty();
    GrassGenerationQueue.Empty();

    int32 limit = FMath::Max(RenderDistance, UnloadDistance);
    int32 iterCount = (limit * 2 + 1) * (limit * 2 + 1);

    int32 x = 0, y = 0, dx = 0, dy = -1;
    TArray<FIntVector> CoordsToUnload;

    int32 GrassRadius = FMath::Min(GrassRenderDistance + 1, RenderDistance);
    int32 GrassRadiusSq = GrassRadius * GrassRadius;

    for (int32 i = 0; i < iterCount; i++)
    {
        if (-limit <= x && x <= limit && -limit <= y && y <= limit)
        {
            FIntVector Coord(NewChunkCoord.X + x, NewChunkCoord.Y + y, 0);
            int32 DistSq = x * x + y * y;

            if (DistSq <= RenderDistance * RenderDistance)
            {
                if (!Chunks.Contains(Coord))
                {
                    DataGenerationQueue.Add(Coord);
                }
                else
                {
                    FVoxelChunk* Chunk = Chunks[Coord].Get();
                    if (Chunk->State == EChunkState::DataReady && CheckNeighborsDataReady(Coord))
                    {
                        MeshGenerationQueue.Add(Coord);
                    }
                    if (DistSq <= GrassRadiusSq)
                    {
                        if (Chunk->State == EChunkState::MeshReady && !Chunk->bGrassGenerated && !Chunk->bGeneratingGrass)
                            GrassGenerationQueue.AddUnique(Coord);
                    }
                }
            }
            else if (DistSq > UnloadDistance * UnloadDistance)
            {
                if (Chunks.Contains(Coord)) CoordsToUnload.Add(Coord);
            }

            if (DistSq > GrassRadiusSq)
            {
                if (Chunks.Contains(Coord))
                {
                    FVoxelChunk* Chunk = Chunks[Coord].Get();
                    if (Chunk->GrassMeshComponent)
                    {
                        ReleaseMeshComponent(Chunk->GrassMeshComponent, 1);
                        Chunk->GrassMeshComponent = nullptr;
                    }
                    Chunk->GrassVoxelTriangles.Empty();
                    Chunk->bGrassGenerated = false;
                    Chunk->bGeneratingGrass = false;
                    GrassGenerationQueue.Remove(Coord);
                }
            }
        }
        if (x == y || (x < 0 && x == -y) || (x > 0 && x == 1 - y)) { int32 temp = dx; dx = -dy; dy = temp; }
        x += dx; y += dy;
    }
    for (const FIntVector& c : CoordsToUnload) UnloadChunk(c);
}

void ASmoothVoxelTerrain::UpdateChunkVisibilityAndShadows()
{
    if (!TrackedPlayerComponent.IsValid()) return;
    FVector PlayerLoc = TrackedPlayerComponent->GetComponentLocation();
    float GrassRadiusSq = FMath::Square(GrassRenderDistance * ChunkSize * CubeSize);
    float ShadowRadiusSq = FMath::Square(ShadowRenderDistance * ChunkSize * CubeSize);

    for (auto& Pair : Chunks)
    {
        FVoxelChunk* Chunk = Pair.Value.Get();
        if (!Chunk) continue;

        FVector ChunkOrigin = ChunkCoordToWorldOrigin(Chunk->Coord);
        float ClosestX = FMath::Clamp(PlayerLoc.X, ChunkOrigin.X, ChunkOrigin.X + (ChunkSize * CubeSize));
        float ClosestY = FMath::Clamp(PlayerLoc.Y, ChunkOrigin.Y, ChunkOrigin.Y + (ChunkSize * CubeSize));
        float DistSq = FVector::DistSquaredXY(PlayerLoc, FVector(ClosestX, ClosestY, 0));

        if (Chunk->GrassMeshComponent && Chunk->bGrassGenerated)
        {
            bool bShouldBeVisible = bEnableGrassGeometry && (DistSq <= GrassRadiusSq);
            if (Chunk->GrassMeshComponent->IsVisible() != bShouldBeVisible) Chunk->GrassMeshComponent->SetVisibility(bShouldBeVisible);
        }

        if (Chunk->MeshComponent)
        {
            bool bShouldCastShadow = bCastShadow && (DistSq <= ShadowRadiusSq);
            if (Chunk->MeshComponent->CastShadow != bShouldCastShadow) Chunk->MeshComponent->SetCastShadow(bShouldCastShadow);
        }
    }
}

void ASmoothVoxelTerrain::UpdateCollisionIfNeeded()
{
    if (bCollisionDirty)
    {
        for (auto& Pair : Chunks) if (Pair.Value && Pair.Value->MeshComponent) Pair.Value->MeshComponent->UpdateCollision(false);
        bCollisionDirty = false;
    }
}

void ASmoothVoxelTerrain::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    bIsDestroyed = true;
    Chunks.Empty();
    MeshComponentPool.Empty();
    GrassMeshComponentPool.Empty();
    WaterMeshComponentPool.Empty();
    DataGenerationQueue.Empty();
    MeshGenerationQueue.Empty();
    GrassGenerationQueue.Empty();
    MeshApplyQueue.Empty();
    GrassApplyQueue.Empty();
    Super::EndPlay(EndPlayReason);
}

UDynamicMeshComponent* ASmoothVoxelTerrain::AcquireMeshComponent(int32 MeshType)
{
    TArray<UDynamicMeshComponent*>* Pool = nullptr;
    if (MeshType == 1) Pool = &GrassMeshComponentPool;
    else if (MeshType == 2) Pool = &WaterMeshComponentPool;
    else Pool = &MeshComponentPool;

    if (Pool->Num() > 0)
    {
        UDynamicMeshComponent* Comp = Pool->Pop();
        if (MeshType != 1) Comp->SetVisibility(true);
        return Comp;
    }

    UDynamicMeshComponent* Comp = NewObject<UDynamicMeshComponent>(this);
    Comp->CreationMethod = EComponentCreationMethod::Instance;
    Comp->SetupAttachment(RootSceneComponent);
    Comp->SetMobility(EComponentMobility::Static);
    Comp->RegisterComponent();

    if (MeshType == 1) {
        Comp->SetCastShadow(false); Comp->SetReceivesDecals(false);
        Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Comp->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
        Comp->bEnableComplexCollision = false; Comp->SetCanEverAffectNavigation(false);
        Comp->SetGenerateOverlapEvents(false);
        if (GrassBladesMaterial) Comp->SetMaterial(0, GrassBladesMaterial);
    }
    else if (MeshType == 2) {
        Comp->SetCastShadow(false); Comp->SetReceivesDecals(false);
        Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Comp->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
        Comp->bEnableComplexCollision = false; Comp->SetCanEverAffectNavigation(false);
        Comp->SetGenerateOverlapEvents(false);
        if (WaterMaterial) Comp->SetMaterial(0, WaterMaterial);
    }
    else {
        Comp->SetCastShadow(bCastShadow); Comp->SetReceivesDecals(bReceivesDecals);
        Comp->EnableComplexAsSimpleCollision(); Comp->bEnableComplexCollision = bEnableComplexCollision;
        Comp->SetCollisionEnabled(CollisionEnabled); Comp->SetCollisionProfileName(CollisionProfileName);
        Comp->SetGenerateOverlapEvents(bGenerateOverlapEvents);
        Comp->bUseAsyncCooking = true; Comp->bDeferCollisionUpdates = true;
        if (GrassMaterial) Comp->SetMaterial(0, GrassMaterial);
        if (DirtMaterial) Comp->SetMaterial(1, DirtMaterial);
        if (StoneMaterial) Comp->SetMaterial(2, StoneMaterial);
    }
    return Comp;
}

void ASmoothVoxelTerrain::ReleaseMeshComponent(UDynamicMeshComponent* Comp, int32 MeshType)
{
    if (Comp && IsValid(Comp))
    {
        Comp->SetVisibility(false);
        if (UDynamicMesh* DynMesh = Comp->GetDynamicMesh()) DynMesh->EditMesh([](FDynamicMesh3& MeshOut) { MeshOut.Clear(); });
        if (MeshType == 1) GrassMeshComponentPool.Add(Comp);
        else if (MeshType == 2) WaterMeshComponentPool.Add(Comp);
        else MeshComponentPool.Add(Comp);
    }
}

void ASmoothVoxelTerrain::GenerateChunks()
{
    for (auto& Pair : Chunks)
    {
        if (Pair.Value)
        {
            if (Pair.Value->MeshComponent) ReleaseMeshComponent(Pair.Value->MeshComponent, 0);
            if (Pair.Value->GrassMeshComponent) ReleaseMeshComponent(Pair.Value->GrassMeshComponent, 1);
            if (Pair.Value->WaterMeshComponent) ReleaseMeshComponent(Pair.Value->WaterMeshComponent, 2);
        }
    }
    Chunks.Empty();
    DataGenerationQueue.Empty();
    MeshGenerationQueue.Empty();
    GrassGenerationQueue.Empty();
    MeshApplyQueue.Empty();
    GrassApplyQueue.Empty();
    LastPlayerChunkCoord = FIntVector(999999, 999999, 999999);
    if (TrackedPlayerComponent.IsValid()) HandleBoundaryCrossing(WorldToChunkCoord(TrackedPlayerComponent->GetComponentLocation()));
}

void ASmoothVoxelTerrain::ProcessTasks()
{
    int32 AppliedCount = 0;
    while (MeshApplyQueue.Num() > 0 && AppliedCount < MaxMeshApplyPerFrame)
    {
        auto Task = MeshApplyQueue[0];
        MeshApplyQueue.RemoveAt(0);

        if (FVoxelChunk* Chunk = GetChunk(Task->Coord))
        {
            if (Chunk->State == EChunkState::GeneratingMesh)
            {
                Chunk->State = EChunkState::MeshReady;
                if (!Chunk->MeshComponent) Chunk->MeshComponent = AcquireMeshComponent(0);

                Chunk->VoxelTriangles = MoveTemp(Task->VoxelTriangles);
                Chunk->MeshComponent->SetMesh(MoveTemp(Task->LocalMesh));
                Chunk->MeshComponent->UpdateCollision(true);

                if (bEnableWater && !Chunk->bWaterGenerated)
                {
                    bool bNeedsWater = false;
                    if (Chunk->HeightMap)
                    {
                        for (int32 i = 0; i < Chunk->HeightMap->Num(); ++i) {
                            if ((*Chunk->HeightMap)[i] < SeaLevel) { bNeedsWater = true; break; }
                        }
                    }
                    else bNeedsWater = true;

                    if (bNeedsWater)
                    {
                        Chunk->bWaterGenerated = true;
                        if (!Chunk->WaterMeshComponent) Chunk->WaterMeshComponent = AcquireMeshComponent(2);

                        FDynamicMesh3 WaterMesh;
                        WaterMesh.EnableAttributes();
                        FDynamicMeshAttributeSet* Attr = WaterMesh.Attributes();
                        Attr->SetNumUVLayers(1); Attr->EnablePrimaryColors();

                        double WorldX = (double)Chunk->Coord.X * ChunkSize * CubeSize;
                        double WorldY = (double)Chunk->Coord.Y * ChunkSize * CubeSize;
                        double Z = (double)SeaLevel * CubeSize;
                        double CSize = (double)ChunkSize * CubeSize;

                        int32 v0 = WaterMesh.AppendVertex(FVector3d(WorldX, WorldY, Z)), v1 = WaterMesh.AppendVertex(FVector3d(WorldX + CSize, WorldY, Z));
                        int32 v2 = WaterMesh.AppendVertex(FVector3d(WorldX + CSize, WorldY + CSize, Z)), v3 = WaterMesh.AppendVertex(FVector3d(WorldX, WorldY + CSize, Z));

                        int32 t1 = WaterMesh.AppendTriangle(v0, v2, v1), t2 = WaterMesh.AppendTriangle(v0, v3, v2);

                        float UMin = ((float)Chunk->Coord.X * ChunkSize) * TextureScale, VMin = ((float)Chunk->Coord.Y * ChunkSize) * TextureScale;
                        float UMax = ((float)(Chunk->Coord.X + 1) * ChunkSize) * TextureScale, VMax = ((float)(Chunk->Coord.Y + 1) * ChunkSize) * TextureScale;

                        FDynamicMeshUVOverlay* UVs = Attr->GetUVLayer(0);
                        int32 uv00 = UVs->AppendElement(FVector2f(UMin, VMin)), uv10 = UVs->AppendElement(FVector2f(UMax, VMin));
                        int32 uv11 = UVs->AppendElement(FVector2f(UMax, VMax)), uv01 = UVs->AppendElement(FVector2f(UMin, VMax));
                        UVs->SetTriangle(t1, FIndex3i(uv00, uv10, uv11)); UVs->SetTriangle(t2, FIndex3i(uv00, uv11, uv01));

                        FDynamicMeshColorOverlay* Colors = Attr->PrimaryColors();
                        int32 c0 = Colors->AppendElement(FVector4f(0.0f, 0.4f, 0.8f, 0.7f));
                        Colors->SetTriangle(t1, FIndex3i(c0, c0, c0)); Colors->SetTriangle(t2, FIndex3i(c0, c0, c0));

                        FDynamicMeshNormalOverlay* Normals = Attr->PrimaryNormals();
                        int32 n0 = Normals->AppendElement(FVector3f(0.0f, 0.0f, 1.0f));
                        Normals->SetTriangle(t1, FIndex3i(n0, n0, n0)); Normals->SetTriangle(t2, FIndex3i(n0, n0, n0));
                        Chunk->WaterMeshComponent->SetMesh(MoveTemp(WaterMesh));
                    }
                }

                int32 DistSq = FMath::Square(Task->Coord.X - LastPlayerChunkCoord.X) + FMath::Square(Task->Coord.Y - LastPlayerChunkCoord.Y);
                if (DistSq <= FMath::Square(FMath::Min(GrassRenderDistance + 1, RenderDistance)) && !Chunk->bGrassGenerated && !Chunk->bGeneratingGrass)
                    GrassGenerationQueue.AddUnique(Task->Coord);
            }
        }
        AppliedCount++;
    }

    int32 AppliedGrassCount = 0;
    while (GrassApplyQueue.Num() > 0 && AppliedGrassCount < MaxMeshApplyPerFrame)
    {
        auto Task = GrassApplyQueue[0];
        GrassApplyQueue.RemoveAt(0);

        if (FVoxelChunk* Chunk = GetChunk(Task->Coord))
        {
            if (Chunk->bGeneratingGrass && (FMath::Square(Task->Coord.X - LastPlayerChunkCoord.X) + FMath::Square(Task->Coord.Y - LastPlayerChunkCoord.Y)) <= FMath::Square(FMath::Min(GrassRenderDistance + 1, RenderDistance)))
            {
                Chunk->bGeneratingGrass = false; Chunk->bGrassGenerated = true;
                if (!Chunk->GrassMeshComponent) Chunk->GrassMeshComponent = AcquireMeshComponent(1);
                Chunk->GrassVoxelTriangles = MoveTemp(Task->GrassVoxelTriangles);
                Chunk->GrassMeshComponent->SetMesh(MoveTemp(Task->LocalGrassMesh));
            }
            else { Chunk->bGeneratingGrass = false; Chunk->bGrassGenerated = false; }
        }
        AppliedGrassCount++;
    }

    int32 DataGenCount = 0;
    while (DataGenerationQueue.Num() > 0 && DataGenCount < MaxChunkDataGenPerFrame)
    {
        FIntVector Coord = DataGenerationQueue[0];
        DataGenerationQueue.RemoveAt(0);
        if (!Chunks.Contains(Coord)) { GenerateChunkData(Coord); DataGenCount++; }
    }

    int32 MeshGenCount = 0;
    for (int32 i = 0; i < MeshGenerationQueue.Num() && MeshGenCount < MaxChunkMeshGenPerFrame; i++)
    {
        FIntVector Coord = MeshGenerationQueue[i];
        if (FVoxelChunk* Chunk = GetChunk(Coord)) {
            if (Chunk->State == EChunkState::DataReady && CheckNeighborsDataReady(Coord)) {
                GenerateChunkMesh(Coord); MeshGenerationQueue.RemoveAt(i); i--; MeshGenCount++;
            }
        }
        else { MeshGenerationQueue.RemoveAt(i); i--; }
    }

    int32 GrassGenCount = 0;
    for (int32 i = 0; i < GrassGenerationQueue.Num() && GrassGenCount < MaxChunkGrassGenPerFrame; i++)
    {
        FIntVector Coord = GrassGenerationQueue[i];
        if (FVoxelChunk* Chunk = GetChunk(Coord)) {
            if (Chunk->State == EChunkState::MeshReady && !Chunk->bGrassGenerated && !Chunk->bGeneratingGrass) {
                GenerateGrassMesh(Coord); GrassGenerationQueue.RemoveAt(i); i--; GrassGenCount++;
            }
        }
        else { GrassGenerationQueue.RemoveAt(i); i--; }
    }
}

bool ASmoothVoxelTerrain::CheckNeighborsDataReady(const FIntVector& ChunkCoord)
{
    const FIntVector Neighbors[8] = {
        FIntVector(ChunkCoord.X - 1, ChunkCoord.Y,     0), FIntVector(ChunkCoord.X + 1, ChunkCoord.Y,     0),
        FIntVector(ChunkCoord.X,     ChunkCoord.Y - 1, 0), FIntVector(ChunkCoord.X,     ChunkCoord.Y + 1, 0),
        FIntVector(ChunkCoord.X - 1, ChunkCoord.Y - 1, 0), FIntVector(ChunkCoord.X + 1, ChunkCoord.Y - 1, 0),
        FIntVector(ChunkCoord.X - 1, ChunkCoord.Y + 1, 0), FIntVector(ChunkCoord.X + 1, ChunkCoord.Y + 1, 0)
    };
    for (const FIntVector& N : Neighbors) {
        FVoxelChunk* C = GetChunk(N);
        if (!C || (C->State != EChunkState::DataReady && C->State != EChunkState::GeneratingMesh && C->State != EChunkState::MeshReady)) return false;
    }
    return true;
}

FTerrainGenConfig ASmoothVoxelTerrain::GetTerrainConfig() const
{
    FTerrainGenConfig Config;
    Config.ChunkSize = ChunkSize; Config.FloorLevel = FloorLevel; Config.BedrockLevel = BedrockLevel;
    Config.MaxHeight = MaxHeight; Config.CubeSize = CubeSize; Config.MinGrassThickness = MinGrassThickness;
    Config.Seed = Seed; Config.bSmoothTerrain = bSmoothTerrain; Config.bEnableWater = bEnableWater;
    Config.SeaLevel = SeaLevel; Config.GrasslandBiome = GrasslandBiome; Config.CaveSettings = CaveSettings; Config.bEnableGrassGeometry = bEnableGrassGeometry;
    Config.GrassMinDensity = GrassMinDensity; Config.GrassMaxDensity = GrassMaxDensity;
    Config.GrassMinHeight = GrassMinHeight; Config.GrassMaxHeight = GrassMaxHeight;
    Config.GrassMinWidth = GrassMinWidth; Config.GrassMaxWidth = GrassMaxWidth;
    Config.GrassDensityNoiseScale = GrassDensityNoiseScale; Config.GrassBladeSegments = GrassBladeSegments;
    Config.bTwoSidedGrass = bTwoSidedGrass; Config.TextureScale = TextureScale;
    return Config;
}

void ASmoothVoxelTerrain::GenerateChunkData(const FIntVector& ChunkCoord)
{
    TSharedPtr<FVoxelChunk> Chunk = MakeShared<FVoxelChunk>();
    Chunk->Coord = ChunkCoord;
    Chunk->State = EChunkState::GeneratingData;
    Chunks.Add(ChunkCoord, Chunk);

    FTerrainGenConfig Config = GetTerrainConfig();
    TWeakObjectPtr<ASmoothVoxelTerrain> WeakThis(this);

    Async(EAsyncExecution::ThreadPool, [WeakThis, ChunkCoord, Config]() mutable
        {
            int32 LocalChunkSize = Config.ChunkSize, LocalMaxHeight = Config.MaxHeight, LocalBedrockLevel = Config.BedrockLevel;

            TSharedPtr<TArray<EVoxelType>, ESPMode::ThreadSafe> LocalVoxelData = MakeShared<TArray<EVoxelType>, ESPMode::ThreadSafe>();
            LocalVoxelData->SetNumZeroed(LocalChunkSize * LocalChunkSize * LocalMaxHeight);

            int32 CacheSize = LocalChunkSize + 5;
            TSharedPtr<TArray<float>, ESPMode::ThreadSafe> LocalHeightMap = MakeShared<TArray<float>, ESPMode::ThreadSafe>();
            LocalHeightMap->SetNumUninitialized(CacheSize * CacheSize);

            for (int32 y = 0; y < CacheSize; ++y) {
                int32 WorldY = ChunkCoord.Y * LocalChunkSize - 2 + y;
                for (int32 x = 0; x < CacheSize; ++x) {
                    (*LocalHeightMap)[x + y * CacheSize] = Config.GetHeightAtWorldCorner(ChunkCoord.X * LocalChunkSize - 2 + x, WorldY);
                }
            }

            FLocalHeightGrid GenHeightGrid;
            GenHeightGrid.Heights = LocalHeightMap->GetData();
            GenHeightGrid.CacheSize = CacheSize;

            for (int32 lx = 0; lx < LocalChunkSize; ++lx) {
                for (int32 ly = 0; ly < LocalChunkSize; ++ly) {
                    int32 GroundLevel = Config.GetGroundLevelLocal(lx, ly, GenHeightGrid);
                    int32 BaseIdx = lx + ly * LocalChunkSize, Step = LocalChunkSize * LocalChunkSize;
                    for (int32 lz = 0; lz < LocalMaxHeight; ++lz) {
                        int32 Index = BaseIdx + lz * Step;
                        if (lz < GroundLevel - 3) (*LocalVoxelData)[Index] = EVoxelType::Stone;
                        else if (lz < GroundLevel) (*LocalVoxelData)[Index] = EVoxelType::Dirt;
                        else if (lz == GroundLevel) (*LocalVoxelData)[Index] = EVoxelType::Grass;
                        else (*LocalVoxelData)[Index] = EVoxelType::Air;
                    }
                }
            }

            // --- Cave Generation Pass ---
            if (Config.CaveSettings.bEnableCaves)
            {
                int32 StepY = LocalChunkSize;
                int32 StepZ = LocalChunkSize * LocalChunkSize;

                for (int32 ly = 0; ly < LocalChunkSize; ++ly) {
                    for (int32 lx = 0; lx < LocalChunkSize; ++lx) {
                        int32 WorldX = ChunkCoord.X * LocalChunkSize + lx;
                        int32 WorldY = ChunkCoord.Y * LocalChunkSize + ly;

                        float SurfaceHeight = Config.GetSurfaceHeightLocal(lx, ly, GenHeightGrid);

                        int32 BaseIdx = lx + ly * StepY;

                        for (int32 lz = 1; lz < LocalMaxHeight; ++lz) {
                            int32 Index = BaseIdx + lz * StepZ;
                            if ((*LocalVoxelData)[Index] == EVoxelType::Air) continue;

                            int32 WorldZ = lz + LocalBedrockLevel;
                            if (Config.IsInsideCave(WorldX, WorldY, WorldZ, SurfaceHeight))
                            {
                                (*LocalVoxelData)[Index] = EVoxelType::Air;
                            }
                        }
                    }
                }

                // Surface pass on newly exposed terrain in cave openings
                for (int32 ly = 0; ly < LocalChunkSize; ++ly) {
                    for (int32 lx = 0; lx < LocalChunkSize; ++lx) {
                        int32 BaseIdx = lx + ly * StepY;
                        float SurfHeight = Config.GetSurfaceHeightLocal(lx, ly, GenHeightGrid);

                        for (int32 lz = LocalMaxHeight - 2; lz >= 1; --lz) {
                            int32 Index = BaseIdx + lz * StepZ;
                            int32 AboveIndex = Index + StepZ;

                            if ((*LocalVoxelData)[Index] == EVoxelType::Stone && (*LocalVoxelData)[AboveIndex] == EVoxelType::Air)
                            {
                                int32 WorldZ = lz + LocalBedrockLevel;
                                if (FMath::Abs(WorldZ - SurfHeight) <= 3.0f)
                                {
                                    (*LocalVoxelData)[Index] = EVoxelType::Grass;
                                    for (int32 d = 1; d <= 2 && (lz - d) > 0; ++d) {
                                        int32 BelowIdx = BaseIdx + (lz - d) * StepZ;
                                        if ((*LocalVoxelData)[BelowIdx] == EVoxelType::Stone)
                                            (*LocalVoxelData)[BelowIdx] = EVoxelType::Dirt;
                                    }
                                }
                                break;
                            }
                        }
                    }
                }
            }

            AsyncTask(ENamedThreads::GameThread, [WeakThis, ChunkCoord, LocalVoxelData, LocalHeightMap]()
                {
                    ASmoothVoxelTerrain* Terrain = WeakThis.Get();
                    if (!Terrain || Terrain->bIsDestroyed) return;
                    if (FVoxelChunk* TargetChunk = Terrain->GetChunk(ChunkCoord)) {
                        TargetChunk->VoxelData = LocalVoxelData; TargetChunk->HeightMap = LocalHeightMap; TargetChunk->State = EChunkState::DataReady;
                        if (Terrain->CheckNeighborsDataReady(ChunkCoord)) Terrain->MeshGenerationQueue.AddUnique(ChunkCoord);

                        const FIntVector Neighbors[8] = {
                            FIntVector(ChunkCoord.X - 1, ChunkCoord.Y,     0), FIntVector(ChunkCoord.X + 1, ChunkCoord.Y,     0),
                            FIntVector(ChunkCoord.X,     ChunkCoord.Y - 1, 0), FIntVector(ChunkCoord.X,     ChunkCoord.Y + 1, 0),
                            FIntVector(ChunkCoord.X - 1, ChunkCoord.Y - 1, 0), FIntVector(ChunkCoord.X + 1, ChunkCoord.Y - 1, 0),
                            FIntVector(ChunkCoord.X - 1, ChunkCoord.Y + 1, 0), FIntVector(ChunkCoord.X + 1, ChunkCoord.Y + 1, 0)
                        };
                        for (const FIntVector& N : Neighbors) {
                            if (FVoxelChunk* NChunk = Terrain->GetChunk(N))
                                if (NChunk->State == EChunkState::DataReady && Terrain->CheckNeighborsDataReady(N)) Terrain->MeshGenerationQueue.AddUnique(N);
                        }
                    }
                });
        });
}

void ASmoothVoxelTerrain::GenerateChunkMesh(const FIntVector& ChunkCoord)
{
    FVoxelChunk* Chunk = GetChunk(ChunkCoord);
    if (!Chunk || Chunk->State != EChunkState::DataReady) return;
    Chunk->State = EChunkState::GeneratingMesh;

    TSharedPtr<TArray<EVoxelType>, ESPMode::ThreadSafe> SelfData = Chunk->VoxelData;
    TSharedPtr<TArray<float>, ESPMode::ThreadSafe> HeightMap = Chunk->HeightMap;
    TSharedPtr<TArray<EVoxelType>, ESPMode::ThreadSafe> WestData, EastData, SouthData, NorthData;
    TSharedPtr<TArray<EVoxelType>, ESPMode::ThreadSafe> SWData, SEData, NWData, NEData;

    if (auto* C = GetChunk(ChunkCoord + FIntVector(-1, 0, 0))) WestData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(1, 0, 0))) EastData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(0, -1, 0))) SouthData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(0, 1, 0))) NorthData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(-1, -1, 0))) SWData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(1, -1, 0))) SEData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(-1, 1, 0))) NWData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(1, 1, 0))) NEData = C->VoxelData;

    FTerrainGenConfig Config = GetTerrainConfig();
    TWeakObjectPtr<ASmoothVoxelTerrain> WeakThis(this);

    Async(EAsyncExecution::ThreadPool, [WeakThis, ChunkCoord, Config, SelfData, HeightMap, WestData, EastData, SouthData, NorthData, SWData, SEData, NWData, NEData]() mutable
        {
            TSharedPtr<FMeshApplyTask, ESPMode::ThreadSafe> ResultTask = MakeShared<FMeshApplyTask, ESPMode::ThreadSafe>();
            ResultTask->Coord = ChunkCoord;
            ResultTask->LocalMesh.EnableAttributes();
            if (FDynamicMeshAttributeSet* Attr = ResultTask->LocalMesh.Attributes()) { Attr->SetNumUVLayers(2); Attr->EnablePrimaryColors(); Attr->EnableMaterialID(); }

            FChunkNeighborhood Neighborhood;
            Neighborhood.SelfData = SelfData->GetData();
            Neighborhood.WestData = WestData ? WestData->GetData() : nullptr; Neighborhood.EastData = EastData ? EastData->GetData() : nullptr;
            Neighborhood.SouthData = SouthData ? SouthData->GetData() : nullptr; Neighborhood.NorthData = NorthData ? NorthData->GetData() : nullptr;
            Neighborhood.SouthWestData = SWData ? SWData->GetData() : nullptr; Neighborhood.SouthEastData = SEData ? SEData->GetData() : nullptr;
            Neighborhood.NorthWestData = NWData ? NWData->GetData() : nullptr; Neighborhood.NorthEastData = NEData ? NEData->GetData() : nullptr;
            Neighborhood.ChunkSize = Config.ChunkSize; Neighborhood.MaxHeight = Config.MaxHeight; Neighborhood.StepY = Config.ChunkSize; Neighborhood.StepZ = Config.ChunkSize * Config.ChunkSize;
            Neighborhood.SelfCoord = ChunkCoord;

            FLocalHeightGrid HeightGrid;
            HeightGrid.Heights = HeightMap->GetData(); HeightGrid.CacheSize = Config.ChunkSize + 5;

            FCaveSmoothCache CaveCache;
            CaveCache.Init(&Config, &HeightGrid, &Neighborhood, ChunkCoord);

            FTriIDArray TempTriIDs;

            for (int32 lz = 0; lz < Config.MaxHeight; ++lz) {
                for (int32 ly = 0; ly < Config.ChunkSize; ++ly) {
                    for (int32 lx = 0; lx < Config.ChunkSize; ++lx) {
                        int32 Index = lx + ly * Config.ChunkSize + lz * Neighborhood.StepZ;
                        if (Neighborhood.SelfData[Index] == EVoxelType::Air) continue;
                        TempTriIDs.Reset();
                        Config.AppendVoxelFacesLocal(lx, ly, lz, ResultTask->LocalMesh, TempTriIDs, HeightGrid, Neighborhood, ChunkCoord, &CaveCache);
                        if (TempTriIDs.Num() > 0) ResultTask->VoxelTriangles.Add(Index, TempTriIDs);
                    }
                }
            }

            AsyncTask(ENamedThreads::GameThread, [WeakThis, ResultTask]() {
                if (ASmoothVoxelTerrain* T = WeakThis.Get()) T->MeshApplyQueue.Add(ResultTask);
                });
        });
}

void ASmoothVoxelTerrain::GenerateGrassMesh(const FIntVector& ChunkCoord)
{
    FVoxelChunk* Chunk = GetChunk(ChunkCoord);
    if (!Chunk || Chunk->State != EChunkState::MeshReady || Chunk->bGeneratingGrass || Chunk->bGrassGenerated) return;
    Chunk->bGeneratingGrass = true;

    TSharedPtr<TArray<EVoxelType>, ESPMode::ThreadSafe> SelfData = Chunk->VoxelData;
    TSharedPtr<TArray<float>, ESPMode::ThreadSafe> HeightMap = Chunk->HeightMap;
    TSharedPtr<TArray<EVoxelType>, ESPMode::ThreadSafe> WestData, EastData, SouthData, NorthData;

    if (auto* C = GetChunk(ChunkCoord + FIntVector(-1, 0, 0))) WestData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(1, 0, 0))) EastData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(0, -1, 0))) SouthData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(0, 1, 0))) NorthData = C->VoxelData;

    FTerrainGenConfig Config = GetTerrainConfig();
    TWeakObjectPtr<ASmoothVoxelTerrain> WeakThis(this);

    Async(EAsyncExecution::ThreadPool, [WeakThis, ChunkCoord, Config, SelfData, HeightMap, WestData, EastData, SouthData, NorthData]() mutable
        {
            if (!Config.bEnableGrassGeometry) return;
            TSharedPtr<FGrassApplyTask, ESPMode::ThreadSafe> ResultTask = MakeShared<FGrassApplyTask, ESPMode::ThreadSafe>();
            ResultTask->Coord = ChunkCoord;
            ResultTask->LocalGrassMesh.EnableAttributes();
            if (FDynamicMeshAttributeSet* GrassAttr = ResultTask->LocalGrassMesh.Attributes()) GrassAttr->SetNumUVLayers(2);

            FChunkNeighborhood Neighborhood;
            Neighborhood.SelfData = SelfData->GetData();
            Neighborhood.WestData = WestData ? WestData->GetData() : nullptr; Neighborhood.EastData = EastData ? EastData->GetData() : nullptr;
            Neighborhood.SouthData = SouthData ? SouthData->GetData() : nullptr; Neighborhood.NorthData = NorthData ? NorthData->GetData() : nullptr;
            Neighborhood.ChunkSize = Config.ChunkSize; Neighborhood.MaxHeight = Config.MaxHeight; Neighborhood.StepY = Config.ChunkSize; Neighborhood.StepZ = Config.ChunkSize * Config.ChunkSize;
            Neighborhood.SelfCoord = ChunkCoord;

            FLocalHeightGrid HeightGrid;
            HeightGrid.Heights = HeightMap->GetData(); HeightGrid.CacheSize = Config.ChunkSize + 5;
            FTriIDArray TempTriIDs;

            for (int32 lz = 0; lz < Config.MaxHeight; ++lz) {
                for (int32 ly = 0; ly < Config.ChunkSize; ++ly) {
                    for (int32 lx = 0; lx < Config.ChunkSize; ++lx) {
                        int32 Index = lx + ly * Config.ChunkSize + lz * Neighborhood.StepZ;
                        if (Neighborhood.SelfData[Index] == EVoxelType::Grass && Neighborhood.SelfData[Index + Neighborhood.StepZ] == EVoxelType::Air) {
                            TempTriIDs.Reset();
                            Config.AppendGrassBladesLocal(lx, ly, lz, ResultTask->LocalGrassMesh, TempTriIDs, HeightGrid, Neighborhood, ChunkCoord);
                            if (TempTriIDs.Num() > 0) ResultTask->GrassVoxelTriangles.Add(Index, TempTriIDs);
                        }
                    }
                }
            }

            FMeshNormals::QuickComputeVertexNormals(ResultTask->LocalGrassMesh);

            AsyncTask(ENamedThreads::GameThread, [WeakThis, ResultTask]() {
                if (ASmoothVoxelTerrain* T = WeakThis.Get()) T->GrassApplyQueue.Add(ResultTask);
                });
        });
}

void ASmoothVoxelTerrain::UnloadChunk(const FIntVector& Coord)
{
    TSharedPtr<FVoxelChunk> Chunk;
    if (Chunks.RemoveAndCopyValue(Coord, Chunk))
    {
        if (Chunk)
        {
            if (Chunk->MeshComponent) ReleaseMeshComponent(Chunk->MeshComponent, 0);
            if (Chunk->GrassMeshComponent) ReleaseMeshComponent(Chunk->GrassMeshComponent, 1);
            if (Chunk->WaterMeshComponent) ReleaseMeshComponent(Chunk->WaterMeshComponent, 2);
        }
    }
}

void ASmoothVoxelTerrain::FVoxelChunk::UpdateSharedFace(int32 LocalX, int32 LocalY, int32 LocalZ, ASmoothVoxelTerrain* TerrainOwner, const FIntVector& NeighborDirection)
{
    if (!VoxelData) return;
    UDynamicMesh* DynamicMesh = MeshComponent ? MeshComponent->GetDynamicMesh() : nullptr;
    UDynamicMesh* GrassDynamicMesh = (bGrassGenerated && GrassMeshComponent) ? GrassMeshComponent->GetDynamicMesh() : nullptr;
    if (!DynamicMesh) return;

    auto UpdateBlockLogic = [&](FDynamicMesh3& MeshOut, FDynamicMesh3* GrassMeshOut)
        {
            MeshOut.EnableAttributes();
            if (GrassMeshOut) GrassMeshOut->EnableAttributes();
            FDynamicMeshAttributeSet* Attr = MeshOut.Attributes();
            if (Attr && Attr->NumUVLayers() < 2) Attr->SetNumUVLayers(2);
            if (Attr && !Attr->PrimaryColors()) Attr->EnablePrimaryColors();
            if (Attr && !Attr->HasMaterialID()) Attr->EnableMaterialID();
            FDynamicMeshAttributeSet* GrassAttr = GrassMeshOut ? GrassMeshOut->Attributes() : nullptr;
            if (GrassAttr && GrassAttr->NumUVLayers() < 2) GrassAttr->SetNumUVLayers(2);

            for (int32 dz = -1; dz <= 1; ++dz) {
                for (int32 dy = -1; dy <= 1; ++dy) {
                    for (int32 dx = -1; dx <= 1; ++dx) {
                        if (NeighborDirection.X != 0 && dx != 0) continue;
                        if (NeighborDirection.Y != 0 && dy != 0) continue;
                        if (NeighborDirection.Z != 0 && dz != 0) continue;
                        int32 nx = LocalX + dx, ny = LocalY + dy, nz = LocalZ + dz;
                        if (nx >= 0 && nx < TerrainOwner->ChunkSize && ny >= 0 && ny < TerrainOwner->ChunkSize && nz >= 0 && nz < TerrainOwner->MaxHeight)
                            RemoveVoxelFaces(nx, ny, nz, MeshOut, GrassMeshOut, TerrainOwner);
                    }
                }
            }

            for (int32 dz = -1; dz <= 1; ++dz) {
                for (int32 dy = -1; dy <= 1; ++dy) {
                    for (int32 dx = -1; dx <= 1; ++dx) {
                        if (NeighborDirection.X != 0 && dx != 0) continue;
                        if (NeighborDirection.Y != 0 && dy != 0) continue;
                        if (NeighborDirection.Z != 0 && dz != 0) continue;
                        int32 nx = LocalX + dx, ny = LocalY + dy, nz = LocalZ + dz;
                        if (nx >= 0 && nx < TerrainOwner->ChunkSize && ny >= 0 && ny < TerrainOwner->ChunkSize && nz >= 0 && nz < TerrainOwner->MaxHeight)
                            if ((*VoxelData)[nx + ny * TerrainOwner->ChunkSize + nz * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize] != EVoxelType::Air)
                                AddVoxelFaces(nx, ny, nz, MeshOut, GrassMeshOut, TerrainOwner);
                    }
                }
            }
        };

    DynamicMesh->EditMesh([&](FDynamicMesh3& MeshOut) {
        if (GrassDynamicMesh) GrassDynamicMesh->EditMesh([&](FDynamicMesh3& GrassMeshOut) { UpdateBlockLogic(MeshOut, &GrassMeshOut); FMeshNormals::QuickComputeVertexNormals(GrassMeshOut); });
        else UpdateBlockLogic(MeshOut, nullptr);
        });
    MeshComponent->UpdateCollision(true);
}

void ASmoothVoxelTerrain::FVoxelChunk::UpdateVoxel(int32 LocalX, int32 LocalY, int32 LocalZ, EVoxelType NewType, ASmoothVoxelTerrain* TerrainOwner)
{
    if (!VoxelData) return;
    int32 Index = LocalX + LocalY * TerrainOwner->ChunkSize + LocalZ * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;
    if ((*VoxelData)[Index] == NewType) return;
    UpdateVoxelMesh(LocalX, LocalY, LocalZ, NewType, TerrainOwner);
}

bool ASmoothVoxelTerrain::GetVoxelAtWorldPoint(const FVector& WorldPoint, int32& OutVoxelX, int32& OutVoxelY, int32& OutVoxelZ, EVoxelType* OutType)
{
    FVector LocalPos = GetActorTransform().InverseTransformPosition(WorldPoint);
    OutVoxelX = FMath::FloorToInt(LocalPos.X / CubeSize);
    OutVoxelY = FMath::FloorToInt(LocalPos.Y / CubeSize);
    OutVoxelZ = FMath::FloorToInt(LocalPos.Z / CubeSize);
    int32 LocalZ = OutVoxelZ - BedrockLevel;
    if (LocalZ < 0 || LocalZ >= MaxHeight) return false;
    if (OutType) *OutType = GetVoxelAtWorld(OutVoxelX, OutVoxelY, OutVoxelZ);
    return true;
}

void ASmoothVoxelTerrain::RemoveVoxel(FVector WorldLocation)
{
    if (bIsDestroyed) return;
    FIntVector ChunkCoord = WorldToChunkCoord(WorldLocation);
    FVoxelChunk* Chunk = GetChunk(ChunkCoord);
    if (!Chunk || !Chunk->VoxelData) return;
    int32 lx, ly, lz; WorldToLocalVoxel(WorldLocation, ChunkCoord, lx, ly, lz);

    if (lx < 0 || lx >= ChunkSize || ly < 0 || ly >= ChunkSize || lz < 0 || lz >= MaxHeight) return;
    int32 Index = lx + ly * ChunkSize + lz * ChunkSize * ChunkSize;
    if ((*Chunk->VoxelData)[Index] == EVoxelType::Air) return;

    Chunk->UpdateVoxel(lx, ly, lz, EVoxelType::Air, this);
    if (lx == 0) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(-1, 0, 0))) Neighbor->UpdateSharedFace(ChunkSize - 1, ly, lz, this, FIntVector(1, 0, 0)); }
    if (lx == ChunkSize - 1) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(1, 0, 0))) Neighbor->UpdateSharedFace(0, ly, lz, this, FIntVector(-1, 0, 0)); }
    if (ly == 0) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, -1, 0))) Neighbor->UpdateSharedFace(lx, ChunkSize - 1, lz, this, FIntVector(0, 1, 0)); }
    if (ly == ChunkSize - 1) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, 1, 0))) Neighbor->UpdateSharedFace(lx, 0, lz, this, FIntVector(0, -1, 0)); }
    if (lz == 0) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, 0, -1))) Neighbor->UpdateSharedFace(lx, ly, MaxHeight - 1, this, FIntVector(0, 0, 1)); }
    if (lz == MaxHeight - 1) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, 0, 1))) Neighbor->UpdateSharedFace(lx, ly, 0, this, FIntVector(0, 0, -1)); }
}

void ASmoothVoxelTerrain::PlaceVoxel(FVector WorldLocation, EVoxelType Type)
{
    if (bIsDestroyed || Type == EVoxelType::Air) return;
    FIntVector ChunkCoord = WorldToChunkCoord(WorldLocation);
    FVoxelChunk* Chunk = GetChunk(ChunkCoord);
    if (!Chunk) return;
    int32 lx, ly, lz; WorldToLocalVoxel(WorldLocation, ChunkCoord, lx, ly, lz);
    if (lx < 0 || lx >= ChunkSize || ly < 0 || ly >= ChunkSize || lz < 0 || lz >= MaxHeight) return;

    Chunk->UpdateVoxel(lx, ly, lz, Type, this);
    if (lx == 0) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(-1, 0, 0))) Neighbor->UpdateSharedFace(ChunkSize - 1, ly, lz, this, FIntVector(1, 0, 0)); }
    if (lx == ChunkSize - 1) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(1, 0, 0))) Neighbor->UpdateSharedFace(0, ly, lz, this, FIntVector(-1, 0, 0)); }
    if (ly == 0) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, -1, 0))) Neighbor->UpdateSharedFace(lx, ChunkSize - 1, lz, this, FIntVector(0, 1, 0)); }
    if (ly == ChunkSize - 1) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, 1, 0))) Neighbor->UpdateSharedFace(lx, 0, lz, this, FIntVector(0, -1, 0)); }
    if (lz == 0) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, 0, -1))) Neighbor->UpdateSharedFace(lx, ly, MaxHeight - 1, this, FIntVector(0, 0, 1)); }
    if (lz == MaxHeight - 1) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, 0, 1))) Neighbor->UpdateSharedFace(lx, ly, 0, this, FIntVector(0, 0, -1)); }
}

void ASmoothVoxelTerrain::RebuildTerrain()
{
    if (bIsDestroyed) return;
    GenerateChunks();
}

FIntVector ASmoothVoxelTerrain::WorldToChunkCoord(const FVector& WorldPos) const
{
    FVector LocalPos = GetActorTransform().InverseTransformPosition(WorldPos);
    int32 WorldX = FMath::FloorToInt(LocalPos.X / CubeSize);
    int32 WorldY = FMath::FloorToInt(LocalPos.Y / CubeSize);
    return FIntVector(FloorDiv(WorldX, ChunkSize), FloorDiv(WorldY, ChunkSize), 0);
}

void ASmoothVoxelTerrain::WorldToLocalVoxel(const FVector& WorldPos, const FIntVector& ChunkCoord, int32& OutX, int32& OutY, int32& OutZ) const
{
    FVector LocalPos = GetActorTransform().InverseTransformPosition(WorldPos);
    int32 WorldX = FMath::FloorToInt(LocalPos.X / CubeSize);
    int32 WorldY = FMath::FloorToInt(LocalPos.Y / CubeSize);
    int32 WorldZ = FMath::FloorToInt(LocalPos.Z / CubeSize);
    OutX = WorldX - ChunkCoord.X * ChunkSize;
    OutY = WorldY - ChunkCoord.Y * ChunkSize;
    OutZ = WorldZ - BedrockLevel;
}

FVector ASmoothVoxelTerrain::ChunkCoordToWorldOrigin(const FIntVector& ChunkCoord) const
{
    FVector LocalOrigin((double)ChunkCoord.X * ChunkSize * CubeSize, (double)ChunkCoord.Y * ChunkSize * CubeSize, 0.0f);
    return GetActorTransform().TransformPosition(LocalOrigin);
}

EVoxelType ASmoothVoxelTerrain::GetVoxelAtWorld(int32 WorldX, int32 WorldY, int32 WorldZ) const
{
    int32 LocalZ = WorldZ - BedrockLevel;
    if (LocalZ < 0 || LocalZ >= MaxHeight) return EVoxelType::Air;
    int32 ChunkX = FloorDiv(WorldX, ChunkSize);
    int32 ChunkY = FloorDiv(WorldY, ChunkSize);
    const FVoxelChunk* Chunk = GetChunk(FIntVector(ChunkX, ChunkY, 0));
    if (!Chunk || !Chunk->VoxelData) return EVoxelType::Air;
    int32 LocalX = WorldX - ChunkX * ChunkSize;
    int32 LocalY = WorldY - ChunkY * ChunkSize;
    if (LocalX < 0 || LocalX >= ChunkSize || LocalY < 0 || LocalY >= ChunkSize) return EVoxelType::Air;
    int32 Index = LocalX + LocalY * ChunkSize + LocalZ * ChunkSize * ChunkSize;
    if (!Chunk->VoxelData->IsValidIndex(Index)) return EVoxelType::Air;
    return (*Chunk->VoxelData)[Index];
}

// ---------------------------------------------------------------------------------
// HEIGHT GENERATION 
// ---------------------------------------------------------------------------------

float FTerrainGenConfig::GetHeightAtWorldCorner(int32 WorldX, int32 WorldY) const
{
    float BaseX = (float)WorldX; float BaseY = (float)WorldY;

    float GlobalBaseNoise = CalculateFBM2D(BaseX, BaseY, 2, GrasslandBiome.GlobalBaseNoiseScale, 1.0f, Seed + 1);

    float SmoothMaskVal = 0.0f;
    if (GrasslandBiome.SmoothHillLikelihood > 0.0f && GrasslandBiome.SmoothHillHeight > 0.0f) {
        float SmoothMask = FastPerlinNoise2D(BaseX * GrasslandBiome.SmoothHillMaskScale + Hash2D(Seed, 10) * 1000, BaseY * GrasslandBiome.SmoothHillMaskScale + Hash2D(Seed, 11) * 1000) * 0.5f + 0.5f;
        float SmoothThreshold = 1.0f - GrasslandBiome.SmoothHillLikelihood;
        if (SmoothMask > SmoothThreshold) {
            SmoothMaskVal = (SmoothMask - SmoothThreshold) / GrasslandBiome.SmoothHillLikelihood;
            SmoothMaskVal = SmoothMaskVal * SmoothMaskVal * (3.0f - 2.0f * SmoothMaskVal);
        }
    }

    float JaggedMaskVal = 0.0f;
    if (GrasslandBiome.JaggedHillLikelihood > 0.0f && GrasslandBiome.JaggedHillHeight > 0.0f) {
        float JaggedMask = FastPerlinNoise2D(BaseX * GrasslandBiome.JaggedHillMaskScale + Hash2D(Seed, 20) * 1000, BaseY * GrasslandBiome.JaggedHillMaskScale + Hash2D(Seed, 21) * 1000) * 0.5f + 0.5f;
        float JaggedThreshold = 1.0f - GrasslandBiome.JaggedHillLikelihood;
        if (JaggedMask > JaggedThreshold) {
            JaggedMaskVal = (JaggedMask - JaggedThreshold) / GrasslandBiome.JaggedHillLikelihood;
            JaggedMaskVal = JaggedMaskVal * JaggedMaskVal * (3.0f - 2.0f * JaggedMaskVal);
        }
    }

    float PlainsMaskVal = 0.0f;
    if (GrasslandBiome.PlainsLikelihood > 0.0f && GrasslandBiome.PlainsHeight > 0.0f) {
        float PlainsMask = FastPerlinNoise2D(BaseX * GrasslandBiome.PlainsMaskScale + Hash2D(Seed, 30) * 1000, BaseY * GrasslandBiome.PlainsMaskScale + Hash2D(Seed, 31) * 1000) * 0.5f + 0.5f;
        float PlainsThreshold = 1.0f - GrasslandBiome.PlainsLikelihood;
        if (PlainsMask > PlainsThreshold) {
            PlainsMaskVal = (PlainsMask - PlainsThreshold) / GrasslandBiome.PlainsLikelihood;
            PlainsMaskVal = PlainsMaskVal * PlainsMaskVal * (3.0f - 2.0f * PlainsMaskVal);
        }
    }

    float EffectiveFloorLevel = FMath::Lerp((float)FloorLevel, GrasslandBiome.PlainsFloorLevel, PlainsMaskVal);
    float TotalHeight = (EffectiveFloorLevel * CubeSize) + (GlobalBaseNoise * GrasslandBiome.GlobalBaseHeight);
    float MaxDominantMask = FMath::Max(SmoothMaskVal, FMath::Max(JaggedMaskVal, PlainsMaskVal));
    float FlatWeight = FMath::Clamp(1.0f - MaxDominantMask, 0.0f, 1.0f);

    if (FlatWeight > 0.0f && GrasslandBiome.FlatFieldHeight > 0.0f) {
        float FieldNoise = CalculateFBM2D(BaseX, BaseY, GrasslandBiome.FlatFieldOctaves, GrasslandBiome.FlatFieldNoiseScale, 1.0f, Seed + 40);
        TotalHeight += FieldNoise * GrasslandBiome.FlatFieldHeight * FlatWeight;
    }

    if (PlainsMaskVal > 0.0f) {
        float PlainsNoise = CalculateFBM2D(BaseX, BaseY, GrasslandBiome.PlainsOctaves, GrasslandBiome.PlainsNoiseScale, 1.0f, Seed + 50);
        TotalHeight += PlainsNoise * GrasslandBiome.PlainsHeight * PlainsMaskVal;
    }

    if (SmoothMaskVal > 0.0f) {
        float HillNoise = CalculateFBM2D(BaseX, BaseY, GrasslandBiome.SmoothHillOctaves, GrasslandBiome.SmoothHillNoiseScale, 1.0f, Seed + 60);
        float SVariance = FastPerlinNoise2D((BaseX + Hash2D(Seed, 61) * 1000) * GrasslandBiome.SmoothHillNoiseScale * 0.73f, (BaseY + Hash2D(Seed, 62) * 1000) * GrasslandBiome.SmoothHillNoiseScale * 0.73f);
        TotalHeight += HillNoise * FMath::Max(0.0f, GrasslandBiome.SmoothHillHeight + (SVariance * GrasslandBiome.SmoothHillHeightVariance)) * SmoothMaskVal;
    }

    if (JaggedMaskVal > 0.0f) {
        float JaggedNoise = CalculateRidgedFBM2D(BaseX, BaseY, GrasslandBiome.JaggedHillOctaves, GrasslandBiome.JaggedHillNoiseScale, 1.0f, Seed + 70);
        float JVariance = FastPerlinNoise2D((BaseX + Hash2D(Seed, 71) * 1000) * GrasslandBiome.JaggedHillNoiseScale * 0.73f, (BaseY + Hash2D(Seed, 72) * 1000) * GrasslandBiome.JaggedHillNoiseScale * 0.73f);
        TotalHeight += JaggedNoise * FMath::Max(0.0f, GrasslandBiome.JaggedHillHeight + (JVariance * GrasslandBiome.JaggedHillHeightVariance)) * JaggedMaskVal;
    }

    // Rivers
    float WarpX = FastPerlinNoise2D((BaseX + Hash2D(Seed, 80) * 1000) * GrasslandBiome.RiverWarpScale, BaseY * GrasslandBiome.RiverWarpScale) * GrasslandBiome.RiverWarpStrength;
    float WarpY = FastPerlinNoise2D((BaseX + Hash2D(Seed, 81) * 1000) * GrasslandBiome.RiverWarpScale, BaseY * GrasslandBiome.RiverWarpScale) * GrasslandBiome.RiverWarpStrength;
    float RiverCenter = FMath::Abs(FastPerlinNoise2D((BaseX + WarpX + Hash2D(Seed, 82) * 1000) * GrasslandBiome.RiverNoiseScale, (BaseY + WarpY + Hash2D(Seed, 83) * 1000) * GrasslandBiome.RiverNoiseScale));

    if (RiverCenter < GrasslandBiome.RiverWidth) {
        float RiverMask = 1.0f - (RiverCenter / GrasslandBiome.RiverWidth);
        RiverMask = RiverMask * RiverMask * (3.0f - 2.0f * RiverMask);
        TotalHeight = FMath::Min(TotalHeight, FMath::Lerp(TotalHeight, ((float)SeaLevel * CubeSize) - GrasslandBiome.RiverDepth, RiverMask));
    }
    return TotalHeight / CubeSize;
}

float FTerrainGenConfig::GetInterpolatedHeightLocal(float LocalX, float LocalY, const FLocalHeightGrid& HeightGrid) const
{
    int32 x0 = FMath::FloorToInt(LocalX), y0 = FMath::FloorToInt(LocalY);
    float fx = LocalX - x0, fy = LocalY - y0;
    return FMath::Lerp(FMath::Lerp(HeightGrid.GetHeight(x0, y0), HeightGrid.GetHeight(x0 + 1, y0), fx), FMath::Lerp(HeightGrid.GetHeight(x0, y0 + 1), HeightGrid.GetHeight(x0 + 1, y0 + 1), fx), fy);
}

float FTerrainGenConfig::GetSurfaceHeightLocal(int32 LocalX, int32 LocalY, const FLocalHeightGrid& G) const
{
    const float H00 = G.GetHeight(LocalX, LocalY);
    const float H10 = G.GetHeight(LocalX + 1, LocalY);
    const float H01 = G.GetHeight(LocalX, LocalY + 1);
    const float H11 = G.GetHeight(LocalX + 1, LocalY + 1);
    return FMath::Min3(H00, H10, FMath::Min(H01, H11));
}

int32 FTerrainGenConfig::GetGroundLevelLocal(int32 LocalX, int32 LocalY, const FLocalHeightGrid& G) const
{
    return FMath::Clamp(FMath::FloorToInt(GetSurfaceHeightLocal(LocalX, LocalY, G) - MinGrassThickness) - BedrockLevel, 0, MaxHeight - 1);
}

FVector FTerrainGenConfig::GetSmoothVertexLocal(int32 VertX, int32 VertY, int32 VertZ, int32 VoxX, int32 VoxY, int32 VoxZ, const FLocalHeightGrid& HeightGrid, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord, FCaveSmoothCache* CaveCache) const
{
    const int32 WorldX = ChunkCoord.X * ChunkSize + VertX;
    const int32 WorldY = ChunkCoord.Y * ChunkSize + VertY;
    const double BaseZ = (double)(VertZ + BedrockLevel);
    const FVector RawGridPos = FVector((double)WorldX, (double)WorldY, BaseZ) * CubeSize;

    if (!bSmoothTerrain) return RawGridPos;

    // --- Rule 1: top-surface displacement. Unchanged. ---
    if (Neighborhood.GetVoxel(VoxX, VoxY, VoxZ) != EVoxelType::Air && VertZ > VoxZ)
    {
        if (Neighborhood.GetVoxel(VoxX, VoxY, VoxZ + 1) == EVoxelType::Air)
        {
            if (VoxZ == GetGroundLevelLocal(VoxX, VoxY, HeightGrid))
            {
                return FVector((double)WorldX, (double)WorldY, (double)HeightGrid.GetHeight(VertX, VertY)) * CubeSize;
            }
        }
    }

    // --- Rule 2: cave-wall displacement. ---
    // Whether this voxel is smoothed at all is decided once, from world generation:
    // it had to be on the generated cave surface. All eight of its corners then take the
    // vertex's offset, which is itself a pure function of the vertex, so every smoothed
    // voxel touching that vertex agrees on where it is. A voxel generation left buried
    // takes no offset on any corner, at any time, and is a perfect cube from the start.
    if (CaveCache && CaveCache->IsReady() && CaveCache->IsCellSmoothSurface(VoxX, VoxY, VoxZ))
    {
        FVector3f Off;
        if (CaveCache->GetVertexOffset(VertX, VertY, VertZ, Off))
        {
            return FVector((double)WorldX + (double)Off.X,
                (double)WorldY + (double)Off.Y,
                BaseZ + (double)Off.Z) * CubeSize;
        }
    }

    return RawGridPos;
}

FVector FTerrainGenConfig::GetSmoothNormalLocal(int32 VertX, int32 VertY, const FLocalHeightGrid& HeightGrid) const
{
    return FVector(HeightGrid.GetHeight(VertX - 1, VertY) - HeightGrid.GetHeight(VertX + 1, VertY), HeightGrid.GetHeight(VertX, VertY - 1) - HeightGrid.GetHeight(VertX, VertY + 1), 2.0f).GetSafeNormal();
}

float FTerrainGenConfig::GetNeighborTopHeightLocal(int32 LocalX, int32 LocalY, int32 LocalZ, const FVector& VertexLocalPos, const FChunkNeighborhood& Neighborhood, const FLocalHeightGrid& HeightGrid) const
{
    EVoxelType neighborType = Neighborhood.GetVoxel(LocalX, LocalY, LocalZ);
    int32 GridX = FMath::RoundToInt(VertexLocalPos.X / CubeSize) - Neighborhood.SelfCoord.X * ChunkSize;
    int32 GridY = FMath::RoundToInt(VertexLocalPos.Y / CubeSize) - Neighborhood.SelfCoord.Y * ChunkSize;

    if (neighborType != EVoxelType::Air) {
        if (Neighborhood.GetVoxel(LocalX, LocalY, LocalZ + 1) != EVoxelType::Air) return FLT_MAX;

        if (LocalZ == GetGroundLevelLocal(LocalX, LocalY, HeightGrid)) {
            return HeightGrid.GetHeight(GridX, GridY) * CubeSize;
        }
        return (LocalZ + 1 + BedrockLevel) * CubeSize;
    }
    else {
        EVoxelType belowType = Neighborhood.GetVoxel(LocalX, LocalY, LocalZ - 1);
        if (belowType != EVoxelType::Air) {
            if ((LocalZ - 1) == GetGroundLevelLocal(LocalX, LocalY, HeightGrid)) {
                return HeightGrid.GetHeight(GridX, GridY) * CubeSize;
            }
            return (LocalZ + BedrockLevel) * CubeSize;
        }
        return -FLT_MAX;
    }
}

// ---------------------------------------------------------------------------------
// CAVE DENSITY FIELD
// ---------------------------------------------------------------------------------

float FTerrainGenConfig::GetCaveDensityAt(float WorldX, float WorldY, float WorldZ, float SurfaceHeight) const
{
    if (!CaveSettings.bEnableCaves) return -1.0f;
    if (WorldZ <= (float)BedrockLevel + (float)CaveSettings.CaveBedrockSafetyMargin) return -1.0f;

    const float DistBelowSurface = SurfaceHeight - WorldZ;
    if (DistBelowSurface <= 0.0f) return -1.0f;

    float DepthFactor = 1.0f;
    if (DistBelowSurface < CaveSettings.CaveMaxHeightOffset)
    {
        const float EntranceNoise = FastPerlinNoise2D(WorldX * 0.01f + Hash2D(Seed, 120), WorldY * 0.01f + Hash2D(Seed, 121)) * 0.5f + 0.5f;
        if (EntranceNoise <= (1.0f - CaveSettings.SurfaceBreakthroughLikelihood))
        {
            DepthFactor = FMath::Clamp(DistBelowSurface / CaveSettings.CaveMaxHeightOffset, 0.0f, 1.0f);
            DepthFactor = DepthFactor * DepthFactor * (3.0f - 2.0f * DepthFactor);
        }
    }
    if (DepthFactor <= 0.01f) return -1.0f;

    const float fx = WorldX;
    const float fy = WorldY;
    const float fz = WorldZ;

    const float ChamberMacroMask = FastPerlinNoise3D(
        fx * CaveSettings.ChamberFrequencyScale + Hash3D(Seed, 131, 1),
        fy * CaveSettings.ChamberFrequencyScale + Hash3D(Seed, 132, 2),
        fz * CaveSettings.ChamberFrequencyScale * 1.2f + Hash3D(Seed, 133, 3)
    ) * 0.5f + 0.5f;

    const float MaskMargin = ChamberMacroMask - 0.42f;

    float RadiusBonus = 0.0f;
    if (ChamberMacroMask > 0.35f)
    {
        RadiusBonus = (ChamberMacroMask - 0.35f) * CaveSettings.TunnelChamberExpansion;
    }
    const float EffectiveRadius = (CaveSettings.TunnelBaseRadius + RadiusBonus) * DepthFactor;

    float Density = -1.0f;

    const float T1A = FastPerlinNoise3D(
        fx * CaveSettings.TunnelNoiseScaleXZ + Hash3D(Seed, 150, 1),
        fy * CaveSettings.TunnelNoiseScaleXZ + Hash3D(Seed, 151, 2),
        fz * CaveSettings.TunnelNoiseScaleY + Hash3D(Seed, 152, 3));
    const float T1B = FastPerlinNoise3D(
        fx * CaveSettings.TunnelNoiseScaleXZ + Hash3D(Seed, 153, 4),
        fy * CaveSettings.TunnelNoiseScaleXZ + Hash3D(Seed, 154, 5),
        fz * CaveSettings.TunnelNoiseScaleY + Hash3D(Seed, 155, 6));
    Density = FMath::Max(Density, EffectiveRadius - FMath::Sqrt(T1A * T1A + T1B * T1B));

    const float T2A = FastPerlinNoise3D(
        (fx + 500.0f) * (CaveSettings.TunnelNoiseScaleXZ * 1.15f) + Hash3D(Seed, 160, 1),
        (fy + 500.0f) * (CaveSettings.TunnelNoiseScaleXZ * 1.15f) + Hash3D(Seed, 161, 2),
        fz * (CaveSettings.TunnelNoiseScaleY * 1.25f) + Hash3D(Seed, 162, 3));
    const float T2B = FastPerlinNoise3D(
        (fx - 500.0f) * (CaveSettings.TunnelNoiseScaleXZ * 1.15f) + Hash3D(Seed, 163, 4),
        (fy - 500.0f) * (CaveSettings.TunnelNoiseScaleXZ * 1.15f) + Hash3D(Seed, 164, 5),
        fz * (CaveSettings.TunnelNoiseScaleY * 1.25f) + Hash3D(Seed, 165, 6));
    Density = FMath::Max(Density, EffectiveRadius - FMath::Sqrt(T2A * T2A + T2B * T2B));

    if (MaskMargin > Density)
    {
        const float ChamberNoise = CalculateFBM3D(
            fx, fy, fz * 1.3f, 2,
            CaveSettings.ChamberNoiseScaleXZ,
            1.0f,
            Seed + 140);

        const float DynamicThreshold = CaveSettings.ChamberThreshold - FMath::Max(0.0f, MaskMargin) * 0.45f;
        Density = FMath::Max(Density, FMath::Min(MaskMargin, ChamberNoise - DynamicThreshold));
    }

    return Density;
}

float FTerrainGenConfig::GetCaveSmoothFieldAt(float VX, float VY, float VZ, float SurfaceHeight) const
{
    return GetCaveDensityAt(VX - 0.5f, VY - 0.5f, VZ - 0.5f, SurfaceHeight);
}

float FTerrainGenConfig::GetCaveDensity(int32 WorldX, int32 WorldY, int32 WorldZ, float SurfaceHeight) const
{
    return GetCaveDensityAt((float)WorldX, (float)WorldY, (float)WorldZ, SurfaceHeight);
}

bool FTerrainGenConfig::IsInsideCave(int32 WorldX, int32 WorldY, int32 WorldZ, float SurfaceHeight) const
{
    return GetCaveDensity(WorldX, WorldY, WorldZ, SurfaceHeight) > 0.0f;
}

// ---------------------------------------------------------------------------------
// AXIS-LOCKED PROJECTION
// ---------------------------------------------------------------------------------

static constexpr float CaveAxisReach = 1.5f;
static constexpr float CaveAxisScanStep = 0.25f;

static FORCEINLINE float SampleCaveFieldOnAxis(const FTerrainGenConfig& Config, const FVector3f& V,
    int32 Axis, float T, float SurfaceHeight)
{
    FVector3f P = V;
    P[Axis] += T;
    return Config.GetCaveSmoothFieldAt(P.X, P.Y, P.Z, SurfaceHeight);
}

static FVector3f CaveFieldGradient(const FTerrainGenConfig& Config, const FVector3f& V, float SurfaceHeight)
{
    static const FVector3f K[4] = {
        FVector3f(1.0f, -1.0f, -1.0f), FVector3f(-1.0f, -1.0f,  1.0f),
        FVector3f(-1.0f,  1.0f, -1.0f), FVector3f(1.0f,  1.0f,  1.0f) };
    const float H = 0.35f;

    FVector3f G(0.0f, 0.0f, 0.0f);
    for (int32 k = 0; k < 4; ++k)
    {
        const FVector3f Q = V + K[k] * H;
        G += K[k] * Config.GetCaveSmoothFieldAt(Q.X, Q.Y, Q.Z, SurfaceHeight);
    }
    return G;
}

static bool FindNearestAxisCrossing(const FTerrainGenConfig& Config, const FVector3f& V,
    int32 Axis, float SurfaceHeight, float& OutT)
{
    const float F0 = SampleCaveFieldOnAxis(Config, V, Axis, 0.0f, SurfaceHeight);
    if (F0 == 0.0f) { OutT = 0.0f; return true; }

    float TPrevP = 0.0f, FPrevP = F0;
    float TPrevN = 0.0f, FPrevN = F0;
    float Lo = 0.0f, Hi = 0.0f, FLo = 0.0f, FHi = 0.0f;
    bool bFound = false;

    for (float S = CaveAxisScanStep; S <= CaveAxisReach + 1e-4f; S += CaveAxisScanStep)
    {
        const float FP = SampleCaveFieldOnAxis(Config, V, Axis, S, SurfaceHeight);
        if ((FP > 0.0f) != (FPrevP > 0.0f)) { Lo = TPrevP; Hi = S; FLo = FPrevP; FHi = FP; bFound = true; break; }
        TPrevP = S; FPrevP = FP;

        const float FN = SampleCaveFieldOnAxis(Config, V, Axis, -S, SurfaceHeight);
        if ((FN > 0.0f) != (FPrevN > 0.0f)) { Lo = -S; Hi = TPrevN; FLo = FN; FHi = FPrevN; bFound = true; break; }
        TPrevN = -S; FPrevN = FN;
    }
    if (!bFound) return false;

    int32 Side = 0;
    for (int32 i = 0; i < 6; ++i)
    {
        const float Denom = FLo - FHi;
        float T = FMath::IsNearlyZero(Denom) ? 0.5f * (Lo + Hi) : Lo + (Hi - Lo) * (FLo / Denom);
        T = FMath::Clamp(T, Lo, Hi);

        const float F = SampleCaveFieldOnAxis(Config, V, Axis, T, SurfaceHeight);
        if ((F > 0.0f) == (FLo > 0.0f)) { Lo = T; FLo = F; if (Side == -1) FHi *= 0.5f; Side = -1; }
        else { Hi = T; FHi = F; if (Side == 1) FLo *= 0.5f; Side = 1; }

        if (Hi - Lo < 1e-4f) break;
    }
    OutT = 0.5f * (Lo + Hi);
    return true;
}

// ---------------------------------------------------------------------------------
// FCaveSmoothCache
// ---------------------------------------------------------------------------------

void FCaveSmoothCache::Init(const FTerrainGenConfig* InConfig, const FLocalHeightGrid* InHeights,
    const FChunkNeighborhood* InNeighborhood, const FIntVector& InChunkCoord)
{
    Config = InConfig;
    Heights = InHeights;
    Neighborhood = InNeighborhood;
    ChunkCoord = InChunkCoord;
    bReady = false;

    if (!Config || !Heights || !Heights->Heights || !Neighborhood) return;
    if (!Config->bSmoothTerrain) return;
    if (!Config->CaveSettings.bEnableCaves || !Config->CaveSettings.bSmoothCaves) return;

    CS = Config->ChunkSize;
    CellW = CS + 4;
    BaseW = CS + 3;
    VertW = CS + 1;

    CellSlabZ.Init(MIN_int32, SLAB_COUNT);
    CellDensityCache.SetNumUninitialized(CellW * CellW * SLAB_COUNT);
    CellValid.SetNumZeroed(CellW * CellW * SLAB_COUNT);

    BaseSlabZ.Init(MIN_int32, SLAB_COUNT);
    BaseOffsetCache.SetNumUninitialized(BaseW * BaseW * SLAB_COUNT);
    BaseState.SetNumZeroed(BaseW * BaseW * SLAB_COUNT);

    VertSlabZ.Init(MIN_int32, SLAB_COUNT);
    VertOffsetCache.SetNumUninitialized(VertW * VertW * SLAB_COUNT);
    VertState.SetNumZeroed(VertW * VertW * SLAB_COUNT);

    SmoothCell[0] = SmoothCell[1] = SmoothCell[2] = MIN_int32;

    bReady = true;
}

bool FCaveSmoothCache::IsCellSmoothSurface(int32 cx, int32 cy, int32 cz)
{
    if (!bReady) return false;

    // Queried eight times in a row for the same voxel while its corners are built.
    if (SmoothCell[0] == cx && SmoothCell[1] == cy && SmoothCell[2] == cz) return bSmoothCellResult;
    SmoothCell[0] = cx; SmoothCell[1] = cy; SmoothCell[2] = cz;

    // Generated state only. A voxel that world generation buried is never smoothed,
    // no matter what the player later digs next to it.
    bSmoothCellResult =
        !IsCellExpectedAir(cx, cy, cz) &&
        (IsCellExpectedAir(cx + 1, cy, cz) || IsCellExpectedAir(cx - 1, cy, cz) ||
            IsCellExpectedAir(cx, cy + 1, cz) || IsCellExpectedAir(cx, cy - 1, cz) ||
            IsCellExpectedAir(cx, cy, cz + 1) || IsCellExpectedAir(cx, cy, cz - 1));

    return bSmoothCellResult;
}

float FCaveSmoothCache::GetCellDensity(int32 cx, int32 cy, int32 cz)
{
    if (cz <= 0 || cz >= Config->MaxHeight) return -1.0f;
    if (cx < -2 || cx > CS + 1 || cy < -2 || cy > CS + 1) return -1.0f;

    const int32 S = cz & (SLAB_COUNT - 1);
    const int32 Plane = CellW * CellW;
    if (CellSlabZ[S] != cz)
    {
        CellSlabZ[S] = cz;
        FMemory::Memzero(CellValid.GetData() + S * Plane, Plane * CellValid.GetTypeSize());
    }

    const int32 I = S * Plane + (cx + 2) + (cy + 2) * CellW;
    if (CellValid[I]) return CellDensityCache[I];

    const float SurfaceHeight = Config->GetSurfaceHeightLocal(cx, cy, *Heights);
    const float D = Config->GetCaveDensity(
        ChunkCoord.X * CS + cx,
        ChunkCoord.Y * CS + cy,
        cz + Config->BedrockLevel,
        SurfaceHeight);

    CellDensityCache[I] = D;
    CellValid[I] = 1;
    return D;
}

bool FCaveSmoothCache::IsCellExpectedAir(int32 cx, int32 cy, int32 cz)
{
    if (cz <= 0) return false;
    if (cz >= Config->MaxHeight) return true;
    if (cz > Config->GetGroundLevelLocal(cx, cy, *Heights)) return true;
    return GetCellDensity(cx, cy, cz) > 0.0f;
}

float FCaveSmoothCache::VertexSurfaceHeight(int32 vx, int32 vy) const
{
    return FMath::Min(
        FMath::Min(Config->GetSurfaceHeightLocal(vx - 1, vy - 1, *Heights),
            Config->GetSurfaceHeightLocal(vx, vy - 1, *Heights)),
        FMath::Min(Config->GetSurfaceHeightLocal(vx - 1, vy, *Heights),
            Config->GetSurfaceHeightLocal(vx, vy, *Heights)));
}

bool FCaveSmoothCache::GetBaseOffset(int32 vx, int32 vy, int32 vz, FVector3f& OutOffset)
{
    if (vx < -1 || vx > CS + 1 || vy < -1 || vy > CS + 1) return false;
    if (vz < 1 || vz >= Config->MaxHeight) return false;

    const int32 S = vz & (SLAB_COUNT - 1);
    const int32 Plane = BaseW * BaseW;
    if (BaseSlabZ[S] != vz)
    {
        BaseSlabZ[S] = vz;
        FMemory::Memzero(BaseState.GetData() + S * Plane, Plane * BaseState.GetTypeSize());
    }

    const int32 I = S * Plane + (vx + 1) + (vy + 1) * BaseW;
    if (BaseState[I] == 1) return false;
    if (BaseState[I] == 2) { OutOffset = BaseOffsetCache[I]; return true; }

    BaseState[I] = 1;

    int32 GroundMin = MAX_int32;
    for (int32 dy = -1; dy <= 0; ++dy)
        for (int32 dx = -1; dx <= 0; ++dx)
        {
            const int32 GL = Config->GetGroundLevelLocal(vx + dx, vy + dy, *Heights);
            if (vz > GL) return false;
            GroundMin = FMath::Min(GroundMin, GL);
        }

    {
        bool bAnyAir = false, bAnySolid = false;
        for (int32 c = 0; c < 8; ++c)
        {
            const float D = GetCellDensity(vx - 1 + (c & 1), vy - 1 + ((c >> 1) & 1), vz - 1 + ((c >> 2) & 1));
            if (D > 0.0f) bAnyAir = true; else bAnySolid = true;
        }
        if (!bAnyAir || !bAnySolid) return false;
    }

    auto SlabMean = [&](int32 Axis, int32 k) -> float
        {
            float Sum = 0.0f;
            for (int32 i = 0; i < 4; ++i)
            {
                int32 o[3];
                o[Axis] = k;
                o[(Axis + 1) % 3] = (i & 1) - 1;
                o[(Axis + 2) % 3] = ((i >> 1) & 1) - 1;
                Sum += GetCellDensity(vx + o[0], vy + o[1], vz + o[2]);
            }
            return Sum * 0.25f;
        };

    constexpr int32 KMin = -2;
    constexpr int32 NSlab = 4;

    float Cross[3] = { 0.0f, 0.0f, 0.0f };
    float Grad[3] = { 0.0f, 0.0f, 0.0f };
    bool  bHas[3] = { false, false, false };

    for (int32 Axis = 0; Axis < 3; ++Axis)
    {
        float M[NSlab];
        for (int32 j = 0; j < NSlab; ++j) M[j] = SlabMean(Axis, KMin + j);

        Grad[Axis] = M[2] - M[1];

        for (int32 j = 0; j + 1 < NSlab; ++j)
        {
            const float A = M[j], B = M[j + 1];
            if ((A > 0.0f) == (B > 0.0f)) continue;

            const float Denom = A - B;
            const float f = FMath::Clamp(FMath::IsNearlyZero(Denom) ? 0.5f : A / Denom, 0.0f, 1.0f);
            const float t = (float)(KMin + j) + 0.5f + f;

            if (!bHas[Axis] || FMath::Abs(t) < FMath::Abs(Cross[Axis]))
            {
                Cross[Axis] = t;
                bHas[Axis] = true;
            }
        }
    }

    const float AG[3] = { FMath::Abs(Grad[0]), FMath::Abs(Grad[1]), FMath::Abs(Grad[2]) };
    int32 Order[3] = { 0, 1, 2 };
    for (int32 i = 1; i < 3; ++i)
        for (int32 j = i; j > 0 && AG[Order[j]] > AG[Order[j - 1]]; --j)
        {
            const int32 Tmp = Order[j]; Order[j] = Order[j - 1]; Order[j - 1] = Tmp;
        }

    int32 Axis = -1;
    for (int32 i = 0; i < 3; ++i)
        if (bHas[Order[i]]) { Axis = Order[i]; break; }
    if (Axis < 0) return false;

    float T = Cross[Axis];
    T *= FMath::Clamp(Config->CaveSettings.SmoothRelaxation, 0.0f, 1.0f);

    if (Axis == 2) T = FMath::Min(T, (float)GroundMin - (float)vz);

    OutOffset = FVector3f(0.0f, 0.0f, 0.0f);
    OutOffset[Axis] = T;

    BaseOffsetCache[I] = OutOffset;
    BaseState[I] = 2;
    return true;
}

bool FCaveSmoothCache::GetVertexOffset(int32 vx, int32 vy, int32 vz, FVector3f& OutOffset)
{
    if (!bReady) return false;
    if (vx < 0 || vx > CS || vy < 0 || vy > CS) return false;
    if (vz < 1 || vz >= Config->MaxHeight) return false;

    const int32 S = vz & (SLAB_COUNT - 1);
    const int32 Plane = VertW * VertW;
    if (VertSlabZ[S] != vz)
    {
        VertSlabZ[S] = vz;
        FMemory::Memzero(VertState.GetData() + S * Plane, Plane * VertState.GetTypeSize());
    }

    const int32 I = S * Plane + vx + vy * VertW;
    if (VertState[I] == 1) return false;
    if (VertState[I] == 2) { OutOffset = VertOffsetCache[I]; return true; }

    VertState[I] = 1;

    FVector3f Off;
    if (!GetBaseOffset(vx, vy, vz, Off)) return false;

    int32 Axis = -1;
    for (int32 a = 0; a < 3; ++a) if (Off[a] != 0.0f) { Axis = a; break; }
    if (Axis < 0) return false;

    // A generation-time cave transition must straddle this vertex along the locked axis.
   // Only generated state is read, so an edit can never move a vertex already in the mesh.
    bool bFace = false;
    for (int32 i = 0; i < 4 && !bFace; ++i)
    {
        int32 o[3];
        o[Axis] = -1;
        o[(Axis + 1) % 3] = (i & 1) - 1;
        o[(Axis + 2) % 3] = ((i >> 1) & 1) - 1;

        const int32 mX = vx + o[0], mY = vy + o[1], mZ = vz + o[2];
        o[Axis] = 0;
        const int32 pX = vx + o[0], pY = vy + o[1], pZ = vz + o[2];

        if (IsCellExpectedAir(mX, mY, mZ) != IsCellExpectedAir(pX, pY, pZ)) bFace = true;
    }
    if (!bFace) return false;

    if (!FMath::IsFinite(Off.X) || !FMath::IsFinite(Off.Y) || !FMath::IsFinite(Off.Z)) return false;

    VertOffsetCache[I] = Off;
    VertState[I] = 2;
    OutOffset = Off;
    return true;
}

FLinearColor FTerrainGenConfig::GetStylizedColorForVoxel(const FVector& WorldPos, EVoxelType VoxelType) const
{
    float VoxX = (float)WorldPos.X / CubeSize, VoxY = (float)WorldPos.Y / CubeSize, VoxZ = (float)WorldPos.Z / CubeSize;
    if (VoxelType == EVoxelType::Grass) return FLinearColor::White;
    else if (VoxelType == EVoxelType::Dirt) return FastColorLerp(FLinearColor(0.12f, 0.07f, 0.05f, 1.0f), FLinearColor(0.20f, 0.12f, 0.08f, 1.0f), FastPerlinNoise2D(VoxX * 0.1f, VoxY * 0.1f) * 0.5f + 0.5f);
    else if (VoxelType == EVoxelType::Stone) return FastColorLerp(FLinearColor(0.18f, 0.20f, 0.22f, 1.0f), FLinearColor(0.30f, 0.32f, 0.34f, 1.0f), FastPerlinNoise3D(VoxX * 0.08f, VoxY * 0.08f, VoxZ * 0.08f) * 0.5f + 0.5f);
    return FLinearColor::White;
}

void FTerrainGenConfig::AppendVoxelFacesLocal(int32 lx, int32 ly, int32 lz, FDynamicMesh3& Mesh, FTriIDArray& OutTriIDs, const FLocalHeightGrid& HeightGrid, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord, FCaveSmoothCache* CaveCache) const
{
    FDynamicMeshAttributeSet* Attr = Mesh.Attributes();
    if (!Attr) return;
    FDynamicMeshUVOverlay* UVOverlay = Attr->GetUVLayer(0);
    FDynamicMeshNormalOverlay* NormalOverlay = Attr->PrimaryNormals();
    FDynamicMeshColorOverlay* ColorOverlay = Attr->PrimaryColors();
    auto* MaterialIDAttribute = Attr->GetMaterialID();

    EVoxelType CurrentType = Neighborhood.GetVoxel(lx, ly, lz);
    int32 WorldX = ChunkCoord.X * ChunkSize + lx;
    int32 WorldY = ChunkCoord.Y * ChunkSize + ly;
    int32 WorldZ = lz + BedrockLevel;

    int32 TopMatID = 1, BottomMatID = 1, SideMatID = 1;
    if (CurrentType == EVoxelType::Grass) { TopMatID = 0; BottomMatID = SideMatID = 1; }
    else if (CurrentType == EVoxelType::Dirt) { TopMatID = BottomMatID = SideMatID = 1; }
    else if (CurrentType == EVoxelType::Stone) { TopMatID = BottomMatID = SideMatID = 2; }

    const bool bAirTop = Neighborhood.GetVoxel(lx, ly, lz + 1) == EVoxelType::Air;
    const bool bAirBottom = Neighborhood.GetVoxel(lx, ly, lz - 1) == EVoxelType::Air;
    const bool bAirEast = Neighborhood.GetVoxel(lx + 1, ly, lz) == EVoxelType::Air;
    const bool bAirWest = Neighborhood.GetVoxel(lx - 1, ly, lz) == EVoxelType::Air;
    const bool bAirNorth = Neighborhood.GetVoxel(lx, ly + 1, lz) == EVoxelType::Air;
    const bool bAirSouth = Neighborhood.GetVoxel(lx, ly - 1, lz) == EVoxelType::Air;

    if (!bSmoothTerrain)
    {
        if (!bAirTop && !bAirBottom && !bAirEast && !bAirWest && !bAirNorth && !bAirSouth) return;

        FLinearColor VoxelColor = GetStylizedColorForVoxel(FVector((double)WorldX * CubeSize + (0.5 * CubeSize), (double)WorldY * CubeSize + (0.5 * CubeSize), (double)WorldZ * CubeSize), CurrentType);
        int32 cIdx = ColorOverlay->AppendElement(FVector4f(VoxelColor));

        FVector Origin((double)WorldX * CubeSize, (double)WorldY * CubeSize, (double)WorldZ * CubeSize);
        FVector p000 = Origin, p100 = Origin + FVector(CubeSize, 0, 0), p010 = Origin + FVector(0, CubeSize, 0), p110 = Origin + FVector(CubeSize, CubeSize, 0);
        FVector p001 = Origin + FVector(0, 0, CubeSize), p101 = Origin + FVector(CubeSize, 0, CubeSize), p011 = Origin + FVector(0, CubeSize, CubeSize), p111 = Origin + FVector(CubeSize, CubeSize, CubeSize);

        float LocalTextureScale = TextureScale; float LocalCubeSize = CubeSize;
        auto AddQuadWorldFast = [&](const FVector& A, const FVector& B, const FVector& C, const FVector& D, const FVector3f& FixedNormal, int32 MatID, int32 UAxis, int32 VAxis)
            {
                FVector2D uvA((float)A[UAxis] / LocalCubeSize * LocalTextureScale, (float)A[VAxis] / LocalCubeSize * LocalTextureScale);
                FVector2D uvB((float)B[UAxis] / LocalCubeSize * LocalTextureScale, (float)B[VAxis] / LocalCubeSize * LocalTextureScale);
                FVector2D uvC((float)C[UAxis] / LocalCubeSize * LocalTextureScale, (float)C[VAxis] / LocalCubeSize * LocalTextureScale);
                FVector2D uvD((float)D[UAxis] / LocalCubeSize * LocalTextureScale, (float)D[VAxis] / LocalCubeSize * LocalTextureScale);

                int32 vA = Mesh.AppendVertex(FVector3d(A)), vB = Mesh.AppendVertex(FVector3d(B));
                int32 vC = Mesh.AppendVertex(FVector3d(C)), vD = Mesh.AppendVertex(FVector3d(D));
                int32 nIdx = NormalOverlay->AppendElement(FixedNormal);

                int32 t1 = Mesh.AppendTriangle(vA, vB, vC);
                if (t1 != FDynamicMesh3::InvalidID) {
                    OutTriIDs.Add(t1); NormalOverlay->SetTriangle(t1, FIndex3i(nIdx, nIdx, nIdx));
                    UVOverlay->SetTriangle(t1, FIndex3i(UVOverlay->AppendElement(FVector2f(uvA)), UVOverlay->AppendElement(FVector2f(uvB)), UVOverlay->AppendElement(FVector2f(uvC))));
                    ColorOverlay->SetTriangle(t1, FIndex3i(cIdx, cIdx, cIdx));
                    if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t1, MatID);
                }
                int32 t2 = Mesh.AppendTriangle(vA, vC, vD);
                if (t2 != FDynamicMesh3::InvalidID) {
                    OutTriIDs.Add(t2); NormalOverlay->SetTriangle(t2, FIndex3i(nIdx, nIdx, nIdx));
                    UVOverlay->SetTriangle(t2, FIndex3i(UVOverlay->AppendElement(FVector2f(uvA)), UVOverlay->AppendElement(FVector2f(uvC)), UVOverlay->AppendElement(FVector2f(uvD))));
                    ColorOverlay->SetTriangle(t2, FIndex3i(cIdx, cIdx, cIdx));
                    if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t2, MatID);
                }
            };

        if (bAirTop) AddQuadWorldFast(p001, p101, p111, p011, FVector3f(0.f, 0.f, 1.f), TopMatID, 0, 1);
        if (bAirBottom) AddQuadWorldFast(p100, p000, p010, p110, FVector3f(0.f, 0.f, -1.f), BottomMatID, 0, 1);
        if (bAirEast) AddQuadWorldFast(p100, p110, p111, p101, FVector3f(1.f, 0.f, 0.f), SideMatID, 1, 2);
        if (bAirWest) AddQuadWorldFast(p010, p000, p001, p011, FVector3f(-1.f, 0.f, 0.f), SideMatID, 1, 2);
        if (bAirNorth) AddQuadWorldFast(p110, p010, p011, p111, FVector3f(0.f, 1.f, 0.f), SideMatID, 0, 2);
        if (bAirSouth) AddQuadWorldFast(p000, p100, p101, p001, FVector3f(0.f, -1.f, 0.f), SideMatID, 0, 2);
    }
    else
    {
        const int32 GroundHere = GetGroundLevelLocal(lx, ly, HeightGrid);
        const int32 GroundEast = GetGroundLevelLocal(lx + 1, ly, HeightGrid);
        const int32 GroundWest = GetGroundLevelLocal(lx - 1, ly, HeightGrid);
        const int32 GroundNorth = GetGroundLevelLocal(lx, ly + 1, HeightGrid);
        const int32 GroundSouth = GetGroundLevelLocal(lx, ly - 1, HeightGrid);

        if (!bAirTop && !bAirBottom && !bAirEast && !bAirWest && !bAirNorth && !bAirSouth &&
            lz <= GroundEast && lz <= GroundWest && lz <= GroundNorth && lz <= GroundSouth)
        {
            return;
        }

        FLinearColor VoxelColor = GetStylizedColorForVoxel(FVector((double)WorldX * CubeSize + (0.5 * CubeSize), (double)WorldY * CubeSize + (0.5 * CubeSize), (double)WorldZ * CubeSize), CurrentType);
        int32 cIdx = ColorOverlay->AppendElement(FVector4f(VoxelColor));

        FVector v000 = GetSmoothVertexLocal(lx, ly, lz, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord, CaveCache);
        FVector v100 = GetSmoothVertexLocal(lx + 1, ly, lz, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord, CaveCache);
        FVector v010 = GetSmoothVertexLocal(lx, ly + 1, lz, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord, CaveCache);
        FVector v110 = GetSmoothVertexLocal(lx + 1, ly + 1, lz, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord, CaveCache);
        FVector v001 = GetSmoothVertexLocal(lx, ly, lz + 1, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord, CaveCache);
        FVector v101 = GetSmoothVertexLocal(lx + 1, ly, lz + 1, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord, CaveCache);
        FVector v011 = GetSmoothVertexLocal(lx, ly + 1, lz + 1, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord, CaveCache);
        FVector v111 = GetSmoothVertexLocal(lx + 1, ly + 1, lz + 1, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord, CaveCache);

        float LocalTextureScale = TextureScale; float LocalCubeSize = CubeSize;
        const double MinCross = 0.01 * (double)LocalCubeSize * (double)LocalCubeSize;

        auto AddQuadWorldSmooth = [&](const FVector& A, const FVector& B, const FVector& C, const FVector& D,
            int32 MatID, int32 UAxis, int32 VAxis, bool bDiagBD)
            {
                const FVector* Tri[2][3];
                if (!bDiagBD) { Tri[0][0] = &A; Tri[0][1] = &B; Tri[0][2] = &C;  Tri[1][0] = &A; Tri[1][1] = &C; Tri[1][2] = &D; }
                else { Tri[0][0] = &B; Tri[0][1] = &C; Tri[0][2] = &D;  Tri[1][0] = &B; Tri[1][1] = &D; Tri[1][2] = &A; }

                auto UVOf = [&](const FVector& P) {
                    return FVector2f((float)P[UAxis] / LocalCubeSize * LocalTextureScale,
                        (float)P[VAxis] / LocalCubeSize * LocalTextureScale);
                    };

                for (int32 i = 0; i < 2; ++i)
                {
                    const FVector& P0 = *Tri[i][0];
                    const FVector& P1 = *Tri[i][1];
                    const FVector& P2 = *Tri[i][2];

                    const FVector X = FVector::CrossProduct(P2 - P0, P1 - P0);
                    if (X.Size() <= MinCross) continue;

                    const int32 i0 = Mesh.AppendVertex(FVector3d(P0));
                    const int32 i1 = Mesh.AppendVertex(FVector3d(P1));
                    const int32 i2 = Mesh.AppendVertex(FVector3d(P2));
                    const int32 t = Mesh.AppendTriangle(i0, i1, i2);
                    if (t == FDynamicMesh3::InvalidID) continue;

                    OutTriIDs.Add(t);
                    const FVector3f N(X.GetSafeNormal());
                    NormalOverlay->SetTriangle(t, FIndex3i(
                        NormalOverlay->AppendElement(N),
                        NormalOverlay->AppendElement(N),
                        NormalOverlay->AppendElement(N)));
                    UVOverlay->SetTriangle(t, FIndex3i(
                        UVOverlay->AppendElement(UVOf(P0)),
                        UVOverlay->AppendElement(UVOf(P1)),
                        UVOverlay->AppendElement(UVOf(P2))));
                    ColorOverlay->SetTriangle(t, FIndex3i(cIdx, cIdx, cIdx));
                    if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t, MatID);
                }
            };

        auto AddTopQuadSmooth = [&](const FVector& A, const FVector& B, const FVector& C, const FVector& D, const FVector& nA, const FVector& nB, const FVector& nC, const FVector& nD, int32 MatID)
            {
                FVector2D uvA((float)A.X / LocalCubeSize * LocalTextureScale, (float)A.Y / LocalCubeSize * LocalTextureScale);
                FVector2D uvB((float)B.X / LocalCubeSize * LocalTextureScale, (float)B.Y / LocalCubeSize * LocalTextureScale);
                FVector2D uvC((float)C.X / LocalCubeSize * LocalTextureScale, (float)C.Y / LocalCubeSize * LocalTextureScale);
                FVector2D uvD((float)D.X / LocalCubeSize * LocalTextureScale, (float)D.Y / LocalCubeSize * LocalTextureScale);

                int32 vA = Mesh.AppendVertex(FVector3d(A)), vB = Mesh.AppendVertex(FVector3d(B));
                int32 vC = Mesh.AppendVertex(FVector3d(C)), vD = Mesh.AppendVertex(FVector3d(D));

                int32 t1 = Mesh.AppendTriangle(vA, vB, vC);
                if (t1 != FDynamicMesh3::InvalidID) {
                    OutTriIDs.Add(t1);
                    NormalOverlay->SetTriangle(t1, FIndex3i(NormalOverlay->AppendElement(FVector3f(nA)), NormalOverlay->AppendElement(FVector3f(nB)), NormalOverlay->AppendElement(FVector3f(nC))));
                    UVOverlay->SetTriangle(t1, FIndex3i(UVOverlay->AppendElement(FVector2f(uvA)), UVOverlay->AppendElement(FVector2f(uvB)), UVOverlay->AppendElement(FVector2f(uvC))));
                    ColorOverlay->SetTriangle(t1, FIndex3i(cIdx, cIdx, cIdx));
                    if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t1, MatID);
                }
                int32 t2 = Mesh.AppendTriangle(vA, vC, vD);
                if (t2 != FDynamicMesh3::InvalidID) {
                    OutTriIDs.Add(t2);
                    NormalOverlay->SetTriangle(t2, FIndex3i(NormalOverlay->AppendElement(FVector3f(nA)), NormalOverlay->AppendElement(FVector3f(nC)), NormalOverlay->AppendElement(FVector3f(nD))));
                    UVOverlay->SetTriangle(t2, FIndex3i(UVOverlay->AppendElement(FVector2f(uvA)), UVOverlay->AppendElement(FVector2f(uvC)), UVOverlay->AppendElement(FVector2f(uvD))));
                    ColorOverlay->SetTriangle(t2, FIndex3i(cIdx, cIdx, cIdx));
                    if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t2, MatID);
                }
            };

        // Face corner order, as a bitmask per corner (bit0 = +X, bit1 = +Y, bit2 = +Z),
        // matching the argument order of the quads emitted below.
        static const int32 CornersTop[4] = { 4, 6, 7, 5 };
        static const int32 CornersBottom[4] = { 1, 3, 2, 0 };
        static const int32 CornersEast[4] = { 1, 5, 7, 3 };
        static const int32 CornersWest[4] = { 2, 6, 4, 0 };
        static const int32 CornersNorth[4] = { 3, 7, 6, 2 };
        static const int32 CornersSouth[4] = { 0, 4, 5, 1 };

        // Where a cubic voxel's face meets a smoothed face, the two rims sit apart by the
        // vertex offset. This band closes that gap without moving either rim. It is always
        // owned by the cubic side, so it is emitted exactly once, and its triangles land in
        // this voxel's OutTriIDs so runtime edits remove them with the rest of the face.
        auto EmitRimBands = [&](int32 FaceAxis, int32 FaceSign, const int32(&Corners)[4],
            int32 MatID, int32 UAxis, int32 VAxis)
            {
                if (!CaveCache || !CaveCache->IsReady()) return;
                if (CaveCache->IsCellSmoothSurface(lx, ly, lz)) return;

                int32 QC[3] = { lx, ly, lz };
                QC[FaceAxis] += FaceSign;

                for (int32 i = 0; i < 4; ++i)
                {
                    const int32 c0 = Corners[i];
                    const int32 c1 = Corners[(i + 1) & 3];

                    const int32 V0[3] = { lx + (c0 & 1), ly + ((c0 >> 1) & 1), lz + ((c0 >> 2) & 1) };
                    const int32 V1[3] = { lx + (c1 & 1), ly + ((c1 >> 1) & 1), lz + ((c1 >> 2) & 1) };

                    int32 EdgeAxis = 0;
                    while (EdgeAxis < 3 && V0[EdgeAxis] == V1[EdgeAxis]) ++EdgeAxis;
                    if (EdgeAxis >= 3) continue;

                    const int32 TAxis = 3 - FaceAxis - EdgeAxis;
                    const int32 TSign = ((c0 >> TAxis) & 1) ? 1 : -1;

                    int32 PT[3] = { lx, ly, lz };          PT[TAxis] += TSign;
                    int32 QT[3] = { QC[0], QC[1], QC[2] }; QT[TAxis] += TSign;

                    const bool bPTAir = Neighborhood.GetVoxel(PT[0], PT[1], PT[2]) == EVoxelType::Air;
                    const bool bQTAir = Neighborhood.GetVoxel(QT[0], QT[1], QT[2]) == EVoxelType::Air;

                    // The emitted face meeting ours at this edge, once per rotation direction.
                    // The two coincide except at a diagonal pinch, where two sheets touch here.
                    const int32* Adj[2] = { nullptr, nullptr };
                    if (!bPTAir) Adj[0] = bQTAir ? PT : QT;
                    if (!bQTAir) Adj[1] = QT; else if (!bPTAir) Adj[1] = PT;

                    for (int32 k = 0; k < 2; ++k)
                    {
                        if (!Adj[k]) continue;
                        if (k == 1 && Adj[0] == Adj[1]) continue;
                        if (!CaveCache->IsCellSmoothSurface(Adj[k][0], Adj[k][1], Adj[k][2])) continue;

                        const FVector R0 = GetSmoothVertexLocal(V0[0], V0[1], V0[2], lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord, CaveCache);
                        const FVector R1 = GetSmoothVertexLocal(V1[0], V1[1], V1[2], lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord, CaveCache);
                        const FVector S0 = GetSmoothVertexLocal(V0[0], V0[1], V0[2], Adj[k][0], Adj[k][1], Adj[k][2], HeightGrid, Neighborhood, ChunkCoord, CaveCache);
                        const FVector S1 = GetSmoothVertexLocal(V1[0], V1[1], V1[2], Adj[k][0], Adj[k][1], Adj[k][2], HeightGrid, Neighborhood, ChunkCoord, CaveCache);

                        // The band continues this face's surface past the rim, so it walks the
                        // shared edge in the opposite direction to the face that owns it.
                        AddQuadWorldSmooth(R1, R0, S0, S1, MatID, UAxis, VAxis, false);
                    }
                }
            };

        // ---- TOP (+Z) ----
        if (bAirTop)
        {
            if (lz == GroundHere)
            {
                FVector n00 = GetSmoothNormalLocal(lx, ly, HeightGrid);
                FVector n10 = GetSmoothNormalLocal(lx + 1, ly, HeightGrid);
                FVector n01 = GetSmoothNormalLocal(lx, ly + 1, HeightGrid);
                FVector n11 = GetSmoothNormalLocal(lx + 1, ly + 1, HeightGrid);
                AddTopQuadSmooth(v001, v011, v111, v101, n00, n01, n11, n10, TopMatID);
            }
            else
            {
                AddQuadWorldSmooth(v001, v011, v111, v101, TopMatID, 0, 1, false);
            }
            EmitRimBands(2, 1, CornersTop, TopMatID, 0, 1);
        }

        // ---- BOTTOM (-Z) ----
        if (bAirBottom)
        {
            AddQuadWorldSmooth(v100, v110, v010, v000, BottomMatID, 0, 1, true);
            EmitRimBands(2, -1, CornersBottom, BottomMatID, 0, 1);
        }

        const float ZOffsetEpsilon = 0.1f;

        auto EmitSide = [&](int32 nx, int32 ny, bool bNeighborAir, int32 NeighborGround,
            const FVector& A, const FVector& B, const FVector& C, const FVector& D,
            int32 UAxis, int32 VAxis, bool bDiagBD,
            int32 FaceAxis, int32 FaceSign, const int32(&Corners)[4])
            {
                if (lz <= NeighborGround)
                {
                    if (bNeighborAir)
                    {
                        AddQuadWorldSmooth(A, B, C, D, SideMatID, UAxis, VAxis, bDiagBD);
                        EmitRimBands(FaceAxis, FaceSign, Corners, SideMatID, UAxis, VAxis);
                    }
                    return;
                }
                if (GetNeighborTopHeightLocal(nx, ny, lz, A, Neighborhood, HeightGrid) < A.Z - ZOffsetEpsilon ||
                    GetNeighborTopHeightLocal(nx, ny, lz, B, Neighborhood, HeightGrid) < B.Z - ZOffsetEpsilon ||
                    GetNeighborTopHeightLocal(nx, ny, lz, C, Neighborhood, HeightGrid) < C.Z - ZOffsetEpsilon ||
                    GetNeighborTopHeightLocal(nx, ny, lz, D, Neighborhood, HeightGrid) < D.Z - ZOffsetEpsilon)
                {
                    AddQuadWorldSmooth(A, B, C, D, SideMatID, UAxis, VAxis, bDiagBD);
                }
            };

        EmitSide(lx + 1, ly, bAirEast, GroundEast, v100, v101, v111, v110, 1, 2, false, 0, 1, CornersEast);
        EmitSide(lx - 1, ly, bAirWest, GroundWest, v010, v011, v001, v000, 1, 2, true, 0, -1, CornersWest);
        EmitSide(lx, ly + 1, bAirNorth, GroundNorth, v110, v111, v011, v010, 0, 2, true, 1, 1, CornersNorth);
        EmitSide(lx, ly - 1, bAirSouth, GroundSouth, v000, v001, v101, v100, 0, 2, false, 1, -1, CornersSouth);
    }
}

void FTerrainGenConfig::AppendGrassBladesLocal(int32 lx, int32 ly, int32 lz, FDynamicMesh3& Mesh, FTriIDArray& OutTriIDs, const FLocalHeightGrid& HeightGrid, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord) const
{
    FDynamicMeshAttributeSet* Attr = Mesh.Attributes();
    if (!Attr) return;

    FDynamicMeshUVOverlay* UVOverlay0 = Attr->GetUVLayer(0);
    if (!UVOverlay0) return;

    if (Attr->NumUVLayers() < 2) Attr->SetNumUVLayers(2);
    FDynamicMeshUVOverlay* UVOverlay1 = Attr->GetUVLayer(1);

    int32 WorldX = ChunkCoord.X * ChunkSize + lx;
    int32 WorldY = ChunkCoord.Y * ChunkSize + ly;

    float DensityNoise = FastPerlinNoise2D((float)WorldX * GrassDensityNoiseScale, (float)WorldY * GrassDensityNoiseScale) * 0.5f + 0.5f;
    float FineNoise = Hash3D(WorldX, WorldY, 999);

    if (FineNoise < 0.20f) DensityNoise *= 0.1f;
    else if (FineNoise > 0.85f) DensityNoise = FMath::Min(1.0f, DensityNoise * 1.5f);

    float TargetDensity = FMath::Lerp((float)GrassMinDensity, (float)GrassMaxDensity, DensityNoise) + (Hash3D(WorldX, WorldY, 888) - 0.5f) * 3.0f;
    int32 Density = FMath::Clamp(FMath::RoundToInt(TargetDensity), 0, GrassMaxDensity + 2);

    for (int32 i = 0; i < Density; ++i)
    {
        FFastRandom FastRand((uint32)WorldX * 73856093U ^ (uint32)WorldY * 19349663U ^ (uint32)i * 83492791U);
        float RandX = FastRand.NextFloat(), RandY = FastRand.NextFloat(), RandHeight = FastRand.NextFloat(), RandWidth = FastRand.NextFloat();
        float RandAngle = FastRand.NextFloat(), RandLeanAngle = FastRand.NextFloat(), RandLeanStrength = FastRand.NextFloat();
        float RandBendAngle = FastRand.NextFloat(), RandBendForce = FastRand.NextFloat();

        float BladeLocalX = (float)lx + RandX, BladeLocalY = (float)ly + RandY, BladeWorldZ = 0.0f;
        FVector GroundNormal(0.f, 0.f, 1.f);

        if (bSmoothTerrain) {
            if (lz == GetGroundLevelLocal(lx, ly, HeightGrid)) {
                BladeWorldZ = GetInterpolatedHeightLocal(BladeLocalX, BladeLocalY, HeightGrid) * CubeSize;
                GroundNormal = GetSmoothNormalLocal(FMath::RoundToInt(BladeLocalX), FMath::RoundToInt(BladeLocalY), HeightGrid);
            }
            else {
                BladeWorldZ = (float)(lz + 1 + BedrockLevel) * CubeSize;
            }
        }
        else BladeWorldZ = (float)(lz + 1 + BedrockLevel) * CubeSize;

        FVector BasePos((double)(WorldX + RandX) * CubeSize, (double)(WorldY + RandY) * CubeSize, (double)BladeWorldZ);

        float Height = GrassMinHeight + (GrassMaxHeight - GrassMinHeight) * RandHeight;
        float Width = GrassMinWidth + (GrassMaxWidth - GrassMinWidth) * RandWidth;

        float Angle = RandAngle * 2.0f * PI, SinAngle, CosAngle; FMath::SinCos(&SinAngle, &CosAngle, Angle);
        FVector BladeRight(CosAngle, SinAngle, 0.0f), BladeForward(-SinAngle, CosAngle, 0.0f);

        float LeanAngle = RandLeanAngle * 2.0f * PI, SinLean, CosLean; FMath::SinCos(&SinLean, &CosLean, LeanAngle);
        FVector TiltingNormal = (GroundNormal + FVector(CosLean, SinLean, 0.0f) * (0.05f + 0.15f * RandLeanStrength)).GetSafeNormal();

        float BendAngle = RandBendAngle * 2.0f * PI, SinBend, CosBend; FMath::SinCos(&SinBend, &CosBend, BendAngle);
        FVector BendDir = (FVector(CosBend, SinBend, 0.0f) * 0.5f + BladeForward * 0.3f + GroundNormal * 0.2f).GetSafeNormal();

        float BendForce = (0.15f + 0.35f * RandBendForce) * Height;

        FDynamicMeshNormalOverlay* NormalOverlay = Attr->PrimaryNormals();
        int32 nGround = NormalOverlay ? NormalOverlay->AppendElement(FVector3f(GroundNormal)) : -1;

        bool bLocalTwoSided = bTwoSidedGrass;
        auto AddTri = [UVOverlay0, UVOverlay1, NormalOverlay, nGround, &Mesh, &OutTriIDs, bLocalTwoSided](
            int32 a, int32 b, int32 c, int32 u0_A, int32 u0_B, int32 u0_C, int32 u1_A, int32 u1_B, int32 u1_C)
            {
                int32 t = Mesh.AppendTriangle(a, b, c);
                if (t != FDynamicMesh3::InvalidID) {
                    OutTriIDs.Add(t);
                    if (NormalOverlay && nGround != -1) NormalOverlay->SetTriangle(t, FIndex3i(nGround, nGround, nGround));
                    if (UVOverlay0 && u0_A != -1) UVOverlay0->SetTriangle(t, FIndex3i(u0_A, u0_B, u0_C));
                    if (UVOverlay1 && u1_A != -1) UVOverlay1->SetTriangle(t, FIndex3i(u1_A, u1_B, u1_C));
                }
                if (!bLocalTwoSided) {
                    int32 tBack = Mesh.AppendTriangle(a, c, b);
                    if (tBack != FDynamicMesh3::InvalidID) {
                        OutTriIDs.Add(tBack);
                        if (NormalOverlay && nGround != -1) NormalOverlay->SetTriangle(tBack, FIndex3i(nGround, nGround, nGround));
                        if (UVOverlay0 && u0_A != -1) UVOverlay0->SetTriangle(tBack, FIndex3i(u0_A, u0_C, u0_B));
                        if (UVOverlay1 && u1_A != -1) UVOverlay1->SetTriangle(tBack, FIndex3i(u1_A, u1_C, u1_B));
                    }
                }
            };

        float RandMat = FastRand.NextFloat();

        if (GrassBladeSegments <= 1)
        {
            int32 v0 = Mesh.AppendVertex(FVector3d(BasePos - BladeRight * (Width * 0.5f))), v1 = Mesh.AppendVertex(FVector3d(BasePos + BladeRight * (Width * 0.5f))), v2 = Mesh.AppendVertex(FVector3d(BasePos + BendDir * BendForce + TiltingNormal * Height));
            int32 uv0_0 = UVOverlay0->AppendElement(FVector2f(0.0f, 0.0f)), uv0_1 = UVOverlay0->AppendElement(FVector2f(1.0f, 0.0f)), uv0_2 = UVOverlay0->AppendElement(FVector2f(0.5f, 1.0f));
            int32 uv1_0 = UVOverlay1 ? UVOverlay1->AppendElement(FVector2f(RandMat, 0.0f)) : -1, uv1_1 = uv1_0, uv1_2 = UVOverlay1 ? UVOverlay1->AppendElement(FVector2f(RandMat, 1.0f)) : -1;
            AddTri(v0, v1, v2, uv0_0, uv0_1, uv0_2, uv1_0, uv1_1, uv1_2);
        }
        else
        {
            int32 v0 = Mesh.AppendVertex(FVector3d(BasePos - BladeRight * (Width * 0.5f))), v1 = Mesh.AppendVertex(FVector3d(BasePos + BladeRight * (Width * 0.5f)));
            int32 v2 = Mesh.AppendVertex(FVector3d(BasePos - BladeRight * (Width * 0.3f) + BendDir * (BendForce * 0.35f) + TiltingNormal * (Height * 0.5f)));
            int32 v3 = Mesh.AppendVertex(FVector3d(BasePos + BladeRight * (Width * 0.3f) + BendDir * (BendForce * 0.35f) + TiltingNormal * (Height * 0.5f)));
            int32 v4 = Mesh.AppendVertex(FVector3d(BasePos + BendDir * BendForce + TiltingNormal * Height));

            int32 uv0_0 = UVOverlay0->AppendElement(FVector2f(0.0f, 0.0f)), uv0_1 = UVOverlay0->AppendElement(FVector2f(1.0f, 0.0f)), uv0_2 = UVOverlay0->AppendElement(FVector2f(0.15f, 0.5f)), uv0_3 = UVOverlay0->AppendElement(FVector2f(0.85f, 0.5f)), uv0_4 = UVOverlay0->AppendElement(FVector2f(0.5f, 1.0f));
            int32 uv1_0 = UVOverlay1 ? UVOverlay1->AppendElement(FVector2f(RandMat, 0.0f)) : -1, uv1_1 = uv1_0, uv1_2 = UVOverlay1 ? UVOverlay1->AppendElement(FVector2f(RandMat, 0.5f)) : -1, uv1_3 = uv1_2, uv1_4 = UVOverlay1 ? UVOverlay1->AppendElement(FVector2f(RandMat, 1.0f)) : -1;

            AddTri(v0, v1, v3, uv0_0, uv0_1, uv0_3, uv1_0, uv1_1, uv1_3);
            AddTri(v0, v3, v2, uv0_0, uv0_3, uv0_2, uv1_0, uv1_3, uv1_2);
            AddTri(v2, v3, v4, uv0_2, uv0_3, uv0_4, uv1_2, uv1_3, uv1_4);
        }
    }
}

void ASmoothVoxelTerrain::FVoxelChunk::UpdateVoxelMesh(int32 LocalX, int32 LocalY, int32 LocalZ, EVoxelType NewType, ASmoothVoxelTerrain* TerrainOwner)
{
    if (!MeshComponent || !VoxelData) return;
    UDynamicMesh* DynamicMesh = MeshComponent->GetDynamicMesh();
    UDynamicMesh* GrassDynamicMesh = (bGrassGenerated && GrassMeshComponent) ? GrassMeshComponent->GetDynamicMesh() : nullptr;
    if (!DynamicMesh) return;

    auto UpdateBlockLogic = [&](FDynamicMesh3& MeshOut, FDynamicMesh3* GrassMeshOut)
        {
            MeshOut.EnableAttributes();
            if (GrassMeshOut) GrassMeshOut->EnableAttributes();
            FDynamicMeshAttributeSet* Attr = MeshOut.Attributes();
            if (Attr && Attr->NumUVLayers() < 2) Attr->SetNumUVLayers(2);
            if (Attr && !Attr->PrimaryColors()) Attr->EnablePrimaryColors();
            if (Attr && !Attr->HasMaterialID()) Attr->EnableMaterialID();
            FDynamicMeshAttributeSet* GrassAttr = GrassMeshOut ? GrassMeshOut->Attributes() : nullptr;
            if (GrassAttr && GrassAttr->NumUVLayers() < 2) GrassAttr->SetNumUVLayers(2);

            for (int32 dz = -1; dz <= 1; ++dz) {
                for (int32 dy = -1; dy <= 1; ++dy) {
                    for (int32 dx = -1; dx <= 1; ++dx) {
                        int32 nx = LocalX + dx, ny = LocalY + dy, nz = LocalZ + dz;
                        if (nx >= 0 && nx < TerrainOwner->ChunkSize && ny >= 0 && ny < TerrainOwner->ChunkSize && nz >= 0 && nz < TerrainOwner->MaxHeight)
                            RemoveVoxelFaces(nx, ny, nz, MeshOut, GrassMeshOut, TerrainOwner);
                    }
                }
            }

            int32 Index = LocalX + LocalY * TerrainOwner->ChunkSize + LocalZ * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;
            (*VoxelData)[Index] = NewType;

            for (int32 dz = -1; dz <= 1; ++dz) {
                for (int32 dy = -1; dy <= 1; ++dy) {
                    for (int32 dx = -1; dx <= 1; ++dx) {
                        int32 nx = LocalX + dx, ny = LocalY + dy, nz = LocalZ + dz;
                        if (nx >= 0 && nx < TerrainOwner->ChunkSize && ny >= 0 && ny < TerrainOwner->ChunkSize && nz >= 0 && nz < TerrainOwner->MaxHeight)
                            if ((*VoxelData)[nx + ny * TerrainOwner->ChunkSize + nz * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize] != EVoxelType::Air)
                                AddVoxelFaces(nx, ny, nz, MeshOut, GrassMeshOut, TerrainOwner);
                    }
                }
            }
        };

    DynamicMesh->EditMesh([&](FDynamicMesh3& MeshOut) {
        if (GrassDynamicMesh) GrassDynamicMesh->EditMesh([&](FDynamicMesh3& GrassMeshOut) { UpdateBlockLogic(MeshOut, &GrassMeshOut); FMeshNormals::QuickComputeVertexNormals(GrassMeshOut); });
        else UpdateBlockLogic(MeshOut, nullptr);
        });
    MeshComponent->UpdateCollision(true);
}

void ASmoothVoxelTerrain::FVoxelChunk::RemoveVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, FDynamicMesh3& Mesh, FDynamicMesh3* GrassMesh, ASmoothVoxelTerrain* TerrainOwner)
{
    int32 VoxelIndex = LocalX + LocalY * TerrainOwner->ChunkSize + LocalZ * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;
    if (auto* TriIDsPtr = VoxelTriangles.Find(VoxelIndex)) {
        for (int32 TriID : *TriIDsPtr) { if (Mesh.IsTriangle(TriID)) Mesh.RemoveTriangle(TriID, false); }
        VoxelTriangles.Remove(VoxelIndex);
    }
    if (GrassMesh) {
        if (auto* GrassTriIDsPtr = GrassVoxelTriangles.Find(VoxelIndex)) {
            for (int32 TriID : *GrassTriIDsPtr) { if (GrassMesh->IsTriangle(TriID)) GrassMesh->RemoveTriangle(TriID, false); }
            GrassVoxelTriangles.Remove(VoxelIndex);
        }
    }
}

void ASmoothVoxelTerrain::FVoxelChunk::AddVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, FDynamicMesh3& Mesh, FDynamicMesh3* GrassMesh, ASmoothVoxelTerrain* TerrainOwner)
{
    if (!VoxelData) return;
    int32 VoxelIndex = LocalX + LocalY * TerrainOwner->ChunkSize + LocalZ * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;

    FChunkNeighborhood Neighborhood;
    Neighborhood.SelfData = VoxelData->GetData();
    Neighborhood.ChunkSize = TerrainOwner->ChunkSize;
    Neighborhood.MaxHeight = TerrainOwner->MaxHeight;
    Neighborhood.StepY = TerrainOwner->ChunkSize;
    Neighborhood.StepZ = TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;
    Neighborhood.SelfCoord = Coord;

    auto RetrieveVoxelDataPtr = [&](const FIntVector& Offset) -> const EVoxelType* {
        if (const FVoxelChunk* Target = TerrainOwner->GetChunk(Coord + Offset))
            if (Target->VoxelData) return Target->VoxelData->GetData();
        return nullptr;
        };

    Neighborhood.WestData = RetrieveVoxelDataPtr(FIntVector(-1, 0, 0));
    Neighborhood.EastData = RetrieveVoxelDataPtr(FIntVector(1, 0, 0));
    Neighborhood.SouthData = RetrieveVoxelDataPtr(FIntVector(0, -1, 0));
    Neighborhood.NorthData = RetrieveVoxelDataPtr(FIntVector(0, 1, 0));
    Neighborhood.SouthWestData = RetrieveVoxelDataPtr(FIntVector(-1, -1, 0));
    Neighborhood.SouthEastData = RetrieveVoxelDataPtr(FIntVector(1, -1, 0));
    Neighborhood.NorthWestData = RetrieveVoxelDataPtr(FIntVector(-1, 1, 0));
    Neighborhood.NorthEastData = RetrieveVoxelDataPtr(FIntVector(1, 1, 0));

    FLocalHeightGrid HeightGrid;
    HeightGrid.Heights = HeightMap ? HeightMap->GetData() : nullptr;
    HeightGrid.CacheSize = TerrainOwner->ChunkSize + 5;
    FTriIDArray NewTriIDs;

    const FTerrainGenConfig Config = TerrainOwner->GetTerrainConfig();

    static thread_local FCaveSmoothCache CaveCache;
    CaveCache.Init(&Config, &HeightGrid, &Neighborhood, Coord);

    Config.AppendVoxelFacesLocal(LocalX, LocalY, LocalZ, Mesh, NewTriIDs, HeightGrid, Neighborhood, Coord,
        CaveCache.IsReady() ? &CaveCache : nullptr);
    if (NewTriIDs.Num() > 0) VoxelTriangles.Add(VoxelIndex, NewTriIDs);

    if (GrassMesh && TerrainOwner->bEnableGrassGeometry && (*VoxelData)[VoxelIndex] == EVoxelType::Grass)
    {
        if (Neighborhood.GetVoxel(LocalX, LocalY, LocalZ + 1) == EVoxelType::Air) {
            FTriIDArray NewGrassTriIDs;
            Config.AppendGrassBladesLocal(LocalX, LocalY, LocalZ, *GrassMesh, NewGrassTriIDs, HeightGrid, Neighborhood, Coord);
            if (NewGrassTriIDs.Num() > 0) GrassVoxelTriangles.Add(VoxelIndex, NewGrassTriIDs);
        }
    }
}

ASmoothVoxelTerrain::FVoxelChunk* ASmoothVoxelTerrain::GetChunk(const FIntVector& Coord)
{
    if (auto* Ptr = Chunks.Find(Coord)) return Ptr->IsValid() ? Ptr->Get() : nullptr;
    return nullptr;
}

const ASmoothVoxelTerrain::FVoxelChunk* ASmoothVoxelTerrain::GetChunk(const FIntVector& Coord) const
{
    if (auto* Ptr = Chunks.Find(Coord)) return Ptr->IsValid() ? Ptr->Get() : nullptr;
    return nullptr;
}

#if WITH_EDITOR
void ASmoothVoxelTerrain::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);
    static const TArray<FName> RelevantProperties = {
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, CollisionEnabled), GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, CollisionProfileName),
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, bGenerateOverlapEvents), GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, bCastShadow),
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, bReceivesDecals), GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, GrassMaterial),
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, DirtMaterial), GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, StoneMaterial),
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, GrassBladesMaterial), GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, WaterMaterial)
    };

    if (RelevantProperties.Contains(PropertyChangedEvent.GetPropertyName()))
    {
        for (auto& Pair : Chunks) {
            if (Pair.Value && Pair.Value->MeshComponent) {
                Pair.Value->MeshComponent->SetCollisionEnabled(CollisionEnabled);
                Pair.Value->MeshComponent->SetCollisionProfileName(CollisionProfileName);
                Pair.Value->MeshComponent->SetGenerateOverlapEvents(bGenerateOverlapEvents);
                Pair.Value->MeshComponent->SetCastShadow(bCastShadow);
                Pair.Value->MeshComponent->SetReceivesDecals(bReceivesDecals);
                if (GrassMaterial) Pair.Value->MeshComponent->SetMaterial(0, GrassMaterial);
                if (DirtMaterial) Pair.Value->MeshComponent->SetMaterial(1, DirtMaterial);
                if (StoneMaterial) Pair.Value->MeshComponent->SetMaterial(2, StoneMaterial);
            }
            if (Pair.Value && Pair.Value->GrassMeshComponent) {
                Pair.Value->GrassMeshComponent->SetReceivesDecals(bReceivesDecals);
                if (GrassBladesMaterial) Pair.Value->GrassMeshComponent->SetMaterial(0, GrassBladesMaterial);
            }
            if (Pair.Value && Pair.Value->WaterMeshComponent) {
                if (WaterMaterial) Pair.Value->WaterMeshComponent->SetMaterial(0, WaterMaterial);
            }
        }
    }
}
#endif