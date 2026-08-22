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
// HASHING & NOISE IMPLEMENTATION
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

        const EVoxelType* TargetData = SelfData;
        int32 LX = LocalX, LY = LocalY;

        if (LX < 0) { TargetData = WestData; LX += ChunkSize; }
        else if (LX >= ChunkSize) { TargetData = EastData; LX -= ChunkSize; }

        if (LY < 0) { TargetData = SouthData; LY += ChunkSize; }
        else if (LY >= ChunkSize) { TargetData = NorthData; LY -= ChunkSize; }

        if (!TargetData) return EVoxelType::Stone;

        return TargetData[LX + LY * ChunkSize + LocalZ * StepZ];
    }
};

// ---------------------------------------------------------------------------------
// ACTOR IMPLEMENTATION
// ---------------------------------------------------------------------------------

ASmoothVoxelTerrain::ASmoothVoxelTerrain()
{
    PrimaryActorTick.bCanEverTick = true;
    RootSceneComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    RootSceneComponent->SetMobility(EComponentMobility::Static);
    RootComponent = RootSceneComponent;
}

ASmoothVoxelTerrain::~ASmoothVoxelTerrain()
{
    bIsDestroyed = true;
}

void ASmoothVoxelTerrain::BeginDestroy()
{
    bIsDestroyed = true;
    CleanupAllComponents();
    Super::BeginDestroy();
}

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
    if (!UpdatedComponent || bIsDestroyed) return;
    FIntVector CurrentChunk = WorldToChunkCoord(UpdatedComponent->GetComponentLocation());
    if (CurrentChunk != LastPlayerChunkCoord) HandleBoundaryCrossing(CurrentChunk);
}

void ASmoothVoxelTerrain::HandleBoundaryCrossing(const FIntVector& NewChunkCoord)
{
    if (bIsDestroyed) return;
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
                    if (Chunk && Chunk->State == EChunkState::DataReady && CheckNeighborsDataReady(Coord))
                    {
                        MeshGenerationQueue.Add(Coord);
                    }
                    if (Chunk && DistSq <= GrassRadiusSq)
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
                    if (Chunk)
                    {
                        if (Chunk->GrassMeshComponent.IsValid())
                        {
                            ReleaseMeshComponent(Chunk->GrassMeshComponent.Get(), 1);
                            Chunk->GrassMeshComponent = nullptr;
                        }
                        Chunk->GrassVoxelTriangles.Empty();
                        Chunk->bGrassGenerated = false;
                        Chunk->bGeneratingGrass = false;
                        GrassGenerationQueue.Remove(Coord);
                    }
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
    if (!TrackedPlayerComponent.IsValid() || bIsDestroyed) return;
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

        if (Chunk->GrassMeshComponent.IsValid() && Chunk->bGrassGenerated)
        {
            bool bShouldBeVisible = bEnableGrassGeometry && (DistSq <= GrassRadiusSq);
            if (Chunk->GrassMeshComponent->IsVisible() != bShouldBeVisible) Chunk->GrassMeshComponent->SetVisibility(bShouldBeVisible);
        }

        if (Chunk->MeshComponent.IsValid())
        {
            bool bShouldCastShadow = bCastShadow && (DistSq <= ShadowRadiusSq);
            if (Chunk->MeshComponent->CastShadow != bShouldCastShadow) Chunk->MeshComponent->SetCastShadow(bShouldCastShadow);
        }
    }
}

void ASmoothVoxelTerrain::UpdateCollisionIfNeeded()
{
    if (bCollisionDirty && !bIsDestroyed)
    {
        for (auto& Pair : Chunks)
        {
            if (Pair.Value && Pair.Value->MeshComponent.IsValid())
            {
                Pair.Value->MeshComponent->UpdateCollision(false);
            }
        }
        bCollisionDirty = false;
    }
}

void ASmoothVoxelTerrain::CleanupAllComponents()
{
    for (TObjectPtr<UDynamicMeshComponent>& Comp : ActiveComponents)
    {
        if (IsValid(Comp))
        {
            Comp->UnregisterComponent();
            Comp->DestroyComponent();
        }
    }
    ActiveComponents.Empty();
    MeshComponentPool.Empty();
    GrassMeshComponentPool.Empty();
    WaterMeshComponentPool.Empty();
    Chunks.Empty();
}

void ASmoothVoxelTerrain::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    bIsDestroyed = true;
    GenerationEpoch++;

    DataGenerationQueue.Empty();
    MeshGenerationQueue.Empty();
    GrassGenerationQueue.Empty();
    MeshApplyQueue.Empty();
    GrassApplyQueue.Empty();

    CleanupAllComponents();
    Super::EndPlay(EndPlayReason);
}

UDynamicMeshComponent* ASmoothVoxelTerrain::AcquireMeshComponent(int32 MeshType)
{
    if (bIsDestroyed) return nullptr;

    TArray<TObjectPtr<UDynamicMeshComponent>>* Pool = nullptr;
    if (MeshType == 1) Pool = &GrassMeshComponentPool;
    else if (MeshType == 2) Pool = &WaterMeshComponentPool;
    else Pool = &MeshComponentPool;

    while (Pool->Num() > 0)
    {
        UDynamicMeshComponent* Comp = Pool->Pop();
        if (IsValid(Comp))
        {
            Comp->SetVisibility(true);
            return Comp;
        }
    }

    UDynamicMeshComponent* Comp = NewObject<UDynamicMeshComponent>(this);
    if (!Comp) return nullptr;

    Comp->CreationMethod = EComponentCreationMethod::Instance;
    Comp->SetupAttachment(RootSceneComponent);
    Comp->SetMobility(EComponentMobility::Static);
    Comp->RegisterComponent();

    // Synchronous cooking to prevent Chaos background thread races
    Comp->bUseAsyncCooking = false;
    Comp->bDeferCollisionUpdates = false;

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

        if (GrassMaterial) Comp->SetMaterial(0, GrassMaterial);
        if (DirtMaterial) Comp->SetMaterial(1, DirtMaterial);
        if (StoneMaterial) Comp->SetMaterial(2, StoneMaterial);
        if (SurfaceMaterial) Comp->SetMaterial(3, SurfaceMaterial);
    }

    ActiveComponents.Add(Comp);
    return Comp;
}

void ASmoothVoxelTerrain::ReleaseMeshComponent(UDynamicMeshComponent* Comp, int32 MeshType)
{
    if (Comp && IsValid(Comp))
    {
        Comp->SetVisibility(false);
        if (UDynamicMesh* DynMesh = Comp->GetDynamicMesh())
        {
            DynMesh->EditMesh([](FDynamicMesh3& MeshOut) { MeshOut.Clear(); });
        }
        if (MeshType == 1) GrassMeshComponentPool.Add(Comp);
        else if (MeshType == 2) WaterMeshComponentPool.Add(Comp);
        else MeshComponentPool.Add(Comp);
    }
}

void ASmoothVoxelTerrain::GenerateChunks()
{
    GenerationEpoch++;
    for (auto& Pair : Chunks)
    {
        if (Pair.Value)
        {
            if (Pair.Value->MeshComponent.IsValid()) ReleaseMeshComponent(Pair.Value->MeshComponent.Get(), 0);
            if (Pair.Value->GrassMeshComponent.IsValid()) ReleaseMeshComponent(Pair.Value->GrassMeshComponent.Get(), 1);
            if (Pair.Value->WaterMeshComponent.IsValid()) ReleaseMeshComponent(Pair.Value->WaterMeshComponent.Get(), 2);
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
    if (bIsDestroyed) return;

    int32 AppliedCount = 0;
    while (MeshApplyQueue.Num() > 0 && AppliedCount < MaxMeshApplyPerFrame)
    {
        auto Task = MeshApplyQueue[0];
        MeshApplyQueue.RemoveAt(0);

        if (!Task.IsValid() || Task->Epoch != GenerationEpoch) continue;

        if (FVoxelChunk* Chunk = GetChunk(Task->Coord))
        {
            if (Chunk->State == EChunkState::GeneratingMesh)
            {
                Chunk->State = EChunkState::MeshReady;
                if (!Chunk->MeshComponent.IsValid()) Chunk->MeshComponent = AcquireMeshComponent(0);

                if (Chunk->MeshComponent.IsValid())
                {
                    Chunk->VoxelTriangles = MoveTemp(Task->VoxelTriangles);
                    Chunk->MeshComponent->SetMesh(MoveTemp(Task->LocalMesh));
                    Chunk->MeshComponent->UpdateCollision(true);
                }

                if (bEnableWater && !Chunk->bWaterGenerated)
                {
                    bool bNeedsWater = false;
                    int32 SeaLevelLocalZ = SeaLevel - BedrockLevel;

                    if (Chunk->VoxelData)
                    {
                        if (SeaLevelLocalZ >= 0 && SeaLevelLocalZ < MaxHeight)
                        {
                            for (int32 i = 0; i < ChunkSize * ChunkSize; ++i) {
                                if ((*Chunk->VoxelData)[i + SeaLevelLocalZ * ChunkSize * ChunkSize] == EVoxelType::Air) {
                                    bNeedsWater = true;
                                    break;
                                }
                            }
                        }
                        else if (SeaLevelLocalZ >= MaxHeight)
                        {
                            bNeedsWater = true;
                        }
                    }

                    if (bNeedsWater)
                    {
                        Chunk->bWaterGenerated = true;
                        if (!Chunk->WaterMeshComponent.IsValid()) Chunk->WaterMeshComponent = AcquireMeshComponent(2);

                        if (Chunk->WaterMeshComponent.IsValid())
                        {
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

                            int32 t1 = WaterMesh.AppendTriangle(v0, v1, v2), t2 = WaterMesh.AppendTriangle(v0, v2, v3);

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

                            FMeshNormals::QuickComputeVertexNormals(WaterMesh);
                            Chunk->WaterMeshComponent->SetMesh(MoveTemp(WaterMesh));
                        }
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

        if (!Task.IsValid() || Task->Epoch != GenerationEpoch) continue;

        if (FVoxelChunk* Chunk = GetChunk(Task->Coord))
        {
            if (Chunk->bGeneratingGrass && (FMath::Square(Task->Coord.X - LastPlayerChunkCoord.X) + FMath::Square(Task->Coord.Y - LastPlayerChunkCoord.Y)) <= FMath::Square(FMath::Min(GrassRenderDistance + 1, RenderDistance)))
            {
                Chunk->bGeneratingGrass = false; Chunk->bGrassGenerated = true;
                if (!Chunk->GrassMeshComponent.IsValid()) Chunk->GrassMeshComponent = AcquireMeshComponent(1);

                if (Chunk->GrassMeshComponent.IsValid())
                {
                    Chunk->GrassVoxelTriangles = MoveTemp(Task->GrassVoxelTriangles);
                    Chunk->GrassMeshComponent->SetMesh(MoveTemp(Task->LocalGrassMesh));
                }
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
    FIntVector Neighbors[4] = { FIntVector(ChunkCoord.X - 1, ChunkCoord.Y, 0), FIntVector(ChunkCoord.X + 1, ChunkCoord.Y, 0), FIntVector(ChunkCoord.X, ChunkCoord.Y - 1, 0), FIntVector(ChunkCoord.X, ChunkCoord.Y + 1, 0) };
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
    Config.Seed = Seed; Config.bEnableWater = bEnableWater;
    Config.SeaLevel = SeaLevel; Config.GrasslandBiome = GrasslandBiome;
    Config.CaveSettings = CaveSettings;
    Config.bEnableGrassGeometry = bEnableGrassGeometry;
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
    uint32 TaskEpoch = GenerationEpoch;

    Async(EAsyncExecution::ThreadPool, [WeakThis, ChunkCoord, Config, TaskEpoch]() mutable
        {
            int32 LocalChunkSize = Config.ChunkSize, LocalMaxHeight = Config.MaxHeight, LocalBedrockLevel = Config.BedrockLevel;

            TSharedPtr<TArray<EVoxelType>, ESPMode::ThreadSafe> LocalVoxelData = MakeShared<TArray<EVoxelType>, ESPMode::ThreadSafe>();
            LocalVoxelData->SetNumZeroed(LocalChunkSize * LocalChunkSize * LocalMaxHeight);

            int32 StepY = LocalChunkSize;
            int32 StepZ = LocalChunkSize * LocalChunkSize;

            TArray<float> ColumnHeights;
            ColumnHeights.SetNumUninitialized(LocalChunkSize * LocalChunkSize);

            // 1. Generate Base Terrain Heights & Initial Core Voxels
            for (int32 ly = 0; ly < LocalChunkSize; ++ly) {
                for (int32 lx = 0; lx < LocalChunkSize; ++lx) {
                    int32 WorldX = ChunkCoord.X * LocalChunkSize + lx;
                    int32 WorldY = ChunkCoord.Y * LocalChunkSize + ly;
                    float Height = Config.GetTerrainHeight(WorldX, WorldY);
                    ColumnHeights[lx + ly * StepY] = Height;

                    int32 IntHeight = FMath::FloorToInt(Height);
                    int32 BaseIdx = lx + ly * StepY;

                    for (int32 lz = 0; lz < LocalMaxHeight; ++lz) {
                        int32 Index = BaseIdx + lz * StepZ;
                        int32 WorldZ = lz + LocalBedrockLevel;

                        if (lz <= 0) {
                            (*LocalVoxelData)[Index] = EVoxelType::Stone; // Bedrock
                        }
                        else if (WorldZ <= IntHeight) {
                            if (WorldZ == IntHeight) {
                                (*LocalVoxelData)[Index] = EVoxelType::Grass;
                            }
                            else if (WorldZ >= IntHeight - 3) {
                                (*LocalVoxelData)[Index] = EVoxelType::Dirt;
                            }
                            else {
                                (*LocalVoxelData)[Index] = EVoxelType::Stone;
                            }
                        }
                        else {
                            (*LocalVoxelData)[Index] = EVoxelType::Air;
                        }
                    }
                }
            }

            // 2. Carve Caves (Tunnels & Caverns)
            if (Config.CaveSettings.bEnableCaves)
            {
                for (int32 ly = 0; ly < LocalChunkSize; ++ly) {
                    for (int32 lx = 0; lx < LocalChunkSize; ++lx) {
                        int32 WorldX = ChunkCoord.X * LocalChunkSize + lx;
                        int32 WorldY = ChunkCoord.Y * LocalChunkSize + ly;
                        float SurfaceHeight = ColumnHeights[lx + ly * StepY];
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
                        for (int32 lz = LocalMaxHeight - 2; lz >= 1; --lz) {
                            int32 Index = BaseIdx + lz * StepZ;
                            int32 AboveIndex = Index + StepZ;

                            if ((*LocalVoxelData)[Index] == EVoxelType::Stone && (*LocalVoxelData)[AboveIndex] == EVoxelType::Air)
                            {
                                int32 WorldZ = lz + LocalBedrockLevel;
                                float SurfHeight = ColumnHeights[lx + ly * StepY];
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

            // 3. Clothed Voxels - Surface Layer Placement (for chunk & caves)
            TSet<int32> SurfaceCandidates;

            for (int32 lz = 0; lz < LocalMaxHeight; ++lz) {
                for (int32 ly = -1; ly <= LocalChunkSize; ++ly) {
                    for (int32 lx = -1; lx <= LocalChunkSize; ++lx) {
                        bool bIsCore = false;

                        if (lx >= 0 && lx < LocalChunkSize && ly >= 0 && ly < LocalChunkSize) {
                            bIsCore = (*LocalVoxelData)[lx + ly * StepY + lz * StepZ] != EVoxelType::Air;
                        }
                        else {
                            int32 WorldX = ChunkCoord.X * LocalChunkSize + lx;
                            int32 WorldY = ChunkCoord.Y * LocalChunkSize + ly;
                            int32 WorldZ = lz + LocalBedrockLevel;
                            float Height = Config.GetTerrainHeight(WorldX, WorldY);
                            int32 IntHeight = FMath::FloorToInt(Height);

                            bIsCore = (lz <= 0) || (WorldZ <= IntHeight);
                            if (bIsCore && lz > 0 && Config.CaveSettings.bEnableCaves) {
                                if (Config.IsInsideCave(WorldX, WorldY, WorldZ, Height)) {
                                    bIsCore = false;
                                }
                            }
                        }

                        if (bIsCore) {
                            int32 Neighbors[6][3] = {
                                {lx + 1, ly, lz}, {lx - 1, ly, lz},
                                {lx, ly + 1, lz}, {lx, ly - 1, lz},
                                {lx, ly, lz + 1}, {lx, ly, lz - 1}
                            };

                            for (int32 i = 0; i < 6; ++i) {
                                int32 nx = Neighbors[i][0];
                                int32 ny = Neighbors[i][1];
                                int32 nz = Neighbors[i][2];

                                if (nx >= 0 && nx < LocalChunkSize && ny >= 0 && ny < LocalChunkSize && nz >= 0 && nz < LocalMaxHeight) {
                                    int32 NIndex = nx + ny * StepY + nz * StepZ;
                                    if ((*LocalVoxelData)[NIndex] == EVoxelType::Air) {
                                        SurfaceCandidates.Add(NIndex);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            for (int32 CandidateIndex : SurfaceCandidates) {
                (*LocalVoxelData)[CandidateIndex] = EVoxelType::Surface;
            }

            AsyncTask(ENamedThreads::GameThread, [WeakThis, ChunkCoord, LocalVoxelData, TaskEpoch]()
                {
                    ASmoothVoxelTerrain* Terrain = WeakThis.Get();
                    if (!Terrain || Terrain->bIsDestroyed || Terrain->GenerationEpoch != TaskEpoch) return;

                    if (FVoxelChunk* TargetChunk = Terrain->GetChunk(ChunkCoord)) {
                        TargetChunk->VoxelData = LocalVoxelData; TargetChunk->State = EChunkState::DataReady;
                        if (Terrain->CheckNeighborsDataReady(ChunkCoord)) Terrain->MeshGenerationQueue.AddUnique(ChunkCoord);
                        FIntVector Neighbors[4] = { FIntVector(ChunkCoord.X - 1, ChunkCoord.Y, 0), FIntVector(ChunkCoord.X + 1, ChunkCoord.Y, 0), FIntVector(ChunkCoord.X, ChunkCoord.Y - 1, 0), FIntVector(ChunkCoord.X, ChunkCoord.Y + 1, 0) };
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
    TSharedPtr<TArray<EVoxelType>, ESPMode::ThreadSafe> WestData, EastData, SouthData, NorthData;

    if (auto* C = GetChunk(ChunkCoord + FIntVector(-1, 0, 0))) WestData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(1, 0, 0))) EastData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(0, -1, 0))) SouthData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(0, 1, 0))) NorthData = C->VoxelData;

    FTerrainGenConfig Config = GetTerrainConfig();
    TWeakObjectPtr<ASmoothVoxelTerrain> WeakThis(this);
    uint32 TaskEpoch = GenerationEpoch;

    Async(EAsyncExecution::ThreadPool, [WeakThis, ChunkCoord, Config, SelfData, WestData, EastData, SouthData, NorthData, TaskEpoch]() mutable
        {
            TSharedPtr<FMeshApplyTask, ESPMode::ThreadSafe> ResultTask = MakeShared<FMeshApplyTask, ESPMode::ThreadSafe>();
            ResultTask->Epoch = TaskEpoch;
            ResultTask->Coord = ChunkCoord;
            ResultTask->LocalMesh.EnableAttributes();
            if (FDynamicMeshAttributeSet* Attr = ResultTask->LocalMesh.Attributes()) { Attr->SetNumUVLayers(2); Attr->EnablePrimaryColors(); Attr->EnableMaterialID(); }

            FChunkNeighborhood Neighborhood;
            Neighborhood.SelfData = SelfData->GetData();
            Neighborhood.WestData = WestData ? WestData->GetData() : nullptr; Neighborhood.EastData = EastData ? EastData->GetData() : nullptr;
            Neighborhood.SouthData = SouthData ? SouthData->GetData() : nullptr; Neighborhood.NorthData = NorthData ? NorthData->GetData() : nullptr;
            Neighborhood.ChunkSize = Config.ChunkSize; Neighborhood.MaxHeight = Config.MaxHeight; Neighborhood.StepY = Config.ChunkSize; Neighborhood.StepZ = Config.ChunkSize * Config.ChunkSize;
            Neighborhood.SelfCoord = ChunkCoord;

            FTriIDArray TempTriIDs;

            for (int32 lz = 0; lz < Config.MaxHeight; ++lz) {
                for (int32 ly = 0; ly < Config.ChunkSize; ++ly) {
                    for (int32 lx = 0; lx < Config.ChunkSize; ++lx) {
                        int32 Index = lx + ly * Config.ChunkSize + lz * Neighborhood.StepZ;
                        if (Neighborhood.SelfData[Index] == EVoxelType::Air) continue;
                        TempTriIDs.Reset();
                        Config.AppendVoxelFacesLocal(lx, ly, lz, ResultTask->LocalMesh, TempTriIDs, Neighborhood, ChunkCoord);
                        if (TempTriIDs.Num() > 0) ResultTask->VoxelTriangles.Add(Index, TempTriIDs);
                    }
                }
            }

            FMeshNormals::QuickComputeVertexNormals(ResultTask->LocalMesh);

            AsyncTask(ENamedThreads::GameThread, [WeakThis, ResultTask, TaskEpoch]() {
                if (ASmoothVoxelTerrain* T = WeakThis.Get())
                {
                    if (!T->bIsDestroyed && T->GenerationEpoch == TaskEpoch)
                    {
                        T->MeshApplyQueue.Add(ResultTask);
                    }
                }
                });
        });
}

void ASmoothVoxelTerrain::GenerateGrassMesh(const FIntVector& ChunkCoord)
{
    FVoxelChunk* Chunk = GetChunk(ChunkCoord);
    if (!Chunk || Chunk->State != EChunkState::MeshReady || Chunk->bGeneratingGrass || Chunk->bGrassGenerated) return;
    Chunk->bGeneratingGrass = true;

    TSharedPtr<TArray<EVoxelType>, ESPMode::ThreadSafe> SelfData = Chunk->VoxelData;
    TSharedPtr<TArray<EVoxelType>, ESPMode::ThreadSafe> WestData, EastData, SouthData, NorthData;

    if (auto* C = GetChunk(ChunkCoord + FIntVector(-1, 0, 0))) WestData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(1, 0, 0))) EastData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(0, -1, 0))) SouthData = C->VoxelData;
    if (auto* C = GetChunk(ChunkCoord + FIntVector(0, 1, 0))) NorthData = C->VoxelData;

    FTerrainGenConfig Config = GetTerrainConfig();
    TWeakObjectPtr<ASmoothVoxelTerrain> WeakThis(this);
    uint32 TaskEpoch = GenerationEpoch;

    Async(EAsyncExecution::ThreadPool, [WeakThis, ChunkCoord, Config, SelfData, WestData, EastData, SouthData, NorthData, TaskEpoch]() mutable
        {
            if (!Config.bEnableGrassGeometry) return;
            TSharedPtr<FGrassApplyTask, ESPMode::ThreadSafe> ResultTask = MakeShared<FGrassApplyTask, ESPMode::ThreadSafe>();
            ResultTask->Epoch = TaskEpoch;
            ResultTask->Coord = ChunkCoord;
            ResultTask->LocalGrassMesh.EnableAttributes();
            if (FDynamicMeshAttributeSet* GrassAttr = ResultTask->LocalGrassMesh.Attributes()) GrassAttr->SetNumUVLayers(2);

            FChunkNeighborhood Neighborhood;
            Neighborhood.SelfData = SelfData->GetData();
            Neighborhood.WestData = WestData ? WestData->GetData() : nullptr; Neighborhood.EastData = EastData ? EastData->GetData() : nullptr;
            Neighborhood.SouthData = SouthData ? SouthData->GetData() : nullptr; Neighborhood.NorthData = NorthData ? NorthData->GetData() : nullptr;
            Neighborhood.ChunkSize = Config.ChunkSize; Neighborhood.MaxHeight = Config.MaxHeight; Neighborhood.StepY = Config.ChunkSize; Neighborhood.StepZ = Config.ChunkSize * Config.ChunkSize;
            Neighborhood.SelfCoord = ChunkCoord;

            FTriIDArray TempTriIDs;

            for (int32 lz = 0; lz < Config.MaxHeight; ++lz) {
                for (int32 ly = 0; ly < Config.ChunkSize; ++ly) {
                    for (int32 lx = 0; lx < Config.ChunkSize; ++lx) {
                        int32 Index = lx + ly * Config.ChunkSize + lz * Neighborhood.StepZ;
                        if (Neighborhood.SelfData[Index] == EVoxelType::Grass && Neighborhood.SelfData[Index + Neighborhood.StepZ] == EVoxelType::Air) {
                            TempTriIDs.Reset();
                            Config.AppendGrassBladesLocal(lx, ly, lz, ResultTask->LocalGrassMesh, TempTriIDs, Neighborhood, ChunkCoord);
                            if (TempTriIDs.Num() > 0) ResultTask->GrassVoxelTriangles.Add(Index, TempTriIDs);
                        }
                    }
                }
            }

            FMeshNormals::QuickComputeVertexNormals(ResultTask->LocalGrassMesh);

            AsyncTask(ENamedThreads::GameThread, [WeakThis, ResultTask, TaskEpoch]() {
                if (ASmoothVoxelTerrain* T = WeakThis.Get())
                {
                    if (!T->bIsDestroyed && T->GenerationEpoch == TaskEpoch)
                    {
                        T->GrassApplyQueue.Add(ResultTask);
                    }
                }
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
            if (Chunk->MeshComponent.IsValid()) ReleaseMeshComponent(Chunk->MeshComponent.Get(), 0);
            if (Chunk->GrassMeshComponent.IsValid()) ReleaseMeshComponent(Chunk->GrassMeshComponent.Get(), 1);
            if (Chunk->WaterMeshComponent.IsValid()) ReleaseMeshComponent(Chunk->WaterMeshComponent.Get(), 2);
        }
    }
}

void ASmoothVoxelTerrain::FVoxelChunk::UpdateSharedFace(int32 LocalX, int32 LocalY, int32 LocalZ, ASmoothVoxelTerrain* TerrainOwner, const FIntVector& NeighborDirection)
{
    if (!VoxelData || !TerrainOwner || TerrainOwner->bIsDestroyed) return;
    UDynamicMeshComponent* MeshComp = MeshComponent.Get();
    UDynamicMeshComponent* GrassComp = GrassMeshComponent.Get();
    if (!MeshComp || !IsValid(MeshComp)) return;

    UDynamicMesh* DynamicMesh = MeshComp->GetDynamicMesh();
    UDynamicMesh* GrassDynamicMesh = (bGrassGenerated && GrassComp && IsValid(GrassComp)) ? GrassComp->GetDynamicMesh() : nullptr;
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

        FMeshNormals::QuickComputeVertexNormals(MeshOut);
        });
    MeshComp->UpdateCollision(true);
}

void ASmoothVoxelTerrain::FVoxelChunk::UpdateVoxel(int32 LocalX, int32 LocalY, int32 LocalZ, EVoxelType NewType, ASmoothVoxelTerrain* TerrainOwner)
{
    if (!VoxelData || !TerrainOwner || TerrainOwner->bIsDestroyed) return;
    int32 Index = LocalX + LocalY * TerrainOwner->ChunkSize + LocalZ * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;
    if (!VoxelData->IsValidIndex(Index) || (*VoxelData)[Index] == NewType) return;
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

void ASmoothVoxelTerrain::RemoveVoxel(FVector WorldLocation, FVector HitNormal)
{
    if (bIsDestroyed) return;

    FVector AdjustedLoc = WorldLocation;
    if (!HitNormal.IsNearlyZero()) {
        AdjustedLoc -= HitNormal * (CubeSize * 0.5f);
    }

    FIntVector ChunkCoord = WorldToChunkCoord(AdjustedLoc);
    FVoxelChunk* Chunk = GetChunk(ChunkCoord);
    if (!Chunk || !Chunk->VoxelData) return;

    int32 lx, ly, lz; WorldToLocalVoxel(AdjustedLoc, ChunkCoord, lx, ly, lz);
    if (lx < 0 || lx >= ChunkSize || ly < 0 || ly >= ChunkSize || lz <= 0 || lz >= MaxHeight) return;

    int32 Index = lx + ly * ChunkSize + lz * ChunkSize * ChunkSize;
    if (!Chunk->VoxelData->IsValidIndex(Index) || (*Chunk->VoxelData)[Index] == EVoxelType::Air) return;

    Chunk->UpdateVoxel(lx, ly, lz, EVoxelType::Air, this);

    if (lx == 0) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(-1, 0, 0))) Neighbor->UpdateSharedFace(ChunkSize - 1, ly, lz, this, FIntVector(1, 0, 0)); }
    if (lx == ChunkSize - 1) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(1, 0, 0))) Neighbor->UpdateSharedFace(0, ly, lz, this, FIntVector(-1, 0, 0)); }
    if (ly == 0) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, -1, 0))) Neighbor->UpdateSharedFace(lx, ChunkSize - 1, lz, this, FIntVector(0, 1, 0)); }
    if (ly == ChunkSize - 1) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, 1, 0))) Neighbor->UpdateSharedFace(lx, 0, lz, this, FIntVector(0, -1, 0)); }
    if (lz == 0) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, 0, -1))) Neighbor->UpdateSharedFace(lx, ly, MaxHeight - 1, this, FIntVector(0, 0, 1)); }
    if (lz == MaxHeight - 1) { if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, 0, 1))) Neighbor->UpdateSharedFace(lx, ly, 0, this, FIntVector(0, 0, -1)); }
}

void ASmoothVoxelTerrain::PlaceVoxel(FVector WorldLocation, EVoxelType Type, FVector HitNormal)
{
    if (bIsDestroyed || Type == EVoxelType::Air) return;

    FVector AdjustedLoc = WorldLocation;
    if (!HitNormal.IsNearlyZero()) {
        AdjustedLoc += HitNormal * (CubeSize * 0.5f);
    }

    FIntVector ChunkCoord = WorldToChunkCoord(AdjustedLoc);
    FVoxelChunk* Chunk = GetChunk(ChunkCoord);
    if (!Chunk) return;

    int32 lx, ly, lz; WorldToLocalVoxel(AdjustedLoc, ChunkCoord, lx, ly, lz);
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
// 2D TERRAIN & 3D CAVE GENERATION (Minecraft Style)
// ---------------------------------------------------------------------------------

float FTerrainGenConfig::GetTerrainHeight(int32 WorldX, int32 WorldY) const
{
    float BaseX = (float)WorldX; float BaseY = (float)WorldY;

    // Global Base Noise
    float GlobalBaseNoise = CalculateFBM2D(BaseX, BaseY, 2, GrasslandBiome.GlobalBaseNoiseScale, 1.0f, Seed + 1);

    // Biome Masks
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

    float FinalHeightVoxels = TotalHeight / CubeSize;
    FinalHeightVoxels = FMath::Max(FinalHeightVoxels, (float)FloorLevel);
    FinalHeightVoxels = FMath::Max(FinalHeightVoxels, (float)BedrockLevel + 1.0f);

    float MaxAbsoluteHeight = (float)(BedrockLevel + MaxHeight - 2);
    FinalHeightVoxels = FMath::Min(FinalHeightVoxels, MaxAbsoluteHeight);

    return FinalHeightVoxels;
}

bool FTerrainGenConfig::IsInsideCave(int32 WorldX, int32 WorldY, int32 WorldZ, float SurfaceHeight) const
{
    if (!CaveSettings.bEnableCaves) return false;
    if (WorldZ <= BedrockLevel + CaveSettings.CaveBedrockSafetyMargin) return false;

    float DistBelowSurface = SurfaceHeight - (float)WorldZ;
    if (DistBelowSurface <= 0.0f) return false;

    float DepthFactor = 1.0f;
    if (DistBelowSurface < CaveSettings.CaveMaxHeightOffset)
    {
        float EntranceNoise = FastPerlinNoise2D((float)WorldX * 0.01f + Hash2D(Seed, 120), (float)WorldY * 0.01f + Hash2D(Seed, 121)) * 0.5f + 0.5f;
        if (EntranceNoise > (1.0f - CaveSettings.SurfaceBreakthroughLikelihood))
        {
            DepthFactor = 1.0f;
        }
        else
        {
            DepthFactor = FMath::Clamp(DistBelowSurface / CaveSettings.CaveMaxHeightOffset, 0.0f, 1.0f);
            DepthFactor = DepthFactor * DepthFactor * (3.0f - 2.0f * DepthFactor);
        }
    }

    if (DepthFactor <= 0.01f) return false;

    float fx = (float)WorldX;
    float fy = (float)WorldY;
    float fz = (float)WorldZ;

    // --- 1. Chambers (Caverns / Cheese Caves) ---
    float ChamberMacroMask = FastPerlinNoise3D(
        fx * CaveSettings.ChamberFrequencyScale + Hash3D(Seed, 131, 1),
        fy * CaveSettings.ChamberFrequencyScale + Hash3D(Seed, 132, 2),
        fz * CaveSettings.ChamberFrequencyScale * 1.2f + Hash3D(Seed, 133, 3)
    ) * 0.5f + 0.5f;

    float ChamberNoise = 0.0f;
    bool bInChamber = false;

    if (ChamberMacroMask > 0.42f)
    {
        ChamberNoise = CalculateFBM3D(
            fx, fy, fz * 1.3f, 2,
            CaveSettings.ChamberNoiseScaleXZ,
            1.0f,
            Seed + 140
        );

        float DynamicThreshold = CaveSettings.ChamberThreshold - (ChamberMacroMask - 0.42f) * 0.45f;
        if (ChamberNoise > DynamicThreshold)
        {
            bInChamber = true;
        }
    }

    // --- 2. Tunnels (Spaghetti / Worm Caves) ---
    float RadiusBonus = 0.0f;
    if (ChamberMacroMask > 0.35f)
    {
        RadiusBonus = (ChamberMacroMask - 0.35f) * CaveSettings.TunnelChamberExpansion;
    }

    float EffectiveRadius = (CaveSettings.TunnelBaseRadius + RadiusBonus) * DepthFactor;
    float EffectiveRadiusSq = EffectiveRadius * EffectiveRadius;

    // Primary Tunnel Worm Noise
    float T1A = FastPerlinNoise3D(
        fx * CaveSettings.TunnelNoiseScaleXZ + Hash3D(Seed, 150, 1),
        fy * CaveSettings.TunnelNoiseScaleXZ + Hash3D(Seed, 151, 2),
        fz * CaveSettings.TunnelNoiseScaleY + Hash3D(Seed, 152, 3)
    );
    float T1B = FastPerlinNoise3D(
        fx * CaveSettings.TunnelNoiseScaleXZ + Hash3D(Seed, 153, 4),
        fy * CaveSettings.TunnelNoiseScaleXZ + Hash3D(Seed, 154, 5),
        fz * CaveSettings.TunnelNoiseScaleY + Hash3D(Seed, 155, 6)
    );

    float DistSq1 = T1A * T1A + T1B * T1B;
    if (DistSq1 < EffectiveRadiusSq) return true;

    // Secondary Cross-Cutting Tunnel
    float T2A = FastPerlinNoise3D(
        (fx + 500.0f) * (CaveSettings.TunnelNoiseScaleXZ * 1.15f) + Hash3D(Seed, 160, 1),
        (fy + 500.0f) * (CaveSettings.TunnelNoiseScaleXZ * 1.15f) + Hash3D(Seed, 161, 2),
        fz * (CaveSettings.TunnelNoiseScaleY * 1.25f) + Hash3D(Seed, 162, 3)
    );
    float T2B = FastPerlinNoise3D(
        (fx - 500.0f) * (CaveSettings.TunnelNoiseScaleXZ * 1.15f) + Hash3D(Seed, 163, 4),
        (fy - 500.0f) * (CaveSettings.TunnelNoiseScaleXZ * 1.15f) + Hash3D(Seed, 164, 5),
        fz * (CaveSettings.TunnelNoiseScaleY * 1.25f) + Hash3D(Seed, 165, 6)
    );

    float DistSq2 = T2A * T2A + T2B * T2B;
    if (DistSq2 < EffectiveRadiusSq) return true;

    return bInChamber;
}

FLinearColor FTerrainGenConfig::GetStylizedColorForVoxel(const FVector& WorldPos, EVoxelType VoxelType) const
{
    float VoxX = (float)WorldPos.X / CubeSize, VoxY = (float)WorldPos.Y / CubeSize, VoxZ = (float)WorldPos.Z / CubeSize;
    if (VoxelType == EVoxelType::Grass) return FLinearColor::White;
    else if (VoxelType == EVoxelType::Dirt) return FastColorLerp(FLinearColor(0.12f, 0.07f, 0.05f, 1.0f), FLinearColor(0.20f, 0.12f, 0.08f, 1.0f), FastPerlinNoise2D(VoxX * 0.1f, VoxY * 0.1f) * 0.5f + 0.5f);
    else if (VoxelType == EVoxelType::Stone) return FastColorLerp(FLinearColor(0.18f, 0.20f, 0.22f, 1.0f), FLinearColor(0.30f, 0.32f, 0.34f, 1.0f), FastPerlinNoise3D(VoxX * 0.08f, VoxY * 0.08f, VoxZ * 0.08f) * 0.5f + 0.5f);
    else if (VoxelType == EVoxelType::Surface) return FLinearColor(0.05f, 0.45f, 0.85f, 1.0f);
    return FLinearColor::White;
}

void FTerrainGenConfig::AppendVoxelFacesLocal(int32 lx, int32 ly, int32 lz, FDynamicMesh3& Mesh, FTriIDArray& OutTriIDs, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord) const
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

    bool bExposedTop = Neighborhood.GetVoxel(lx, ly, lz + 1) == EVoxelType::Air;
    bool bExposedBottom = Neighborhood.GetVoxel(lx, ly, lz - 1) == EVoxelType::Air;
    bool bExposedEast = Neighborhood.GetVoxel(lx + 1, ly, lz) == EVoxelType::Air;
    bool bExposedWest = Neighborhood.GetVoxel(lx - 1, ly, lz) == EVoxelType::Air;
    bool bExposedNorth = Neighborhood.GetVoxel(lx, ly + 1, lz) == EVoxelType::Air;
    bool bExposedSouth = Neighborhood.GetVoxel(lx, ly - 1, lz) == EVoxelType::Air;

    if (!bExposedTop && !bExposedBottom && !bExposedEast && !bExposedWest && !bExposedNorth && !bExposedSouth) return;

    FLinearColor VoxelColor = GetStylizedColorForVoxel(FVector((double)WorldX * CubeSize + (0.5 * CubeSize), (double)WorldY * CubeSize + (0.5 * CubeSize), (double)WorldZ * CubeSize), CurrentType);
    int32 cIdx = ColorOverlay->AppendElement(FVector4f(VoxelColor));

    int32 TopMatID = 1, BottomMatID = 1, SideMatID = 1;
    if (CurrentType == EVoxelType::Grass) { TopMatID = 0; BottomMatID = SideMatID = 1; }
    else if (CurrentType == EVoxelType::Dirt) { TopMatID = BottomMatID = SideMatID = 1; }
    else if (CurrentType == EVoxelType::Stone) { TopMatID = BottomMatID = SideMatID = 2; }
    else if (CurrentType == EVoxelType::Surface) { TopMatID = BottomMatID = SideMatID = 3; }

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

            int32 t1 = Mesh.AppendTriangle(vA, vD, vC);
            if (t1 != FDynamicMesh3::InvalidID) {
                OutTriIDs.Add(t1); NormalOverlay->SetTriangle(t1, FIndex3i(nIdx, nIdx, nIdx));
                UVOverlay->SetTriangle(t1, FIndex3i(UVOverlay->AppendElement(FVector2f(uvA)), UVOverlay->AppendElement(FVector2f(uvD)), UVOverlay->AppendElement(FVector2f(uvC))));
                ColorOverlay->SetTriangle(t1, FIndex3i(cIdx, cIdx, cIdx));
                if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t1, MatID);
            }
            int32 t2 = Mesh.AppendTriangle(vA, vC, vB);
            if (t2 != FDynamicMesh3::InvalidID) {
                OutTriIDs.Add(t2); NormalOverlay->SetTriangle(t2, FIndex3i(nIdx, nIdx, nIdx));
                UVOverlay->SetTriangle(t2, FIndex3i(UVOverlay->AppendElement(FVector2f(uvA)), UVOverlay->AppendElement(FVector2f(uvC)), UVOverlay->AppendElement(FVector2f(uvB))));
                ColorOverlay->SetTriangle(t2, FIndex3i(cIdx, cIdx, cIdx));
                if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t2, MatID);
            }
        };

    if (bExposedTop) AddQuadWorldFast(p001, p101, p111, p011, FVector3f(0.f, 0.f, 1.f), TopMatID, 0, 1);
    if (bExposedBottom) AddQuadWorldFast(p100, p000, p010, p110, FVector3f(0.f, 0.f, -1.f), BottomMatID, 0, 1);
    if (bExposedEast) AddQuadWorldFast(p100, p110, p111, p101, FVector3f(1.f, 0.f, 0.f), SideMatID, 1, 2);
    if (bExposedWest) AddQuadWorldFast(p010, p000, p001, p011, FVector3f(-1.f, 0.f, 0.f), SideMatID, 1, 2);
    if (bExposedNorth) AddQuadWorldFast(p110, p010, p011, p111, FVector3f(0.f, 1.f, 0.f), SideMatID, 0, 2);
    if (bExposedSouth) AddQuadWorldFast(p000, p100, p101, p001, FVector3f(0.f, -1.f, 0.f), SideMatID, 0, 2);
}

void FTerrainGenConfig::AppendGrassBladesLocal(int32 lx, int32 ly, int32 lz, FDynamicMesh3& Mesh, FTriIDArray& OutTriIDs, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord) const
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

        float BladeWorldZ = (float)(lz + 1 + BedrockLevel) * CubeSize;
        FVector GroundNormal(0.f, 0.f, 1.f);
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
    if (!VoxelData || !TerrainOwner || TerrainOwner->bIsDestroyed) return;
    UDynamicMeshComponent* MeshComp = MeshComponent.Get();
    UDynamicMeshComponent* GrassComp = GrassMeshComponent.Get();
    if (!MeshComp || !IsValid(MeshComp)) return;

    UDynamicMesh* DynamicMesh = MeshComp->GetDynamicMesh();
    UDynamicMesh* GrassDynamicMesh = (bGrassGenerated && GrassComp && IsValid(GrassComp)) ? GrassComp->GetDynamicMesh() : nullptr;
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

        FMeshNormals::QuickComputeVertexNormals(MeshOut);
        });
    MeshComp->UpdateCollision(true);
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

    FTriIDArray NewTriIDs;

    TerrainOwner->GetTerrainConfig().AppendVoxelFacesLocal(LocalX, LocalY, LocalZ, Mesh, NewTriIDs, Neighborhood, Coord);
    if (NewTriIDs.Num() > 0) VoxelTriangles.Add(VoxelIndex, NewTriIDs);

    if (GrassMesh && TerrainOwner->bEnableGrassGeometry && (*VoxelData)[VoxelIndex] == EVoxelType::Grass)
    {
        if (Neighborhood.GetVoxel(LocalX, LocalY, LocalZ + 1) == EVoxelType::Air) {
            FTriIDArray NewGrassTriIDs;
            TerrainOwner->GetTerrainConfig().AppendGrassBladesLocal(LocalX, LocalY, LocalZ, *GrassMesh, NewGrassTriIDs, Neighborhood, Coord);
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
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, SurfaceMaterial),
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, GrassBladesMaterial), GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, WaterMaterial)
    };

    if (RelevantProperties.Contains(PropertyChangedEvent.GetPropertyName()))
    {
        for (auto& Pair : Chunks) {
            if (Pair.Value && Pair.Value->MeshComponent.IsValid()) {
                Pair.Value->MeshComponent->SetCollisionEnabled(CollisionEnabled);
                Pair.Value->MeshComponent->SetCollisionProfileName(CollisionProfileName);
                Pair.Value->MeshComponent->SetGenerateOverlapEvents(bGenerateOverlapEvents);
                Pair.Value->MeshComponent->SetCastShadow(bCastShadow);
                Pair.Value->MeshComponent->SetReceivesDecals(bReceivesDecals);
                if (GrassMaterial) Pair.Value->MeshComponent->SetMaterial(0, GrassMaterial);
                if (DirtMaterial) Pair.Value->MeshComponent->SetMaterial(1, DirtMaterial);
                if (StoneMaterial) Pair.Value->MeshComponent->SetMaterial(2, StoneMaterial);
                if (SurfaceMaterial) Pair.Value->MeshComponent->SetMaterial(3, SurfaceMaterial);
            }
            if (Pair.Value && Pair.Value->GrassMeshComponent.IsValid()) {
                Pair.Value->GrassMeshComponent->SetReceivesDecals(bReceivesDecals);
                if (GrassBladesMaterial) Pair.Value->GrassMeshComponent->SetMaterial(0, GrassBladesMaterial);
            }
            if (Pair.Value && Pair.Value->WaterMeshComponent.IsValid()) {
                if (WaterMaterial) Pair.Value->WaterMeshComponent->SetMaterial(0, WaterMaterial);
            }
        }
    }
}
#endif