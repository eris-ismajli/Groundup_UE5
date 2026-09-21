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


// A hash, a multiply and a SinCos per lattice corner — four per 2D sample, eight per
// 3D sample — was the single largest cost in the whole generator. Classic Perlin
// gradient tables give the same distribution with a mask and three multiply-adds.
FORCEINLINE uint32 MixHash(uint32 h)
{
    h ^= h >> 13;
    h *= 1274126177U;
    return h ^ (h >> 15);
}


// 16 directions on the unit circle. Unit length, so the old 1.414 scale still applies.
static const float GGrad2D[16][2] = {
    {  1.00000f,  0.00000f }, {  0.92388f,  0.38268f }, {  0.70711f,  0.70711f }, {  0.38268f,  0.92388f },
    {  0.00000f,  1.00000f }, { -0.38268f,  0.92388f }, { -0.70711f,  0.70711f }, { -0.92388f,  0.38268f },
    { -1.00000f,  0.00000f }, { -0.92388f, -0.38268f }, { -0.70711f, -0.70711f }, { -0.38268f, -0.92388f },
    {  0.00000f, -1.00000f }, {  0.38268f, -0.92388f }, {  0.70711f, -0.70711f }, {  0.92388f, -0.38268f }
};

// The 12 cube-edge gradients, padded to 16 so the index is a mask. Length sqrt(2),
// which absorbs the 1.414 the old unit-vector version applied at the end.
static const float GGrad3D[16][3] = {
    {  1,  1,  0 }, { -1,  1,  0 }, {  1, -1,  0 }, { -1, -1,  0 },
    {  1,  0,  1 }, { -1,  0,  1 }, {  1,  0, -1 }, { -1,  0, -1 },
    {  0,  1,  1 }, {  0, -1,  1 }, {  0,  1, -1 }, {  0, -1, -1 },
    {  1,  1,  0 }, {  0, -1,  1 }, { -1,  1,  0 }, {  0, -1, -1 }
};

// Quintic curve for smoother interpolation
FORCEINLINE float Fade(float t) { return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); }
FORCEINLINE float FastPerlinNoise2D(float x, float y)
{
    const int32 ix = FMath::FloorToInt(x), iy = FMath::FloorToInt(y);
    const float fx = x - ix, fy = y - iy;

    // Per-axis products are shared by the four corners: six multiplies instead of eight
    // full hashes.
    const uint32 hx0 = (uint32)ix * 374761393U, hx1 = (uint32)(ix + 1) * 374761393U;
    const uint32 hy0 = (uint32)iy * 668265263U, hy1 = (uint32)(iy + 1) * 668265263U;

    auto G = [](uint32 h, float dx, float dy) -> float
        {
            const float* g = GGrad2D[MixHash(h) & 15u];
            return g[0] * dx + g[1] * dy;
        };

    const float n00 = G(hx0 + hy0, fx, fy);
    const float n10 = G(hx1 + hy0, fx - 1.0f, fy);
    const float n01 = G(hx0 + hy1, fx, fy - 1.0f);
    const float n11 = G(hx1 + hy1, fx - 1.0f, fy - 1.0f);

    const float ux = Fade(fx), uy = Fade(fy);
    return FMath::Lerp(FMath::Lerp(n00, n10, ux), FMath::Lerp(n01, n11, ux), uy) * 1.414f;
}


FORCEINLINE float FastPerlinNoise3D(float x, float y, float z)
{
    const int32 ix = FMath::FloorToInt(x), iy = FMath::FloorToInt(y), iz = FMath::FloorToInt(z);
    const float fx = x - ix, fy = y - iy, fz = z - iz;

    const uint32 hx0 = (uint32)ix * 73856093U, hx1 = (uint32)(ix + 1) * 73856093U;
    const uint32 hy0 = (uint32)iy * 19349663U, hy1 = (uint32)(iy + 1) * 19349663U;
    const uint32 hz0 = (uint32)iz * 83492791U, hz1 = (uint32)(iz + 1) * 83492791U;

    auto G = [](uint32 h, float dx, float dy, float dz) -> float
        {
            const float* g = GGrad3D[MixHash(h) & 15u];
            return g[0] * dx + g[1] * dy + g[2] * dz;
        };

    const float x1 = fx - 1.0f, y1 = fy - 1.0f, z1 = fz - 1.0f;

    const float c000 = G(hx0 ^ hy0 ^ hz0, fx, fy, fz);
    const float c100 = G(hx1 ^ hy0 ^ hz0, x1, fy, fz);
    const float c010 = G(hx0 ^ hy1 ^ hz0, fx, y1, fz);
    const float c110 = G(hx1 ^ hy1 ^ hz0, x1, y1, fz);
    const float c001 = G(hx0 ^ hy0 ^ hz1, fx, fy, z1);
    const float c101 = G(hx1 ^ hy0 ^ hz1, x1, fy, z1);
    const float c011 = G(hx0 ^ hy1 ^ hz1, fx, y1, z1);
    const float c111 = G(hx1 ^ hy1 ^ hz1, x1, y1, z1);

    const float ux = Fade(fx), uy = Fade(fy), uz = Fade(fz);
    return FMath::Lerp(
        FMath::Lerp(FMath::Lerp(c000, c100, ux), FMath::Lerp(c010, c110, ux), uy),
        FMath::Lerp(FMath::Lerp(c001, c101, ux), FMath::Lerp(c011, c111, ux), uy), uz);
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

// Ridged multifractal read as a carve depth rather than a height. Returns [0,1]:
// 0 along the crest network (which is a connected set of curves, so it stays exactly on
// the original smooth surface) and 1 at the bottom of a pit. Because the result is only
// ever subtracted along the surface normal, the rock gains cracks and facets without
// gaining a single protrusion.
static float CalculateRockCarve3D(float x, float y, float z,
    int32 Octaves, float Freq, float Lacunarity, float Gain,
    float RidgeWeight, float Sharpness, const float(&Off)[6][3])
{
    float total = 0.0f, maxAmp = 0.0f, amp = 1.0f, w = 1.0f;
    const int32 Count = FMath::Clamp(Octaves, 1, 6);

    for (int32 i = 0; i < Count; ++i)
    {
        float n = FastPerlinNoise3D(x * Freq + Off[i][0], y * Freq + Off[i][1], z * Freq + Off[i][2]);
        n = 1.0f - FMath::Abs(n);   // crest where the noise crosses zero
        n *= n;                     // squaring is what makes the crest a sharp edge
        n *= w;                     // feedback: detail only survives near a crest
        w = FMath::Clamp(n * RidgeWeight, 0.0f, 1.0f);

        total += n * amp;
        maxAmp += amp;
        Freq *= Lacunarity;
        amp *= Gain;
    }

    const float Ridge = (maxAmp > 0.0f) ? FMath::Clamp(total / maxAmp, 0.0f, 1.0f) : 0.0f;
    const float Carve = 1.0f - Ridge;
    return FMath::IsNearlyEqual(Sharpness, 1.0f) ? Carve : FMath::Pow(Carve, Sharpness);
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

// Everything AddVoxelFaces needs that is identical for all 27 voxels an edit touches.
// It was previously rebuilt per voxel: a full FTerrainGenConfig copy, eight TMap lookups
// for the neighbour pointers, and an FCaveSmoothCache::Init that allocates about half a
// megabyte. One block break did that 27 times.
struct FVoxelEditContext
{
    FTerrainGenConfig  Config;
    FChunkNeighborhood Neighborhood;
    FLocalHeightGrid   HeightGrid;
    FCaveSmoothCache   CaveCache;
    bool bValid = false;
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
                        int32 n1 = Normals->AppendElement(FVector3f(0.0f, 0.0f, 1.0f));
                        int32 n2 = Normals->AppendElement(FVector3f(0.0f, 0.0f, 1.0f));
                        int32 n3 = Normals->AppendElement(FVector3f(0.0f, 0.0f, 1.0f));
                        Normals->SetTriangle(t1, FIndex3i(n0, n1, n2));
                        Normals->SetTriangle(t2, FIndex3i(n0, n2, n3));
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
    Config.bBendCaveFaces = bBendCaveFaces; Config.CavePatchSubdiv = CavePatchSubdiv;
    Config.CavePatchFlatDot = CavePatchFlatDot;
    Config.CaveRoughness = CaveRoughness;
    Config.PrepareDerived();
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
            const int32 CS = Config.ChunkSize;
            const int32 MH = Config.MaxHeight;
            const int32 Bedrock = Config.BedrockLevel;
            const int32 StepZ = CS * CS;

            // Air is enum value 0, so the zeroed buffer is already the air part of every
            // column. Only the solid prefix is written: ~2/3 fewer stores.
            TSharedPtr<TArray<EVoxelType>, ESPMode::ThreadSafe> LocalVoxelData = MakeShared<TArray<EVoxelType>, ESPMode::ThreadSafe>();
            LocalVoxelData->SetNumZeroed(StepZ * MH);
            EVoxelType* Data = LocalVoxelData->GetData();

            const int32 CacheSize = CS + 5;
            TSharedPtr<TArray<float>, ESPMode::ThreadSafe> LocalHeightMap = MakeShared<TArray<float>, ESPMode::ThreadSafe>();
            LocalHeightMap->SetNumUninitialized(CacheSize * CacheSize);

            for (int32 y = 0; y < CacheSize; ++y)
            {
                const int32 WorldY = ChunkCoord.Y * CS - 2 + y;
                for (int32 x = 0; x < CacheSize; ++x)
                {
                    (*LocalHeightMap)[x + y * CacheSize] = Config.GetHeightAtWorldCorner(ChunkCoord.X * CS - 2 + x, WorldY);
                }
            }

            FLocalHeightGrid HeightGrid;
            HeightGrid.Heights = LocalHeightMap->GetData();
            HeightGrid.CacheSize = CacheSize;

            // Column cache over [-2 .. CS+1]. Built from the raw heights, not through
            // GetSurfaceHeightLocal, because that would read the cache being built.
            const int32 ColSize = CS + 4;
            TSharedPtr<TArray<FColumnCache>, ESPMode::ThreadSafe> LocalColumns = MakeShared<TArray<FColumnCache>, ESPMode::ThreadSafe>();
            LocalColumns->SetNumUninitialized(ColSize * ColSize);
            {
                FColumnCache* C = LocalColumns->GetData();
                for (int32 y = -2; y <= CS + 1; ++y)
                {
                    for (int32 x = -2; x <= CS + 1; ++x)
                    {
                        const float H00 = HeightGrid.GetHeight(x, y);
                        const float H10 = HeightGrid.GetHeight(x + 1, y);
                        const float H01 = HeightGrid.GetHeight(x, y + 1);
                        const float H11 = HeightGrid.GetHeight(x + 1, y + 1);
                        const float S = FMath::Min3(H00, H10, FMath::Min(H01, H11));

                        FColumnCache& E = C[(x + 2) + (y + 2) * ColSize];
                        E.Surface = S;
                        E.Ground = FMath::Clamp(FMath::FloorToInt(S - Config.MinGrassThickness) - Bedrock, 0, MH - 1);
                    }
                }
            }
            HeightGrid.Columns = LocalColumns->GetData();
            HeightGrid.ColumnSize = ColSize;

            int32 MaxGround = 0;
            for (int32 ly = 0; ly < CS; ++ly)
            {
                for (int32 lx = 0; lx < CS; ++lx)
                {
                    const int32 G = HeightGrid.GetColumn(lx, ly).Ground;
                    MaxGround = FMath::Max(MaxGround, G);

                    EVoxelType* Col = Data + lx + ly * CS;
                    const int32 StoneTop = FMath::Max(0, G - 3);
                    for (int32 lz = 0; lz < StoneTop; ++lz) Col[lz * StepZ] = EVoxelType::Stone;
                    for (int32 lz = StoneTop; lz < G; ++lz)  Col[lz * StepZ] = EVoxelType::Dirt;
                    Col[G * StepZ] = EVoxelType::Grass;
                }
            }

            // --- Cave Generation Pass ---
            if (Config.CaveSettings.bEnableCaves)
            {
                const int32 ZTop = FMath::Min(MaxGround, MH - 1);

                // z outermost: one contiguous 1 KB plane per iteration instead of a
                // 4 KB stride per voxel, and the world Z term is hoisted out of the column.
                for (int32 lz = 1; lz <= ZTop; ++lz)
                {
                    const int32 WorldZ = lz + Bedrock;
                    EVoxelType* Plane = Data + lz * StepZ;

                    for (int32 ly = 0; ly < CS; ++ly)
                    {
                        const int32 WorldY = ChunkCoord.Y * CS + ly;
                        for (int32 lx = 0; lx < CS; ++lx)
                        {
                            const int32 Idx = lx + ly * CS;
                            if (Plane[Idx] == EVoxelType::Air) continue;

                            if (Config.IsInsideCave(ChunkCoord.X * CS + lx, WorldY, WorldZ,
                                HeightGrid.GetColumn(lx, ly).Surface))
                            {
                                Plane[Idx] = EVoxelType::Air;
                            }
                        }
                    }
                }

                // Surface pass on newly exposed terrain in cave openings. Starts at the
                // column's ground level: nothing above it is ever Stone.
                for (int32 ly = 0; ly < CS; ++ly)
                {
                    for (int32 lx = 0; lx < CS; ++lx)
                    {
                        const FColumnCache& Col = HeightGrid.GetColumn(lx, ly);
                        EVoxelType* C = Data + lx + ly * CS;

                        for (int32 lz = FMath::Min(Col.Ground, MH - 2); lz >= 1; --lz)
                        {
                            if (C[lz * StepZ] == EVoxelType::Stone && C[(lz + 1) * StepZ] == EVoxelType::Air)
                            {
                                const int32 WorldZ = lz + Bedrock;
                                if (FMath::Abs(WorldZ - Col.Surface) <= 3.0f)
                                {
                                    C[lz * StepZ] = EVoxelType::Grass;
                                    for (int32 d = 1; d <= 2 && (lz - d) > 0; ++d)
                                    {
                                        if (C[(lz - d) * StepZ] == EVoxelType::Stone)
                                            C[(lz - d) * StepZ] = EVoxelType::Dirt;
                                    }
                                }
                                break;
                            }
                        }
                    }
                }
            }

            const int32 MaxSolidZ = FMath::Min(MaxGround + 1, MH);

            AsyncTask(ENamedThreads::GameThread, [WeakThis, ChunkCoord, LocalVoxelData, LocalHeightMap, LocalColumns, MaxSolidZ]()
                {
                    ASmoothVoxelTerrain* Terrain = WeakThis.Get();
                    if (!Terrain || Terrain->bIsDestroyed) return;
                    if (FVoxelChunk* TargetChunk = Terrain->GetChunk(ChunkCoord))
                    {
                        TargetChunk->VoxelData = LocalVoxelData;
                        TargetChunk->HeightMap = LocalHeightMap;
                        TargetChunk->Columns = LocalColumns;
                        TargetChunk->MaxSolidZ = MaxSolidZ;
                        TargetChunk->State = EChunkState::DataReady;
                        if (Terrain->CheckNeighborsDataReady(ChunkCoord)) Terrain->MeshGenerationQueue.AddUnique(ChunkCoord);

                        const FIntVector Neighbors[8] = {
                            FIntVector(ChunkCoord.X - 1, ChunkCoord.Y,     0), FIntVector(ChunkCoord.X + 1, ChunkCoord.Y,     0),
                            FIntVector(ChunkCoord.X,     ChunkCoord.Y - 1, 0), FIntVector(ChunkCoord.X,     ChunkCoord.Y + 1, 0),
                            FIntVector(ChunkCoord.X - 1, ChunkCoord.Y - 1, 0), FIntVector(ChunkCoord.X + 1, ChunkCoord.Y - 1, 0),
                            FIntVector(ChunkCoord.X - 1, ChunkCoord.Y + 1, 0), FIntVector(ChunkCoord.X + 1, ChunkCoord.Y + 1, 0)
                        };
                        for (const FIntVector& N : Neighbors)
                        {
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
    TSharedPtr<TArray<FColumnCache>, ESPMode::ThreadSafe> ColumnData = Chunk->Columns;
    const int32 MaxSolidZ = Chunk->MaxSolidZ;

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

    Async(EAsyncExecution::ThreadPool, [WeakThis, ChunkCoord, Config, SelfData, HeightMap, ColumnData, MaxSolidZ, WestData, EastData, SouthData, NorthData, SWData, SEData, NWData, NEData]() mutable
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
            HeightGrid.Heights = HeightMap->GetData();
            HeightGrid.CacheSize = Config.ChunkSize + 5;
            HeightGrid.Columns = ColumnData ? ColumnData->GetData() : nullptr;
            HeightGrid.ColumnSize = Config.ChunkSize + 4;

            FCaveSmoothCache CaveCache;
            CaveCache.Init(&Config, &HeightGrid, &Neighborhood, ChunkCoord);

            FTriIDArray TempTriIDs;

            // Everything at or above MaxSolidZ is air by construction, so the old loop
            // spent more than half its iterations proving that 1024 times per plane.
            const int32 ZLimit = FMath::Clamp(MaxSolidZ, 0, Config.MaxHeight);

            for (int32 lz = 0; lz < ZLimit; ++lz) {
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

            for (int32 lz = 0; lz < Config.MaxHeight - 1; ++lz) {
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

    FVoxelEditContext Ctx;
    BuildEditContext(Ctx, TerrainOwner);
    if (!Ctx.bValid) return;

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
                        const int32 nx = LocalX + dx, ny = LocalY + dy, nz = LocalZ + dz;
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
                        const int32 nx = LocalX + dx, ny = LocalY + dy, nz = LocalZ + dz;
                        if (nx >= 0 && nx < TerrainOwner->ChunkSize && ny >= 0 && ny < TerrainOwner->ChunkSize && nz >= 0 && nz < TerrainOwner->MaxHeight)
                            if ((*VoxelData)[nx + ny * TerrainOwner->ChunkSize + nz * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize] != EVoxelType::Air)
                                AddVoxelFaces(nx, ny, nz, MeshOut, GrassMeshOut, TerrainOwner, Ctx);
                    }
                }
            }
        };

    DynamicMesh->EditMesh([&](FDynamicMesh3& MeshOut) {
        if (GrassDynamicMesh) GrassDynamicMesh->EditMesh([&](FDynamicMesh3& GrassMeshOut) { UpdateBlockLogic(MeshOut, &GrassMeshOut); FMeshNormals::QuickComputeVertexNormals(GrassMeshOut); });
        else UpdateBlockLogic(MeshOut, nullptr);
        });

    TerrainOwner->MarkCollisionDirty(MeshComponent);
}

void ASmoothVoxelTerrain::MarkCollisionDirty(UDynamicMeshComponent* Comp)
{
    if (Comp && IsValid(Comp)) PendingCollisionUpdates.AddUnique(Comp);
}

void ASmoothVoxelTerrain::UpdateCollisionIfNeeded()
{
    int32 Budget = FMath::Max(1, MaxCollisionUpdatesPerFrame);
    while (Budget-- > 0 && PendingCollisionUpdates.Num() > 0)
    {
        TWeakObjectPtr<UDynamicMeshComponent> Weak = PendingCollisionUpdates[0];
        PendingCollisionUpdates.RemoveAt(0, 1, EAllowShrinking::No);

        UDynamicMeshComponent* Comp = Weak.Get();
        if (!Comp || !IsValid(Comp)) continue;
        Comp->UpdateCollision(false);
    }
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

    // Self-healing fallback: If the hit position landed on Air (e.g., on smooth terrain 
    // where vertices are displaced above the voxel grid), find the solid voxel supporting it.
    if ((*Chunk->VoxelData)[Index] == EVoxelType::Air)
    {
        bool bFound = false;

        // 1. Search downward (for smooth hills and slopes where the surface sits above GroundLevel)
        for (int32 dz = -1; dz >= -4 && (lz + dz) >= 0; --dz)
        {
            int32 CheckIdx = lx + ly * ChunkSize + (lz + dz) * ChunkSize * ChunkSize;
            if ((*Chunk->VoxelData)[CheckIdx] != EVoxelType::Air)
            {
                lz += dz;
                Index = CheckIdx;
                bFound = true;
                break;
            }
        }

        // 2. Search immediate cardinal neighbors (for walls, cliffs, and cave surfaces)
        if (!bFound)
        {
            const int32 Dirs[6][3] = { {0,0,-1}, {0,0,1}, {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0} };
            for (const auto& d : Dirs)
            {
                int32 nx = lx + d[0], ny = ly + d[1], nz = lz + d[2];
                if (nx >= 0 && nx < ChunkSize && ny >= 0 && ny < ChunkSize && nz >= 0 && nz < MaxHeight)
                {
                    int32 CheckIdx = nx + ny * ChunkSize + nz * ChunkSize * ChunkSize;
                    if ((*Chunk->VoxelData)[CheckIdx] != EVoxelType::Air)
                    {
                        lx = nx; ly = ny; lz = nz;
                        Index = CheckIdx;
                        bFound = true;
                        break;
                    }
                }
            }
        }

        if (!bFound) return;
    }

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
    if (!Chunk || !Chunk->VoxelData) return;

    int32 lx, ly, lz;
    WorldToLocalVoxel(WorldLocation, ChunkCoord, lx, ly, lz);

    if (lx < 0 || lx >= ChunkSize || ly < 0 || ly >= ChunkSize || lz < 0 || lz >= MaxHeight) return;

    int32 Index = lx + ly * ChunkSize + lz * ChunkSize * ChunkSize;

    // If the target voxel is already solid, the hit location fell inside the visual surface 
    // of an existing block. We must step to the best adjacent Air voxel.
    if ((*Chunk->VoxelData)[Index] != EVoxelType::Air)
    {
        FVector LocalPos = GetActorTransform().InverseTransformPosition(WorldLocation);
        int32 WorldX = ChunkCoord.X * ChunkSize + lx;
        int32 WorldY = ChunkCoord.Y * ChunkSize + ly;
        int32 WorldZ = lz + BedrockLevel;
        FVector VoxelCenter = FVector(WorldX + 0.5f, WorldY + 0.5f, WorldZ + 0.5f) * CubeSize;
        FVector Delta = LocalPos - VoxelCenter;

        float BestDot = -FLT_MAX;
        int32 FoundX = lx, FoundY = ly, FoundZ = lz;
        FVoxelChunk* FoundChunk = nullptr;
        bool bFound = false;

        const int32 Dirs[6][3] = { {0,0,1}, {0,0,-1}, {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0} };
        for (const auto& d : Dirs)
        {
            int32 cx = lx + d[0], cy = ly + d[1], cz = lz + d[2];
            FVoxelChunk* C = Chunk;

            if (cx < 0) { cx += ChunkSize; C = GetChunk(ChunkCoord + FIntVector(-1, 0, 0)); }
            else if (cx >= ChunkSize) { cx -= ChunkSize; C = GetChunk(ChunkCoord + FIntVector(1, 0, 0)); }

            if (cy < 0) { cy += ChunkSize; if (C) C = GetChunk(C->Coord + FIntVector(0, -1, 0)); }
            else if (cy >= ChunkSize) { cy -= ChunkSize; if (C) C = GetChunk(C->Coord + FIntVector(0, 1, 0)); }

            if (!C || !C->VoxelData || cz < 0 || cz >= MaxHeight) continue;

            int32 nIdx = cx + cy * ChunkSize + cz * ChunkSize * ChunkSize;
            if ((*C->VoxelData)[nIdx] == EVoxelType::Air)
            {
                FVector Dir(d[0], d[1], d[2]);
                float Dot = FVector::DotProduct(Delta, Dir);
                if (Dot > BestDot)
                {
                    BestDot = Dot;
                    FoundX = cx; FoundY = cy; FoundZ = cz;
                    FoundChunk = C;
                    bFound = true;
                }
            }
        }

        if (bFound)
        {
            Chunk = FoundChunk;
            lx = FoundX; ly = FoundY; lz = FoundZ;
            ChunkCoord = Chunk->Coord;
        }
        else return; // No adjacent Air block found
    }

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
    if (G.Columns) return G.GetColumn(LocalX, LocalY).Surface;

    const float H00 = G.GetHeight(LocalX, LocalY);
    const float H10 = G.GetHeight(LocalX + 1, LocalY);
    const float H01 = G.GetHeight(LocalX, LocalY + 1);
    const float H11 = G.GetHeight(LocalX + 1, LocalY + 1);
    return FMath::Min3(H00, H10, FMath::Min(H01, H11));
}

int32 FTerrainGenConfig::GetGroundLevelLocal(int32 LocalX, int32 LocalY, const FLocalHeightGrid& G) const
{
    if (G.Columns) return G.GetColumn(LocalX, LocalY).Ground;

    return FMath::Clamp(FMath::FloorToInt(GetSurfaceHeightLocal(LocalX, LocalY, G) - MinGrassThickness) - BedrockLevel, 0, MaxHeight - 1);
}

FSmoothVertex FTerrainGenConfig::GetSmoothVertexEx(
    int32 VertX, int32 VertY, int32 VertZ,
    int32 VoxX, int32 VoxY, int32 VoxZ,
    const FLocalHeightGrid& HeightGrid,
    const FChunkNeighborhood& Neighborhood,
    const FIntVector& ChunkCoord,
    FCaveSmoothCache* CaveCache) const
{
    FSmoothVertex Out;

    const int32  WorldX = ChunkCoord.X * ChunkSize + VertX;
    const int32  WorldY = ChunkCoord.Y * ChunkSize + VertY;
    const double BaseZ = (double)(VertZ + BedrockLevel);
    Out.P = FVector((double)WorldX, (double)WorldY, BaseZ) * CubeSize;

    if (!bSmoothTerrain) return Out;

    // Every call site passes a solid cell (the mesher skips Air, and the rim bands
    // pick their adjacent cell behind a !bAir test), so an Air cell here is a
    // caller bug, not a vertex to displace.
    if (Neighborhood.GetVoxel(VoxX, VoxY, VoxZ) == EVoxelType::Air) return Out;

    // --- Surface displacement: top cap of the generated ground voxel only ---
    // A cell above the ground voxel is something the player placed. It keeps its
    // lattice position, so its vertical edges are always exactly one voxel long.
    // The lip between its bottom plane and the terrain surface is closed by the
    // skirt bands in AppendVoxelFacesLocal, not by dragging vertices off the grid.
    // This branch never reports bCurved: the height field owns that geometry and its
    // edges must stay straight so they meet the flat top cap exactly.
    if (VertZ > VoxZ && VoxZ == GetGroundLevelLocal(VoxX, VoxY, HeightGrid))
    {
        Out.P = FVector((double)WorldX,
            (double)WorldY,
            (double)HeightGrid.GetHeight(VertX, VertY)) * CubeSize;
        return Out;
    }

    // --- Cave-wall displacement (generated state only) ---
    if (CaveCache && CaveCache->IsReady() && CaveCache->IsCellSmoothSurface(VoxX, VoxY, VoxZ))
    {
        FVector3f Off, Nrm;
        if (CaveCache->GetVertexOffset(VertX, VertY, VertZ, Off, Nrm))
        {
            Out.P = FVector((double)WorldX + (double)Off.X,
                (double)WorldY + (double)Off.Y,
                BaseZ + (double)Off.Z) * CubeSize;
            Out.N = Nrm;
            Out.bCurved = !Nrm.IsNearlyZero();

            // Macro roughness only. It is a pure function of the final world position and
            // the field normal, both of which two neighbouring chunks agree on exactly,
            // so a shared lattice vertex lands in the same place from either side.
            // The micro layer is deliberately NOT applied here: leaving the lattice cage
            // alone at sub-voxel scale is what keeps the collision hull walkable.
            if (Out.bCurved)
            {
                Out.P = ApplyCaveRoughness(Out.P, Nrm, 1.0f, 0.0f);
            }
            return Out;
        }
    }

    return Out;
}

FVector FTerrainGenConfig::GetSmoothVertexLocal(
    int32 VertX, int32 VertY, int32 VertZ,
    int32 VoxX, int32 VoxY, int32 VoxZ,
    const FLocalHeightGrid& HeightGrid,
    const FChunkNeighborhood& Neighborhood,
    const FIntVector& ChunkCoord,
    FCaveSmoothCache* CaveCache) const
{
    return GetSmoothVertexEx(VertX, VertY, VertZ, VoxX, VoxY, VoxZ,
        HeightGrid, Neighborhood, ChunkCoord, CaveCache).P;
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

void FTerrainGenConfig::PrepareDerived()
{
    InvCubeSize = (CubeSize != 0.0f) ? (1.0f / CubeSize) : 0.01f;

    CaveOff.Entrance[0] = Hash2D(Seed, 120);
    CaveOff.Entrance[1] = Hash2D(Seed, 121);

    CaveOff.Macro[0] = Hash3D(Seed, 131, 1);
    CaveOff.Macro[1] = Hash3D(Seed, 132, 2);
    CaveOff.Macro[2] = Hash3D(Seed, 133, 3);

    CaveOff.T1[0] = Hash3D(Seed, 150, 1); CaveOff.T1[1] = Hash3D(Seed, 151, 2);
    CaveOff.T1[2] = Hash3D(Seed, 152, 3); CaveOff.T1[3] = Hash3D(Seed, 153, 4);
    CaveOff.T1[4] = Hash3D(Seed, 154, 5); CaveOff.T1[5] = Hash3D(Seed, 155, 6);

    CaveOff.T2[0] = Hash3D(Seed, 160, 1); CaveOff.T2[1] = Hash3D(Seed, 161, 2);
    CaveOff.T2[2] = Hash3D(Seed, 162, 3); CaveOff.T2[3] = Hash3D(Seed, 163, 4);
    CaveOff.T2[4] = Hash3D(Seed, 164, 5); CaveOff.T2[5] = Hash3D(Seed, 165, 6);

    // Matches CalculateFBM3D(..., octaves = 2, layerSeed = Seed + 140).
    for (int32 i = 0; i < 2; ++i)
    {
        CaveOff.Chamber[i][0] = Hash3D(Seed + 140, i, 11) * 3000.0f;
        CaveOff.Chamber[i][1] = Hash3D(Seed + 141, i, 22) * 3000.0f;
        CaveOff.Chamber[i][2] = Hash3D(Seed + 142, i, 33) * 3000.0f;
    }

    // Wall roughness. Separate offsets per layer so macro lumps and micro chipping do
    // not line up with each other or with the tunnel field.
    RoughOff.Warp[0] = Hash3D(Seed, 201, 1) * 4000.0f;
    RoughOff.Warp[1] = Hash3D(Seed, 202, 2) * 4000.0f;
    RoughOff.Warp[2] = Hash3D(Seed, 203, 3) * 4000.0f;

    for (int32 i = 0; i < 6; ++i)
    {
        RoughOff.Macro[i][0] = Hash3D(Seed + 210, i, 11) * 4000.0f;
        RoughOff.Macro[i][1] = Hash3D(Seed + 211, i, 22) * 4000.0f;
        RoughOff.Macro[i][2] = Hash3D(Seed + 212, i, 33) * 4000.0f;

        RoughOff.Micro[i][0] = Hash3D(Seed + 220, i, 11) * 4000.0f;
        RoughOff.Micro[i][1] = Hash3D(Seed + 221, i, 22) * 4000.0f;
        RoughOff.Micro[i][2] = Hash3D(Seed + 222, i, 33) * 4000.0f;
    }

    RoughOff.Strata = Hash2D(Seed, 230) * 4000.0f;
}
float FTerrainGenConfig::GetCaveDensityAt(float WorldX, float WorldY, float WorldZ, float SurfaceHeight) const
{
    if (!CaveSettings.bEnableCaves) return -1.0f;
    if (WorldZ <= (float)BedrockLevel + (float)CaveSettings.CaveBedrockSafetyMargin) return -1.0f;

    const float DistBelowSurface = SurfaceHeight - WorldZ;
    if (DistBelowSurface <= 0.0f) return -1.0f;

    float DepthFactor = 1.0f;
    if (DistBelowSurface < CaveSettings.CaveMaxHeightOffset)
    {
        const float EntranceNoise = FastPerlinNoise2D(WorldX * 0.01f + CaveOff.Entrance[0],
            WorldY * 0.01f + CaveOff.Entrance[1]) * 0.5f + 0.5f;
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
        fx * CaveSettings.ChamberFrequencyScale + CaveOff.Macro[0],
        fy * CaveSettings.ChamberFrequencyScale + CaveOff.Macro[1],
        fz * CaveSettings.ChamberFrequencyScale * 1.2f + CaveOff.Macro[2]) * 0.5f + 0.5f;

    const float MaskMargin = ChamberMacroMask - 0.42f;

    float RadiusBonus = 0.0f;
    if (ChamberMacroMask > 0.35f)
    {
        RadiusBonus = (ChamberMacroMask - 0.35f) * CaveSettings.TunnelChamberExpansion;
    }
    const float EffectiveRadius = (CaveSettings.TunnelBaseRadius + RadiusBonus) * DepthFactor;

    float Density = -1.0f;

    const float SXZ1 = CaveSettings.TunnelNoiseScaleXZ;
    const float SY1 = CaveSettings.TunnelNoiseScaleY;

    const float T1A = FastPerlinNoise3D(fx * SXZ1 + CaveOff.T1[0], fy * SXZ1 + CaveOff.T1[1], fz * SY1 + CaveOff.T1[2]);
    const float T1B = FastPerlinNoise3D(fx * SXZ1 + CaveOff.T1[3], fy * SXZ1 + CaveOff.T1[4], fz * SY1 + CaveOff.T1[5]);
    Density = FMath::Max(Density, EffectiveRadius - FMath::Sqrt(T1A * T1A + T1B * T1B));

    const float SXZ2 = SXZ1 * 1.15f;
    const float SY2 = SY1 * 1.25f;

    const float T2A = FastPerlinNoise3D((fx + 500.0f) * SXZ2 + CaveOff.T2[0], (fy + 500.0f) * SXZ2 + CaveOff.T2[1], fz * SY2 + CaveOff.T2[2]);
    const float T2B = FastPerlinNoise3D((fx - 500.0f) * SXZ2 + CaveOff.T2[3], (fy - 500.0f) * SXZ2 + CaveOff.T2[4], fz * SY2 + CaveOff.T2[5]);
    Density = FMath::Max(Density, EffectiveRadius - FMath::Sqrt(T2A * T2A + T2B * T2B));

    if (MaskMargin > Density)
    {
        // CalculateFBM3D inlined with its seed offsets hoisted: two octaves, freq *= 2.02,
        // amp *= 0.5, z pre-scaled by 1.3 exactly as the old call site did.
        float freq = CaveSettings.ChamberNoiseScaleXZ;
        float amp = 1.0f;
        float total = 0.0f;
        float maxAmp = 0.0f;
        const float cz = fz * 1.3f;

        for (int32 i = 0; i < 2; ++i)
        {
            total += FastPerlinNoise3D(fx * freq + CaveOff.Chamber[i][0],
                fy * freq + CaveOff.Chamber[i][1],
                cz * freq + CaveOff.Chamber[i][2]) * amp;
            maxAmp += amp;
            freq *= 2.02f;
            amp *= 0.5f;
        }
        const float ChamberNoise = (maxAmp > 0.0f) ? total / maxAmp : 0.0f;

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

float FTerrainGenConfig::GetCaveRoughDepth(float VoxX, float VoxY, float VoxZ,
    const FVector3f& Normal, float MacroWeight, float MicroWeight) const
{
    const FCaveRoughnessSettings& R = CaveRoughness;
    if (!R.bEnableCaveRoughness) return 0.0f;

    const bool bWantMacro = (MacroWeight > 0.0f) && (R.MacroDepth > 0.0f);
    const bool bWantMicro = (MicroWeight > 0.0f) && (R.MicroDepth > 0.0f);
    if (!bWantMacro && !bWantMicro) return 0.0f;

    // Up-facing surfaces are the ones the player walks on, so they get their own
    // multiplier. Keeping FloorRoughness low leaves cave floors readable underfoot
    // while the walls and the roof can be as broken as you like.
    const float nz = FMath::Clamp(Normal.Z, -1.0f, 1.0f);
    const float Vert = nz * nz;
    const float Orient = (nz >= 0.0f)
        ? FMath::Lerp(R.WallRoughness, R.FloorRoughness, Vert)
        : FMath::Lerp(R.WallRoughness, R.CeilingRoughness, Vert);
    if (Orient <= 0.0f) return 0.0f;

    float px = VoxX, py = VoxY, pz = VoxZ;

    // Warping the sample point, not the result, is what stops the fractures from
    // running parallel to the noise lattice and reading as a grid.
    if (R.WarpStrength > 0.0f)
    {
        const float ws = R.WarpScale;
        const float wx = FastPerlinNoise3D(px * ws + RoughOff.Warp[0], py * ws + RoughOff.Warp[1], pz * ws + RoughOff.Warp[2]);
        const float wy = FastPerlinNoise3D(py * ws + RoughOff.Warp[1], pz * ws + RoughOff.Warp[2], px * ws + RoughOff.Warp[0]);
        const float wz = FastPerlinNoise3D(pz * ws + RoughOff.Warp[2], px * ws + RoughOff.Warp[0], py * ws + RoughOff.Warp[1]);
        px += wx * R.WarpStrength;
        py += wy * R.WarpStrength;
        pz += wz * R.WarpStrength;
    }

    // Bedding planes: bands of harder and softer rock stacked along Z, optionally tilted.
    float Strata = 1.0f;
    if (R.StrataStrength > 0.0f && R.StrataScale > 0.0f)
    {
        const float Band = (pz + R.StrataTilt * (px * 0.6f + py * 0.8f)) * R.StrataScale;
        const float Bed = FastPerlinNoise2D(Band, RoughOff.Strata) * 0.5f + 0.5f;
        Strata = FMath::Lerp(1.0f, Bed, FMath::Clamp(R.StrataStrength, 0.0f, 1.0f));
    }

    float Depth = 0.0f;

    if (bWantMacro)
    {
        Depth += R.MacroDepth * MacroWeight * CalculateRockCarve3D(px, py, pz,
            R.MacroOctaves, R.MacroScale, R.Lacunarity, R.Gain, R.RidgeWeight,
            R.MacroSharpness, RoughOff.Macro);
    }

    if (bWantMicro)
    {
        Depth += R.MicroDepth * MicroWeight * CalculateRockCarve3D(px, py, pz,
            R.MicroOctaves, R.MicroScale, R.Lacunarity, R.Gain, R.RidgeWeight,
            R.MicroSharpness, RoughOff.Micro);
    }

    Depth *= Orient * Strata;
    return FMath::Clamp(Depth, 0.0f, R.MaxDepth);
}

FVector FTerrainGenConfig::ApplyCaveRoughness(const FVector& WorldPos, const FVector3f& Normal,
    float MacroWeight, float MicroWeight) const
{
    if (!CaveRoughness.bEnableCaveRoughness) return WorldPos;
    if (Normal.IsNearlyZero()) return WorldPos;

    const float Depth = GetCaveRoughDepth(
        (float)WorldPos.X * InvCubeSize,
        (float)WorldPos.Y * InvCubeSize,
        (float)WorldPos.Z * InvCubeSize,
        Normal, MacroWeight, MicroWeight);

    if (Depth <= 0.0f) return WorldPos;

    // The density gradient points out of the solid, so subtracting always cuts into the
    // rock. That one sign is the whole collision guarantee: the carved surface can never
    // cross in front of the smooth one, so nothing new ever sticks out into the tunnel.
    const double Scale = (double)Depth * (double)CubeSize;
    return WorldPos - FVector((double)Normal.X, (double)Normal.Y, (double)Normal.Z) * Scale;
}


// ---------------------------------------------------------------------------------
// CAVE SURFACE PROJECTION
// ---------------------------------------------------------------------------------
// The eight cells sharing a lattice vertex form a unit cube centred on it: cell
// (vx-1+i, vy-1+j, vz-1+k) sits at (i-0.5, j-0.5, k-0.5) voxels away. A vertex is placed
// at the mean of the density crossings on that cube's twelve edges.

static constexpr float CaveCornerCoord[2] = { -0.5f, 0.5f };

FORCEINLINE FVector3f CaveCubeCorner(int32 c)
{
    return FVector3f(CaveCornerCoord[c & 1], CaveCornerCoord[(c >> 1) & 1], CaveCornerCoord[(c >> 2) & 1]);
}

// Corner index pairs for the twelve edges: four along X, four along Y, four along Z.
static const int32 CaveCubeEdges[12][2] = {
    { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 },
    { 0, 2 }, { 1, 3 }, { 4, 6 }, { 5, 7 },
    { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 }
};

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
    BaseNormalCache.SetNumUninitialized(BaseW * BaseW * SLAB_COUNT);
    BaseState.SetNumZeroed(BaseW * BaseW * SLAB_COUNT);

    VertSlabZ.Init(MIN_int32, SLAB_COUNT);
    VertOffsetCache.SetNumUninitialized(VertW * VertW * SLAB_COUNT);
    VertNormalCache.SetNumUninitialized(VertW * VertW * SLAB_COUNT);
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

bool FCaveSmoothCache::GetBaseOffset(int32 vx, int32 vy, int32 vz, FVector3f& OutOffset, FVector3f& OutNormal)
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
    if (BaseState[I] == 2) { OutOffset = BaseOffsetCache[I]; OutNormal = BaseNormalCache[I]; return true; }

    BaseState[I] = 1;

    int32 GroundMin = MAX_int32;
    for (int32 dy = -1; dy <= 0; ++dy)
        for (int32 dx = -1; dx <= 0; ++dx)
        {
            const int32 GL = Config->GetGroundLevelLocal(vx + dx, vy + dy, *Heights);
            if (vz > GL) return false;
            GroundMin = FMath::Min(GroundMin, GL);
        }

    float D[8];
    {
        bool bAnyAir = false, bAnySolid = false;
        for (int32 c = 0; c < 8; ++c)
        {
            D[c] = GetCellDensity(vx - 1 + (c & 1), vy - 1 + ((c >> 1) & 1), vz - 1 + ((c >> 2) & 1));
            if (D[c] > 0.0f) bAnyAir = true; else bAnySolid = true;
        }
        if (!bAnyAir || !bAnySolid) return false;
    }

    // Central difference of the eight cell densities across the unit cube. The corner bit
    // layout is the same one CaveCubeCorner uses, so each sum is one face group of the cube.
    // D > 0 is air, so the gradient already points out of the solid and needs no negation.
    {
        const FVector3f Grad(
            (D[1] + D[3] + D[5] + D[7]) - (D[0] + D[2] + D[4] + D[6]),
            (D[2] + D[3] + D[6] + D[7]) - (D[0] + D[1] + D[4] + D[5]),
            (D[4] + D[5] + D[6] + D[7]) - (D[0] + D[1] + D[2] + D[3]));

        OutNormal = (Grad.SizeSquared() > 1.e-12f) ? Grad.GetUnsafeNormal() : FVector3f::ZeroVector;
    }

    // Every edge that changes sign contributes its linearly interpolated crossing, and the
    // vertex lands on their average. The crossings come from all three axes at once, so an
    // acute wall junction pulls the vertex diagonally into the corner instead of sliding it
    // along whichever single axis happened to have the steepest slab gradient.
    FVector3f Sum(0.0f, 0.0f, 0.0f);
    int32 Count = 0;

    for (int32 e = 0; e < 12; ++e)
    {
        const int32 c0 = CaveCubeEdges[e][0];
        const int32 c1 = CaveCubeEdges[e][1];
        const float F0 = D[c0];
        const float F1 = D[c1];
        if ((F0 > 0.0f) == (F1 > 0.0f)) continue;

        const float Denom = F0 - F1;
        const float t = FMath::Clamp(FMath::IsNearlyZero(Denom) ? 0.5f : F0 / Denom, 0.0f, 1.0f);

        const FVector3f P0 = CaveCubeCorner(c0);
        Sum += P0 + (CaveCubeCorner(c1) - P0) * t;
        ++Count;
    }

    if (Count == 0) return false;

    OutOffset = Sum * (1.0f / (float)Count);

    // The mass point is a convex combination of points on the cube, so the vertex stays
    // inside its own dual cell and X/Y need no clamp. Z keeps the ground guard: GroundMin
    // is never below vz here, so this only ever limits upward travel into the height-field
    // surface, which owns that geometry.
    OutOffset.Z = FMath::Min(OutOffset.Z, (float)GroundMin - (float)vz);

    BaseOffsetCache[I] = OutOffset;
    BaseNormalCache[I] = OutNormal;
    BaseState[I] = 2;
    return true;
}

bool FCaveSmoothCache::GetVertexOffset(int32 vx, int32 vy, int32 vz, FVector3f& OutOffset, FVector3f& OutNormal)
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
    if (VertState[I] == 2) { OutOffset = VertOffsetCache[I]; OutNormal = VertNormalCache[I]; return true; }

    VertState[I] = 1;
    FVector3f Off, Nrm;
    if (!GetBaseOffset(vx, vy, vz, Off, Nrm)) return false;

    // GetBaseOffset succeeds only when at least one edge of the cell cube crosses the
    // generated air/solid boundary, which is the old straddle condition without a preferred
    // axis. It still reads generated state only, so an edit can never move a vertex that is
    // already in the mesh.
    if (!FMath::IsFinite(Off.X) || !FMath::IsFinite(Off.Y) || !FMath::IsFinite(Off.Z)) return false;

    VertOffsetCache[I] = Off;
    VertNormalCache[I] = Nrm;
    VertState[I] = 2;
    OutOffset = Off;
    OutNormal = Nrm;
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

    // The six neighbour probes are array reads; the ground levels are cache reads but
    // still six of them, so they are only fetched once the cheap test fails to reject.
    const bool bAirTop = Neighborhood.GetVoxel(lx, ly, lz + 1) == EVoxelType::Air;
    // GetVoxel reports Air below the bottom of the column, which would otherwise give every
    // chunk a full sheet of downward quads under the world that nothing can ever see.
    const bool bAirBottom = lz > 0 && Neighborhood.GetVoxel(lx, ly, lz - 1) == EVoxelType::Air;
    const bool bAirEast = Neighborhood.GetVoxel(lx + 1, ly, lz) == EVoxelType::Air;
    const bool bAirWest = Neighborhood.GetVoxel(lx - 1, ly, lz) == EVoxelType::Air;
    const bool bAirNorth = Neighborhood.GetVoxel(lx, ly + 1, lz) == EVoxelType::Air;
    const bool bAirSouth = Neighborhood.GetVoxel(lx, ly - 1, lz) == EVoxelType::Air;

    const bool bAnyAir = bAirTop || bAirBottom || bAirEast || bAirWest || bAirNorth || bAirSouth;

    int32 GroundHere = 0, GroundEast = 0, GroundWest = 0, GroundNorth = 0, GroundSouth = 0;
    if (bSmoothTerrain)
    {
        GroundHere = HeightGrid.GetColumn(lx, ly).Ground;
        GroundEast = HeightGrid.GetColumn(lx + 1, ly).Ground;
        GroundWest = HeightGrid.GetColumn(lx - 1, ly).Ground;
        GroundNorth = HeightGrid.GetColumn(lx, ly + 1).Ground;
        GroundSouth = HeightGrid.GetColumn(lx, ly - 1).Ground;

        if (!bAnyAir && lz <= GroundEast && lz <= GroundWest && lz <= GroundNorth && lz <= GroundSouth) return;
    }
    else if (!bAnyAir)
    {
        return;
    }

    FDynamicMeshUVOverlay* UVOverlay = Attr->GetUVLayer(0);
    FDynamicMeshNormalOverlay* NormalOverlay = Attr->PrimaryNormals();
    FDynamicMeshColorOverlay* ColorOverlay = Attr->PrimaryColors();
    auto* MaterialIDAttribute = Attr->GetMaterialID();

    const EVoxelType CurrentType = Neighborhood.GetVoxel(lx, ly, lz);
    const int32 WorldX = ChunkCoord.X * ChunkSize + lx;
    const int32 WorldY = ChunkCoord.Y * ChunkSize + ly;
    const int32 WorldZ = lz + BedrockLevel;

    int32 TopMatID = 1, BottomMatID = 1, SideMatID = 1;
    if (CurrentType == EVoxelType::Grass) { TopMatID = 0; BottomMatID = SideMatID = 1; }
    else if (CurrentType == EVoxelType::Dirt) { TopMatID = BottomMatID = SideMatID = 1; }
    else if (CurrentType == EVoxelType::Stone) { TopMatID = BottomMatID = SideMatID = 2; }

    const FLinearColor VoxelColor = GetStylizedColorForVoxel(FVector((double)WorldX * CubeSize + (0.5 * CubeSize), (double)WorldY * CubeSize + (0.5 * CubeSize), (double)WorldZ * CubeSize), CurrentType);
    const int32 cIdx = ColorOverlay->AppendElement(FVector4f(VoxelColor));

    // One multiply replaces a divide per UV component, of which there are two per mesh corner.
    const float UVScale = TextureScale * InvCubeSize;

    auto UVAt = [&](const FVector& P, int32 UAxis, int32 VAxis)
        {
            return FVector2f((float)P[UAxis] * UVScale, (float)P[VAxis] * UVScale);
        };

    if (!bSmoothTerrain)
    {
        FVector Origin((double)WorldX * CubeSize, (double)WorldY * CubeSize, (double)WorldZ * CubeSize);
        FVector p000 = Origin, p100 = Origin + FVector(CubeSize, 0, 0), p010 = Origin + FVector(0, CubeSize, 0), p110 = Origin + FVector(CubeSize, CubeSize, 0);
        FVector p001 = Origin + FVector(0, 0, CubeSize), p101 = Origin + FVector(CubeSize, 0, CubeSize), p011 = Origin + FVector(0, CubeSize, CubeSize), p111 = Origin + FVector(CubeSize, CubeSize, CubeSize);

        // Four vertices, one normal element and four UV elements per quad. The old version
        // emitted eight, two and six respectively for identical geometry.
        auto AddQuadWorldFast = [&](const FVector& A, const FVector& B, const FVector& C, const FVector& D, const FVector3f& FixedNormal, int32 MatID, int32 UAxis, int32 VAxis)
            {
                const int32 vA = Mesh.AppendVertex(FVector3d(A)), vB = Mesh.AppendVertex(FVector3d(B));
                const int32 vC = Mesh.AppendVertex(FVector3d(C)), vD = Mesh.AppendVertex(FVector3d(D));
                int32 n0 = NormalOverlay->AppendElement(FixedNormal);
                int32 n1 = NormalOverlay->AppendElement(FixedNormal);
                int32 n2 = NormalOverlay->AppendElement(FixedNormal);
                int32 n3 = NormalOverlay->AppendElement(FixedNormal);

                const int32 uA = UVOverlay->AppendElement(UVAt(A, UAxis, VAxis));
                const int32 uB = UVOverlay->AppendElement(UVAt(B, UAxis, VAxis));
                const int32 uC = UVOverlay->AppendElement(UVAt(C, UAxis, VAxis));
                const int32 uD = UVOverlay->AppendElement(UVAt(D, UAxis, VAxis));

                const int32 t1 = Mesh.AppendTriangle(vA, vB, vC);
                if (t1 != FDynamicMesh3::InvalidID) {
                    OutTriIDs.Add(t1);
                    NormalOverlay->SetTriangle(t1, FIndex3i(n0, n1, n2));
                    UVOverlay->SetTriangle(t1, FIndex3i(uA, uB, uC));
                    ColorOverlay->SetTriangle(t1, FIndex3i(cIdx, cIdx, cIdx));
                    if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t1, MatID);
                }
                const int32 t2 = Mesh.AppendTriangle(vA, vC, vD);
                if (t2 != FDynamicMesh3::InvalidID) {
                    OutTriIDs.Add(t2);
                    NormalOverlay->SetTriangle(t2, FIndex3i(n0, n2, n3));
                    UVOverlay->SetTriangle(t1, FIndex3i(uA, uB, uC));
                    ColorOverlay->SetTriangle(t1, FIndex3i(cIdx, cIdx, cIdx));
                    if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t1, MatID);
                }
            };

        if (bAirTop) AddQuadWorldFast(p001, p101, p111, p011, FVector3f(0.f, 0.f, 1.f), TopMatID, 0, 1);
        if (bAirBottom) AddQuadWorldFast(p100, p000, p010, p110, FVector3f(0.f, 0.f, -1.f), BottomMatID, 0, 1);
        if (bAirEast) AddQuadWorldFast(p100, p110, p111, p101, FVector3f(1.f, 0.f, 0.f), SideMatID, 1, 2);
        if (bAirWest) AddQuadWorldFast(p010, p000, p001, p011, FVector3f(-1.f, 0.f, 0.f), SideMatID, 1, 2);
        if (bAirNorth) AddQuadWorldFast(p110, p010, p011, p111, FVector3f(0.f, 1.f, 0.f), SideMatID, 0, 2);
        if (bAirSouth) AddQuadWorldFast(p000, p100, p101, p001, FVector3f(0.f, -1.f, 0.f), SideMatID, 0, 2);
        return;
    }

    // Corner code c = x | (y << 1) | (z << 2) — the same layout CaveCubeCorner and the
    // Corners* tables below use. Built on demand: a voxel that only emits a lip quad
    // used to pay for all eight, each of which walks the cave cache.
    FSmoothVertex CV[8];
    uint8 bCVReady = 0;
    auto Corner = [&](int32 c) -> const FSmoothVertex&
        {
            if (!(bCVReady & (1u << c)))
            {
                CV[c] = GetSmoothVertexEx(lx + (c & 1), ly + ((c >> 1) & 1), lz + ((c >> 2) & 1),
                    lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord, CaveCache);
                bCVReady |= (uint8)(1u << c);
            }
            return CV[c];
        };

    // Hoisted: EmitRimBands asked this once per face, and the single-entry cache inside
    // the cave cache was being clobbered by the adjacent-cell queries in between.
    const bool bSelfSmooth = CaveCache && CaveCache->IsReady() && CaveCache->IsCellSmoothSurface(lx, ly, lz);

    const double MinCross = 0.01 * (double)CubeSize * (double)CubeSize;

    auto AddQuadWorldSmooth = [&](const FVector& A, const FVector& B, const FVector& C, const FVector& D,
        int32 MatID, int32 UAxis, int32 VAxis, bool bDiagBD)
        {
            // Both splits preserve the loop winding, so the diagonal is free to follow the
            // geometry. On a displaced quad the shorter diagonal runs along the wall instead
            // of cutting across it, which is what gives cave surfaces their creased look.
            // Planar quads tie exactly and keep the caller's choice.
            const double DiagAC = FVector::DistSquared(A, C);
            const double DiagBD = FVector::DistSquared(B, D);
            if (DiagAC < DiagBD)      bDiagBD = false;
            else if (DiagBD < DiagAC) bDiagBD = true;

            const FVector* P[4] = { &A, &B, &C, &D };
            int32 T[2][3];
            if (!bDiagBD) { T[0][0] = 0; T[0][1] = 1; T[0][2] = 2;  T[1][0] = 0; T[1][1] = 2; T[1][2] = 3; }
            else { T[0][0] = 1; T[0][1] = 2; T[0][2] = 3;  T[1][0] = 1; T[1][1] = 3; T[1][2] = 0; }

            // Degeneracy is decided before anything is appended, so a dropped triangle
            // never leaves an orphan vertex behind in the buffer.
            FVector X[2];
            bool bKeep[2];
            bool bUsed[4] = { false, false, false, false };
            for (int32 i = 0; i < 2; ++i)
            {
                const FVector& P0 = *P[T[i][0]];
                X[i] = FVector::CrossProduct(*P[T[i][2]] - P0, *P[T[i][1]] - P0);
                bKeep[i] = X[i].Size() > MinCross;
                if (bKeep[i]) { bUsed[T[i][0]] = true; bUsed[T[i][1]] = true; bUsed[T[i][2]] = true; }
            }
            if (!bKeep[0] && !bKeep[1]) return;

            int32 VId[4] = { -1,-1,-1,-1 }, UId[4] = { -1,-1,-1,-1 };
            for (int32 c = 0; c < 4; ++c)
            {
                if (!bUsed[c]) continue;
                VId[c] = Mesh.AppendVertex(FVector3d(*P[c]));
                UId[c] = UVOverlay->AppendElement(UVAt(*P[c], UAxis, VAxis));
            }

            for (int32 i = 0; i < 2; ++i)
            {
                if (!bKeep[i]) continue;
                const int32 t = Mesh.AppendTriangle(VId[T[i][0]], VId[T[i][1]], VId[T[i][2]]);
                if (t == FDynamicMesh3::InvalidID) continue;

                OutTriIDs.Add(t);
                const FVector3f N(X[i].GetSafeNormal());
                NormalOverlay->SetTriangle(t, FIndex3i(
                    NormalOverlay->AppendElement(N),
                    NormalOverlay->AppendElement(N),
                    NormalOverlay->AppendElement(N)
                ));
                UVOverlay->SetTriangle(t, FIndex3i(UId[T[i][0]], UId[T[i][1]], UId[T[i][2]]));
                ColorOverlay->SetTriangle(t, FIndex3i(cIdx, cIdx, cIdx));
                if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t, MatID);
            }
        };

    // A curved replacement for AddQuadWorldSmooth. The quad's corners map to
 // A=(u0,v0) B=(u1,v0) C=(u1,v1) D=(u0,v1), so the sub-cell winding below matches
 // the caller's loop order exactly and the shorter-diagonal rule still applies.
    auto AddPatchSmooth = [&](const FSmoothVertex& A, const FSmoothVertex& B,
        const FSmoothVertex& C, const FSmoothVertex& Dv,
        int32 MatID, int32 UAxis, int32 VAxis, bool bDiagBD)
        {
            // Whether an edge bends must depend only on that lattice edge and its two
            // endpoints, never on the patch that happens to own it. Both tests here read the
            // endpoints alone: their provenance, and the angle between their field normals.
            auto EdgeBends = [&](const FSmoothVertex& P0, const FSmoothVertex& P1)
                {
                    if (!bBendCaveFaces) return false;
                    if (!P0.bCurved || !P1.bCurved) return false;
                    return FVector3f::DotProduct(P0.N, P1.N) < CavePatchFlatDot;
                };

            // Same rule for roughness: an edge is rough when both endpoints are cave-surface
            // vertices. The patch on the other side reads the same two endpoints, so it
            // reaches the same answer and cuts the shared edge the same way.
            const bool bRoughOn = CaveRoughness.bEnableCaveRoughness
                && CaveRoughness.MicroDepth > 0.0f
                && CaveRoughness.DetailSubdivisions > 1;

            auto EdgeRough = [&](const FSmoothVertex& P0, const FSmoothVertex& P1)
                {
                    return bRoughOn && P0.bCurved && P1.bCurved;
                };

            const bool bEdgeAB = EdgeBends(A, B);    // v = 0
            const bool bEdgeBC = EdgeBends(B, C);    // u = 1
            const bool bEdgeDC = EdgeBends(Dv, C);   // v = 1
            const bool bEdgeAD = EdgeBends(A, Dv);   // u = 0

            const bool bRoughAB = EdgeRough(A, B);
            const bool bRoughBC = EdgeRough(B, C);
            const bool bRoughDC = EdgeRough(Dv, C);
            const bool bRoughAD = EdgeRough(A, Dv);

            const bool bAnyBend = bEdgeAB || bEdgeBC || bEdgeDC || bEdgeAD;
            const bool bAnyRough = bRoughAB || bRoughBC || bRoughDC || bRoughAD;

            // One subdivision count shared by every patch that subdivides at all. Two patches
            // meeting on a bent or rough edge have to cut it identically or the seam opens,
            // and a straight edge stays collinear at any N, so a single constant is both
            // sufficient and the only safe choice.
            const int32 N = (bAnyBend || bAnyRough)
                ? FMath::Clamp(FMath::Max(CavePatchSubdiv, bRoughOn ? CaveRoughness.DetailSubdivisions : 1), 1, 6)
                : 1;

            if (N == 1)
            {
                AddQuadWorldSmooth(A.P, B.P, C.P, Dv.P, MatID, UAxis, VAxis, bDiagBD);
                return;
            }

            auto EdgePoint = [](const FSmoothVertex& P0, const FSmoothVertex& P1, double t, bool bCurve) -> FVector
                {
                    if (!bCurve) return FMath::Lerp(P0.P, P1.P, t);
                    const FVector d = P1.P - P0.P;
                    const FVector n0((double)P0.N.X, (double)P0.N.Y, (double)P0.N.Z);
                    const FVector n1((double)P1.N.X, (double)P1.N.Y, (double)P1.N.Z);
                    const FVector b1 = (2.0 * P0.P + P1.P - FVector::DotProduct(d, n0) * n0) / 3.0;
                    const FVector b2 = (2.0 * P1.P + P0.P + FVector::DotProduct(d, n1) * n1) / 3.0;
                    const double s = 1.0 - t;
                    return s * s * s * P0.P + 3.0 * s * s * t * b1 + 3.0 * s * t * t * b2 + t * t * t * P1.P;
                };

            auto PatchPoint = [&](double u, double v) -> FVector
                {
                    const FVector Cu0 = EdgePoint(A, B, u, bEdgeAB);
                    const FVector Cu1 = EdgePoint(Dv, C, u, bEdgeDC);
                    const FVector Cv0 = EdgePoint(A, Dv, v, bEdgeAD);
                    const FVector Cv1 = EdgePoint(B, C, v, bEdgeBC);
                    const FVector Bl = (1.0 - u) * (1.0 - v) * A.P + u * (1.0 - v) * B.P
                        + u * v * C.P + (1.0 - u) * v * Dv.P;
                    return (1.0 - v) * Cu0 + v * Cu1 + (1.0 - u) * Cv0 + u * Cv1 - Bl;
                };

            // Micro roughness weight. Zero at all four lattice corners, which is what leaves
            // the one-voxel cage of the mesh exactly where it was and gives a character
            // capsule an unbroken surface to ride. On a boundary this collapses to a function
            // of that edge's parameter alone, and the hat is symmetric, so it does not matter
            // which direction the neighbouring patch walks the shared edge.
            auto RoughHat = [](double t) { const double h = 4.0 * t * (1.0 - t); return h * h; };

            auto MicroWeight = [&](double u, double v) -> float
                {
                    if (!bAnyRough) return 0.0f;
                    const double wAB = bRoughAB ? RoughHat(u) : 0.0;
                    const double wDC = bRoughDC ? RoughHat(u) : 0.0;
                    const double wAD = bRoughAD ? RoughHat(v) : 0.0;
                    const double wBC = bRoughBC ? RoughHat(v) : 0.0;
                    const double W = (1.0 - v) * wAB + v * wDC + (1.0 - u) * wAD + u * wBC;
                    return (float)FMath::Min(1.0, W);
                };

            // A corner that is not a cave vertex contributes nothing to the carve direction,
            // so on the boundary of the rough region the direction is driven by the cave side
            // alone and the weight has already faded to zero anyway.
            const FVector3f NA = A.bCurved ? A.N : FVector3f::ZeroVector;
            const FVector3f NB = B.bCurved ? B.N : FVector3f::ZeroVector;
            const FVector3f NC = C.bCurved ? C.N : FVector3f::ZeroVector;
            const FVector3f ND = Dv.bCurved ? Dv.N : FVector3f::ZeroVector;

            auto PatchNormal = [&](double u, double v) -> FVector3f
                {
                    const FVector3f Nn =
                        NA * (float)((1.0 - u) * (1.0 - v)) + NB * (float)(u * (1.0 - v)) +
                        NC * (float)(u * v) + ND * (float)((1.0 - u) * v);
                    return Nn.GetSafeNormal();
                };

            const int32 Stride = N + 1;
            TArray<FVector, TInlineAllocator<49>> VPos; VPos.SetNumUninitialized(Stride * Stride);
            TArray<int32, TInlineAllocator<49>>   VIdx; VIdx.SetNumUninitialized(Stride * Stride);
            // UVs are a function of position, so they are shared per grid vertex exactly as
            // the positions are. At N=2 that is 9 elements instead of 24.
            TArray<int32, TInlineAllocator<49>>   UIdx; UIdx.SetNumUninitialized(Stride * Stride);

            const double InvN = 1.0 / (double)N;
            for (int32 j = 0; j <= N; ++j)
            {
                for (int32 i = 0; i <= N; ++i)
                {
                    const double u = (double)i * InvN;
                    const double v = (double)j * InvN;

                    FVector P = PatchPoint(u, v);

                    const float W = MicroWeight(u, v);
                    if (W > 0.0f)
                    {
                        const FVector3f Nn = PatchNormal(u, v);
                        if (!Nn.IsNearlyZero()) P = ApplyCaveRoughness(P, Nn, 0.0f, W);
                    }

                    const int32 k = i + j * Stride;
                    VPos[k] = P;
                    VIdx[k] = Mesh.AppendVertex(FVector3d(P));
                    UIdx[k] = UVOverlay->AppendElement(UVAt(P, UAxis, VAxis));
                }
            }

            // Sub-triangles are 1/N^2 of the original area, so the degeneracy floor has to
            // shrink with them or valid slivers get dropped and punch holes in the wall.
            const double SubMinCross = MinCross / (double)(N * N);

            for (int32 j = 0; j < N; ++j)
            {
                for (int32 i = 0; i < N; ++i)
                {
                    const int32 q0 = i + j * Stride;                 // A-like
                    const int32 q1 = (i + 1) + j * Stride;           // B-like
                    const int32 q2 = (i + 1) + (j + 1) * Stride;     // C-like
                    const int32 q3 = i + (j + 1) * Stride;           // D-like

                    const double dAC = FVector::DistSquared(VPos[q0], VPos[q2]);
                    const double dBD = FVector::DistSquared(VPos[q1], VPos[q3]);
                    const bool bSub = (dAC < dBD) ? false : ((dBD < dAC) ? true : bDiagBD);

                    int32 T[2][3];
                    if (!bSub) { T[0][0] = q0; T[0][1] = q1; T[0][2] = q2;  T[1][0] = q0; T[1][1] = q2; T[1][2] = q3; }
                    else { T[0][0] = q1; T[0][1] = q2; T[0][2] = q3;  T[1][0] = q1; T[1][1] = q3; T[1][2] = q0; }

                    for (int32 k = 0; k < 2; ++k)
                    {
                        const FVector& P0 = VPos[T[k][0]];
                        const FVector& P1 = VPos[T[k][1]];
                        const FVector& P2 = VPos[T[k][2]];

                        const FVector X = FVector::CrossProduct(P2 - P0, P1 - P0);
                        if (X.Size() <= SubMinCross) continue;

                        const int32 t = Mesh.AppendTriangle(VIdx[T[k][0]], VIdx[T[k][1]], VIdx[T[k][2]]);
                        if (t == FDynamicMesh3::InvalidID) continue;

                        OutTriIDs.Add(t);
                        const FVector3f Nf(X.GetSafeNormal());
                        NormalOverlay->SetTriangle(t, FIndex3i(
                            NormalOverlay->AppendElement(Nf),
                            NormalOverlay->AppendElement(Nf),
                            NormalOverlay->AppendElement(Nf)
                        ));
                        UVOverlay->SetTriangle(t, FIndex3i(UIdx[T[k][0]], UIdx[T[k][1]], UIdx[T[k][2]]));
                        ColorOverlay->SetTriangle(t, FIndex3i(cIdx, cIdx, cIdx));
                        if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t, MatID);
                    }
                }
            }
        };

    auto AddTopQuadSmooth = [&](const FVector& A, const FVector& B, const FVector& C, const FVector& D,
        const FVector& nA, const FVector& nB, const FVector& nC, const FVector& nD, int32 MatID)
        {
            const int32 vA = Mesh.AppendVertex(FVector3d(A)), vB = Mesh.AppendVertex(FVector3d(B));
            const int32 vC = Mesh.AppendVertex(FVector3d(C)), vD = Mesh.AppendVertex(FVector3d(D));

            const int32 nIA = NormalOverlay->AppendElement(FVector3f(nA));
            const int32 nIB = NormalOverlay->AppendElement(FVector3f(nB));
            const int32 nIC = NormalOverlay->AppendElement(FVector3f(nC));
            const int32 nID = NormalOverlay->AppendElement(FVector3f(nD));

            const int32 uA = UVOverlay->AppendElement(FVector2f((float)A.X * UVScale, (float)A.Y * UVScale));
            const int32 uB = UVOverlay->AppendElement(FVector2f((float)B.X * UVScale, (float)B.Y * UVScale));
            const int32 uC = UVOverlay->AppendElement(FVector2f((float)C.X * UVScale, (float)C.Y * UVScale));
            const int32 uD = UVOverlay->AppendElement(FVector2f((float)D.X * UVScale, (float)D.Y * UVScale));

            const int32 t1 = Mesh.AppendTriangle(vA, vB, vC);
            if (t1 != FDynamicMesh3::InvalidID) {
                OutTriIDs.Add(t1);
                NormalOverlay->SetTriangle(t1, FIndex3i(nIA, nIB, nIC));
                UVOverlay->SetTriangle(t1, FIndex3i(uA, uB, uC));
                ColorOverlay->SetTriangle(t1, FIndex3i(cIdx, cIdx, cIdx));
                if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t1, MatID);
            }
            const int32 t2 = Mesh.AppendTriangle(vA, vC, vD);
            if (t2 != FDynamicMesh3::InvalidID) {
                OutTriIDs.Add(t2);
                NormalOverlay->SetTriangle(t2, FIndex3i(nIA, nIC, nID));
                UVOverlay->SetTriangle(t2, FIndex3i(uA, uC, uD));
                ColorOverlay->SetTriangle(t2, FIndex3i(cIdx, cIdx, cIdx));
                if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t2, MatID);
            }
        };

    static const int32 CornersTop[4] = { 4, 6, 7, 5 };
    static const int32 CornersBottom[4] = { 1, 3, 2, 0 };
    static const int32 CornersEast[4] = { 1, 5, 7, 3 };
    static const int32 CornersWest[4] = { 2, 6, 4, 0 };
    static const int32 CornersNorth[4] = { 3, 7, 6, 2 };
    static const int32 CornersSouth[4] = { 0, 4, 5, 1 };

    // Faces of a cell share edges, so the same (edge, neighbouring cell) band is reachable
    // from two faces - 12 of the 36 possible bands are - and the two copies land on top of
    // each other with opposite windings. A cell emits at most 24 bands, so this is enough.
    uint16 EmittedBands[24];
    int32 NumEmittedBands = 0;

    auto EmitRimBands = [&](int32 FaceAxis, int32 FaceSign, const int32(&Corners)[4],
        int32 MatID, int32 UAxis, int32 VAxis)
        {
            if (!CaveCache || !CaveCache->IsReady()) return;
            if (bSelfSmooth) return;

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

                const int32* Adj[2] = { nullptr, nullptr };
                if (!bPTAir) Adj[0] = bQTAir ? PT : QT;
                if (!bQTAir) Adj[1] = QT; else if (!bPTAir) Adj[1] = PT;

                for (int32 k = 0; k < 2; ++k)
                {
                    if (!Adj[k]) continue;
                    if (k == 1 && Adj[0] == Adj[1]) continue;
                    if (!CaveCache->IsCellSmoothSurface(Adj[k][0], Adj[k][1], Adj[k][2])) continue;

                    const int32 EdgeKey = FMath::Min(c0, c1) * 8 + FMath::Max(c0, c1);
                    const int32 AdjKey = (Adj[k][0] - lx + 1) + (Adj[k][1] - ly + 1) * 3 + (Adj[k][2] - lz + 1) * 9;
                    const uint16 BandKey = (uint16)(EdgeKey * 27 + AdjKey);

                    bool bAlready = false;
                    for (int32 s = 0; s < NumEmittedBands; ++s)
                        if (EmittedBands[s] == BandKey) { bAlready = true; break; }
                    if (bAlready) continue;
                    if (NumEmittedBands < UE_ARRAY_COUNT(EmittedBands)) EmittedBands[NumEmittedBands++] = BandKey;

                    const FSmoothVertex& R0 = Corner(c0);
                    const FSmoothVertex& R1 = Corner(c1);
                    const FSmoothVertex S0 = GetSmoothVertexEx(V0[0], V0[1], V0[2], Adj[k][0], Adj[k][1], Adj[k][2], HeightGrid, Neighborhood, ChunkCoord, CaveCache);
                    const FSmoothVertex S1 = GetSmoothVertexEx(V1[0], V1[1], V1[2], Adj[k][0], Adj[k][1], Adj[k][2], HeightGrid, Neighborhood, ChunkCoord, CaveCache);

                    // The band continues this face's surface past the rim, so it walks the
                    // shared edge in the opposite direction to the face that owns it.
                    AddPatchSmooth(R1, R0, S0, S1, MatID, UAxis, VAxis, false);
                }
            }
        };

    // ---- TOP (+Z) ----
    if (bAirTop)
    {
        if (lz == GroundHere)
        {
            const FVector n00 = GetSmoothNormalLocal(lx, ly, HeightGrid);
            const FVector n10 = GetSmoothNormalLocal(lx + 1, ly, HeightGrid);
            const FVector n01 = GetSmoothNormalLocal(lx, ly + 1, HeightGrid);
            const FVector n11 = GetSmoothNormalLocal(lx + 1, ly + 1, HeightGrid);
            AddTopQuadSmooth(Corner(4).P, Corner(6).P, Corner(7).P, Corner(5).P, n00, n01, n11, n10, TopMatID);
        }
        else
        {
            AddPatchSmooth(Corner(4), Corner(6), Corner(7), Corner(5), TopMatID, 0, 1, false);
        }
        EmitRimBands(2, 1, CornersTop, TopMatID, 0, 1);
    }

    // ---- BOTTOM (-Z) ----
    if (bAirBottom)
    {
        AddPatchSmooth(Corner(1), Corner(3), Corner(2), Corner(0), BottomMatID, 0, 1, true);
        EmitRimBands(2, -1, CornersBottom, BottomMatID, 0, 1);
    }

    const float ZOffsetEpsilon = 0.1f;

    auto EmitSide = [&](int32 nx, int32 ny, bool bNeighborAir, int32 NeighborGround,
        const int32(&Corners)[4], int32 UAxis, int32 VAxis, bool bDiagBD,
        int32 FaceAxis, int32 FaceSign)
        {
            // A solid neighbour inside its own generated column hides this side completely.
            if (!bNeighborAir && lz <= NeighborGround) return;

            const FSmoothVertex& A = Corner(Corners[0]);
            const FSmoothVertex& B = Corner(Corners[1]);
            const FSmoothVertex& C = Corner(Corners[2]);
            const FSmoothVertex& D = Corner(Corners[3]);

            const float nZA = GetNeighborTopHeightLocal(nx, ny, lz, A.P, Neighborhood, HeightGrid);
            const float nZB = GetNeighborTopHeightLocal(nx, ny, lz, B.P, Neighborhood, HeightGrid);
            const float nZC = GetNeighborTopHeightLocal(nx, ny, lz, C.P, Neighborhood, HeightGrid);
            const float nZD = GetNeighborTopHeightLocal(nx, ny, lz, D.P, Neighborhood, HeightGrid);

            if (nZA < A.P.Z - ZOffsetEpsilon ||
                nZB < B.P.Z - ZOffsetEpsilon ||
                nZC < C.P.Z - ZOffsetEpsilon ||
                nZD < D.P.Z - ZOffsetEpsilon)
            {
                AddPatchSmooth(A, B, C, D, SideMatID, UAxis, VAxis, bDiagBD);
                if (bNeighborAir && lz <= NeighborGround)
                {
                    EmitRimBands(FaceAxis, FaceSign, Corners, SideMatID, UAxis, VAxis);
                }
            }
        };

    EmitSide(lx + 1, ly, bAirEast, GroundEast, CornersEast, 1, 2, false, 0, 1);
    EmitSide(lx - 1, ly, bAirWest, GroundWest, CornersWest, 1, 2, true, 0, -1);
    EmitSide(lx, ly + 1, bAirNorth, GroundNorth, CornersNorth, 0, 2, true, 1, 1);
    EmitSide(lx, ly - 1, bAirSouth, GroundSouth, CornersSouth, 0, 2, false, 1, -1);

    if (!bAirBottom && lz - 1 == GroundHere)
    {
        const double BottomZ = (double)(lz + BedrockLevel);

        auto SkirtVert = [&](int32 vx, int32 vy) -> FVector
            {
                return FVector((double)(ChunkCoord.X * ChunkSize + vx),
                    (double)(ChunkCoord.Y * ChunkSize + vy),
                    FMath::Min((double)HeightGrid.GetHeight(vx, vy), BottomZ)) * CubeSize;
            };

        const FVector s00 = SkirtVert(lx, ly);
        const FVector s10 = SkirtVert(lx + 1, ly);
        const FVector s01 = SkirtVert(lx, ly + 1);
        const FVector s11 = SkirtVert(lx + 1, ly + 1);

        // The lip belongs to the terrain column, not to the placed block, so it takes
        // the supporting voxel's side material.
        const EVoxelType BelowType = Neighborhood.GetVoxel(lx, ly, lz - 1);
        const int32 SkirtMatID = (BelowType == EVoxelType::Stone) ? 2 : 1;

        // Straight quads by construction: this block only runs when lz - 1 == GroundHere,
        // which makes IsCellExpectedAir(lx,ly,lz) true, so all eight corners report
        // bCurved false and the shared edges are straight on both sides.
        if (bAirEast)  AddQuadWorldSmooth(s10, Corner(1).P, Corner(3).P, s11, SkirtMatID, 1, 2, false);
        if (bAirWest)  AddQuadWorldSmooth(s01, Corner(2).P, Corner(0).P, s00, SkirtMatID, 1, 2, true);
        if (bAirNorth) AddQuadWorldSmooth(s11, Corner(3).P, Corner(2).P, s01, SkirtMatID, 0, 2, true);
        if (bAirSouth) AddQuadWorldSmooth(s00, Corner(0).P, Corner(1).P, s10, SkirtMatID, 0, 2, false);
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

        bool bLocalTwoSided = bTwoSidedGrass;
        auto AddTri = [UVOverlay0, UVOverlay1, NormalOverlay, GroundNormal, &Mesh, &OutTriIDs, bLocalTwoSided](
            int32 a, int32 b, int32 c, int32 u0_A, int32 u0_B, int32 u0_C, int32 u1_A, int32 u1_B, int32 u1_C)
            {
                int32 t = Mesh.AppendTriangle(a, b, c);
                if (t != FDynamicMesh3::InvalidID) {
                    OutTriIDs.Add(t);
                    if (NormalOverlay) {
                        NormalOverlay->SetTriangle(t, FIndex3i(
                            NormalOverlay->AppendElement(FVector3f(GroundNormal)),
                            NormalOverlay->AppendElement(FVector3f(GroundNormal)),
                            NormalOverlay->AppendElement(FVector3f(GroundNormal))
                        ));
                    }
                    if (UVOverlay0 && u0_A != -1) UVOverlay0->SetTriangle(t, FIndex3i(u0_A, u0_B, u0_C));
                    if (UVOverlay1 && u1_A != -1) UVOverlay1->SetTriangle(t, FIndex3i(u1_A, u1_B, u1_C));
                }
                if (!bLocalTwoSided) {
                    int32 tBack = Mesh.AppendTriangle(a, c, b);
                    if (tBack != FDynamicMesh3::InvalidID) {
                        OutTriIDs.Add(tBack);
                        if (NormalOverlay) {
                            NormalOverlay->SetTriangle(tBack, FIndex3i(
                                NormalOverlay->AppendElement(FVector3f(GroundNormal)),
                                NormalOverlay->AppendElement(FVector3f(GroundNormal)),
                                NormalOverlay->AppendElement(FVector3f(GroundNormal))
                            ));
                        }
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


void ASmoothVoxelTerrain::FVoxelChunk::BuildEditContext(FVoxelEditContext& Ctx, ASmoothVoxelTerrain* TerrainOwner)
{
    Ctx.bValid = false;
    if (!VoxelData || !TerrainOwner) return;

    Ctx.Config = TerrainOwner->GetTerrainConfig();

    Ctx.Neighborhood.SelfData = VoxelData->GetData();
    Ctx.Neighborhood.ChunkSize = TerrainOwner->ChunkSize;
    Ctx.Neighborhood.MaxHeight = TerrainOwner->MaxHeight;
    Ctx.Neighborhood.StepY = TerrainOwner->ChunkSize;
    Ctx.Neighborhood.StepZ = TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;
    Ctx.Neighborhood.SelfCoord = Coord;

    auto RetrieveVoxelDataPtr = [&](const FIntVector& Offset) -> const EVoxelType*
        {
            if (const FVoxelChunk* Target = TerrainOwner->GetChunk(Coord + Offset))
                if (Target->VoxelData) return Target->VoxelData->GetData();
            return nullptr;
        };

    Ctx.Neighborhood.WestData = RetrieveVoxelDataPtr(FIntVector(-1, 0, 0));
    Ctx.Neighborhood.EastData = RetrieveVoxelDataPtr(FIntVector(1, 0, 0));
    Ctx.Neighborhood.SouthData = RetrieveVoxelDataPtr(FIntVector(0, -1, 0));
    Ctx.Neighborhood.NorthData = RetrieveVoxelDataPtr(FIntVector(0, 1, 0));
    Ctx.Neighborhood.SouthWestData = RetrieveVoxelDataPtr(FIntVector(-1, -1, 0));
    Ctx.Neighborhood.SouthEastData = RetrieveVoxelDataPtr(FIntVector(1, -1, 0));
    Ctx.Neighborhood.NorthWestData = RetrieveVoxelDataPtr(FIntVector(-1, 1, 0));
    Ctx.Neighborhood.NorthEastData = RetrieveVoxelDataPtr(FIntVector(1, 1, 0));

    Ctx.HeightGrid.Heights = HeightMap ? HeightMap->GetData() : nullptr;
    Ctx.HeightGrid.CacheSize = TerrainOwner->ChunkSize + 5;
    Ctx.HeightGrid.Columns = Columns ? Columns->GetData() : nullptr;
    Ctx.HeightGrid.ColumnSize = TerrainOwner->ChunkSize + 4;

    // Reads generated state only, so it stays valid across every voxel this edit touches
    // and across the write to VoxelData in the middle of UpdateVoxelMesh.
    Ctx.CaveCache.Init(&Ctx.Config, &Ctx.HeightGrid, &Ctx.Neighborhood, Coord);

    Ctx.bValid = true;
}

void ASmoothVoxelTerrain::FVoxelChunk::AddVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, FDynamicMesh3& Mesh, FDynamicMesh3* GrassMesh, ASmoothVoxelTerrain* TerrainOwner, FVoxelEditContext& Ctx)
{
    if (!VoxelData || !Ctx.bValid) return;
    const int32 VoxelIndex = LocalX + LocalY * TerrainOwner->ChunkSize + LocalZ * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;

    FTriIDArray NewTriIDs;
    Ctx.Config.AppendVoxelFacesLocal(LocalX, LocalY, LocalZ, Mesh, NewTriIDs, Ctx.HeightGrid, Ctx.Neighborhood, Coord,
        Ctx.CaveCache.IsReady() ? &Ctx.CaveCache : nullptr);
    if (NewTriIDs.Num() > 0) VoxelTriangles.Add(VoxelIndex, NewTriIDs);

    if (GrassMesh && TerrainOwner->bEnableGrassGeometry && (*VoxelData)[VoxelIndex] == EVoxelType::Grass)
    {
        if (Ctx.Neighborhood.GetVoxel(LocalX, LocalY, LocalZ + 1) == EVoxelType::Air)
        {
            FTriIDArray NewGrassTriIDs;
            Ctx.Config.AppendGrassBladesLocal(LocalX, LocalY, LocalZ, *GrassMesh, NewGrassTriIDs, Ctx.HeightGrid, Ctx.Neighborhood, Coord);
            if (NewGrassTriIDs.Num() > 0) GrassVoxelTriangles.Add(VoxelIndex, NewGrassTriIDs);
        }
    }
}

void ASmoothVoxelTerrain::FVoxelChunk::RemoveVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, FDynamicMesh3& Mesh, FDynamicMesh3* GrassMesh, ASmoothVoxelTerrain* TerrainOwner)
{
    const int32 VoxelIndex = LocalX + LocalY * TerrainOwner->ChunkSize + LocalZ * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;
    if (auto* TriIDsPtr = VoxelTriangles.Find(VoxelIndex)) {
        // Isolated vertices were being left behind on every removal, so an edited chunk's
        // vertex buffer only ever grew and every subsequent collision cook got slower.
        for (int32 TriID : *TriIDsPtr) { if (Mesh.IsTriangle(TriID)) Mesh.RemoveTriangle(TriID, true); }
        VoxelTriangles.Remove(VoxelIndex);
    }
    if (GrassMesh) {
        if (auto* GrassTriIDsPtr = GrassVoxelTriangles.Find(VoxelIndex)) {
            for (int32 TriID : *GrassTriIDsPtr) { if (GrassMesh->IsTriangle(TriID)) GrassMesh->RemoveTriangle(TriID, true); }
            GrassVoxelTriangles.Remove(VoxelIndex);
        }
    }
}

void ASmoothVoxelTerrain::FVoxelChunk::UpdateVoxelMesh(int32 LocalX, int32 LocalY, int32 LocalZ, EVoxelType NewType, ASmoothVoxelTerrain* TerrainOwner)
{
    if (!MeshComponent || !VoxelData) return;
    UDynamicMesh* DynamicMesh = MeshComponent->GetDynamicMesh();
    UDynamicMesh* GrassDynamicMesh = (bGrassGenerated && GrassMeshComponent) ? GrassMeshComponent->GetDynamicMesh() : nullptr;
    if (!DynamicMesh) return;

    FVoxelEditContext Ctx;
    BuildEditContext(Ctx, TerrainOwner);
    if (!Ctx.bValid) return;

    if (NewType != EVoxelType::Air) MaxSolidZ = FMath::Max(MaxSolidZ, LocalZ + 1);

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
                        const int32 nx = LocalX + dx, ny = LocalY + dy, nz = LocalZ + dz;
                        if (nx >= 0 && nx < TerrainOwner->ChunkSize && ny >= 0 && ny < TerrainOwner->ChunkSize && nz >= 0 && nz < TerrainOwner->MaxHeight)
                            RemoveVoxelFaces(nx, ny, nz, MeshOut, GrassMeshOut, TerrainOwner);
                    }
                }
            }

            const int32 Index = LocalX + LocalY * TerrainOwner->ChunkSize + LocalZ * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;
            (*VoxelData)[Index] = NewType;

            for (int32 dz = -1; dz <= 1; ++dz) {
                for (int32 dy = -1; dy <= 1; ++dy) {
                    for (int32 dx = -1; dx <= 1; ++dx) {
                        const int32 nx = LocalX + dx, ny = LocalY + dy, nz = LocalZ + dz;
                        if (nx >= 0 && nx < TerrainOwner->ChunkSize && ny >= 0 && ny < TerrainOwner->ChunkSize && nz >= 0 && nz < TerrainOwner->MaxHeight)
                            if ((*VoxelData)[nx + ny * TerrainOwner->ChunkSize + nz * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize] != EVoxelType::Air)
                                AddVoxelFaces(nx, ny, nz, MeshOut, GrassMeshOut, TerrainOwner, Ctx);
                    }
                }
            }
        };

    DynamicMesh->EditMesh([&](FDynamicMesh3& MeshOut) {
        if (GrassDynamicMesh) GrassDynamicMesh->EditMesh([&](FDynamicMesh3& GrassMeshOut) { UpdateBlockLogic(MeshOut, &GrassMeshOut); FMeshNormals::QuickComputeVertexNormals(GrassMeshOut); });
        else UpdateBlockLogic(MeshOut, nullptr);
        });

    TerrainOwner->MarkCollisionDirty(MeshComponent);
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