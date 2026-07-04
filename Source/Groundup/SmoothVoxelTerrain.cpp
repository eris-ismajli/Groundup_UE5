#include "SmoothVoxelTerrain.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "DynamicMesh/MeshNormals.h"
#include "DynamicMesh/MeshTangents.h"
#include "UDynamicMesh.h"
#include "Engine/World.h"
#include "Components/DynamicMeshComponent.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/Pawn.h"
#include "Async/Async.h" // Added for Async multi-threading

using namespace UE::Geometry;

static int32 FloorDiv(int32 Dividend, int32 Divisor)
{
    int32 Quotient = Dividend / Divisor;
    if ((Dividend ^ Divisor) < 0 && Dividend % Divisor != 0)
        Quotient--;
    return Quotient;
}

// Ultra-fast value-noise hash functions with floating-point multiplication replacements
FORCEINLINE float Hash2D(int32 x, int32 y)
{
    uint32 h = (uint32)x * 374761393U + (uint32)y * 668265263U;
    h = (h ^ (h >> 13)) * 1274126177U;
    return (float)(h & 0x7FFFFFFF) * 4.656612873077392578125e-10f;
}

FORCEINLINE float FastValueNoise2D(float x, float y)
{
    int32 ix = FMath::FloorToInt(x);
    int32 iy = FMath::FloorToInt(y);
    float fx = x - ix;
    float fy = y - iy;

    float ux = fx * fx * (3.0f - 2.0f * fx);
    float uy = fy * fy * (3.0f - 2.0f * fy);

    float a = Hash2D(ix, iy);
    float b = Hash2D(ix + 1, iy);
    float c = Hash2D(ix, iy + 1);
    float d = Hash2D(ix + 1, iy + 1);

    return FMath::Lerp(FMath::Lerp(a, b, ux), FMath::Lerp(c, d, ux), uy);
}

FORCEINLINE float Hash3D(int32 x, int32 y, int32 z)
{
    uint32 h = (uint32)x * 73856093U ^ (uint32)y * 19349663U ^ (uint32)z * 83492791U;
    h = (h ^ (h >> 13)) * 1274126177U;
    return (float)(h & 0x7FFFFFFF) * 4.656612873077392578125e-10f;
}

FORCEINLINE float FastValueNoise3D(float x, float y, float z)
{
    int32 ix = FMath::FloorToInt(x);
    int32 iy = FMath::FloorToInt(y);
    int32 iz = FMath::FloorToInt(z);
    float fx = x - ix;
    float fy = y - iy;
    float fz = z - iz;

    float ux = fx * fx * (3.0f - 2.0f * fx);
    float uy = fy * fy * (3.0f - 2.0f * fy);
    float uz = fz * fz * (3.0f - 2.0f * fz);

    float c000 = Hash3D(ix, iy, iz);
    float c100 = Hash3D(ix + 1, iy, iz);
    float c010 = Hash3D(ix, iy + 1, iz);
    float c110 = Hash3D(ix + 1, iy + 1, iz);
    float c001 = Hash3D(ix, iy, iz + 1);
    float c101 = Hash3D(ix + 1, iy, iz + 1);
    float c011 = Hash3D(ix, iy + 1, iz + 1);
    float c111 = Hash3D(ix + 1, iy + 1, iz + 1);

    float r0 = FMath::Lerp(FMath::Lerp(c000, c100, ux), FMath::Lerp(c010, c110, ux), uy);
    float r1 = FMath::Lerp(FMath::Lerp(c001, c101, ux), FMath::Lerp(c011, c111, ux), uy);
    return FMath::Lerp(r0, r1, uz);
}

// Ultra-fast pseudo-random generator
struct FFastRandom
{
    uint32 State;
    FORCEINLINE FFastRandom(uint32 Seed) : State(Seed) {}
    FORCEINLINE float NextFloat()
    {
        State = State * 1664525U + 1013904223U;
        return (float)(State & 0x7FFFFFFF) * 4.656612873077392578125e-10f;
    }
};

// Optimized local boundary cache supporting full-chunk and fast localized voxel initialization
struct FHeightCache
{
    int32 StartWorldX = 0;
    int32 StartWorldY = 0;
    int32 Size = 0;
    TArray<float> Heights;

    void Init(int32 InStartWorldX, int32 InStartWorldY, int32 InSize, const ASmoothVoxelTerrain* Terrain)
    {
        StartWorldX = InStartWorldX;
        StartWorldY = InStartWorldY;
        Size = InSize;
        Heights.SetNumUninitialized(Size * Size);
        for (int32 y = 0; y < Size; ++y)
        {
            int32 RowOffset = y * Size;
            int32 WorldY = StartWorldY + y;
            for (int32 x = 0; x < Size; ++x)
            {
                Heights[x + RowOffset] = Terrain->GetHeightAtWorldCorner(StartWorldX + x, WorldY);
            }
        }
    }

    void InitForVoxel(int32 VoxelWorldX, int32 VoxelWorldY, const ASmoothVoxelTerrain* Terrain)
    {
        StartWorldX = VoxelWorldX - 2;
        StartWorldY = VoxelWorldY - 2;
        Size = 6;
        Heights.SetNumUninitialized(Size * Size);
        for (int32 y = 0; y < Size; ++y)
        {
            int32 RowOffset = y * Size;
            int32 WorldY = StartWorldY + y;
            for (int32 x = 0; x < Size; ++x)
            {
                Heights[x + RowOffset] = Terrain->GetHeightAtWorldCorner(StartWorldX + x, WorldY);
            }
        }
    }

    FORCEINLINE float GetHeight(int32 WorldX, int32 WorldY) const
    {
        int32 lx = WorldX - StartWorldX;
        int32 ly = WorldY - StartWorldY;
#if UE_BUILD_SHIPPING
        return Heights[lx + ly * Size];
#else
        if (lx >= 0 && lx < Size && ly >= 0 && ly < Size)
        {
            return Heights[lx + ly * Size];
        }
        return 0.0f;
#endif
    }
};

struct FChunkNeighborhood
{
    const ASmoothVoxelTerrain::FVoxelChunk* Self = nullptr;
    const ASmoothVoxelTerrain::FVoxelChunk* West = nullptr;
    const ASmoothVoxelTerrain::FVoxelChunk* East = nullptr;
    const ASmoothVoxelTerrain::FVoxelChunk* South = nullptr;
    const ASmoothVoxelTerrain::FVoxelChunk* North = nullptr;

    const EVoxelType* SelfData = nullptr;
    const EVoxelType* WestData = nullptr;
    const EVoxelType* EastData = nullptr;
    const EVoxelType* SouthData = nullptr;
    const EVoxelType* NorthData = nullptr;

    int32 ChunkSize = 32;
    int32 MaxHeight = 64;
    int32 StepY = 32;
    int32 StepZ = 32 * 32;

    FORCEINLINE EVoxelType GetVoxel(int32 LocalX, int32 LocalY, int32 LocalZ) const
    {
        if (LocalZ < 0 || LocalZ >= MaxHeight) return EVoxelType::Air;

        if (uint32(LocalX) < uint32(ChunkSize) && uint32(LocalY) < uint32(ChunkSize))
        {
            return SelfData[LocalX + LocalY * StepY + LocalZ * StepZ];
        }

        const EVoxelType* TargetData = SelfData;
        int32 LX = LocalX;
        int32 LY = LocalY;

        if (LX < 0)
        {
            TargetData = WestData;
            LX += ChunkSize;
        }
        else if (LX >= ChunkSize)
        {
            TargetData = EastData;
            LX -= ChunkSize;
        }

        if (LY < 0)
        {
            TargetData = SouthData;
            LY += ChunkSize;
        }
        else if (LY >= ChunkSize)
        {
            TargetData = NorthData;
            LY -= ChunkSize;
        }

        if (!TargetData) return EVoxelType::Air;
        return TargetData[LX + LY * ChunkSize + LocalZ * StepZ];
    }
};

ASmoothVoxelTerrain::ASmoothVoxelTerrain()
{
    PrimaryActorTick.bCanEverTick = true;
    RootSceneComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    RootComponent = RootSceneComponent;
}

ASmoothVoxelTerrain::~ASmoothVoxelTerrain()
{
    bIsDestroyed = true;
}

void ASmoothVoxelTerrain::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);

    if (bIsDestroyed || HasAnyFlags(RF_ClassDefaultObject | RF_ArchetypeObject) || !GetWorld() || GetWorld()->bIsTearingDown || IsActorBeingDestroyed())
        return;

    RebuildTerrain();
}

void ASmoothVoxelTerrain::BeginPlay()
{
    Super::BeginPlay();
    if (Chunks.Num() == 0)
    {
        RebuildTerrain();
    }
}

void ASmoothVoxelTerrain::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (bIsDestroyed) return;

    TimeSinceLastUpdate += DeltaTime;
    if (TimeSinceLastUpdate >= UpdateInterval)
    {
        TimeSinceLastUpdate = 0.0f;
        UpdateProceduralTerrain();
    }

    ProcessGenerationQueue();
    UpdateCollisionIfNeeded();
}

void ASmoothVoxelTerrain::UpdateCollisionIfNeeded()
{
    if (bCollisionDirty)
    {
        for (auto& Pair : Chunks)
        {
            if (Pair.Value && Pair.Value->MeshComponent)
                Pair.Value->MeshComponent->UpdateCollision(false);
        }
        bCollisionDirty = false;
    }
}

void ASmoothVoxelTerrain::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    bIsDestroyed = true;
    for (auto& Pair : Chunks)
    {
        if (Pair.Value)
        {
            if (Pair.Value->MeshComponent && IsValid(Pair.Value->MeshComponent))
            {
                Pair.Value->MeshComponent->DestroyComponent();
            }
            if (Pair.Value->GrassMeshComponent && IsValid(Pair.Value->GrassMeshComponent))
            {
                Pair.Value->GrassMeshComponent->DestroyComponent();
            }
        }
    }
    Chunks.Empty();
    GenerationQueue.Empty();
    Super::EndPlay(EndPlayReason);
}

void ASmoothVoxelTerrain::GenerateChunks()
{
    for (auto& Pair : Chunks)
    {
        if (Pair.Value)
        {
            if (Pair.Value->MeshComponent)
            {
                Pair.Value->MeshComponent->UnregisterComponent();
                Pair.Value->MeshComponent->DestroyComponent();
            }
            if (Pair.Value->GrassMeshComponent)
            {
                Pair.Value->GrassMeshComponent->UnregisterComponent();
                Pair.Value->GrassMeshComponent->DestroyComponent();
            }
        }
    }
    Chunks.Empty();
    GenerationQueue.Empty();

    TArray<UDynamicMeshComponent*> OldComps;
    GetComponents<UDynamicMeshComponent>(OldComps);
    for (UDynamicMeshComponent* Comp : OldComps)
    {
        Comp->UnregisterComponent();
        Comp->DestroyComponent();
    }

    FVector PlayerPos = GetActorLocation();
    if (GetWorld())
    {
        if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
        {
            if (APawn* Pawn = PC->GetPawnOrSpectator())
            {
                PlayerPos = Pawn->GetActorLocation();
            }
        }
    }

    FIntVector PlayerChunk = WorldToChunkCoord(PlayerPos);
    LastPlayerChunkCoord = PlayerChunk;

    const int32 SafeSpawnRadius = 1;
    for (int32 dx = -SafeSpawnRadius; dx <= SafeSpawnRadius; ++dx)
    {
        for (int32 dy = -SafeSpawnRadius; dy <= SafeSpawnRadius; ++dy)
        {
            FIntVector SpawnCoord(PlayerChunk.X + dx, PlayerChunk.Y + dy, 0);
            GenerateSingleChunk(SpawnCoord);
        }
    }

    TSet<FIntVector> TargetCoords;
    for (int32 dx = -RenderDistance; dx <= RenderDistance; ++dx)
    {
        for (int32 dy = -RenderDistance; dy <= RenderDistance; ++dy)
        {
            if (dx * dx + dy * dy <= RenderDistance * RenderDistance)
            {
                TargetCoords.Add(FIntVector(PlayerChunk.X + dx, PlayerChunk.Y + dy, 0));
            }
        }
    }

    for (const FIntVector& Coord : TargetCoords)
    {
        if (!Chunks.Contains(Coord))
        {
            GenerationQueue.Add(Coord);
        }
    }

    GenerationQueue.Sort([PlayerChunk](const FIntVector& A, const FIntVector& B) {
        int32 DistA = FMath::Square(A.X - PlayerChunk.X) + FMath::Square(A.Y - PlayerChunk.Y);
        int32 DistB = FMath::Square(B.X - PlayerChunk.X) + FMath::Square(B.Y - PlayerChunk.Y);
        return DistA < DistB;
        });
}

void ASmoothVoxelTerrain::UpdateProceduralTerrain()
{
    FVector PlayerPos = GetActorLocation();
    if (GetWorld())
    {
        if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
        {
            if (APawn* Pawn = PC->GetPawnOrSpectator())
            {
                PlayerPos = Pawn->GetActorLocation();
            }
        }
    }

    FIntVector PlayerChunk = WorldToChunkCoord(PlayerPos);

    if (PlayerChunk != LastPlayerChunkCoord)
    {
        LastPlayerChunkCoord = PlayerChunk;

        TSet<FIntVector> DesiredCoords;
        for (int32 dx = -RenderDistance; dx <= RenderDistance; ++dx)
        {
            for (int32 dy = -RenderDistance; dy <= RenderDistance; ++dy)
            {
                if (dx * dx + dy * dy <= RenderDistance * RenderDistance)
                {
                    DesiredCoords.Add(FIntVector(PlayerChunk.X + dx, PlayerChunk.Y + dy, 0));
                }
            }
        }

        TArray<FIntVector> CoordsToUnload;
        for (const auto& Pair : Chunks)
        {
            FIntVector Coord = Pair.Key;
            float DistSq = FMath::Square(Coord.X - PlayerChunk.X) + FMath::Square(Coord.Y - PlayerChunk.Y);
            if (DistSq > FMath::Square(UnloadDistance))
            {
                CoordsToUnload.Add(Coord);
            }
        }

        for (const FIntVector& Coord : CoordsToUnload)
        {
            UnloadChunk(Coord);
        }

        GenerationQueue.Empty();
        for (const FIntVector& Coord : DesiredCoords)
        {
            if (!Chunks.Contains(Coord))
            {
                GenerationQueue.Add(Coord);
            }
        }

        GenerationQueue.Sort([PlayerChunk](const FIntVector& A, const FIntVector& B) {
            int32 DistA = FMath::Square(A.X - PlayerChunk.X) + FMath::Square(A.Y - PlayerChunk.Y);
            int32 DistB = FMath::Square(B.X - PlayerChunk.X) + FMath::Square(B.Y - PlayerChunk.Y);
            return DistA < DistB;
            });
    }
}

void ASmoothVoxelTerrain::ProcessGenerationQueue()
{
    int32 ProcessedThisFrame = 0;
    while (GenerationQueue.Num() > 0 && ProcessedThisFrame < MaxChunkGenPerFrame)
    {
        FIntVector TargetCoord = GenerationQueue[0];
        GenerationQueue.RemoveAt(0);

        if (!Chunks.Contains(TargetCoord))
        {
            GenerateSingleChunk(TargetCoord);
            ProcessedThisFrame++;
        }
    }
}

void ASmoothVoxelTerrain::GenerateSingleChunk(const FIntVector& ChunkCoord)
{
    double GameThreadStart = FPlatformTime::Seconds();

    // Instantiate structures safely on the Game Thread
    TUniquePtr<FVoxelChunk> Chunk = MakeUnique<FVoxelChunk>();
    Chunk->Coord = ChunkCoord;
    Chunk->VoxelData.SetNumZeroed(ChunkSize * ChunkSize * MaxHeight);

    UDynamicMeshComponent* MeshComp = NewObject<UDynamicMeshComponent>(this);
    MeshComp->CreationMethod = EComponentCreationMethod::Instance;
    MeshComp->AttachToComponent(RootSceneComponent, FAttachmentTransformRules::KeepRelativeTransform);
    MeshComp->SetRelativeTransform(FTransform());
    MeshComp->RegisterComponent();
    MeshComp->SetVisibility(true);
    MeshComp->SetCastShadow(bCastShadow);
    MeshComp->SetReceivesDecals(bReceivesDecals);
    MeshComp->EnableComplexAsSimpleCollision();
    MeshComp->bEnableComplexCollision = bEnableComplexCollision;
    MeshComp->SetCollisionEnabled(CollisionEnabled);
    MeshComp->SetCollisionProfileName(CollisionProfileName);
    MeshComp->SetGenerateOverlapEvents(bGenerateOverlapEvents);
    MeshComp->bUseAsyncCooking = true;

    if (GrassMaterial) MeshComp->SetMaterial(0, GrassMaterial);
    if (DirtMaterial) MeshComp->SetMaterial(1, DirtMaterial);
    if (StoneMaterial) MeshComp->SetMaterial(2, StoneMaterial);
    Chunk->MeshComponent = MeshComp;

    UDynamicMeshComponent* GrassMeshComp = NewObject<UDynamicMeshComponent>(this);
    GrassMeshComp->CreationMethod = EComponentCreationMethod::Instance;
    GrassMeshComp->AttachToComponent(RootSceneComponent, FAttachmentTransformRules::KeepRelativeTransform);
    GrassMeshComp->SetRelativeTransform(FTransform());
    GrassMeshComp->RegisterComponent();
    GrassMeshComp->SetVisibility(true);
    GrassMeshComp->SetCastShadow(false);
    GrassMeshComp->SetReceivesDecals(false);
    GrassMeshComp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    GrassMeshComp->bEnableComplexCollision = false;

    if (GrassBladesMaterial) GrassMeshComp->SetMaterial(0, GrassBladesMaterial);
    Chunk->GrassMeshComponent = GrassMeshComp;

    FVoxelChunk* ChunkPtr = Chunk.Get();
    Chunks.Add(ChunkCoord, MoveTemp(Chunk));

    TWeakObjectPtr<ASmoothVoxelTerrain> WeakThis(this);

    // Completely offload Voxel Data Noise computations to background threads
    Async(EAsyncExecution::ThreadPool, [WeakThis, ChunkCoord, GameThreadStart]() mutable
        {
            ASmoothVoxelTerrain* Terrain = WeakThis.Get();
            if (!Terrain || Terrain->bIsDestroyed) return;

            double DataStart = FPlatformTime::Seconds();

            int32 LocalChunkSize = Terrain->ChunkSize;
            int32 LocalMaxHeight = Terrain->MaxHeight;
            float LocalMinGrassThickness = Terrain->MinGrassThickness;

            TArray<EVoxelType> LocalVoxelData;
            LocalVoxelData.SetNumZeroed(LocalChunkSize * LocalChunkSize * LocalMaxHeight);

            TArray<float> LocalHeights;
            LocalHeights.SetNumUninitialized((LocalChunkSize + 1) * (LocalChunkSize + 1));

            for (int32 ly = 0; ly <= LocalChunkSize; ++ly)
            {
                int32 RowOffset = ly * (LocalChunkSize + 1);
                int32 WorldY = ChunkCoord.Y * LocalChunkSize + ly;
                for (int32 lx = 0; lx <= LocalChunkSize; ++lx)
                {
                    int32 WorldX = ChunkCoord.X * LocalChunkSize + lx;
                    LocalHeights[lx + RowOffset] = Terrain->GetHeightAtWorldCorner(WorldX, WorldY);
                }
            }

            for (int32 lx = 0; lx < LocalChunkSize; ++lx)
            {
                for (int32 ly = 0; ly < LocalChunkSize; ++ly)
                {
                    float h00 = LocalHeights[lx + ly * (LocalChunkSize + 1)];
                    float h10 = LocalHeights[(lx + 1) + ly * (LocalChunkSize + 1)];
                    float h01 = LocalHeights[lx + (ly + 1) * (LocalChunkSize + 1)];
                    float h11 = LocalHeights[(lx + 1) + (ly + 1) * (LocalChunkSize + 1)];
                    float MinCorner = FMath::Min3(h00, h10, FMath::Min(h01, h11));
                    int32 GroundLevel = FMath::Clamp(FMath::FloorToInt(MinCorner - LocalMinGrassThickness), 0, LocalMaxHeight - 1);

                    int32 BaseIdx = lx + ly * LocalChunkSize;
                    int32 Step = LocalChunkSize * LocalChunkSize;

                    int32 StoneBound = GroundLevel - 3;
                    int32 DirtBound = GroundLevel;

                    for (int32 lz = 0; lz < LocalMaxHeight; ++lz)
                    {
                        int32 Index = BaseIdx + lz * Step;
                        if (lz < StoneBound) LocalVoxelData[Index] = EVoxelType::Stone;
                        else if (lz < DirtBound) LocalVoxelData[Index] = EVoxelType::Dirt;
                        else if (lz == GroundLevel) LocalVoxelData[Index] = EVoxelType::Grass;
                        else LocalVoxelData[Index] = EVoxelType::Air;
                    }
                }
            }

            double DataEnd = FPlatformTime::Seconds();

            // Dispatch back to Game Thread to apply data and initiate asynchronous mesh compilation
            AsyncTask(ENamedThreads::GameThread, [WeakThis, ChunkCoord, LocalVoxelData = MoveTemp(LocalVoxelData), DataStart, DataEnd]() mutable
                {
                    ASmoothVoxelTerrain* Terrain = WeakThis.Get();
                    if (!Terrain || Terrain->bIsDestroyed) return;

                    FVoxelChunk* TargetChunk = Terrain->GetChunk(ChunkCoord);
                    if (!TargetChunk) return;

                    TargetChunk->VoxelData = MoveTemp(LocalVoxelData);

                    UE_LOG(LogTemp, Warning, TEXT("[PERF] Chunk %s - Voxel Data Noise Gen: %.2f ms"), *ChunkCoord.ToString(), (DataEnd - DataStart) * 1000.0);

                    // Trigger threaded build mesh pipeline
                    TargetChunk->BuildMesh(Terrain);

                    // Cascade neighboring chunk seam resolution if they are loaded
                    FIntVector Neighbors[4] = {
                        FIntVector(ChunkCoord.X - 1, ChunkCoord.Y, 0),
                        FIntVector(ChunkCoord.X + 1, ChunkCoord.Y, 0),
                        FIntVector(ChunkCoord.X, ChunkCoord.Y - 1, 0),
                        FIntVector(ChunkCoord.X, ChunkCoord.Y + 1, 0)
                    };

                    for (const FIntVector& NeighborCoord : Neighbors)
                    {
                        if (FVoxelChunk* Neighbor = Terrain->GetChunk(NeighborCoord))
                        {
                            Neighbor->BuildMesh(Terrain);
                        }
                    }
                });
        });
}

void ASmoothVoxelTerrain::UnloadChunk(const FIntVector& Coord)
{
    TUniquePtr<FVoxelChunk> Chunk;
    if (Chunks.RemoveAndCopyValue(Coord, Chunk))
    {
        if (Chunk)
        {
            if (Chunk->MeshComponent && IsValid(Chunk->MeshComponent))
            {
                Chunk->MeshComponent->UnregisterComponent();
                Chunk->MeshComponent->DestroyComponent();
            }
            if (Chunk->GrassMeshComponent && IsValid(Chunk->GrassMeshComponent))
            {
                Chunk->GrassMeshComponent->UnregisterComponent();
                Chunk->GrassMeshComponent->DestroyComponent();
            }
        }
    }
}

void ASmoothVoxelTerrain::FVoxelChunk::BuildMesh(ASmoothVoxelTerrain* TerrainOwner)
{
    if (!MeshComponent || !GrassMeshComponent) return;

    double StartTime = FPlatformTime::Seconds();

    // Take localized snapshots of neighbor voxel arrays to prevent invalid memory pointers across threads
    TArray<EVoxelType> SelfDataCopy = VoxelData;
    TArray<EVoxelType> WestData, EastData, SouthData, NorthData;

    auto RetrieveData = [&](const FIntVector& Offset, TArray<EVoxelType>& OutData) {
        if (const FVoxelChunk* Target = TerrainOwner->GetChunk(Coord + Offset)) {
            OutData = Target->VoxelData;
        }
        };
    RetrieveData(FIntVector(-1, 0, 0), WestData);
    RetrieveData(FIntVector(1, 0, 0), EastData);
    RetrieveData(FIntVector(0, -1, 0), SouthData);
    RetrieveData(FIntVector(0, 1, 0), NorthData);

    FIntVector LocalCoord = Coord;
    TWeakObjectPtr<ASmoothVoxelTerrain> WeakTerrain(TerrainOwner);

    // Completely offload Vertex Generation, Loops, Math, and Geometry Construction to the thread pool
    Async(EAsyncExecution::ThreadPool, [WeakTerrain, LocalCoord, SelfDataCopy = MoveTemp(SelfDataCopy),
        WestData = MoveTemp(WestData), EastData = MoveTemp(EastData),
        SouthData = MoveTemp(SouthData), NorthData = MoveTemp(NorthData), StartTime]() mutable
        {
            ASmoothVoxelTerrain* Terrain = WeakTerrain.Get();
            if (!Terrain || Terrain->bIsDestroyed) return;

            double AsyncMeshStart = FPlatformTime::Seconds();

            // 1. Render Setup
            FDynamicMesh3 LocalMesh;
            LocalMesh.EnableAttributes();
            FDynamicMeshAttributeSet* Attr = LocalMesh.Attributes();
            Attr->SetNumUVLayers(2);
            Attr->EnablePrimaryColors();
            Attr->EnableMaterialID();

            FDynamicMesh3 LocalGrassMesh;
            LocalGrassMesh.EnableAttributes();
            FDynamicMeshAttributeSet* GrassAttr = LocalGrassMesh.Attributes();
            GrassAttr->SetNumUVLayers(2);
            GrassAttr->EnablePrimaryColors();

            // Stitch memory-safe neighborhood proxy wrapper
            FChunkNeighborhood Neighborhood;
            Neighborhood.SelfData = SelfDataCopy.GetData();
            Neighborhood.WestData = WestData.Num() > 0 ? WestData.GetData() : nullptr;
            Neighborhood.EastData = EastData.Num() > 0 ? EastData.GetData() : nullptr;
            Neighborhood.SouthData = SouthData.Num() > 0 ? SouthData.GetData() : nullptr;
            Neighborhood.NorthData = NorthData.Num() > 0 ? NorthData.GetData() : nullptr;
            Neighborhood.ChunkSize = Terrain->ChunkSize;
            Neighborhood.MaxHeight = Terrain->MaxHeight;
            Neighborhood.StepY = Terrain->ChunkSize;
            Neighborhood.StepZ = Terrain->ChunkSize * Terrain->ChunkSize;

            FVoxelChunk MockSelf;
            MockSelf.Coord = LocalCoord;
            Neighborhood.Self = &MockSelf;

            FHeightCache HeightCache;
            HeightCache.Init(LocalCoord.X * Terrain->ChunkSize - 1, LocalCoord.Y * Terrain->ChunkSize - 1, Terrain->ChunkSize + 3, Terrain);

            TMap<int32, FTriIDArray> NewVoxelTriangles;
            TMap<int32, FTriIDArray> NewGrassTriangles;

            // 2. Main Geometry Engine Processor Loops
            FTriIDArray TempTriIDs;
            for (int32 lx = 0; lx < Terrain->ChunkSize; ++lx)
            {
                for (int32 ly = 0; ly < Terrain->ChunkSize; ++ly)
                {
                    int32 BaseIdx = lx + ly * Terrain->ChunkSize;
                    int32 StepZ = Terrain->ChunkSize * Terrain->ChunkSize;

                    for (int32 lz = 0; lz < Terrain->MaxHeight; ++lz)
                    {
                        int32 Index = BaseIdx + lz * StepZ;
                        EVoxelType VType = Neighborhood.SelfData[Index];

                        if (VType == EVoxelType::Air) continue;

                        int32 WorldX = LocalCoord.X * Terrain->ChunkSize + lx;
                        int32 WorldY = LocalCoord.Y * Terrain->ChunkSize + ly;

                        TempTriIDs.Reset();
                        Terrain->AppendVoxelFacesWorld(WorldX, WorldY, lz, LocalMesh, TempTriIDs, HeightCache, Neighborhood);

                        if (TempTriIDs.Num() > 0)
                        {
                            NewVoxelTriangles.Add(Index, TempTriIDs);
                        }

                        if (Terrain->bEnableGrassGeometry && VType == EVoxelType::Grass)
                        {
                            if (Neighborhood.GetVoxel(lx, ly, lz + 1) == EVoxelType::Air)
                            {
                                TempTriIDs.Reset();
                                Terrain->AppendGrassBladesWorld(WorldX, WorldY, lz, LocalGrassMesh, TempTriIDs, HeightCache, Neighborhood);

                                if (TempTriIDs.Num() > 0)
                                {
                                    NewGrassTriangles.Add(Index, TempTriIDs);
                                }
                            }
                        }
                    }
                }
            }

            FMeshNormals::QuickComputeVertexNormals(LocalGrassMesh);

            double AsyncMeshEnd = FPlatformTime::Seconds();

            // Dispatch Geometry assignments efficiently onto the Game Thread memory allocations via fast Move Semantics
            AsyncTask(ENamedThreads::GameThread, [WeakTerrain, LocalCoord, LocalMesh = MoveTemp(LocalMesh), LocalGrassMesh = MoveTemp(LocalGrassMesh), NewVoxelTriangles = MoveTemp(NewVoxelTriangles), NewGrassTriangles = MoveTemp(NewGrassTriangles), StartTime, AsyncMeshStart, AsyncMeshEnd]() mutable
                {
                    ASmoothVoxelTerrain* Terrain = WeakTerrain.Get();
                    if (!Terrain || Terrain->bIsDestroyed) return;

                    FVoxelChunk* Chunk = Terrain->GetChunk(LocalCoord);
                    if (!Chunk || !Chunk->MeshComponent || !Chunk->GrassMeshComponent) return;

                    Chunk->VoxelTriangles = MoveTemp(NewVoxelTriangles);
                    Chunk->GrassVoxelTriangles = MoveTemp(NewGrassTriangles);

                    // Zero-overhead assignment leveraging FDynamicMesh Move operations directly avoiding re-allocations
                    Chunk->MeshComponent->GetDynamicMesh()->EditMesh([&](FDynamicMesh3& MeshOut) {
                        MeshOut = MoveTemp(LocalMesh);
                        });

                    Chunk->GrassMeshComponent->GetDynamicMesh()->EditMesh([&](FDynamicMesh3& GrassMeshOut) {
                        GrassMeshOut = MoveTemp(LocalGrassMesh);
                        });

                    Terrain->bCollisionDirty = true;

                    double FinalTime = FPlatformTime::Seconds();
                    float AsyncMS = (AsyncMeshEnd - AsyncMeshStart) * 1000.0f;
                    float TotalLatencyMS = (FinalTime - StartTime) * 1000.0f;

                    UE_LOG(LogTemp, Warning, TEXT("[PERF] Chunk %s - Mesh Build Async: %.2f ms (Total Frame Latency Wait: %.2f ms)"), *LocalCoord.ToString(), AsyncMS, TotalLatencyMS);
                });
        });
}

void ASmoothVoxelTerrain::FVoxelChunk::UpdateSharedFace(int32 LocalX, int32 LocalY, int32 LocalZ, ASmoothVoxelTerrain* TerrainOwner, const FIntVector& NeighborDirection)
{
    int32 Index = LocalX + LocalY * TerrainOwner->ChunkSize + LocalZ * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;
    if (VoxelData[Index] == EVoxelType::Air) return;

    UDynamicMesh* DynamicMesh = MeshComponent->GetDynamicMesh();
    UDynamicMesh* GrassDynamicMesh = GrassMeshComponent->GetDynamicMesh();
    if (!DynamicMesh || !GrassDynamicMesh) return;

    DynamicMesh->EditMesh([&](FDynamicMesh3& MeshOut)
        {
            GrassDynamicMesh->EditMesh([&](FDynamicMesh3& GrassMeshOut)
                {
                    RemoveVoxelFaces(LocalX, LocalY, LocalZ, MeshOut, GrassMeshOut, TerrainOwner);
                    AddVoxelFaces(LocalX, LocalY, LocalZ, MeshOut, GrassMeshOut, TerrainOwner);
                });
        });
}

void ASmoothVoxelTerrain::FVoxelChunk::UpdateVoxel(int32 LocalX, int32 LocalY, int32 LocalZ, EVoxelType NewType, ASmoothVoxelTerrain* TerrainOwner)
{
    int32 Index = LocalX + LocalY * TerrainOwner->ChunkSize + LocalZ * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;
    if (VoxelData[Index] == NewType) return;

    UpdateVoxelMesh(LocalX, LocalY, LocalZ, NewType, TerrainOwner);
}

bool ASmoothVoxelTerrain::GetVoxelAtWorldPoint(const FVector& WorldPoint, int32& OutVoxelX, int32& OutVoxelY, int32& OutVoxelZ, EVoxelType* OutType)
{
    FVector LocalPos = GetActorTransform().InverseTransformPosition(WorldPoint);

    OutVoxelX = FMath::FloorToInt(LocalPos.X / CubeSize);
    OutVoxelY = FMath::FloorToInt(LocalPos.Y / CubeSize);
    OutVoxelZ = FMath::FloorToInt(LocalPos.Z / CubeSize);

    if (OutVoxelZ < 0 || OutVoxelZ >= MaxHeight)
    {
        return false;
    }

    if (OutType)
    {
        *OutType = GetVoxelAtWorld(OutVoxelX, OutVoxelY, OutVoxelZ);
    }

    return true;
}

void ASmoothVoxelTerrain::RemoveVoxel(FVector WorldLocation)
{
    if (bIsDestroyed) return;

    FIntVector ChunkCoord = WorldToChunkCoord(WorldLocation);
    FVoxelChunk* Chunk = GetChunk(ChunkCoord);
    if (!Chunk) return;

    int32 lx, ly, lz;
    WorldToLocalVoxel(WorldLocation, ChunkCoord, lx, ly, lz);
    if (lx < 0 || lx >= ChunkSize || ly < 0 || ly >= ChunkSize || lz < 0 || lz >= MaxHeight) return;

    int32 Index = lx + ly * ChunkSize + lz * ChunkSize * ChunkSize;
    if (Chunk->VoxelData[Index] == EVoxelType::Air) {
        return;
    };

    double Start = FPlatformTime::Seconds();

    Chunk->UpdateVoxel(lx, ly, lz, EVoxelType::Air, this);

    if (lx == 0) {
        if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(-1, 0, 0)))
            Neighbor->UpdateSharedFace(ChunkSize - 1, ly, lz, this, FIntVector(1, 0, 0));
    }
    if (lx == ChunkSize - 1) {
        if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(1, 0, 0)))
            Neighbor->UpdateSharedFace(0, ly, lz, this, FIntVector(-1, 0, 0));
    }
    if (ly == 0) {
        if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, -1, 0)))
            Neighbor->UpdateSharedFace(lx, ChunkSize - 1, lz, this, FIntVector(0, 1, 0));
    }
    if (ly == ChunkSize - 1) {
        if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, 1, 0)))
            Neighbor->UpdateSharedFace(lx, 0, lz, this, FIntVector(0, -1, 0));
    }
    if (lz == 0) {
        if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, 0, -1)))
            Neighbor->UpdateSharedFace(lx, ly, MaxHeight - 1, this, FIntVector(0, 0, 1));
    }
    if (lz == MaxHeight - 1) {
        if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, 0, 1)))
            Neighbor->UpdateSharedFace(lx, ly, 0, this, FIntVector(0, 0, -1));
    }

    double End = FPlatformTime::Seconds();
    UE_LOG(LogTemp, Warning, TEXT("[PERF] RemoveVoxel Local Modify took %.2f ms"), (End - Start) * 1000.0);
}

void ASmoothVoxelTerrain::PlaceVoxel(FVector WorldLocation, EVoxelType Type)
{
    if (bIsDestroyed || Type == EVoxelType::Air) return;

    FIntVector ChunkCoord = WorldToChunkCoord(WorldLocation);
    FVoxelChunk* Chunk = GetChunk(ChunkCoord);
    if (!Chunk) return;

    int32 lx, ly, lz;
    WorldToLocalVoxel(WorldLocation, ChunkCoord, lx, ly, lz);

    if (lx < 0 || lx >= ChunkSize || ly < 0 || ly >= ChunkSize || lz < 0 || lz >= MaxHeight) return;

    Chunk->UpdateVoxel(lx, ly, lz, Type, this);

    if (lx == 0) {
        if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(-1, 0, 0)))
            Neighbor->UpdateSharedFace(ChunkSize - 1, ly, lz, this, FIntVector(1, 0, 0));
    }
    if (lx == ChunkSize - 1) {
        if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(1, 0, 0)))
            Neighbor->UpdateSharedFace(0, ly, lz, this, FIntVector(-1, 0, 0));
    }
    if (ly == 0) {
        if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, -1, 0)))
            Neighbor->UpdateSharedFace(lx, ChunkSize - 1, lz, this, FIntVector(0, 1, 0));
    }
    if (ly == ChunkSize - 1) {
        if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, 1, 0)))
            Neighbor->UpdateSharedFace(lx, 0, lz, this, FIntVector(0, -1, 0));
    }
    if (lz == 0) {
        if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, 0, -1)))
            Neighbor->UpdateSharedFace(lx, ly, MaxHeight - 1, this, FIntVector(0, 0, 1));
    }
    if (lz == MaxHeight - 1) {
        if (FVoxelChunk* Neighbor = GetChunk(ChunkCoord + FIntVector(0, 0, 1)))
            Neighbor->UpdateSharedFace(lx, ly, 0, this, FIntVector(0, 0, -1));
    }
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
    return FIntVector(
        FloorDiv(WorldX, ChunkSize),
        FloorDiv(WorldY, ChunkSize),
        0
    );
}

void ASmoothVoxelTerrain::WorldToLocalVoxel(const FVector& WorldPos, const FIntVector& ChunkCoord, int32& OutX, int32& OutY, int32& OutZ) const
{
    FVector LocalPos = GetActorTransform().InverseTransformPosition(WorldPos);
    int32 WorldX = FMath::FloorToInt(LocalPos.X / CubeSize);
    int32 WorldY = FMath::FloorToInt(LocalPos.Y / CubeSize);
    int32 WorldZ = FMath::FloorToInt(LocalPos.Z / CubeSize);
    OutX = WorldX - ChunkCoord.X * ChunkSize;
    OutY = WorldY - ChunkCoord.Y * ChunkSize;
    OutZ = WorldZ;
}

FVector ASmoothVoxelTerrain::ChunkCoordToWorldOrigin(const FIntVector& ChunkCoord) const
{
    FVector LocalOrigin((double)ChunkCoord.X * ChunkSize * CubeSize, (double)ChunkCoord.Y * ChunkSize * CubeSize, 0.0f);
    return GetActorTransform().TransformPosition(LocalOrigin);
}

EVoxelType ASmoothVoxelTerrain::GetVoxelAtWorld(int32 WorldX, int32 WorldY, int32 WorldZ) const
{
    if (WorldZ < 0 || WorldZ >= MaxHeight) return EVoxelType::Air;

    int32 ChunkX = FloorDiv(WorldX, ChunkSize);
    int32 ChunkY = FloorDiv(WorldY, ChunkSize);
    FIntVector ChunkCoord(ChunkX, ChunkY, 0);
    const FVoxelChunk* Chunk = GetChunk(ChunkCoord);
    if (!Chunk) return EVoxelType::Air;

    int32 LocalX = WorldX - ChunkX * ChunkSize;
    int32 LocalY = WorldY - ChunkY * ChunkSize;
    if (LocalX < 0 || LocalX >= ChunkSize || LocalY < 0 || LocalY >= ChunkSize) return EVoxelType::Air;

    int32 Index = LocalX + LocalY * ChunkSize + WorldZ * ChunkSize * ChunkSize;
    if (!Chunk->VoxelData.IsValidIndex(Index)) return EVoxelType::Air;
    return Chunk->VoxelData[Index];
}

float ASmoothVoxelTerrain::GetHeightAtWorldCorner(int32 WorldX, int32 WorldY) const
{
    float NoiseValue = FastValueNoise2D(((float)WorldX + Seed) * NoiseScale, ((float)WorldY + Seed) * NoiseScale) * 2.0f - 1.0f;
    return (NoiseValue + 1.0f) * HeightMultiplier / CubeSize;
}

float ASmoothVoxelTerrain::GetInterpolatedHeight(float WorldX, float WorldY) const
{
    int32 x0 = FMath::FloorToInt(WorldX);
    int32 y0 = FMath::FloorToInt(WorldY);
    int32 x1 = x0 + 1;
    int32 y1 = y0 + 1;
    float fx = WorldX - x0;
    float fy = WorldY - y0;

    float h00 = GetHeightAtWorldCorner(x0, y0);
    float h10 = GetHeightAtWorldCorner(x1, y0);
    float h01 = GetHeightAtWorldCorner(x0, y1);
    float h11 = GetHeightAtWorldCorner(x1, y1);

    return FMath::Lerp(FMath::Lerp(h00, h10, fx), FMath::Lerp(h01, h11, fx), fy);
}

float ASmoothVoxelTerrain::GetInterpolatedHeightCached(float WorldX, float WorldY, const FHeightCache& HeightCache) const
{
    int32 x0 = FMath::FloorToInt(WorldX);
    int32 y0 = FMath::FloorToInt(WorldY);
    int32 x1 = x0 + 1;
    int32 y1 = y0 + 1;
    float fx = WorldX - x0;
    float fy = WorldY - y0;

    float h00 = HeightCache.GetHeight(x0, y0);
    float h10 = HeightCache.GetHeight(x1, y0);
    float h01 = HeightCache.GetHeight(x0, y1);
    float h11 = HeightCache.GetHeight(x1, y1);

    return FMath::Lerp(FMath::Lerp(h00, h10, fx), FMath::Lerp(h01, h11, fx), fy);
}

FVector ASmoothVoxelTerrain::GetSmoothVertexWorld(int32 WorldX, int32 WorldY, int32 WorldZ, int32 VoxX, int32 VoxY, int32 VoxZ, const FHeightCache& HeightCache, const FChunkNeighborhood& Neighborhood) const
{
    if (!bSmoothTerrain)
        return FVector(WorldX, WorldY, WorldZ) * CubeSize;

    float TargetH = HeightCache.GetHeight(WorldX, WorldY);
    float FinalZ = (float)WorldZ;

    int32 LocalVoxX = VoxX - Neighborhood.Self->Coord.X * ChunkSize;
    int32 LocalVoxY = VoxY - Neighborhood.Self->Coord.Y * ChunkSize;

    if (Neighborhood.GetVoxel(LocalVoxX, LocalVoxY, VoxZ) == EVoxelType::Grass && WorldZ > VoxZ)
    {
        if (Neighborhood.GetVoxel(LocalVoxX, LocalVoxY, VoxZ + 1) != EVoxelType::Air)
            return FVector(WorldX, WorldY, WorldZ) * CubeSize;
        FinalZ = TargetH;
    }

    return FVector(WorldX, WorldY, FinalZ) * CubeSize;
}

FVector ASmoothVoxelTerrain::GetSmoothNormalWorld(int32 WorldX, int32 WorldY, const FHeightCache& HeightCache) const
{
    float hL = HeightCache.GetHeight(WorldX - 1, WorldY);
    float hR = HeightCache.GetHeight(WorldX + 1, WorldY);
    float hD = HeightCache.GetHeight(WorldX, WorldY - 1);
    float hU = HeightCache.GetHeight(WorldX, WorldY + 1);
    return FVector(hL - hR, hD - hU, 2.0f).GetSafeNormal();
}

float ASmoothVoxelTerrain::GetNeighborTopHeightWorld(int32 WorldX, int32 WorldY, int32 WorldZ, const FVector& Vertex, const FChunkNeighborhood& Neighborhood, const FHeightCache& HeightCache) const
{
    int32 LocalX = WorldX - Neighborhood.Self->Coord.X * ChunkSize;
    int32 LocalY = WorldY - Neighborhood.Self->Coord.Y * ChunkSize;
    EVoxelType neighborType = Neighborhood.GetVoxel(LocalX, LocalY, WorldZ);

    if (neighborType == EVoxelType::Air)
        return -FLT_MAX;
    else if (neighborType == EVoxelType::Grass)
    {
        float VX = (float)Vertex.X / CubeSize;
        float VY = (float)Vertex.Y / CubeSize;
        return GetInterpolatedHeightCached(VX, VY, HeightCache) * CubeSize;
    }
    else
        return (WorldZ + 1) * CubeSize;
}

FLinearColor ASmoothVoxelTerrain::GetStylizedColorForVoxel(const FVector& WorldPos, EVoxelType VoxelType) const
{
    float VoxX = (float)WorldPos.X / CubeSize;
    float VoxY = (float)WorldPos.Y / CubeSize;
    float VoxZ = (float)WorldPos.Z / CubeSize;

    if (VoxelType == EVoxelType::Grass)
    {
        float ColorNoise = FastValueNoise2D(VoxX * GrassColorNoiseScale, VoxY * GrassColorNoiseScale);
        return FLinearColor::LerpUsingHSV(GrassBaseColorDark, GrassBaseColorLight, ColorNoise);
    }
    else if (VoxelType == EVoxelType::Dirt)
    {
        float DirtNoise = FastValueNoise2D(VoxX * 0.1f, VoxY * 0.1f);
        FLinearColor DirtDark(0.12f, 0.07f, 0.05f, 1.0f);
        FLinearColor DirtLight(0.20f, 0.12f, 0.08f, 1.0f);
        return FLinearColor::LerpUsingHSV(DirtDark, DirtLight, DirtNoise);
    }
    else if (VoxelType == EVoxelType::Stone)
    {
        float StoneNoise = FastValueNoise3D(VoxX * 0.08f, VoxY * 0.08f, VoxZ * 0.08f);
        FLinearColor StoneDark(0.18f, 0.20f, 0.22f, 1.0f);
        FLinearColor StoneLight(0.30f, 0.32f, 0.34f, 1.0f);
        return FLinearColor::LerpUsingHSV(StoneDark, StoneLight, StoneNoise);
    }
    return FLinearColor::White;
}

void ASmoothVoxelTerrain::AppendVoxelFacesWorld(int32 WorldX, int32 WorldY, int32 WorldZ, FDynamicMesh3& Mesh, FTriIDArray& OutTriIDs, const FHeightCache& HeightCache, const FChunkNeighborhood& Neighborhood)
{
    FDynamicMeshAttributeSet* Attr = Mesh.Attributes();
    if (!Attr) return;
    FDynamicMeshUVOverlay* UVOverlay = Attr->GetUVLayer(0);
    FDynamicMeshNormalOverlay* NormalOverlay = Attr->PrimaryNormals();
    FDynamicMeshColorOverlay* ColorOverlay = Attr->PrimaryColors();
    if (!UVOverlay || !NormalOverlay || !ColorOverlay) return;

    if (!Attr->HasMaterialID())
    {
        Attr->EnableMaterialID();
    }
    auto* MaterialIDAttribute = Attr->GetMaterialID();

    FVector v000 = GetSmoothVertexWorld(WorldX, WorldY, WorldZ, WorldX, WorldY, WorldZ, HeightCache, Neighborhood);
    FVector v100 = GetSmoothVertexWorld(WorldX + 1, WorldY, WorldZ, WorldX, WorldY, WorldZ, HeightCache, Neighborhood);
    FVector v010 = GetSmoothVertexWorld(WorldX, WorldY + 1, WorldZ, WorldX, WorldY, WorldZ, HeightCache, Neighborhood);
    FVector v110 = GetSmoothVertexWorld(WorldX + 1, WorldY + 1, WorldZ, WorldX, WorldY, WorldZ, HeightCache, Neighborhood);
    FVector v001 = GetSmoothVertexWorld(WorldX, WorldY, WorldZ + 1, WorldX, WorldY, WorldZ, HeightCache, Neighborhood);
    FVector v101 = GetSmoothVertexWorld(WorldX + 1, WorldY, WorldZ + 1, WorldX, WorldY, WorldZ, HeightCache, Neighborhood);
    FVector v011 = GetSmoothVertexWorld(WorldX, WorldY + 1, WorldZ + 1, WorldX, WorldY, WorldZ, HeightCache, Neighborhood);
    FVector v111 = GetSmoothVertexWorld(WorldX + 1, WorldY + 1, WorldZ + 1, WorldX, WorldY, WorldZ, HeightCache, Neighborhood);

    FVector VoxelOrigin((double)WorldX * CubeSize, (double)WorldY * CubeSize, (double)WorldZ * CubeSize);

    auto GetUVForVertex = [&](const FVector& Pos, const FVector& FaceNormal) -> FVector2D
        {
            FVector LocalPos = (Pos - VoxelOrigin) / CubeSize;
            FVector AbsN = FaceNormal.GetAbs();
            float U, V;
            if (AbsN.Z > 0.9f)
            {
                U = (float)LocalPos.X * TextureScale;
                V = (float)LocalPos.Y * TextureScale;
            }
            else if (AbsN.X > 0.9f)
            {
                U = (float)LocalPos.Y * TextureScale;
                V = (float)LocalPos.Z * TextureScale;
            }
            else
            {
                U = (float)LocalPos.X * TextureScale;
                V = (float)LocalPos.Z * TextureScale;
            }
            return FVector2D(U, V);
        };

    auto ComputeTriangleNormal = [](const FVector& A, const FVector& B, const FVector& C) -> FVector
        {
            return FVector::CrossProduct(C - A, B - A).GetSafeNormal();
        };

    int32 LocalX = WorldX - Neighborhood.Self->Coord.X * ChunkSize;
    int32 LocalY = WorldY - Neighborhood.Self->Coord.Y * ChunkSize;

    EVoxelType CurrentType = Neighborhood.GetVoxel(LocalX, LocalY, WorldZ);
    FLinearColor VoxelColor = GetStylizedColorForVoxel(VoxelOrigin + FVector(0.5 * CubeSize), CurrentType);

    auto AddQuadWorld = [&](const FVector& A, const FVector& B, const FVector& C, const FVector& D,
        const FVector& FaceNormal, int32 MatID)
        {
            FVector2D uvA = GetUVForVertex(A, FaceNormal);
            FVector2D uvB = GetUVForVertex(B, FaceNormal);
            FVector2D uvC = GetUVForVertex(C, FaceNormal);
            FVector2D uvD = GetUVForVertex(D, FaceNormal);

            int32 vA = Mesh.AppendVertex(FVector3d(A));
            int32 vB = Mesh.AppendVertex(FVector3d(B));
            int32 vC = Mesh.AppendVertex(FVector3d(C));
            int32 vD = Mesh.AppendVertex(FVector3d(D));

            int32 cA = ColorOverlay->AppendElement(FVector4f(VoxelColor));
            int32 cB = ColorOverlay->AppendElement(FVector4f(VoxelColor));
            int32 cC = ColorOverlay->AppendElement(FVector4f(VoxelColor));
            int32 cD = ColorOverlay->AppendElement(FVector4f(VoxelColor));

            FVector n1 = ComputeTriangleNormal(A, B, C);
            int32 t1 = Mesh.AppendTriangle(vA, vB, vC);
            if (t1 != FDynamicMesh3::InvalidID)
            {
                OutTriIDs.Add(t1);
                int32 nA1 = NormalOverlay->AppendElement(FVector3f(n1));
                int32 nB1 = NormalOverlay->AppendElement(FVector3f(n1));
                int32 nC1 = NormalOverlay->AppendElement(FVector3f(n1));
                NormalOverlay->SetTriangle(t1, FIndex3i(nA1, nB1, nC1));

                int32 uvA1 = UVOverlay->AppendElement(FVector2f(uvA));
                int32 uvB1 = UVOverlay->AppendElement(FVector2f(uvB));
                int32 uvC1 = UVOverlay->AppendElement(FVector2f(uvC));
                UVOverlay->SetTriangle(t1, FIndex3i(uvA1, uvB1, uvC1));

                ColorOverlay->SetTriangle(t1, FIndex3i(cA, cB, cC));

                if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t1, MatID);
            }

            FVector n2 = ComputeTriangleNormal(A, C, D);
            int32 t2 = Mesh.AppendTriangle(vA, vC, vD);
            if (t2 != FDynamicMesh3::InvalidID)
            {
                OutTriIDs.Add(t2);
                int32 nA2 = NormalOverlay->AppendElement(FVector3f(n2));
                int32 nC2 = NormalOverlay->AppendElement(FVector3f(n2));
                int32 nD2 = NormalOverlay->AppendElement(FVector3f(n2));
                NormalOverlay->SetTriangle(t2, FIndex3i(nA2, nC2, nD2));

                int32 uvA2 = UVOverlay->AppendElement(FVector2f(uvA));
                int32 uvC2 = UVOverlay->AppendElement(FVector2f(uvC));
                int32 uvD2 = UVOverlay->AppendElement(FVector2f(uvD));
                UVOverlay->SetTriangle(t2, FIndex3i(uvA2, uvC2, uvD2));

                ColorOverlay->SetTriangle(t2, FIndex3i(cA, cC, cD));

                if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t2, MatID);
            }
        };

    bool bExposedTop = Neighborhood.GetVoxel(LocalX, LocalY, WorldZ + 1) == EVoxelType::Air;

    int32 TopMatID = 1;
    int32 BottomMatID = 1;
    int32 SideMatID = 1;

    if (CurrentType == EVoxelType::Grass)
    {
        TopMatID = 0;
        BottomMatID = 1;
        SideMatID = 1;
    }
    else if (CurrentType == EVoxelType::Dirt)
    {
        TopMatID = BottomMatID = SideMatID = 1;
    }
    else if (CurrentType == EVoxelType::Stone)
    {
        TopMatID = BottomMatID = SideMatID = 2;
    }

    if (bExposedTop)
    {
        if (bSmoothTerrain)
        {
            FVector n00 = GetSmoothNormalWorld(WorldX, WorldY, HeightCache);
            FVector n10 = GetSmoothNormalWorld(WorldX + 1, WorldY, HeightCache);
            FVector n01 = GetSmoothNormalWorld(WorldX, WorldY + 1, HeightCache);
            FVector n11 = GetSmoothNormalWorld(WorldX + 1, WorldY + 1, HeightCache);

            auto AddTopQuadSmooth = [&](const FVector& A, const FVector& B, const FVector& C, const FVector& D,
                const FVector& nA, const FVector& nB, const FVector& nC, const FVector& nD, int32 MatID)
                {
                    FVector2D uvA = GetUVForVertex(A, FVector(0.f, 0.f, 1.f));
                    FVector2D uvB = GetUVForVertex(B, FVector(0.f, 0.f, 1.f));
                    FVector2D uvC = GetUVForVertex(C, FVector(0.f, 0.f, 1.f));
                    FVector2D uvD = GetUVForVertex(D, FVector(0.f, 0.f, 1.f));

                    int32 vA = Mesh.AppendVertex(FVector3d(A));
                    int32 vB = Mesh.AppendVertex(FVector3d(B));
                    int32 vC = Mesh.AppendVertex(FVector3d(C));
                    int32 vD = Mesh.AppendVertex(FVector3d(D));

                    int32 cA = ColorOverlay->AppendElement(FVector4f(VoxelColor));
                    int32 cB = ColorOverlay->AppendElement(FVector4f(VoxelColor));
                    int32 cC = ColorOverlay->AppendElement(FVector4f(VoxelColor));
                    int32 cD = ColorOverlay->AppendElement(FVector4f(VoxelColor));

                    int32 t1 = Mesh.AppendTriangle(vA, vB, vC);
                    if (t1 != FDynamicMesh3::InvalidID)
                    {
                        OutTriIDs.Add(t1);
                        int32 nA1 = NormalOverlay->AppendElement(FVector3f(nA));
                        int32 nB1 = NormalOverlay->AppendElement(FVector3f(nB));
                        int32 nC1 = NormalOverlay->AppendElement(FVector3f(nC));
                        NormalOverlay->SetTriangle(t1, FIndex3i(nA1, nB1, nC1));
                        int32 uvA1 = UVOverlay->AppendElement(FVector2f(uvA));
                        int32 uvB1 = UVOverlay->AppendElement(FVector2f(uvB));
                        int32 uvC1 = UVOverlay->AppendElement(FVector2f(uvC));
                        UVOverlay->SetTriangle(t1, FIndex3i(uvA1, uvB1, uvC1));

                        ColorOverlay->SetTriangle(t1, FIndex3i(cA, cB, cC));
                        if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t1, MatID);
                    }
                    int32 t2 = Mesh.AppendTriangle(vA, vC, vD);
                    if (t2 != FDynamicMesh3::InvalidID)
                    {
                        OutTriIDs.Add(t2);
                        int32 nA2 = NormalOverlay->AppendElement(FVector3f(nA));
                        int32 nC2 = NormalOverlay->AppendElement(FVector3f(nA));
                        int32 nD2 = NormalOverlay->AppendElement(FVector3f(nD));
                        NormalOverlay->SetTriangle(t2, FIndex3i(nA2, nC2, nD2));
                        int32 uvA2 = UVOverlay->AppendElement(FVector2f(uvA));
                        int32 uvC2 = UVOverlay->AppendElement(FVector2f(uvC));
                        int32 uvD2 = UVOverlay->AppendElement(FVector2f(uvD));
                        UVOverlay->SetTriangle(t2, FIndex3i(uvA2, uvC2, uvD2));

                        ColorOverlay->SetTriangle(t2, FIndex3i(cA, cC, cD));
                        if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t2, MatID);
                    }
                };
            AddTopQuadSmooth(v001, v011, v111, v101, n00, n01, n11, n10, TopMatID);
        }
        else
        {
            AddQuadWorld(v001, v011, v111, v101, FVector(0.f, 0.f, 1.f), TopMatID);
        }
    }

    if (Neighborhood.GetVoxel(LocalX, LocalY, WorldZ - 1) == EVoxelType::Air)
        AddQuadWorld(v100, v110, v010, v000, FVector(0.f, 0.f, -1.f), BottomMatID);

    if (Neighborhood.GetVoxel(LocalX + 1, LocalY, WorldZ) == EVoxelType::Air ||
        GetNeighborTopHeightWorld(WorldX + 1, WorldY, WorldZ, v100, Neighborhood, HeightCache) < v100.Z ||
        GetNeighborTopHeightWorld(WorldX + 1, WorldY, WorldZ, v101, Neighborhood, HeightCache) < v101.Z ||
        GetNeighborTopHeightWorld(WorldX + 1, WorldY, WorldZ, v111, Neighborhood, HeightCache) < v111.Z ||
        GetNeighborTopHeightWorld(WorldX + 1, WorldY, WorldZ, v110, Neighborhood, HeightCache) < v110.Z)
    {
        AddQuadWorld(v100, v101, v111, v110, FVector(1.f, 0.f, 0.f), SideMatID);
    }

    if (Neighborhood.GetVoxel(LocalX - 1, LocalY, WorldZ) == EVoxelType::Air ||
        GetNeighborTopHeightWorld(WorldX - 1, WorldY, WorldZ, v010, Neighborhood, HeightCache) < v010.Z ||
        GetNeighborTopHeightWorld(WorldX - 1, WorldY, WorldZ, v011, Neighborhood, HeightCache) < v011.Z ||
        GetNeighborTopHeightWorld(WorldX - 1, WorldY, WorldZ, v001, Neighborhood, HeightCache) < v001.Z ||
        GetNeighborTopHeightWorld(WorldX - 1, WorldY, WorldZ, v000, Neighborhood, HeightCache) < v000.Z)
    {
        AddQuadWorld(v010, v011, v001, v000, FVector(-1.f, 0.f, 0.f), SideMatID);
    }

    if (Neighborhood.GetVoxel(LocalX, LocalY + 1, WorldZ) == EVoxelType::Air ||
        GetNeighborTopHeightWorld(WorldX, WorldY + 1, WorldZ, v110, Neighborhood, HeightCache) < v110.Z ||
        GetNeighborTopHeightWorld(WorldX, WorldY + 1, WorldZ, v111, Neighborhood, HeightCache) < v111.Z ||
        GetNeighborTopHeightWorld(WorldX, WorldY + 1, WorldZ, v011, Neighborhood, HeightCache) < v011.Z ||
        GetNeighborTopHeightWorld(WorldX, WorldY + 1, WorldZ, v010, Neighborhood, HeightCache) < v010.Z)
    {
        AddQuadWorld(v110, v111, v011, v010, FVector(0.f, 1.f, 0.f), SideMatID);
    }

    if (Neighborhood.GetVoxel(LocalX, LocalY - 1, WorldZ) == EVoxelType::Air ||
        GetNeighborTopHeightWorld(WorldX, WorldY - 1, WorldZ, v000, Neighborhood, HeightCache) < v000.Z ||
        GetNeighborTopHeightWorld(WorldX, WorldY - 1, WorldZ, v001, Neighborhood, HeightCache) < v001.Z ||
        GetNeighborTopHeightWorld(WorldX, WorldY - 1, WorldZ, v101, Neighborhood, HeightCache) < v101.Z ||
        GetNeighborTopHeightWorld(WorldX, WorldY - 1, WorldZ, v100, Neighborhood, HeightCache) < v100.Z)
    {
        AddQuadWorld(v000, v001, v101, v100, FVector(0.f, -1.f, 0.f), SideMatID);
    }
}

void ASmoothVoxelTerrain::AppendGrassBladesWorld(int32 WorldX, int32 WorldY, int32 WorldZ, FDynamicMesh3& Mesh, FTriIDArray& OutTriIDs, const FHeightCache& HeightCache, const FChunkNeighborhood& Neighborhood)
{
    FDynamicMeshAttributeSet* Attr = Mesh.Attributes();
    if (!Attr) return;

    FDynamicMeshUVOverlay* UVOverlay0 = Attr->GetUVLayer(0);
    if (!UVOverlay0) return;

    if (Attr->NumUVLayers() < 2)
    {
        Attr->SetNumUVLayers(2);
    }
    FDynamicMeshUVOverlay* UVOverlay1 = Attr->GetUVLayer(1);

    if (!Attr->PrimaryColors())
    {
        Attr->EnablePrimaryColors();
    }
    FDynamicMeshColorOverlay* ColorOverlay = Attr->PrimaryColors();

    float DensityNoise = FastValueNoise2D((float)WorldX * GrassDensityNoiseScale, (float)WorldY * GrassDensityNoiseScale);

    float FineNoise = Hash3D(WorldX, WorldY, 999);
    if (FineNoise < 0.20f)
    {
        DensityNoise *= 0.1f;
    }
    else if (FineNoise > 0.85f)
    {
        DensityNoise = FMath::Min(1.0f, DensityNoise * 1.5f);
    }

    float DensityRand = Hash3D(WorldX, WorldY, 888);
    float TargetDensity = FMath::Lerp((float)GrassMinDensity, (float)GrassMaxDensity, DensityNoise);
    TargetDensity += (DensityRand - 0.5f) * 3.0f;
    int32 Density = FMath::Clamp(FMath::RoundToInt(TargetDensity), 0, GrassMaxDensity + 2);

    float ColorNoise = FastValueNoise2D((float)WorldX * GrassColorNoiseScale, (float)WorldY * GrassColorNoiseScale);

    FLinearColor BaseColor = FLinearColor::LerpUsingHSV(GrassBaseColorDark, GrassBaseColorLight, ColorNoise);
    FLinearColor TipColor = FLinearColor::LerpUsingHSV(GrassTipColorDark, GrassTipColorLight, ColorNoise);

    const float LocalGrassMinHeight = GrassMinHeight;
    const float LocalGrassMaxHeight = GrassMaxHeight;
    const float LocalGrassMinWidth = GrassMinWidth;
    const float LocalGrassMaxWidth = GrassMaxWidth;
    const float LocalCubeSize = CubeSize;
    const bool bLocalSmoothTerrain = bSmoothTerrain;

    for (int32 i = 0; i < Density; ++i)
    {
        uint32 BaseSeed = (uint32)WorldX * 73856093U ^ (uint32)WorldY * 19349663U ^ (uint32)i * 83492791U;
        FFastRandom FastRand(BaseSeed);

        float RandX = FastRand.NextFloat();
        float RandY = FastRand.NextFloat();
        float RandHeight = FastRand.NextFloat();
        float RandWidth = FastRand.NextFloat();
        float RandAngle = FastRand.NextFloat();
        float RandTint = FastRand.NextFloat();
        float RandLeanAngle = FastRand.NextFloat();
        float RandLeanStrength = FastRand.NextFloat();
        float RandBendAngle = FastRand.NextFloat();
        float RandBendForce = FastRand.NextFloat();

        float dx = RandX;
        float dy = RandY;

        float BladeWorldX = (float)WorldX + dx;
        float BladeWorldY = (float)WorldY + dy;

        float BladeWorldZ = 0.0f;
        FVector GroundNormal(0.f, 0.f, 1.f);

        if (bLocalSmoothTerrain)
        {
            BladeWorldZ = GetInterpolatedHeightCached(BladeWorldX, BladeWorldY, HeightCache) * LocalCubeSize;
            GroundNormal = GetSmoothNormalWorld(FMath::RoundToInt(BladeWorldX), FMath::RoundToInt(BladeWorldY), HeightCache);
        }
        else
        {
            BladeWorldZ = (float)(WorldZ + 1) * LocalCubeSize;
        }

        FVector BasePos((double)BladeWorldX * LocalCubeSize, (double)BladeWorldY * LocalCubeSize, (double)BladeWorldZ);

        float Height = LocalGrassMinHeight + (LocalGrassMaxHeight - LocalGrassMinHeight) * RandHeight;
        float Width = LocalGrassMinWidth + (LocalGrassMaxWidth - LocalGrassMinWidth) * RandWidth;

        float Angle = RandAngle * 2.0f * PI;
        float SinAngle, CosAngle;
        FMath::SinCos(&SinAngle, &CosAngle, Angle);
        FVector BladeRight(CosAngle, SinAngle, 0.0f);
        FVector BladeForward(-SinAngle, CosAngle, 0.0f);

        float BladeTint = (RandTint * 0.2f) - 0.1f;
        FLinearColor CustomBaseColor = FLinearColor::LerpUsingHSV(BaseColor, FLinearColor::Black, FMath::Max(0.0f, -BladeTint));
        CustomBaseColor = FLinearColor::LerpUsingHSV(CustomBaseColor, FLinearColor::White, FMath::Max(0.0f, BladeTint));
        FLinearColor CustomTipColor = FLinearColor::LerpUsingHSV(TipColor, FLinearColor::Black, FMath::Max(0.0f, -BladeTint));
        CustomTipColor = FLinearColor::LerpUsingHSV(CustomTipColor, FLinearColor::White, FMath::Max(0.0f, BladeTint));
        FLinearColor MidColor = FLinearColor::LerpUsingHSV(CustomBaseColor, CustomTipColor, 0.5f);

        float LeanAngle = RandLeanAngle * 2.0f * PI;
        float SinLean, CosLean;
        FMath::SinCos(&SinLean, &CosLean, LeanAngle);
        float LeanStrength = 0.05f + 0.15f * RandLeanStrength;
        FVector LeanDir(CosLean, SinLean, 0.0f);
        FVector TiltingNormal = (GroundNormal + LeanDir * LeanStrength).GetSafeNormal();

        float BendAngle = RandBendAngle * 2.0f * PI;
        float SinBend, CosBend;
        FMath::SinCos(&SinBend, &CosBend, BendAngle);
        FVector RandomBendDir(CosBend, SinBend, 0.0f);
        FVector BendDir = (RandomBendDir * 0.5f + BladeForward * 0.3f + GroundNormal * 0.2f).GetSafeNormal();

        float BendForce = (0.15f + 0.35f * RandBendForce) * Height;

        FDynamicMeshNormalOverlay* NormalOverlay = Attr->PrimaryNormals();
        int32 nGround = NormalOverlay ? NormalOverlay->AppendElement(FVector3f(GroundNormal)) : -1;

        auto AddTri = [UVOverlay0, UVOverlay1, ColorOverlay, NormalOverlay, nGround, &Mesh, &OutTriIDs, this](
            int32 a, int32 b, int32 c,
            int32 u0_A, int32 u0_B, int32 u0_C,
            int32 u1_A, int32 u1_B, int32 u1_C,
            int32 cA, int32 cB, int32 cC)
            {
                int32 t = Mesh.AppendTriangle(a, b, c);
                if (t != FDynamicMesh3::InvalidID)
                {
                    OutTriIDs.Add(t);
                    if (NormalOverlay && nGround != -1 && Mesh.IsTriangle(t)) NormalOverlay->SetTriangle(t, FIndex3i(nGround, nGround, nGround));
                    if (UVOverlay0 && u0_A != -1 && u0_B != -1 && u0_C != -1 && Mesh.IsTriangle(t)) UVOverlay0->SetTriangle(t, FIndex3i(u0_A, u0_B, u0_C));
                    if (UVOverlay1 && u1_A != -1 && u1_B != -1 && u1_C != -1 && Mesh.IsTriangle(t)) UVOverlay1->SetTriangle(t, FIndex3i(u1_A, u1_B, u1_C));
                    if (ColorOverlay && cA != -1 && cB != -1 && cC != -1 && Mesh.IsTriangle(t)) ColorOverlay->SetTriangle(t, FIndex3i(cA, cB, cC));
                }

                if (!bTwoSidedGrass)
                {
                    int32 tBack = Mesh.AppendTriangle(a, c, b);
                    if (tBack != FDynamicMesh3::InvalidID)
                    {
                        OutTriIDs.Add(tBack);
                        if (NormalOverlay && nGround != -1 && Mesh.IsTriangle(tBack)) NormalOverlay->SetTriangle(tBack, FIndex3i(nGround, nGround, nGround));
                        if (UVOverlay0 && u0_A != -1 && u0_B != -1 && u0_C != -1 && Mesh.IsTriangle(tBack)) UVOverlay0->SetTriangle(tBack, FIndex3i(u0_A, u0_C, u0_B));
                        if (UVOverlay1 && u1_A != -1 && u1_B != -1 && u1_C != -1 && Mesh.IsTriangle(tBack)) UVOverlay1->SetTriangle(tBack, FIndex3i(u1_A, u1_C, u1_B));
                        if (ColorOverlay && cA != -1 && cB != -1 && cC != -1 && Mesh.IsTriangle(tBack)) ColorOverlay->SetTriangle(tBack, FIndex3i(cA, cC, cB));
                    }
                }
            };

        if (GrassBladeSegments <= 1)
        {
            FVector V0 = BasePos - BladeRight * (Width * 0.5f);
            FVector V1 = BasePos + BladeRight * (Width * 0.5f);
            FVector V2 = BasePos + BendDir * BendForce + TiltingNormal * Height;

            int32 v0 = Mesh.AppendVertex(FVector3d(V0));
            int32 v1 = Mesh.AppendVertex(FVector3d(V1));
            int32 v2 = Mesh.AppendVertex(FVector3d(V2));

            int32 uv0_0 = UVOverlay0->AppendElement(FVector2f(0.0f, 0.0f));
            int32 uv0_1 = UVOverlay0->AppendElement(FVector2f(1.0f, 0.0f));
            int32 uv0_2 = UVOverlay0->AppendElement(FVector2f(0.5f, 1.0f));

            int32 uv1_0 = UVOverlay1 ? UVOverlay1->AppendElement(FVector2f(ColorNoise, 0.0f)) : -1;
            int32 uv1_1 = UVOverlay1 ? UVOverlay1->AppendElement(FVector2f(ColorNoise, 0.0f)) : -1;
            int32 uv1_2 = UVOverlay1 ? UVOverlay1->AppendElement(FVector2f(ColorNoise, 1.0f)) : -1;

            int32 c0 = ColorOverlay ? ColorOverlay->AppendElement(FVector4f(CustomBaseColor)) : -1;
            int32 c1 = ColorOverlay ? ColorOverlay->AppendElement(FVector4f(CustomBaseColor)) : -1;
            int32 c2 = ColorOverlay ? ColorOverlay->AppendElement(FVector4f(CustomTipColor)) : -1;

            AddTri(v0, v1, v2, uv0_0, uv0_1, uv0_2, uv1_0, uv1_1, uv1_2, c0, c1, c2);
        }
        else
        {
            FVector V0 = BasePos - BladeRight * (Width * 0.5f);
            FVector V1 = BasePos + BladeRight * (Width * 0.5f);
            FVector V2 = BasePos - BladeRight * (Width * 0.3f) + BendDir * (BendForce * 0.35f) + TiltingNormal * (Height * 0.5f);
            FVector V3 = BasePos + BladeRight * (Width * 0.3f) + BendDir * (BendForce * 0.35f) + TiltingNormal * (Height * 0.5f);
            FVector V4 = BasePos + BendDir * BendForce + TiltingNormal * Height;

            int32 v0 = Mesh.AppendVertex(FVector3d(V0));
            int32 v1 = Mesh.AppendVertex(FVector3d(V1));
            int32 v2 = Mesh.AppendVertex(FVector3d(V2));
            int32 v3 = Mesh.AppendVertex(FVector3d(V3));
            int32 v4 = Mesh.AppendVertex(FVector3d(V4));

            int32 uv0_0 = UVOverlay0->AppendElement(FVector2f(0.0f, 0.0f));
            int32 uv0_1 = UVOverlay0->AppendElement(FVector2f(1.0f, 0.0f));
            int32 uv0_2 = UVOverlay0->AppendElement(FVector2f(0.15f, 0.5f));
            int32 uv0_3 = UVOverlay0->AppendElement(FVector2f(0.85f, 0.5f));
            int32 uv0_4 = UVOverlay0->AppendElement(FVector2f(0.5f, 1.0f));

            int32 uv1_0 = UVOverlay1 ? UVOverlay1->AppendElement(FVector2f(ColorNoise, 0.0f)) : -1;
            int32 uv1_1 = UVOverlay1 ? UVOverlay1->AppendElement(FVector2f(ColorNoise, 0.0f)) : -1;
            int32 uv1_2 = UVOverlay1 ? UVOverlay1->AppendElement(FVector2f(ColorNoise, 0.5f)) : -1;
            int32 uv1_3 = UVOverlay1 ? UVOverlay1->AppendElement(FVector2f(ColorNoise, 0.5f)) : -1;
            int32 uv1_4 = UVOverlay1 ? UVOverlay1->AppendElement(FVector2f(ColorNoise, 1.0f)) : -1;

            int32 c0 = ColorOverlay ? ColorOverlay->AppendElement(FVector4f(CustomBaseColor)) : -1;
            int32 c1 = ColorOverlay ? ColorOverlay->AppendElement(FVector4f(CustomBaseColor)) : -1;
            int32 c2 = ColorOverlay ? ColorOverlay->AppendElement(FVector4f(MidColor)) : -1;
            int32 c3 = ColorOverlay ? ColorOverlay->AppendElement(FVector4f(MidColor)) : -1;
            int32 c4 = ColorOverlay ? ColorOverlay->AppendElement(FVector4f(CustomTipColor)) : -1;

            AddTri(v0, v1, v3, uv0_0, uv0_1, uv0_3, uv1_0, uv1_1, uv1_3, c0, c1, c3);
            AddTri(v0, v3, v2, uv0_0, uv0_3, uv0_2, uv1_0, uv1_3, uv1_2, c0, c3, c2);
            AddTri(v2, v3, v4, uv0_2, uv0_3, uv0_4, uv1_2, uv1_3, uv1_4, c2, c3, c4);
        }
    }
}

void ASmoothVoxelTerrain::FVoxelChunk::UpdateVoxelMesh(int32 LocalX, int32 LocalY, int32 LocalZ, EVoxelType NewType, ASmoothVoxelTerrain* TerrainOwner)
{
    if (!MeshComponent || !GrassMeshComponent) return;

    UDynamicMesh* DynamicMesh = MeshComponent->GetDynamicMesh();
    UDynamicMesh* GrassDynamicMesh = GrassMeshComponent->GetDynamicMesh();
    if (!DynamicMesh || !GrassDynamicMesh) return;

    DynamicMesh->EditMesh([&](FDynamicMesh3& MeshOut)
        {
            GrassDynamicMesh->EditMesh([&](FDynamicMesh3& GrassMeshOut)
                {
                    FDynamicMeshAttributeSet* Attr = MeshOut.Attributes();
                    FDynamicMeshAttributeSet* GrassAttr = GrassMeshOut.Attributes();
                    if (!Attr || !GrassAttr) return;

                    if (Attr->NumUVLayers() < 2) Attr->SetNumUVLayers(2);
                    if (GrassAttr->NumUVLayers() < 2) GrassAttr->SetNumUVLayers(2);
                    if (!Attr->PrimaryColors()) Attr->EnablePrimaryColors();
                    if (!GrassAttr->PrimaryColors()) GrassAttr->EnablePrimaryColors();
                    if (!Attr->HasMaterialID()) Attr->EnableMaterialID();

                    for (int32 dz = -1; dz <= 1; ++dz)
                    {
                        for (int32 dy = -1; dy <= 1; ++dy)
                        {
                            for (int32 dx = -1; dx <= 1; ++dx)
                            {
                                int32 dist = FMath::Abs(dx) + FMath::Abs(dy) + FMath::Abs(dz);
                                if (dist != 0 && dist != 1) continue;

                                int32 nx = LocalX + dx;
                                int32 ny = LocalY + dy;
                                int32 nz = LocalZ + dz;
                                if (nx >= 0 && nx < TerrainOwner->ChunkSize && ny >= 0 && ny < TerrainOwner->ChunkSize && nz >= 0 && nz < TerrainOwner->MaxHeight)
                                {
                                    RemoveVoxelFaces(nx, ny, nz, MeshOut, GrassMeshOut, TerrainOwner);
                                }
                            }
                        }
                    }

                    int32 Index = LocalX + LocalY * TerrainOwner->ChunkSize + LocalZ * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;
                    VoxelData[Index] = NewType;

                    for (int32 dz = -1; dz <= 1; ++dz)
                    {
                        for (int32 dy = -1; dy <= 1; ++dy)
                        {
                            for (int32 dx = -1; dx <= 1; ++dx)
                            {
                                int32 dist = FMath::Abs(dx) + FMath::Abs(dy) + FMath::Abs(dz);
                                if (dist != 0 && dist != 1) continue;

                                int32 nx = LocalX + dx;
                                int32 ny = LocalY + dy;
                                int32 nz = LocalZ + dz;
                                if (nx >= 0 && nx < TerrainOwner->ChunkSize && ny >= 0 && ny < TerrainOwner->ChunkSize && nz >= 0 && nz < TerrainOwner->MaxHeight)
                                {
                                    int32 neighborIndex = nx + ny * TerrainOwner->ChunkSize + nz * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;
                                    if (VoxelData[neighborIndex] != EVoxelType::Air)
                                    {
                                        AddVoxelFaces(nx, ny, nz, MeshOut, GrassMeshOut, TerrainOwner);
                                    }
                                }
                            }
                        }
                    }
                });
        });
}

void ASmoothVoxelTerrain::FVoxelChunk::RemoveVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, FDynamicMesh3& Mesh, FDynamicMesh3& GrassMesh, ASmoothVoxelTerrain* TerrainOwner)
{
    int32 VoxelIndex = LocalX + LocalY * TerrainOwner->ChunkSize + LocalZ * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;

    if (auto* TriIDsPtr = VoxelTriangles.Find(VoxelIndex))
    {
        for (int32 TriID : *TriIDsPtr)
        {
            if (Mesh.IsTriangle(TriID)) Mesh.RemoveTriangle(TriID, false);
        }
        VoxelTriangles.Remove(VoxelIndex);
    }

    if (auto* GrassTriIDsPtr = GrassVoxelTriangles.Find(VoxelIndex))
    {
        for (int32 TriID : *GrassTriIDsPtr)
        {
            if (GrassMesh.IsTriangle(TriID)) GrassMesh.RemoveTriangle(TriID, false);
        }
        GrassVoxelTriangles.Remove(VoxelIndex);
    }
}

void ASmoothVoxelTerrain::FVoxelChunk::AddVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, FDynamicMesh3& Mesh, FDynamicMesh3& GrassMesh, ASmoothVoxelTerrain* TerrainOwner)
{
    int32 WorldX = Coord.X * TerrainOwner->ChunkSize + LocalX;
    int32 WorldY = Coord.Y * TerrainOwner->ChunkSize + LocalY;
    int32 WorldZ = LocalZ;
    int32 VoxelIndex = LocalX + LocalY * TerrainOwner->ChunkSize + LocalZ * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;

    FChunkNeighborhood Neighborhood;
    Neighborhood.Self = this;
    Neighborhood.SelfData = VoxelData.GetData();
    Neighborhood.ChunkSize = TerrainOwner->ChunkSize;
    Neighborhood.MaxHeight = TerrainOwner->MaxHeight;
    Neighborhood.StepY = TerrainOwner->ChunkSize;
    Neighborhood.StepZ = TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;

    auto RetrieveVoxelDataPtr = [&](const FIntVector& Offset) -> const EVoxelType* {
        if (const FVoxelChunk* Target = TerrainOwner->GetChunk(Coord + Offset)) return Target->VoxelData.GetData();
        return nullptr;
        };

    Neighborhood.West = TerrainOwner->GetChunk(Coord + FIntVector(-1, 0, 0));
    Neighborhood.East = TerrainOwner->GetChunk(Coord + FIntVector(1, 0, 0));
    Neighborhood.South = TerrainOwner->GetChunk(Coord + FIntVector(0, -1, 0));
    Neighborhood.North = TerrainOwner->GetChunk(Coord + FIntVector(0, 1, 0));

    Neighborhood.WestData = RetrieveVoxelDataPtr(FIntVector(-1, 0, 0));
    Neighborhood.EastData = RetrieveVoxelDataPtr(FIntVector(1, 0, 0));
    Neighborhood.SouthData = RetrieveVoxelDataPtr(FIntVector(0, -1, 0));
    Neighborhood.NorthData = RetrieveVoxelDataPtr(FIntVector(0, 1, 0));

    FHeightCache HeightCache;
    HeightCache.InitForVoxel(WorldX, WorldY, TerrainOwner);

    FTriIDArray NewTriIDs;
    TerrainOwner->AppendVoxelFacesWorld(WorldX, WorldY, WorldZ, Mesh, NewTriIDs, HeightCache, Neighborhood);
    if (NewTriIDs.Num() > 0)
    {
        VoxelTriangles.Add(VoxelIndex, NewTriIDs);
    }

    FTriIDArray NewGrassTriIDs;
    if (TerrainOwner->bEnableGrassGeometry && VoxelData[VoxelIndex] == EVoxelType::Grass)
    {
        if (Neighborhood.GetVoxel(LocalX, LocalY, LocalZ + 1) == EVoxelType::Air)
        {
            TerrainOwner->AppendGrassBladesWorld(WorldX, WorldY, WorldZ, GrassMesh, NewGrassTriIDs, HeightCache, Neighborhood);
        }
    }
    if (NewGrassTriIDs.Num() > 0)
    {
        GrassVoxelTriangles.Add(VoxelIndex, NewGrassTriIDs);
    }
}

ASmoothVoxelTerrain::FVoxelChunk* ASmoothVoxelTerrain::GetChunk(const FIntVector& Coord)
{
    if (auto* Ptr = Chunks.Find(Coord))
    {
        return Ptr->IsValid() ? Ptr->Get() : nullptr;
    }
    return nullptr;
}

const ASmoothVoxelTerrain::FVoxelChunk* ASmoothVoxelTerrain::GetChunk(const FIntVector& Coord) const
{
    if (auto* Ptr = Chunks.Find(Coord))
    {
        return Ptr->IsValid() ? Ptr->Get() : nullptr;
    }
    return nullptr;
}

#if WITH_EDITOR
void ASmoothVoxelTerrain::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);

    static const TArray<FName> RelevantProperties = {
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, CollisionEnabled),
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, CollisionProfileName),
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, bGenerateOverlapEvents),
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, bCastShadow),
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, bReceivesDecals),
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, GrassMaterial),
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, DirtMaterial),
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, StoneMaterial),
        GET_MEMBER_NAME_CHECKED(ASmoothVoxelTerrain, GrassBladesMaterial)
    };

    if (RelevantProperties.Contains(PropertyChangedEvent.GetPropertyName()))
    {
        for (auto& Pair : Chunks)
        {
            if (Pair.Value && Pair.Value->MeshComponent)
            {
                Pair.Value->MeshComponent->SetCollisionEnabled(CollisionEnabled);
                Pair.Value->MeshComponent->SetCollisionProfileName(CollisionProfileName);
                Pair.Value->MeshComponent->SetGenerateOverlapEvents(bGenerateOverlapEvents);
                Pair.Value->MeshComponent->SetCastShadow(bCastShadow);
                Pair.Value->MeshComponent->SetReceivesDecals(bReceivesDecals);
                if (GrassMaterial) Pair.Value->MeshComponent->SetMaterial(0, GrassMaterial);
                if (DirtMaterial) Pair.Value->MeshComponent->SetMaterial(1, DirtMaterial);
                if (StoneMaterial) Pair.Value->MeshComponent->SetMaterial(2, StoneMaterial);
            }
            if (Pair.Value && Pair.Value->GrassMeshComponent)
            {
                Pair.Value->GrassMeshComponent->SetReceivesDecals(bReceivesDecals);
                if (GrassBladesMaterial) Pair.Value->GrassMeshComponent->SetMaterial(0, GrassBladesMaterial);
            }
        }
    }
}
#endif