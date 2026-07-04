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
#include "Async/Async.h" 

using namespace UE::Geometry;

static int32 FloorDiv(int32 Dividend, int32 Divisor)
{
    int32 Quotient = Dividend / Divisor;
    if ((Dividend ^ Divisor) < 0 && Dividend % Divisor != 0) Quotient--;
    return Quotient;
}

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
    float a = Hash2D(ix, iy), b = Hash2D(ix + 1, iy), c = Hash2D(ix, iy + 1), d = Hash2D(ix + 1, iy + 1);
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
    int32 ix = FMath::FloorToInt(x), iy = FMath::FloorToInt(y), iz = FMath::FloorToInt(z);
    float fx = x - ix, fy = y - iy, fz = z - iz;
    float ux = fx * fx * (3.0f - 2.0f * fx), uy = fy * fy * (3.0f - 2.0f * fy), uz = fz * fz * (3.0f - 2.0f * fz);
    float c000 = Hash3D(ix, iy, iz), c100 = Hash3D(ix + 1, iy, iz), c010 = Hash3D(ix, iy + 1, iz), c110 = Hash3D(ix + 1, iy + 1, iz);
    float c001 = Hash3D(ix, iy, iz + 1), c101 = Hash3D(ix + 1, iy, iz + 1), c011 = Hash3D(ix, iy + 1, iz + 1), c111 = Hash3D(ix + 1, iy + 1, iz + 1);
    return FMath::Lerp(FMath::Lerp(FMath::Lerp(c000, c100, ux), FMath::Lerp(c010, c110, ux), uy), FMath::Lerp(FMath::Lerp(c001, c101, ux), FMath::Lerp(c011, c111, ux), uy), uz);
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
    const ASmoothVoxelTerrain::FVoxelChunk* Self = nullptr;
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
            return SelfData[LocalX + LocalY * StepY + LocalZ * StepZ];

        const EVoxelType* TargetData = SelfData;
        int32 LX = LocalX, LY = LocalY;

        if (LX < 0) { TargetData = WestData; LX += ChunkSize; }
        else if (LX >= ChunkSize) { TargetData = EastData; LX -= ChunkSize; }

        if (LY < 0) { TargetData = SouthData; LY += ChunkSize; }
        else if (LY >= ChunkSize) { TargetData = NorthData; LY -= ChunkSize; }

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
            if (Pair.Value && Pair.Value->MeshComponent) Pair.Value->MeshComponent->UpdateCollision(false);
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
            if (Pair.Value->MeshComponent && IsValid(Pair.Value->MeshComponent)) Pair.Value->MeshComponent->DestroyComponent();
            if (Pair.Value->GrassMeshComponent && IsValid(Pair.Value->GrassMeshComponent)) Pair.Value->GrassMeshComponent->DestroyComponent();
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
            if (Pair.Value->MeshComponent) { Pair.Value->MeshComponent->UnregisterComponent(); Pair.Value->MeshComponent->DestroyComponent(); }
            if (Pair.Value->GrassMeshComponent) { Pair.Value->GrassMeshComponent->UnregisterComponent(); Pair.Value->GrassMeshComponent->DestroyComponent(); }
        }
    }
    Chunks.Empty();
    GenerationQueue.Empty();

    TArray<UDynamicMeshComponent*> OldComps;
    GetComponents<UDynamicMeshComponent>(OldComps);
    for (UDynamicMeshComponent* Comp : OldComps) { Comp->UnregisterComponent(); Comp->DestroyComponent(); }

    FVector PlayerPos = GetActorLocation();
    if (GetWorld())
    {
        if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
            if (APawn* Pawn = PC->GetPawnOrSpectator()) PlayerPos = Pawn->GetActorLocation();
    }

    FIntVector PlayerChunk = WorldToChunkCoord(PlayerPos);
    LastPlayerChunkCoord = PlayerChunk;

    const int32 SafeSpawnRadius = 1;
    for (int32 dx = -SafeSpawnRadius; dx <= SafeSpawnRadius; ++dx)
        for (int32 dy = -SafeSpawnRadius; dy <= SafeSpawnRadius; ++dy)
            GenerateSingleChunk(FIntVector(PlayerChunk.X + dx, PlayerChunk.Y + dy, 0));

    TSet<FIntVector> TargetCoords;
    for (int32 dx = -RenderDistance; dx <= RenderDistance; ++dx)
        for (int32 dy = -RenderDistance; dy <= RenderDistance; ++dy)
            if (dx * dx + dy * dy <= RenderDistance * RenderDistance) TargetCoords.Add(FIntVector(PlayerChunk.X + dx, PlayerChunk.Y + dy, 0));

    for (const FIntVector& Coord : TargetCoords)
        if (!Chunks.Contains(Coord)) GenerationQueue.Add(Coord);

    GenerationQueue.Sort([PlayerChunk](const FIntVector& A, const FIntVector& B) { return FMath::Square(A.X - PlayerChunk.X) + FMath::Square(A.Y - PlayerChunk.Y) < FMath::Square(B.X - PlayerChunk.X) + FMath::Square(B.Y - PlayerChunk.Y); });
}

void ASmoothVoxelTerrain::UpdateProceduralTerrain()
{
    FVector PlayerPos = GetActorLocation();
    if (GetWorld())
    {
        if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
            if (APawn* Pawn = PC->GetPawnOrSpectator()) PlayerPos = Pawn->GetActorLocation();
    }

    FIntVector PlayerChunk = WorldToChunkCoord(PlayerPos);
    int32 GrassDistSq = FMath::Square(GrassRenderDistance);

    for (auto& Pair : Chunks)
    {
        if (Pair.Value && Pair.Value->GrassMeshComponent)
        {
            int32 DistSq = FMath::Square(Pair.Key.X - PlayerChunk.X) + FMath::Square(Pair.Key.Y - PlayerChunk.Y);
            bool bShouldBeVisible = bEnableGrassGeometry && (DistSq <= GrassDistSq);
            if (Pair.Value->GrassMeshComponent->IsVisible() != bShouldBeVisible) Pair.Value->GrassMeshComponent->SetVisibility(bShouldBeVisible);
        }
    }

    if (PlayerChunk != LastPlayerChunkCoord)
    {
        LastPlayerChunkCoord = PlayerChunk;
        TSet<FIntVector> DesiredCoords;
        for (int32 dx = -RenderDistance; dx <= RenderDistance; ++dx)
            for (int32 dy = -RenderDistance; dy <= RenderDistance; ++dy)
                if (dx * dx + dy * dy <= RenderDistance * RenderDistance) DesiredCoords.Add(FIntVector(PlayerChunk.X + dx, PlayerChunk.Y + dy, 0));

        TArray<FIntVector> CoordsToUnload;
        for (const auto& Pair : Chunks)
            if (FMath::Square(Pair.Key.X - PlayerChunk.X) + FMath::Square(Pair.Key.Y - PlayerChunk.Y) > FMath::Square(UnloadDistance)) CoordsToUnload.Add(Pair.Key);

        for (const FIntVector& Coord : CoordsToUnload) UnloadChunk(Coord);

        GenerationQueue.Empty();
        for (const FIntVector& Coord : DesiredCoords)
            if (!Chunks.Contains(Coord)) GenerationQueue.Add(Coord);

        GenerationQueue.Sort([PlayerChunk](const FIntVector& A, const FIntVector& B) { return FMath::Square(A.X - PlayerChunk.X) + FMath::Square(A.Y - PlayerChunk.Y) < FMath::Square(B.X - PlayerChunk.X) + FMath::Square(B.Y - PlayerChunk.Y); });
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
    MeshComp->bDeferCollisionUpdates = true;

    if (GrassMaterial) MeshComp->SetMaterial(0, GrassMaterial);
    if (DirtMaterial) MeshComp->SetMaterial(1, DirtMaterial);
    if (StoneMaterial) MeshComp->SetMaterial(2, StoneMaterial);
    Chunk->MeshComponent = MeshComp;

    UDynamicMeshComponent* GrassMeshComp = NewObject<UDynamicMeshComponent>(this);
    GrassMeshComp->CreationMethod = EComponentCreationMethod::Instance;
    GrassMeshComp->AttachToComponent(RootSceneComponent, FAttachmentTransformRules::KeepRelativeTransform);
    GrassMeshComp->SetRelativeTransform(FTransform());
    GrassMeshComp->RegisterComponent();

    int32 DistSq = FMath::Square(ChunkCoord.X - LastPlayerChunkCoord.X) + FMath::Square(ChunkCoord.Y - LastPlayerChunkCoord.Y);
    GrassMeshComp->SetVisibility(bEnableGrassGeometry && (DistSq <= FMath::Square(GrassRenderDistance)));

    GrassMeshComp->SetCastShadow(false);
    GrassMeshComp->SetReceivesDecals(false);
    GrassMeshComp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    GrassMeshComp->bEnableComplexCollision = false;

    if (GrassBladesMaterial) GrassMeshComp->SetMaterial(0, GrassBladesMaterial);
    Chunk->GrassMeshComponent = GrassMeshComp;

    FVoxelChunk* ChunkPtr = Chunk.Get();
    Chunks.Add(ChunkCoord, MoveTemp(Chunk));

    TWeakObjectPtr<ASmoothVoxelTerrain> WeakThis(this);

    Async(EAsyncExecution::ThreadPool, [WeakThis, ChunkCoord, GameThreadStart]() mutable
        {
            ASmoothVoxelTerrain* Terrain = WeakThis.Get();
            if (!Terrain || Terrain->bIsDestroyed) return;

            int32 LocalChunkSize = Terrain->ChunkSize;
            int32 LocalMaxHeight = Terrain->MaxHeight;
            float LocalMinGrassThickness = Terrain->MinGrassThickness;

            TArray<EVoxelType> LocalVoxelData;
            LocalVoxelData.SetNumZeroed(LocalChunkSize * LocalChunkSize * LocalMaxHeight);

            int32 CacheSize = LocalChunkSize + 3;
            TArray<float> LocalHeightMap;
            LocalHeightMap.SetNumUninitialized(CacheSize * CacheSize);

            for (int32 y = 0; y < CacheSize; ++y)
            {
                int32 WorldY = ChunkCoord.Y * LocalChunkSize - 1 + y;
                for (int32 x = 0; x < CacheSize; ++x)
                {
                    int32 WorldX = ChunkCoord.X * LocalChunkSize - 1 + x;
                    LocalHeightMap[x + y * CacheSize] = Terrain->GetHeightAtWorldCorner(WorldX, WorldY);
                }
            }

            for (int32 lx = 0; lx < LocalChunkSize; ++lx)
            {
                for (int32 ly = 0; ly < LocalChunkSize; ++ly)
                {
                    float h00 = LocalHeightMap[(lx + 1) + (ly + 1) * CacheSize];
                    float h10 = LocalHeightMap[(lx + 2) + (ly + 1) * CacheSize];
                    float h01 = LocalHeightMap[(lx + 1) + (ly + 2) * CacheSize];
                    float h11 = LocalHeightMap[(lx + 2) + (ly + 2) * CacheSize];

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

            AsyncTask(ENamedThreads::GameThread, [WeakThis, ChunkCoord, LocalVoxelData = MoveTemp(LocalVoxelData), LocalHeightMap = MoveTemp(LocalHeightMap)]() mutable
                {
                    ASmoothVoxelTerrain* Terrain = WeakThis.Get();
                    if (!Terrain || Terrain->bIsDestroyed) return;

                    FVoxelChunk* TargetChunk = Terrain->GetChunk(ChunkCoord);
                    if (!TargetChunk) return;

                    TargetChunk->VoxelData = MoveTemp(LocalVoxelData);
                    TargetChunk->HeightMap = MoveTemp(LocalHeightMap);

                    TargetChunk->BuildMesh(Terrain);

                    FIntVector Neighbors[4] = {
                        FIntVector(ChunkCoord.X - 1, ChunkCoord.Y, 0),
                        FIntVector(ChunkCoord.X + 1, ChunkCoord.Y, 0),
                        FIntVector(ChunkCoord.X, ChunkCoord.Y - 1, 0),
                        FIntVector(ChunkCoord.X, ChunkCoord.Y + 1, 0)
                    };

                    for (const FIntVector& NeighborCoord : Neighbors)
                    {
                        if (FVoxelChunk* Neighbor = Terrain->GetChunk(NeighborCoord))
                            Neighbor->BuildMesh(Terrain);
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
            if (Chunk->MeshComponent && IsValid(Chunk->MeshComponent)) { Chunk->MeshComponent->UnregisterComponent(); Chunk->MeshComponent->DestroyComponent(); }
            if (Chunk->GrassMeshComponent && IsValid(Chunk->GrassMeshComponent)) { Chunk->GrassMeshComponent->UnregisterComponent(); Chunk->GrassMeshComponent->DestroyComponent(); }
        }
    }
}

void ASmoothVoxelTerrain::FVoxelChunk::BuildMesh(ASmoothVoxelTerrain* TerrainOwner)
{
    if (!MeshComponent || !GrassMeshComponent) return;

    TArray<EVoxelType> SelfDataCopy = VoxelData;
    TArray<float> HeightMapCopy = HeightMap;
    TArray<EVoxelType> WestData, EastData, SouthData, NorthData;

    auto RetrieveData = [&](const FIntVector& Offset, TArray<EVoxelType>& OutData) {
        if (const FVoxelChunk* Target = TerrainOwner->GetChunk(Coord + Offset)) OutData = Target->VoxelData;
        };
    RetrieveData(FIntVector(-1, 0, 0), WestData);
    RetrieveData(FIntVector(1, 0, 0), EastData);
    RetrieveData(FIntVector(0, -1, 0), SouthData);
    RetrieveData(FIntVector(0, 1, 0), NorthData);

    FIntVector LocalCoord = Coord;
    TWeakObjectPtr<ASmoothVoxelTerrain> WeakTerrain(TerrainOwner);

    Async(EAsyncExecution::ThreadPool, [WeakTerrain, LocalCoord, SelfDataCopy = MoveTemp(SelfDataCopy), HeightMapCopy = MoveTemp(HeightMapCopy),
        WestData = MoveTemp(WestData), EastData = MoveTemp(EastData), SouthData = MoveTemp(SouthData), NorthData = MoveTemp(NorthData)]() mutable
        {
            ASmoothVoxelTerrain* Terrain = WeakTerrain.Get();
            if (!Terrain || Terrain->bIsDestroyed) return;

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

            FLocalHeightGrid HeightGrid;
            HeightGrid.Heights = HeightMapCopy.GetData();
            HeightGrid.CacheSize = Terrain->ChunkSize + 3;

            TMap<int32, FTriIDArray> NewVoxelTriangles;
            TMap<int32, FTriIDArray> NewGrassTriangles;

            FTriIDArray TempTriIDs;

            for (int32 lz = 0; lz < Terrain->MaxHeight; ++lz)
            {
                int32 ZOffset = lz * Neighborhood.StepZ;
                for (int32 ly = 0; ly < Terrain->ChunkSize; ++ly)
                {
                    int32 YOffset = ly * Terrain->ChunkSize;
                    for (int32 lx = 0; lx < Terrain->ChunkSize; ++lx)
                    {
                        int32 Index = lx + YOffset + ZOffset;
                        EVoxelType VType = Neighborhood.SelfData[Index];

                        if (VType == EVoxelType::Air) continue;

                        TempTriIDs.Reset();
                        Terrain->AppendVoxelFacesLocal(lx, ly, lz, LocalMesh, TempTriIDs, HeightGrid, Neighborhood, LocalCoord);
                        if (TempTriIDs.Num() > 0) NewVoxelTriangles.Add(Index, TempTriIDs);

                        if (Terrain->bEnableGrassGeometry && VType == EVoxelType::Grass)
                        {
                            if (Neighborhood.SelfData[Index + Neighborhood.StepZ] == EVoxelType::Air)
                            {
                                TempTriIDs.Reset();
                                Terrain->AppendGrassBladesLocal(lx, ly, lz, LocalGrassMesh, TempTriIDs, HeightGrid, Neighborhood, LocalCoord);
                                if (TempTriIDs.Num() > 0) NewGrassTriangles.Add(Index, TempTriIDs);
                            }
                        }
                    }
                }
            }

            FMeshNormals::QuickComputeVertexNormals(LocalGrassMesh);

            AsyncTask(ENamedThreads::GameThread, [WeakTerrain, LocalCoord, LocalMesh = MoveTemp(LocalMesh), LocalGrassMesh = MoveTemp(LocalGrassMesh), NewVoxelTriangles = MoveTemp(NewVoxelTriangles), NewGrassTriangles = MoveTemp(NewGrassTriangles)]() mutable
                {
                    ASmoothVoxelTerrain* Terrain = WeakTerrain.Get();
                    if (!Terrain || Terrain->bIsDestroyed) return;

                    FVoxelChunk* Chunk = Terrain->GetChunk(LocalCoord);
                    if (!Chunk || !Chunk->MeshComponent || !Chunk->GrassMeshComponent) return;

                    Chunk->VoxelTriangles = MoveTemp(NewVoxelTriangles);
                    Chunk->GrassVoxelTriangles = MoveTemp(NewGrassTriangles);

                    Chunk->MeshComponent->SetMesh(MoveTemp(LocalMesh));
                    Chunk->GrassMeshComponent->SetMesh(MoveTemp(LocalGrassMesh));

                    Chunk->MeshComponent->UpdateCollision(true);
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

    MeshComponent->UpdateCollision(true);
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
    if (OutVoxelZ < 0 || OutVoxelZ >= MaxHeight) return false;
    if (OutType) *OutType = GetVoxelAtWorld(OutVoxelX, OutVoxelY, OutVoxelZ);
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
    if (Chunk->VoxelData[Index] == EVoxelType::Air) return;

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

    int32 lx, ly, lz;
    WorldToLocalVoxel(WorldLocation, ChunkCoord, lx, ly, lz);
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

float ASmoothVoxelTerrain::GetInterpolatedHeightLocal(float LocalX, float LocalY, const FLocalHeightGrid& HeightGrid) const
{
    int32 x0 = FMath::FloorToInt(LocalX);
    int32 y0 = FMath::FloorToInt(LocalY);
    float fx = LocalX - x0;
    float fy = LocalY - y0;
    float h00 = HeightGrid.GetHeight(x0, y0);
    float h10 = HeightGrid.GetHeight(x0 + 1, y0);
    float h01 = HeightGrid.GetHeight(x0, y0 + 1);
    float h11 = HeightGrid.GetHeight(x0 + 1, y0 + 1);
    return FMath::Lerp(FMath::Lerp(h00, h10, fx), FMath::Lerp(h01, h11, fx), fy);
}

FVector ASmoothVoxelTerrain::GetSmoothVertexLocal(int32 VertX, int32 VertY, int32 VertZ, int32 VoxX, int32 VoxY, int32 VoxZ, const FLocalHeightGrid& HeightGrid, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord) const
{
    int32 WorldX = ChunkCoord.X * ChunkSize + VertX;
    int32 WorldY = ChunkCoord.Y * ChunkSize + VertY;

    if (!bSmoothTerrain) return FVector(WorldX, WorldY, VertZ) * CubeSize;

    float TargetH = HeightGrid.GetHeight(VertX, VertY);
    float FinalZ = (float)VertZ;

    if (Neighborhood.GetVoxel(VoxX, VoxY, VoxZ) == EVoxelType::Grass && VertZ > VoxZ)
    {
        if (Neighborhood.GetVoxel(VoxX, VoxY, VoxZ + 1) != EVoxelType::Air) return FVector(WorldX, WorldY, VertZ) * CubeSize;
        FinalZ = TargetH;
    }
    return FVector(WorldX, WorldY, FinalZ) * CubeSize;
}

FVector ASmoothVoxelTerrain::GetSmoothNormalLocal(int32 VertX, int32 VertY, const FLocalHeightGrid& HeightGrid) const
{
    float hL = HeightGrid.GetHeight(VertX - 1, VertY);
    float hR = HeightGrid.GetHeight(VertX + 1, VertY);
    float hD = HeightGrid.GetHeight(VertX, VertY - 1);
    float hU = HeightGrid.GetHeight(VertX, VertY + 1);
    return FVector(hL - hR, hD - hU, 2.0f).GetSafeNormal();
}

float ASmoothVoxelTerrain::GetNeighborTopHeightLocal(int32 LocalX, int32 LocalY, int32 LocalZ, const FVector& VertexLocalPos, const FChunkNeighborhood& Neighborhood, const FLocalHeightGrid& HeightGrid) const
{
    EVoxelType neighborType = Neighborhood.GetVoxel(LocalX, LocalY, LocalZ);
    if (neighborType == EVoxelType::Air) return -FLT_MAX;
    else if (neighborType == EVoxelType::Grass)
    {
        float LocalVertX = (float)VertexLocalPos.X / CubeSize - Neighborhood.Self->Coord.X * ChunkSize;
        float LocalVertY = (float)VertexLocalPos.Y / CubeSize - Neighborhood.Self->Coord.Y * ChunkSize;
        return GetInterpolatedHeightLocal(LocalVertX, LocalVertY, HeightGrid) * CubeSize;
    }
    return (LocalZ + 1) * CubeSize;
}

FLinearColor ASmoothVoxelTerrain::GetStylizedColorForVoxel(const FVector& WorldPos, EVoxelType VoxelType) const
{
    float VoxX = (float)WorldPos.X / CubeSize;
    float VoxY = (float)WorldPos.Y / CubeSize;
    float VoxZ = (float)WorldPos.Z / CubeSize;

    if (VoxelType == EVoxelType::Grass) return FLinearColor::White;
    else if (VoxelType == EVoxelType::Dirt)
    {
        float DirtNoise = FastValueNoise2D(VoxX * 0.1f, VoxY * 0.1f);
        return FastColorLerp(FLinearColor(0.12f, 0.07f, 0.05f, 1.0f), FLinearColor(0.20f, 0.12f, 0.08f, 1.0f), DirtNoise);
    }
    else if (VoxelType == EVoxelType::Stone)
    {
        float StoneNoise = FastValueNoise3D(VoxX * 0.08f, VoxY * 0.08f, VoxZ * 0.08f);
        return FastColorLerp(FLinearColor(0.18f, 0.20f, 0.22f, 1.0f), FLinearColor(0.30f, 0.32f, 0.34f, 1.0f), StoneNoise);
    }
    return FLinearColor::White;
}

void ASmoothVoxelTerrain::AppendVoxelFacesLocal(int32 lx, int32 ly, int32 lz, FDynamicMesh3& Mesh, FTriIDArray& OutTriIDs, const FLocalHeightGrid& HeightGrid, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord)
{
    FDynamicMeshAttributeSet* Attr = Mesh.Attributes();
    if (!Attr) return;
    FDynamicMeshUVOverlay* UVOverlay = Attr->GetUVLayer(0);
    FDynamicMeshNormalOverlay* NormalOverlay = Attr->PrimaryNormals();
    FDynamicMeshColorOverlay* ColorOverlay = Attr->PrimaryColors();
    auto* MaterialIDAttribute = Attr->GetMaterialID();

    EVoxelType CurrentType = Neighborhood.GetVoxel(lx, ly, lz);
    if (CurrentType == EVoxelType::Air) return;

    int32 WorldX = ChunkCoord.X * ChunkSize + lx;
    int32 WorldY = ChunkCoord.Y * ChunkSize + ly;
    FLinearColor VoxelColor = GetStylizedColorForVoxel(FVector((double)WorldX * CubeSize + (0.5 * CubeSize), (double)WorldY * CubeSize + (0.5 * CubeSize), (double)lz * CubeSize), CurrentType);

    int32 cIdx = ColorOverlay->AppendElement(FVector4f(VoxelColor));

    int32 TopMatID = 1, BottomMatID = 1, SideMatID = 1;
    if (CurrentType == EVoxelType::Grass) { TopMatID = 0; BottomMatID = SideMatID = 1; }
    else if (CurrentType == EVoxelType::Dirt) { TopMatID = BottomMatID = SideMatID = 1; }
    else if (CurrentType == EVoxelType::Stone) { TopMatID = BottomMatID = SideMatID = 2; }

    if (!bSmoothTerrain)
    {
        bool bExposedTop = Neighborhood.GetVoxel(lx, ly, lz + 1) == EVoxelType::Air;
        bool bExposedBottom = Neighborhood.GetVoxel(lx, ly, lz - 1) == EVoxelType::Air;
        bool bExposedEast = Neighborhood.GetVoxel(lx + 1, ly, lz) == EVoxelType::Air;
        bool bExposedWest = Neighborhood.GetVoxel(lx - 1, ly, lz) == EVoxelType::Air;
        bool bExposedNorth = Neighborhood.GetVoxel(lx, ly + 1, lz) == EVoxelType::Air;
        bool bExposedSouth = Neighborhood.GetVoxel(lx, ly - 1, lz) == EVoxelType::Air;

        if (!bExposedTop && !bExposedBottom && !bExposedEast && !bExposedWest && !bExposedNorth && !bExposedSouth) return;

        FVector Origin((double)WorldX * CubeSize, (double)WorldY * CubeSize, (double)lz * CubeSize);
        FVector p000 = Origin, p100 = Origin + FVector(CubeSize, 0, 0), p010 = Origin + FVector(0, CubeSize, 0), p110 = Origin + FVector(CubeSize, CubeSize, 0);
        FVector p001 = Origin + FVector(0, 0, CubeSize), p101 = Origin + FVector(CubeSize, 0, CubeSize), p011 = Origin + FVector(0, CubeSize, CubeSize), p111 = Origin + FVector(CubeSize, CubeSize, CubeSize);

        auto AddQuadWorldFast = [&](const FVector& A, const FVector& B, const FVector& C, const FVector& D, const FVector3f& FixedNormal, int32 MatID, int32 UAxis, int32 VAxis)
            {
                FVector2D uvA((float)A[UAxis] / CubeSize * TextureScale, (float)A[VAxis] / CubeSize * TextureScale);
                FVector2D uvB((float)B[UAxis] / CubeSize * TextureScale, (float)B[VAxis] / CubeSize * TextureScale);
                FVector2D uvC((float)C[UAxis] / CubeSize * TextureScale, (float)C[VAxis] / CubeSize * TextureScale);
                FVector2D uvD((float)D[UAxis] / CubeSize * TextureScale, (float)D[VAxis] / CubeSize * TextureScale);

                int32 vA = Mesh.AppendVertex(FVector3d(A)), vB = Mesh.AppendVertex(FVector3d(B));
                int32 vC = Mesh.AppendVertex(FVector3d(C)), vD = Mesh.AppendVertex(FVector3d(D));

                int32 nIdx = NormalOverlay->AppendElement(FixedNormal);

                int32 t1 = Mesh.AppendTriangle(vA, vB, vC);
                if (t1 != FDynamicMesh3::InvalidID)
                {
                    OutTriIDs.Add(t1);
                    NormalOverlay->SetTriangle(t1, FIndex3i(nIdx, nIdx, nIdx));
                    UVOverlay->SetTriangle(t1, FIndex3i(UVOverlay->AppendElement(FVector2f(uvA)), UVOverlay->AppendElement(FVector2f(uvB)), UVOverlay->AppendElement(FVector2f(uvC))));
                    ColorOverlay->SetTriangle(t1, FIndex3i(cIdx, cIdx, cIdx));
                    if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t1, MatID);
                }

                int32 t2 = Mesh.AppendTriangle(vA, vC, vD);
                if (t2 != FDynamicMesh3::InvalidID)
                {
                    OutTriIDs.Add(t2);
                    NormalOverlay->SetTriangle(t2, FIndex3i(nIdx, nIdx, nIdx));
                    UVOverlay->SetTriangle(t2, FIndex3i(UVOverlay->AppendElement(FVector2f(uvA)), UVOverlay->AppendElement(FVector2f(uvC)), UVOverlay->AppendElement(FVector2f(uvD))));
                    ColorOverlay->SetTriangle(t2, FIndex3i(cIdx, cIdx, cIdx));
                    if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t2, MatID);
                }
            };

        if (bExposedTop) AddQuadWorldFast(p001, p011, p111, p101, FVector3f(0.f, 0.f, 1.f), TopMatID, 0, 1);
        if (bExposedBottom) AddQuadWorldFast(p100, p110, p010, p000, FVector3f(0.f, 0.f, -1.f), BottomMatID, 0, 1);
        if (bExposedEast) AddQuadWorldFast(p100, p101, p111, p110, FVector3f(1.f, 0.f, 0.f), SideMatID, 1, 2);
        if (bExposedWest) AddQuadWorldFast(p010, p011, p001, p000, FVector3f(-1.f, 0.f, 0.f), SideMatID, 1, 2);
        if (bExposedNorth) AddQuadWorldFast(p110, p111, p011, p010, FVector3f(0.f, 1.f, 0.f), SideMatID, 0, 2);
        if (bExposedSouth) AddQuadWorldFast(p000, p001, p101, p100, FVector3f(0.f, -1.f, 0.f), SideMatID, 0, 2);
    }
    else
    {
        FVector v000 = GetSmoothVertexLocal(lx, ly, lz, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord);
        FVector v100 = GetSmoothVertexLocal(lx + 1, ly, lz, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord);
        FVector v010 = GetSmoothVertexLocal(lx, ly + 1, lz, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord);
        FVector v110 = GetSmoothVertexLocal(lx + 1, ly + 1, lz, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord);
        FVector v001 = GetSmoothVertexLocal(lx, ly, lz + 1, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord);
        FVector v101 = GetSmoothVertexLocal(lx + 1, ly, lz + 1, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord);
        FVector v011 = GetSmoothVertexLocal(lx, ly + 1, lz + 1, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord);
        FVector v111 = GetSmoothVertexLocal(lx + 1, ly + 1, lz + 1, lx, ly, lz, HeightGrid, Neighborhood, ChunkCoord);

        auto ComputeTriangleNormal = [](const FVector& A, const FVector& B, const FVector& C) -> FVector { return FVector::CrossProduct(C - A, B - A).GetSafeNormal(); };
        auto AddQuadWorldSmooth = [&](const FVector& A, const FVector& B, const FVector& C, const FVector& D, int32 MatID, int32 UAxis, int32 VAxis)
            {
                FVector2D uvA((float)A[UAxis] / CubeSize * TextureScale, (float)A[VAxis] / CubeSize * TextureScale);
                FVector2D uvB((float)B[UAxis] / CubeSize * TextureScale, (float)B[VAxis] / CubeSize * TextureScale);
                FVector2D uvC((float)C[UAxis] / CubeSize * TextureScale, (float)C[VAxis] / CubeSize * TextureScale);
                FVector2D uvD((float)D[UAxis] / CubeSize * TextureScale, (float)D[VAxis] / CubeSize * TextureScale);

                int32 vA = Mesh.AppendVertex(FVector3d(A)), vB = Mesh.AppendVertex(FVector3d(B));
                int32 vC = Mesh.AppendVertex(FVector3d(C)), vD = Mesh.AppendVertex(FVector3d(D));

                FVector n1 = ComputeTriangleNormal(A, B, C);
                int32 t1 = Mesh.AppendTriangle(vA, vB, vC);
                if (t1 != FDynamicMesh3::InvalidID) {
                    OutTriIDs.Add(t1);
                    int32 nA1 = NormalOverlay->AppendElement(FVector3f(n1)), nB1 = NormalOverlay->AppendElement(FVector3f(n1)), nC1 = NormalOverlay->AppendElement(FVector3f(n1));
                    NormalOverlay->SetTriangle(t1, FIndex3i(nA1, nB1, nC1));
                    UVOverlay->SetTriangle(t1, FIndex3i(UVOverlay->AppendElement(FVector2f(uvA)), UVOverlay->AppendElement(FVector2f(uvB)), UVOverlay->AppendElement(FVector2f(uvC))));
                    ColorOverlay->SetTriangle(t1, FIndex3i(cIdx, cIdx, cIdx));
                    if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t1, MatID);
                }
                FVector n2 = ComputeTriangleNormal(A, C, D);
                int32 t2 = Mesh.AppendTriangle(vA, vC, vD);
                if (t2 != FDynamicMesh3::InvalidID) {
                    OutTriIDs.Add(t2);
                    int32 nA2 = NormalOverlay->AppendElement(FVector3f(n2)), nC2 = NormalOverlay->AppendElement(FVector3f(n2)), nD2 = NormalOverlay->AppendElement(FVector3f(n2));
                    NormalOverlay->SetTriangle(t2, FIndex3i(nA2, nC2, nD2));
                    UVOverlay->SetTriangle(t2, FIndex3i(UVOverlay->AppendElement(FVector2f(uvA)), UVOverlay->AppendElement(FVector2f(uvC)), UVOverlay->AppendElement(FVector2f(uvD))));
                    ColorOverlay->SetTriangle(t2, FIndex3i(cIdx, cIdx, cIdx));
                    if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t2, MatID);
                }
            };

        if (Neighborhood.GetVoxel(lx, ly, lz + 1) == EVoxelType::Air)
        {
            FVector n00 = GetSmoothNormalLocal(lx, ly, HeightGrid);
            FVector n10 = GetSmoothNormalLocal(lx + 1, ly, HeightGrid);
            FVector n01 = GetSmoothNormalLocal(lx, ly + 1, HeightGrid);
            FVector n11 = GetSmoothNormalLocal(lx + 1, ly + 1, HeightGrid);

            auto AddTopQuadSmooth = [&](const FVector& A, const FVector& B, const FVector& C, const FVector& D, const FVector& nA, const FVector& nB, const FVector& nC, const FVector& nD, int32 MatID)
                {
                    FVector2D uvA((float)A.X / CubeSize * TextureScale, (float)A.Y / CubeSize * TextureScale);
                    FVector2D uvB((float)B.X / CubeSize * TextureScale, (float)B.Y / CubeSize * TextureScale);
                    FVector2D uvC((float)C.X / CubeSize * TextureScale, (float)C.Y / CubeSize * TextureScale);
                    FVector2D uvD((float)D.X / CubeSize * TextureScale, (float)D.Y / CubeSize * TextureScale);

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
                        NormalOverlay->SetTriangle(t2, FIndex3i(NormalOverlay->AppendElement(FVector3f(nA)), NormalOverlay->AppendElement(FVector3f(nA)), NormalOverlay->AppendElement(FVector3f(nD))));
                        UVOverlay->SetTriangle(t2, FIndex3i(UVOverlay->AppendElement(FVector2f(uvA)), UVOverlay->AppendElement(FVector2f(uvC)), UVOverlay->AppendElement(FVector2f(uvD))));
                        ColorOverlay->SetTriangle(t2, FIndex3i(cIdx, cIdx, cIdx));
                        if (MaterialIDAttribute) MaterialIDAttribute->SetValue(t2, MatID);
                    }
                };
            AddTopQuadSmooth(v001, v011, v111, v101, n00, n01, n11, n10, TopMatID);
        }

        if (Neighborhood.GetVoxel(lx, ly, lz - 1) == EVoxelType::Air)
            AddQuadWorldSmooth(v100, v110, v010, v000, BottomMatID, 0, 1);

        if (Neighborhood.GetVoxel(lx + 1, ly, lz) == EVoxelType::Air ||
            GetNeighborTopHeightLocal(lx + 1, ly, lz, v100, Neighborhood, HeightGrid) < v100.Z ||
            GetNeighborTopHeightLocal(lx + 1, ly, lz, v101, Neighborhood, HeightGrid) < v101.Z ||
            GetNeighborTopHeightLocal(lx + 1, ly, lz, v111, Neighborhood, HeightGrid) < v111.Z ||
            GetNeighborTopHeightLocal(lx + 1, ly, lz, v110, Neighborhood, HeightGrid) < v110.Z)
        {
            AddQuadWorldSmooth(v100, v101, v111, v110, SideMatID, 1, 2);
        }

        if (Neighborhood.GetVoxel(lx - 1, ly, lz) == EVoxelType::Air ||
            GetNeighborTopHeightLocal(lx - 1, ly, lz, v010, Neighborhood, HeightGrid) < v010.Z ||
            GetNeighborTopHeightLocal(lx - 1, ly, lz, v011, Neighborhood, HeightGrid) < v011.Z ||
            GetNeighborTopHeightLocal(lx - 1, ly, lz, v001, Neighborhood, HeightGrid) < v001.Z ||
            GetNeighborTopHeightLocal(lx - 1, ly, lz, v000, Neighborhood, HeightGrid) < v000.Z)
        {
            AddQuadWorldSmooth(v010, v011, v001, v000, SideMatID, 1, 2);
        }

        if (Neighborhood.GetVoxel(lx, ly + 1, lz) == EVoxelType::Air ||
            GetNeighborTopHeightLocal(lx, ly + 1, lz, v110, Neighborhood, HeightGrid) < v110.Z ||
            GetNeighborTopHeightLocal(lx, ly + 1, lz, v111, Neighborhood, HeightGrid) < v111.Z ||
            GetNeighborTopHeightLocal(lx, ly + 1, lz, v011, Neighborhood, HeightGrid) < v011.Z ||
            GetNeighborTopHeightLocal(lx, ly + 1, lz, v010, Neighborhood, HeightGrid) < v010.Z)
        {
            AddQuadWorldSmooth(v110, v111, v011, v010, SideMatID, 0, 2);
        }

        if (Neighborhood.GetVoxel(lx, ly - 1, lz) == EVoxelType::Air ||
            GetNeighborTopHeightLocal(lx, ly - 1, lz, v000, Neighborhood, HeightGrid) < v000.Z ||
            GetNeighborTopHeightLocal(lx, ly - 1, lz, v001, Neighborhood, HeightGrid) < v001.Z ||
            GetNeighborTopHeightLocal(lx, ly - 1, lz, v101, Neighborhood, HeightGrid) < v101.Z ||
            GetNeighborTopHeightLocal(lx, ly - 1, lz, v100, Neighborhood, HeightGrid) < v100.Z)
        {
            AddQuadWorldSmooth(v000, v001, v101, v100, SideMatID, 0, 2);
        }
    }
}

void ASmoothVoxelTerrain::AppendGrassBladesLocal(int32 lx, int32 ly, int32 lz, FDynamicMesh3& Mesh, FTriIDArray& OutTriIDs, const FLocalHeightGrid& HeightGrid, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord)
{
    FDynamicMeshAttributeSet* Attr = Mesh.Attributes();
    if (!Attr) return;

    FDynamicMeshUVOverlay* UVOverlay0 = Attr->GetUVLayer(0);
    if (!UVOverlay0) return;

    if (Attr->NumUVLayers() < 2) Attr->SetNumUVLayers(2);
    FDynamicMeshUVOverlay* UVOverlay1 = Attr->GetUVLayer(1);

    int32 WorldX = ChunkCoord.X * ChunkSize + lx;
    int32 WorldY = ChunkCoord.Y * ChunkSize + ly;

    float DensityNoise = FastValueNoise2D((float)WorldX * GrassDensityNoiseScale, (float)WorldY * GrassDensityNoiseScale);
    float FineNoise = Hash3D(WorldX, WorldY, 999);

    if (FineNoise < 0.20f) DensityNoise *= 0.1f;
    else if (FineNoise > 0.85f) DensityNoise = FMath::Min(1.0f, DensityNoise * 1.5f);

    float TargetDensity = FMath::Lerp((float)GrassMinDensity, (float)GrassMaxDensity, DensityNoise) + (Hash3D(WorldX, WorldY, 888) - 0.5f) * 3.0f;
    int32 Density = FMath::Clamp(FMath::RoundToInt(TargetDensity), 0, GrassMaxDensity + 2);

    const float LocalGrassMinHeight = GrassMinHeight, LocalGrassMaxHeight = GrassMaxHeight, LocalGrassMinWidth = GrassMinWidth, LocalGrassMaxWidth = GrassMaxWidth, LocalCubeSize = CubeSize;
    const bool bLocalSmoothTerrain = bSmoothTerrain;

    for (int32 i = 0; i < Density; ++i)
    {
        FFastRandom FastRand((uint32)WorldX * 73856093U ^ (uint32)WorldY * 19349663U ^ (uint32)i * 83492791U);
        float RandX = FastRand.NextFloat(), RandY = FastRand.NextFloat(), RandHeight = FastRand.NextFloat(), RandWidth = FastRand.NextFloat();
        float RandAngle = FastRand.NextFloat(), RandLeanAngle = FastRand.NextFloat(), RandLeanStrength = FastRand.NextFloat();
        float RandBendAngle = FastRand.NextFloat(), RandBendForce = FastRand.NextFloat();

        float BladeLocalX = (float)lx + RandX;
        float BladeLocalY = (float)ly + RandY;
        float BladeWorldZ = 0.0f;
        FVector GroundNormal(0.f, 0.f, 1.f);

        if (bLocalSmoothTerrain)
        {
            BladeWorldZ = GetInterpolatedHeightLocal(BladeLocalX, BladeLocalY, HeightGrid) * LocalCubeSize;
            GroundNormal = GetSmoothNormalLocal(FMath::RoundToInt(BladeLocalX), FMath::RoundToInt(BladeLocalY), HeightGrid);
        }
        else BladeWorldZ = (float)(lz + 1) * LocalCubeSize;

        FVector BasePos((double)(WorldX + RandX) * LocalCubeSize, (double)(WorldY + RandY) * LocalCubeSize, (double)BladeWorldZ);

        float Height = LocalGrassMinHeight + (LocalGrassMaxHeight - LocalGrassMinHeight) * RandHeight;
        float Width = LocalGrassMinWidth + (LocalGrassMaxWidth - LocalGrassMinWidth) * RandWidth;

        float Angle = RandAngle * 2.0f * PI, SinAngle, CosAngle;
        FMath::SinCos(&SinAngle, &CosAngle, Angle);
        FVector BladeRight(CosAngle, SinAngle, 0.0f), BladeForward(-SinAngle, CosAngle, 0.0f);

        float LeanAngle = RandLeanAngle * 2.0f * PI, SinLean, CosLean;
        FMath::SinCos(&SinLean, &CosLean, LeanAngle);
        FVector TiltingNormal = (GroundNormal + FVector(CosLean, SinLean, 0.0f) * (0.05f + 0.15f * RandLeanStrength)).GetSafeNormal();

        float BendAngle = RandBendAngle * 2.0f * PI, SinBend, CosBend;
        FMath::SinCos(&SinBend, &CosBend, BendAngle);
        FVector BendDir = (FVector(CosBend, SinBend, 0.0f) * 0.5f + BladeForward * 0.3f + GroundNormal * 0.2f).GetSafeNormal();

        float BendForce = (0.15f + 0.35f * RandBendForce) * Height;

        FDynamicMeshNormalOverlay* NormalOverlay = Attr->PrimaryNormals();
        int32 nGround = NormalOverlay ? NormalOverlay->AppendElement(FVector3f(GroundNormal)) : -1;

        auto AddTri = [UVOverlay0, UVOverlay1, NormalOverlay, nGround, &Mesh, &OutTriIDs, this](
            int32 a, int32 b, int32 c, int32 u0_A, int32 u0_B, int32 u0_C, int32 u1_A, int32 u1_B, int32 u1_C)
            {
                int32 t = Mesh.AppendTriangle(a, b, c);
                if (t != FDynamicMesh3::InvalidID)
                {
                    OutTriIDs.Add(t);
                    if (NormalOverlay && nGround != -1) NormalOverlay->SetTriangle(t, FIndex3i(nGround, nGround, nGround));
                    if (UVOverlay0 && u0_A != -1) UVOverlay0->SetTriangle(t, FIndex3i(u0_A, u0_B, u0_C));
                    if (UVOverlay1 && u1_A != -1) UVOverlay1->SetTriangle(t, FIndex3i(u1_A, u1_B, u1_C));
                }
                if (!bTwoSidedGrass)
                {
                    int32 tBack = Mesh.AppendTriangle(a, c, b);
                    if (tBack != FDynamicMesh3::InvalidID)
                    {
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
            int32 v0 = Mesh.AppendVertex(FVector3d(BasePos - BladeRight * (Width * 0.5f)));
            int32 v1 = Mesh.AppendVertex(FVector3d(BasePos + BladeRight * (Width * 0.5f)));
            int32 v2 = Mesh.AppendVertex(FVector3d(BasePos + BendDir * BendForce + TiltingNormal * Height));

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
                    if (!Attr->HasMaterialID()) Attr->EnableMaterialID();

                    for (int32 dz = -1; dz <= 1; ++dz)
                    {
                        for (int32 dy = -1; dy <= 1; ++dy)
                        {
                            for (int32 dx = -1; dx <= 1; ++dx)
                            {
                                int32 dist = FMath::Abs(dx) + FMath::Abs(dy) + FMath::Abs(dz);
                                if (dist != 0 && dist != 1) continue;

                                int32 nx = LocalX + dx, ny = LocalY + dy, nz = LocalZ + dz;
                                if (nx >= 0 && nx < TerrainOwner->ChunkSize && ny >= 0 && ny < TerrainOwner->ChunkSize && nz >= 0 && nz < TerrainOwner->MaxHeight)
                                    RemoveVoxelFaces(nx, ny, nz, MeshOut, GrassMeshOut, TerrainOwner);
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

                                int32 nx = LocalX + dx, ny = LocalY + dy, nz = LocalZ + dz;
                                if (nx >= 0 && nx < TerrainOwner->ChunkSize && ny >= 0 && ny < TerrainOwner->ChunkSize && nz >= 0 && nz < TerrainOwner->MaxHeight)
                                {
                                    if (VoxelData[nx + ny * TerrainOwner->ChunkSize + nz * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize] != EVoxelType::Air)
                                        AddVoxelFaces(nx, ny, nz, MeshOut, GrassMeshOut, TerrainOwner);
                                }
                            }
                        }
                    }
                });
        });

    MeshComponent->UpdateCollision(true);
}

void ASmoothVoxelTerrain::FVoxelChunk::RemoveVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, FDynamicMesh3& Mesh, FDynamicMesh3& GrassMesh, ASmoothVoxelTerrain* TerrainOwner)
{
    int32 VoxelIndex = LocalX + LocalY * TerrainOwner->ChunkSize + LocalZ * TerrainOwner->ChunkSize * TerrainOwner->ChunkSize;
    if (auto* TriIDsPtr = VoxelTriangles.Find(VoxelIndex))
    {
        for (int32 TriID : *TriIDsPtr) { if (Mesh.IsTriangle(TriID)) Mesh.RemoveTriangle(TriID, false); }
        VoxelTriangles.Remove(VoxelIndex);
    }
    if (auto* GrassTriIDsPtr = GrassVoxelTriangles.Find(VoxelIndex))
    {
        for (int32 TriID : *GrassTriIDsPtr) { if (GrassMesh.IsTriangle(TriID)) GrassMesh.RemoveTriangle(TriID, false); }
        GrassVoxelTriangles.Remove(VoxelIndex);
    }
}

void ASmoothVoxelTerrain::FVoxelChunk::AddVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, FDynamicMesh3& Mesh, FDynamicMesh3& GrassMesh, ASmoothVoxelTerrain* TerrainOwner)
{
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
    Neighborhood.WestData = RetrieveVoxelDataPtr(FIntVector(-1, 0, 0));
    Neighborhood.EastData = RetrieveVoxelDataPtr(FIntVector(1, 0, 0));
    Neighborhood.SouthData = RetrieveVoxelDataPtr(FIntVector(0, -1, 0));
    Neighborhood.NorthData = RetrieveVoxelDataPtr(FIntVector(0, 1, 0));

    FLocalHeightGrid HeightGrid;
    HeightGrid.Heights = HeightMap.GetData();
    HeightGrid.CacheSize = TerrainOwner->ChunkSize + 3;

    FTriIDArray NewTriIDs;
    TerrainOwner->AppendVoxelFacesLocal(LocalX, LocalY, LocalZ, Mesh, NewTriIDs, HeightGrid, Neighborhood, Coord);
    if (NewTriIDs.Num() > 0) VoxelTriangles.Add(VoxelIndex, NewTriIDs);

    FTriIDArray NewGrassTriIDs;
    if (TerrainOwner->bEnableGrassGeometry && VoxelData[VoxelIndex] == EVoxelType::Grass)
    {
        if (Neighborhood.GetVoxel(LocalX, LocalY, LocalZ + 1) == EVoxelType::Air)
            TerrainOwner->AppendGrassBladesLocal(LocalX, LocalY, LocalZ, GrassMesh, NewGrassTriIDs, HeightGrid, Neighborhood, Coord);
    }
    if (NewGrassTriIDs.Num() > 0) GrassVoxelTriangles.Add(VoxelIndex, NewGrassTriIDs);
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