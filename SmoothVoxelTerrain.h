#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SceneComponent.h"
#include "Components/DynamicMeshComponent.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "DynamicMesh/DynamicMeshOverlay.h"
#include "SmoothVoxelTerrain.generated.h"

namespace UE::Geometry { class FDynamicMesh3; }

struct FChunkNeighborhood;

struct FLocalHeightGrid
{
    const float* Heights;
    int32 CacheSize;
    FORCEINLINE float GetHeight(int32 LocalX, int32 LocalY) const
    {
        return Heights[(LocalX + 1) + (LocalY + 1) * CacheSize];
    }
};

UENUM(BlueprintType)
enum class EVoxelType : uint8
{
    Air   UMETA(DisplayName = "Air"),
    Grass UMETA(DisplayName = "Grass"),
    Dirt  UMETA(DisplayName = "Dirt"),
    Stone UMETA(DisplayName = "Stone")
};

enum class EChunkState : uint8
{
    Unloaded,
    GeneratingData,
    DataReady,
    GeneratingMesh,
    MeshReady
};

USTRUCT(BlueprintType)
struct FBiomeGrasslandSettings
{
    GENERATED_BODY()

    // 1. GLOBAL ELEVATION: Affects the entire world smoothly so the "sea level" isn't a perfectly flat plane
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Global Base Elevation")
    float GlobalBaseNoiseScale = 0.0005f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Global Base Elevation")
    float GlobalBaseHeight = 400.0f;

    // 2. FLAT FIELDS: Adds bumps and uneven ground ONLY in flat areas. Fades out when hills/mountains appear.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Flat Fields")
    float FlatFieldNoiseScale = 0.004f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Flat Fields")
    float FlatFieldHeight = 150.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Flat Fields", meta = (ClampMin = "1", ClampMax = "6"))
    int32 FlatFieldOctaves = 3;

    // 3. SMOOTH HILLS
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Smooth Hills", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float SmoothHillLikelihood = 0.4f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Smooth Hills")
    float SmoothHillMaskScale = 0.0008f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Smooth Hills")
    float SmoothHillNoiseScale = 0.003f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Smooth Hills")
    float SmoothHillHeight = 1500.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Smooth Hills")
    float SmoothHillHeightVariance = 700.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Smooth Hills", meta = (ClampMin = "1", ClampMax = "8"))
    int32 SmoothHillOctaves = 3;

    // 4. JAGGED HILLS
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Jagged Hills", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float JaggedHillLikelihood = 0.15f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Jagged Hills")
    float JaggedHillMaskScale = 0.0015f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Jagged Hills")
    float JaggedHillNoiseScale = 0.015f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Jagged Hills")
    float JaggedHillHeight = 2500.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Jagged Hills")
    float JaggedHillHeightVariance = 1200.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Jagged Hills", meta = (ClampMin = "1", ClampMax = "8"))
    int32 JaggedHillOctaves = 4;
};

UCLASS()
class GROUNDUP_API ASmoothVoxelTerrain : public AActor
{
    GENERATED_BODY()

public:
    using FTriIDArray = TArray<int32, TInlineAllocator<64>>;

    struct FVoxelChunk
    {
        FIntVector Coord;
        EChunkState State = EChunkState::Unloaded;

        bool bGeneratingGrass = false;
        bool bGrassGenerated = false;
        bool bWaterGenerated = false;

        TSharedPtr<TArray<EVoxelType>, ESPMode::ThreadSafe> VoxelData;
        TSharedPtr<TArray<float>, ESPMode::ThreadSafe> HeightMap;

        TMap<int32, FTriIDArray> VoxelTriangles;
        TMap<int32, FTriIDArray> GrassVoxelTriangles;

        UDynamicMeshComponent* MeshComponent = nullptr;
        UDynamicMeshComponent* GrassMeshComponent = nullptr;
        UDynamicMeshComponent* WaterMeshComponent = nullptr;

        void UpdateVoxel(int32 LocalX, int32 LocalY, int32 LocalZ, EVoxelType NewType, ASmoothVoxelTerrain* TerrainOwner);
        void UpdateVoxelMesh(int32 LocalX, int32 LocalY, int32 LocalZ, EVoxelType NewType, ASmoothVoxelTerrain* TerrainOwner);
        void RemoveVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, UE::Geometry::FDynamicMesh3& Mesh, UE::Geometry::FDynamicMesh3* GrassMesh, ASmoothVoxelTerrain* TerrainOwner);
        void AddVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, UE::Geometry::FDynamicMesh3& Mesh, UE::Geometry::FDynamicMesh3* GrassMesh, ASmoothVoxelTerrain* TerrainOwner);
        void UpdateSharedFace(int32 LocalX, int32 LocalY, int32 LocalZ, ASmoothVoxelTerrain* TerrainOwner, const FIntVector& NeighborDirection);
    };

    ASmoothVoxelTerrain();
    ~ASmoothVoxelTerrain();

protected:
    virtual void BeginPlay() override;
    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaTime) override;
    void OnPlayerMoved(USceneComponent* UpdatedComponent, EUpdateTransformFlags UpdateTransformFlags, ETeleportType Teleport);

public:
    UFUNCTION(BlueprintCallable, Category = "Procedural Generation|Events")
    void RegisterPlayer(APawn* PlayerPawn);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Generation")
    int32 RenderDistance = 12;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Generation")
    int32 UnloadDistance = 14;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Generation")
    int32 MaxChunkDataGenPerFrame = 5;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Generation")
    int32 MaxChunkMeshGenPerFrame = 2;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Generation")
    int32 MaxChunkGrassGenPerFrame = 2;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Generation")
    int32 MaxMeshApplyPerFrame = 1;

    FIntVector LastPlayerChunkCoord = FIntVector(999999, 999999, 999999);

    void HandleBoundaryCrossing(const FIntVector& NewChunkCoord);
    void ProcessTasks();
    void GenerateChunkData(const FIntVector& ChunkCoord);
    void GenerateChunkMesh(const FIntVector& ChunkCoord);
    void GenerateGrassMesh(const FIntVector& ChunkCoord);
    void UnloadChunk(const FIntVector& Coord);
    bool CheckNeighborsDataReady(const FIntVector& ChunkCoord);

    void UpdateChunkVisibilityAndShadows();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials")
    UMaterialInterface* GrassMaterial = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials")
    UMaterialInterface* DirtMaterial = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials")
    UMaterialInterface* StoneMaterial = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials")
    UMaterialInterface* GrassBladesMaterial = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials")
    UMaterialInterface* WaterMaterial = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    int32 ChunkSize = 32;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    int32 FloorLevel = 40;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    int32 BedrockLevel = -64;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    int32 MaxHeight = 256;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    float CubeSize = 100.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    float MinGrassThickness = 1.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    int32 Seed = 1337;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    bool bSmoothTerrain = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Water")
    bool bEnableWater = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Water")
    int32 SeaLevel = 38;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Biomes")
    FBiomeGrasslandSettings GrasslandBiome;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Grass")
    bool bEnableGrassGeometry = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Grass")
    int32 GrassRenderDistance = 3;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Grass")
    int32 GrassMinDensity = 2;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Grass")
    int32 GrassMaxDensity = 6;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Grass")
    float GrassMinHeight = 35.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Grass")
    float GrassMaxHeight = 75.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Grass")
    float GrassMinWidth = 6.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Grass")
    float GrassMaxWidth = 12.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Grass")
    float GrassDensityNoiseScale = 0.03f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Grass", meta = (ClampMin = "1", ClampMax = "2"))
    int32 GrassBladeSegments = 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Grass")
    bool bTwoSidedGrass = true;

    UFUNCTION(BlueprintCallable, Category = "Terrain")
    void RebuildTerrain();

    UFUNCTION(BlueprintCallable, Category = "Terrain")
    void RemoveVoxel(FVector WorldLocation);

    bool GetVoxelAtWorldPoint(const FVector& WorldPoint, int32& OutVoxelX, int32& OutVoxelY, int32& OutVoxelZ, EVoxelType* OutType = nullptr);
    EVoxelType GetVoxelAtWorld(int32 WorldX, int32 WorldY, int32 WorldZ) const;

    UFUNCTION(BlueprintCallable, Category = "Terrain")
    void PlaceVoxel(FVector WorldLocation, EVoxelType Type = EVoxelType::Stone);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision")
    TEnumAsByte<ECollisionEnabled::Type> CollisionEnabled = ECollisionEnabled::QueryAndPhysics;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision")
    FName CollisionProfileName = "BlockAll";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision")
    bool bGenerateOverlapEvents = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision")
    bool bEnableComplexCollision = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Rendering")
    bool bCastShadow = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Rendering", meta = (EditCondition = "bCastShadow"))
    int32 ShadowRenderDistance = 8;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Rendering")
    bool bReceivesDecals = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Rendering")
    float TextureScale = 0.1f;

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

public:
    float GetHeightAtWorldCorner(int32 WorldX, int32 WorldY) const;
    float GetInterpolatedHeightLocal(float LocalX, float LocalY, const FLocalHeightGrid& HeightGrid) const;

private:
    FVector GetSmoothVertexLocal(int32 VertX, int32 VertY, int32 VertZ, int32 VoxX, int32 VoxY, int32 VoxZ, const FLocalHeightGrid& HeightGrid, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord) const;
    FVector GetSmoothNormalLocal(int32 VertX, int32 VertY, const FLocalHeightGrid& HeightGrid) const;
    float GetNeighborTopHeightLocal(int32 LocalX, int32 LocalY, int32 LocalZ, const FVector& VertexLocalPos, const FChunkNeighborhood& Neighborhood, const FLocalHeightGrid& HeightGrid) const;

    FLinearColor GetStylizedColorForVoxel(const FVector& WorldPos, EVoxelType VoxelType) const;

    void AppendVoxelFacesLocal(int32 LocalX, int32 LocalY, int32 LocalZ, UE::Geometry::FDynamicMesh3& Mesh, FTriIDArray& OutTriIDs, const FLocalHeightGrid& HeightGrid, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord);
    void AppendGrassBladesLocal(int32 LocalX, int32 LocalY, int32 LocalZ, UE::Geometry::FDynamicMesh3& Mesh, FTriIDArray& OutTriIDs, const FLocalHeightGrid& HeightGrid, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord);

    FVoxelChunk* GetChunk(const FIntVector& Coord);
    const FVoxelChunk* GetChunk(const FIntVector& Coord) const;

    TMap<FIntVector, TUniquePtr<FVoxelChunk>> Chunks;

    UPROPERTY(VisibleAnywhere)
    USceneComponent* RootSceneComponent = nullptr;

    void GenerateChunks();
    FIntVector WorldToChunkCoord(const FVector& WorldPos) const;
    void WorldToLocalVoxel(const FVector& WorldPos, const FIntVector& ChunkCoord, int32& OutX, int32& OutY, int32& OutZ) const;
    FVector ChunkCoordToWorldOrigin(const FIntVector& ChunkCoord) const;

    bool bCollisionDirty = false;
    void UpdateCollisionIfNeeded();
    bool bIsDestroyed = false;

    TWeakObjectPtr<USceneComponent> TrackedPlayerComponent;

private:
    TArray<FIntVector> DataGenerationQueue;
    TArray<FIntVector> MeshGenerationQueue;
    TArray<FIntVector> GrassGenerationQueue;

    struct FMeshApplyTask
    {
        FIntVector Coord;
        UE::Geometry::FDynamicMesh3 LocalMesh;
        TMap<int32, FTriIDArray> VoxelTriangles;
    };

    struct FGrassApplyTask
    {
        FIntVector Coord;
        UE::Geometry::FDynamicMesh3 LocalGrassMesh;
        TMap<int32, FTriIDArray> GrassVoxelTriangles;
    };

    TArray<TSharedPtr<FMeshApplyTask, ESPMode::ThreadSafe>> MeshApplyQueue;
    TArray<TSharedPtr<FGrassApplyTask, ESPMode::ThreadSafe>> GrassApplyQueue;

    TArray<UDynamicMeshComponent*> MeshComponentPool;
    TArray<UDynamicMeshComponent*> GrassMeshComponentPool;
    TArray<UDynamicMeshComponent*> WaterMeshComponentPool;

    // MeshType: 0 = Terrain, 1 = Grass, 2 = Water
    UDynamicMeshComponent* AcquireMeshComponent(int32 MeshType);
    void ReleaseMeshComponent(UDynamicMeshComponent* Comp, int32 MeshType);
};