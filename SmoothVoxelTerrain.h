#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SceneComponent.h"
#include "Components/DynamicMeshComponent.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "DynamicMesh/DynamicMeshOverlay.h"
#include "HAL/CriticalSection.h"
#include "HAL/ThreadSafeBool.h"
#include <atomic>
#include "SmoothVoxelTerrain.generated.h"

namespace UE::Geometry { class FDynamicMesh3; }

using FTriIDArray = TArray<int32, TInlineAllocator<64>>;

struct FChunkNeighborhood;

UENUM(BlueprintType)
enum class EVoxelType : uint8
{
    Air       UMETA(DisplayName = "Air"),
    Surface   UMETA(DisplayName = "Surface"),
    Grass     UMETA(DisplayName = "Grass"),
    Dirt      UMETA(DisplayName = "Dirt"),
    Stone     UMETA(DisplayName = "Stone")
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
struct FCaveSettings
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Caves")
    bool bEnableCaves = true;

    // --- Tunnels (Spaghetti / Worm Caves) ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Caves|Tunnels", meta = (ClampMin = "0.001", ClampMax = "0.1"))
    float TunnelNoiseScaleXZ = 0.018f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Caves|Tunnels", meta = (ClampMin = "0.001", ClampMax = "0.1"))
    float TunnelNoiseScaleY = 0.022f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Caves|Tunnels", meta = (ClampMin = "0.01", ClampMax = "0.5"))
    float TunnelBaseRadius = 0.065f;

    // --- Chambers (Caverns / Cheese Caves) ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Caves|Chambers", meta = (ClampMin = "0.001", ClampMax = "0.1"))
    float ChamberNoiseScaleXZ = 0.012f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Caves|Chambers", meta = (ClampMin = "0.001", ClampMax = "0.1"))
    float ChamberNoiseScaleY = 0.016f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Caves|Chambers", meta = (ClampMin = "0.0005", ClampMax = "0.05"))
    float ChamberFrequencyScale = 0.006f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Caves|Chambers", meta = (ClampMin = "0.1", ClampMax = "0.9"))
    float ChamberThreshold = 0.52f;

    // --- Tunnel & Chamber Relationship ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Caves|Relationship", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float TunnelChamberExpansion = 0.08f;

    // --- Depth & Surface Constraints ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Caves|Surface & Depth", meta = (ClampMin = "1.0", ClampMax = "30.0"))
    float CaveMaxHeightOffset = 6.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Caves|Surface & Depth", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float SurfaceBreakthroughLikelihood = 0.08f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Caves|Surface & Depth")
    int32 CaveBedrockSafetyMargin = 2;
};

USTRUCT(BlueprintType)
struct FBiomeGrasslandSettings
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Global Base Elevation")
    float GlobalBaseNoiseScale = 0.0005f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Global Base Elevation")
    float GlobalBaseHeight = 400.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Flat Fields")
    float FlatFieldNoiseScale = 0.004f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Flat Fields")
    float FlatFieldHeight = 150.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Flat Fields", meta = (ClampMin = "1", ClampMax = "6"))
    int32 FlatFieldOctaves = 3;

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

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Plains", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float PlainsLikelihood = 0.3f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Plains")
    float PlainsMaskScale = 0.0006f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Plains")
    float PlainsNoiseScale = 0.0004f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Plains")
    float PlainsHeight = 350.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Plains")
    float PlainsFloorLevel = 15.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Plains", meta = (ClampMin = "1", ClampMax = "8"))
    int32 PlainsOctaves = 2;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Rivers")
    float RiverNoiseScale = 0.0005f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Rivers")
    float RiverWidth = 0.025f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Rivers")
    float RiverDepth = 400.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Rivers")
    float RiverWarpScale = 0.002f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grassland|Rivers")
    float RiverWarpStrength = 250.0f;
};

struct FTerrainGenConfig
{
    int32 ChunkSize = 32;
    int32 FloorLevel = 0;
    int32 BedrockLevel = -64;
    int32 MaxHeight = 256;
    float CubeSize = 100.0f;
    float MinGrassThickness = 1.5f;
    int32 Seed = 1337;
    bool bEnableWater = true;
    int32 SeaLevel = 38;
    FBiomeGrasslandSettings GrasslandBiome;
    FCaveSettings CaveSettings;
    bool bEnableGrassGeometry = true;
    int32 GrassMinDensity = 2;
    int32 GrassMaxDensity = 6;
    float GrassMinHeight = 35.0f;
    float GrassMaxHeight = 75.0f;
    float GrassMinWidth = 6.0f;
    float GrassMaxWidth = 12.0f;
    float GrassDensityNoiseScale = 0.03f;
    int32 GrassBladeSegments = 1;
    bool bTwoSidedGrass = true;
    float TextureScale = 0.1f;

    float GetTerrainHeight(int32 WorldX, int32 WorldY) const;
    bool IsInsideCave(int32 WorldX, int32 WorldY, int32 WorldZ, float SurfaceHeight) const;

    FLinearColor GetStylizedColorForVoxel(const FVector& WorldPos, EVoxelType VoxelType) const;

    void AppendVoxelFacesLocal(int32 lx, int32 ly, int32 lz, UE::Geometry::FDynamicMesh3& Mesh, FTriIDArray& OutTriIDs, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord) const;
    void AppendGrassBladesLocal(int32 lx, int32 ly, int32 lz, UE::Geometry::FDynamicMesh3& Mesh, FTriIDArray& OutTriIDs, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord) const;
};

UCLASS()
class GROUNDUP_API ASmoothVoxelTerrain : public AActor
{
    GENERATED_BODY()

public:
    struct FVoxelChunk
    {
        FIntVector Coord = FIntVector::ZeroValue;
        EChunkState State = EChunkState::Unloaded;

        bool bGeneratingGrass = false;
        bool bGrassGenerated = false;
        bool bWaterGenerated = false;

        TSharedPtr<TArray<EVoxelType>, ESPMode::ThreadSafe> VoxelData;

        TMap<int32, FTriIDArray> VoxelTriangles;
        TMap<int32, FTriIDArray> GrassVoxelTriangles;

        TWeakObjectPtr<UDynamicMeshComponent> MeshComponent;
        TWeakObjectPtr<UDynamicMeshComponent> GrassMeshComponent;
        TWeakObjectPtr<UDynamicMeshComponent> WaterMeshComponent;

        void UpdateVoxel(int32 LocalX, int32 LocalY, int32 LocalZ, EVoxelType NewType, ASmoothVoxelTerrain* TerrainOwner);
        void UpdateVoxelMesh(int32 LocalX, int32 LocalY, int32 LocalZ, EVoxelType NewType, ASmoothVoxelTerrain* TerrainOwner);
        void RemoveVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, UE::Geometry::FDynamicMesh3& Mesh, UE::Geometry::FDynamicMesh3* GrassMesh, ASmoothVoxelTerrain* TerrainOwner);
        void AddVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, UE::Geometry::FDynamicMesh3& Mesh, UE::Geometry::FDynamicMesh3* GrassMesh, ASmoothVoxelTerrain* TerrainOwner);
        void UpdateSharedFace(int32 LocalX, int32 LocalY, int32 LocalZ, ASmoothVoxelTerrain* TerrainOwner, const FIntVector& NeighborDirection);
    };

    ASmoothVoxelTerrain();
    virtual ~ASmoothVoxelTerrain() override;

protected:
    virtual void BeginPlay() override;
    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void BeginDestroy() override;
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
    TObjectPtr<UMaterialInterface> SurfaceMaterial = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials")
    TObjectPtr<UMaterialInterface> GrassMaterial = nullptr;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials")
    TObjectPtr<UMaterialInterface> DirtMaterial = nullptr;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials")
    TObjectPtr<UMaterialInterface> StoneMaterial = nullptr;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials")
    TObjectPtr<UMaterialInterface> GrassBladesMaterial = nullptr;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials")
    TObjectPtr<UMaterialInterface> WaterMaterial = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    int32 ChunkSize = 32;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    int32 FloorLevel = 0;
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

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Water")
    bool bEnableWater = true;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Water")
    int32 SeaLevel = 38;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Biomes")
    FBiomeGrasslandSettings GrasslandBiome;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Caves")
    FCaveSettings CaveSettings;

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
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Rendering")
    float TextureScale = 0.1f;

    UFUNCTION(BlueprintCallable, Category = "Terrain")
    void RebuildTerrain();

    UFUNCTION(BlueprintCallable, Category = "Terrain")
    void RemoveVoxel(FVector WorldLocation, FVector HitNormal = FVector::ZeroVector);

    bool GetVoxelAtWorldPoint(const FVector& WorldPoint, int32& OutVoxelX, int32& OutVoxelY, int32& OutVoxelZ, EVoxelType* OutType = nullptr);
    EVoxelType GetVoxelAtWorld(int32 WorldX, int32 WorldY, int32 WorldZ) const;

    UFUNCTION(BlueprintCallable, Category = "Terrain")
    void PlaceVoxel(FVector WorldLocation, EVoxelType Type = EVoxelType::Stone, FVector HitNormal = FVector::ZeroVector);

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

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

    FTerrainGenConfig GetTerrainConfig() const;

private:
    FVoxelChunk* GetChunk(const FIntVector& Coord);
    const FVoxelChunk* GetChunk(const FIntVector& Coord) const;

    TMap<FIntVector, TSharedPtr<FVoxelChunk>> Chunks;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<USceneComponent> RootSceneComponent = nullptr;

    void GenerateChunks();
    FIntVector WorldToChunkCoord(const FVector& WorldPos) const;
    void WorldToLocalVoxel(const FVector& WorldPos, const FIntVector& ChunkCoord, int32& OutX, int32& OutY, int32& OutZ) const;
    FVector ChunkCoordToWorldOrigin(const FIntVector& ChunkCoord) const;

    bool bCollisionDirty = false;
    void UpdateCollisionIfNeeded();

    std::atomic<bool> bIsDestroyed{ false };
    std::atomic<uint32> GenerationEpoch{ 0 };
    std::atomic<int32> InFlightTasksCount{ 0 };

    TWeakObjectPtr<USceneComponent> TrackedPlayerComponent;

private:
    TArray<FIntVector> DataGenerationQueue;
    TArray<FIntVector> MeshGenerationQueue;
    TArray<FIntVector> GrassGenerationQueue;

    struct FMeshApplyTask
    {
        uint32 Epoch = 0;
        FIntVector Coord;
        UE::Geometry::FDynamicMesh3 LocalMesh;
        TMap<int32, FTriIDArray> VoxelTriangles;
    };

    struct FGrassApplyTask
    {
        uint32 Epoch = 0;
        FIntVector Coord;
        UE::Geometry::FDynamicMesh3 LocalGrassMesh;
        TMap<int32, FTriIDArray> GrassVoxelTriangles;
    };

    FCriticalSection QueueLock;
    TArray<TSharedPtr<FMeshApplyTask, ESPMode::ThreadSafe>> MeshApplyQueue;
    TArray<TSharedPtr<FGrassApplyTask, ESPMode::ThreadSafe>> GrassApplyQueue;

    UPROPERTY(Transient)
    TArray<TObjectPtr<UDynamicMeshComponent>> MeshComponentPool;

    UPROPERTY(Transient)
    TArray<TObjectPtr<UDynamicMeshComponent>> GrassMeshComponentPool;

    UPROPERTY(Transient)
    TArray<TObjectPtr<UDynamicMeshComponent>> WaterMeshComponentPool;

    UPROPERTY(Transient)
    TArray<TObjectPtr<UDynamicMeshComponent>> ActiveComponents;

    UDynamicMeshComponent* AcquireMeshComponent(int32 MeshType);
    void ReleaseMeshComponent(UDynamicMeshComponent* Comp, int32 MeshType);
    void CleanupAllComponents();
    void WaitForAllTasks();
};