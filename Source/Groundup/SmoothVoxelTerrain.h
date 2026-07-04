#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/DynamicMeshComponent.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "DynamicMesh/DynamicMeshOverlay.h"
#include "SmoothVoxelTerrain.generated.h"

namespace UE::Geometry { class FDynamicMesh3; }

struct FChunkNeighborhood;

// Lightweight wrapper for pre-calculated height access
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

UCLASS()
class GROUNDUP_API ASmoothVoxelTerrain : public AActor
{
    GENERATED_BODY()

public:
    using FTriIDArray = TArray<int32, TInlineAllocator<12>>;

    struct FVoxelChunk
    {
        FIntVector Coord;
        TArray<EVoxelType> VoxelData;
        TArray<float> HeightMap; // Cached noise evaluations
        TMap<int32, FTriIDArray> VoxelTriangles;
        TMap<int32, FTriIDArray> GrassVoxelTriangles;
        UDynamicMeshComponent* MeshComponent = nullptr;
        UDynamicMeshComponent* GrassMeshComponent = nullptr;

        void BuildMesh(ASmoothVoxelTerrain* TerrainOwner);
        void UpdateVoxel(int32 LocalX, int32 LocalY, int32 LocalZ, EVoxelType NewType, ASmoothVoxelTerrain* TerrainOwner);

        void UpdateVoxelMesh(int32 LocalX, int32 LocalY, int32 LocalZ, EVoxelType NewType, ASmoothVoxelTerrain* TerrainOwner);
        void RemoveVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, UE::Geometry::FDynamicMesh3& Mesh, UE::Geometry::FDynamicMesh3& GrassMesh, ASmoothVoxelTerrain* TerrainOwner);
        void AddVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, UE::Geometry::FDynamicMesh3& Mesh, UE::Geometry::FDynamicMesh3& GrassMesh, ASmoothVoxelTerrain* TerrainOwner);
        void UpdateSharedFace(int32 LocalX, int32 LocalY, int32 LocalZ, ASmoothVoxelTerrain* TerrainOwner, const FIntVector& NeighborDirection);
    };

    ASmoothVoxelTerrain();
    ~ASmoothVoxelTerrain();

protected:
    virtual void BeginPlay() override;
    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaTime) override;

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Generation")
    int32 RenderDistance = 6;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Generation")
    int32 UnloadDistance = 8;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Generation")
    float UpdateInterval = 0.3f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Generation")
    int32 MaxChunkGenPerFrame = 1;

    FIntVector LastPlayerChunkCoord = FIntVector(999999, 999999, 999999);
    float TimeSinceLastUpdate = 0.0f;
    TArray<FIntVector> GenerationQueue;

    void UpdateProceduralTerrain();
    void ProcessGenerationQueue();
    void GenerateSingleChunk(const FIntVector& ChunkCoord);
    void UnloadChunk(const FIntVector& Coord);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials")
    UMaterialInterface* GrassMaterial = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials")
    UMaterialInterface* DirtMaterial = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials")
    UMaterialInterface* StoneMaterial = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials")
    UMaterialInterface* GrassBladesMaterial = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    int32 ChunkSize = 32;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    int32 MaxHeight = 64;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    float CubeSize = 100.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    float NoiseScale = 0.01f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    float HeightMultiplier = 2000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    float MinGrassThickness = 1.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    int32 Seed = 1337;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    bool bSmoothTerrain = false;

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
    void PlaceVoxel(FVector WorldLocation, EVoxelType Type = EVoxelType::Dirt);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision")
    TEnumAsByte<ECollisionEnabled::Type> CollisionEnabled = ECollisionEnabled::QueryAndPhysics;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision")
    FName CollisionProfileName = "BlockAll";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision")
    bool bGenerateOverlapEvents = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Rendering")
    bool bCastShadow = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Rendering")
    bool bReceivesDecals = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision")
    bool bEnableComplexCollision = true;

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
};