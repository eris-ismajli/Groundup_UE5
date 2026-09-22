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

using FTriIDArray = TArray<int32, TInlineAllocator<64>>;

struct FChunkNeighborhood;
struct FCaveSmoothCache;
struct FVoxelEditContext;

// One entry per voxel column over the chunk's halo, range [-2 .. ChunkSize+1].
// Surface height and ground level are pure functions of the height field but were
// recomputed from four corner reads on every query — ~30 times per voxel in the
// mesher, plus once per cell per z in the cave cache. Built once, read everywhere.
struct FColumnCache
{
    float Surface;
    int32 Ground;
};

struct FLocalHeightGrid
{
    const float* Heights = nullptr;
    const FColumnCache* Columns = nullptr;   // may be null: callers fall back to recompute
    int32               CacheSize = 0;         // (ChunkSize + 5), indices [-2 .. CS+2]
    int32               ColumnSize = 0;         // (ChunkSize + 4), indices [-2 .. CS+1]

    FORCEINLINE float GetHeight(int32 LocalX, int32 LocalY) const
    {
        // Halo is 2 columns: the relaxation pass reads cells two out from a
        // chunk-edge vertex, and those cells need their own corner heights.
        return Heights[(LocalX + 2) + (LocalY + 2) * CacheSize];
    }

    FORCEINLINE const FColumnCache& GetColumn(int32 LocalX, int32 LocalY) const
    {
        // Clamped rather than asserted: IsCellExpectedAir probes one cell past the
        // density cache's own range and only uses the result to reject.
        const int32 cx = FMath::Clamp(LocalX + 2, 0, ColumnSize - 1);
        const int32 cy = FMath::Clamp(LocalY + 2, 0, ColumnSize - 1);
        return Columns[cx + cy * ColumnSize];
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

// ---------------------------------------------------------------------------------
// CAVE WALL ROUGHNESS
// ---------------------------------------------------------------------------------
// Roughness is always carved INWARD from the smooth cave surface, and the micro layer
// is forced to zero at every voxel-lattice vertex. The collision hull is therefore the
// original smooth surface with pits cut into it - there is nothing sticking out for a
// character capsule to snag on.
USTRUCT(BlueprintType)
struct FCaveRoughnessSettings
{
    GENERATED_BODY()

    /** Master switch for cave wall roughness. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness")
    bool bEnableCaveRoughness = true;

    /**
     * Tessellation of a cave face while roughness is on. 1 = micro layer off,
     * 2 = 4x triangles on cave faces, 3 = 9x, 4 = 16x. This is your whole fine-detail
     * budget and it costs both render triangles and collision cook time.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness", meta = (ClampMin = "1", ClampMax = "6"))
    int32 DetailSubdivisions = 2;

    // ---- MACRO layer: moves the lattice vertices. Collision follows it. ----

    /** Carve depth of the large lumps, in voxels. 0 = collision identical to before. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Macro", meta = (ClampMin = "0.0", ClampMax = "1.5"))
    float MacroDepth = 0.30f;

    /** Cycles per voxel. Lower = broader boulder-like shapes. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Macro", meta = (ClampMin = "0.01", ClampMax = "2.0"))
    float MacroScale = 0.32f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Macro", meta = (ClampMin = "1", ClampMax = "6"))
    int32 MacroOctaves = 2;

    /** >1 widens the flat crests and localises the pits. <1 gives thin knife ridges. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Macro", meta = (ClampMin = "0.25", ClampMax = "4.0"))
    float MacroSharpness = 1.0f;

    // ---- MICRO layer: sub-voxel chipping. Never moves a lattice vertex. ----

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Micro", meta = (ClampMin = "0.0", ClampMax = "1.5"))
    float MicroDepth = 0.30f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Micro", meta = (ClampMin = "0.1", ClampMax = "8.0"))
    float MicroScale = 1.60f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Micro", meta = (ClampMin = "1", ClampMax = "6"))
    int32 MicroOctaves = 3;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Micro", meta = (ClampMin = "0.25", ClampMax = "4.0"))
    float MicroSharpness = 1.30f;

    // ---- Shared fractal shape ----

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Shape", meta = (ClampMin = "1.5", ClampMax = "4.0"))
    float Lacunarity = 2.13f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Shape", meta = (ClampMin = "0.1", ClampMax = "0.9"))
    float Gain = 0.5f;

    /** Ridged-multifractal feedback. Higher = crests carry more fine detail, more shattered. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Shape", meta = (ClampMin = "0.0", ClampMax = "4.0"))
    float RidgeWeight = 2.0f;

    /** Domain warp, in voxels. Breaks the noise grid so fractures wander instead of lining up. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Shape", meta = (ClampMin = "0.0", ClampMax = "4.0"))
    float WarpStrength = 0.55f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Shape", meta = (ClampMin = "0.01", ClampMax = "2.0"))
    float WarpScale = 0.22f;

    // ---- Bedding planes / strata ----

    /** 0 = uniform rock, 1 = strong horizontal banding of rough and smooth layers. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Strata", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float StrataStrength = 0.40f;

    /** Cycles per voxel along Z. Lower = thicker beds. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Strata", meta = (ClampMin = "0.01", ClampMax = "2.0"))
    float StrataScale = 0.25f;

    /** Tilts the beds off horizontal. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Strata", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float StrataTilt = 0.12f;

    // ---- Orientation shaping ----

    /** Multiplier on up-facing surfaces. This is what the player walks on - keep it low. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Orientation", meta = (ClampMin = "0.0", ClampMax = "2.0"))
    float FloorRoughness = 0.30f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Orientation", meta = (ClampMin = "0.0", ClampMax = "2.0"))
    float WallRoughness = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness|Orientation", meta = (ClampMin = "0.0", ClampMax = "2.0"))
    float CeilingRoughness = 1.0f;

    /** Hard clamp on total carve depth, in voxels. Raise past ~0.7 and thin walls can self-intersect. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cave Roughness", meta = (ClampMin = "0.0", ClampMax = "1.5"))
    float MaxDepth = 0.60f;
};

// Seed-derived noise offsets, filled once in PrepareDerived.
struct FCaveRoughOffsets
{
    float Warp[3] = { 0.0f, 0.0f, 0.0f };
    float Macro[6][3] = {};
    float Micro[6][3] = {};
    float Strata = 0.0f;
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

    // --- Wall Smoothing ---
    // Master switch for cave-wall vertex displacement. Off = classic hard cubes.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Caves|Smoothing")
    bool bSmoothCaves = true;
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

struct FSmoothVertex
{
    FVector   P = FVector::ZeroVector;
    FVector3f N = FVector3f::ZeroVector;
    bool      bCurved = false;

    // Where the sample came from and how the cave solve placed it. Merge is -1 for an anchor,
    // a mass-point fallback or any non-cave vertex; otherwise axis * 2 + (1 if the riser end
    // merged towards +axis).
    FIntVector Lattice = FIntVector::ZeroValue;
    bool       bHasLattice = false;
    int8       Merge = -1;
};
// --- Totally stateless thread-safe generation configuration struct ---
struct FTerrainGenConfig
{
    int32 ChunkSize;
    int32 FloorLevel;
    int32 BedrockLevel;
    int32 MaxHeight;
    float CubeSize;
    float MinGrassThickness;
    int32 Seed;
    bool bSmoothTerrain;
    bool bEnableWater;
    int32 SeaLevel;
    FBiomeGrasslandSettings GrasslandBiome;
    FCaveSettings CaveSettings;
    bool bEnableGrassGeometry;
    int32 GrassMinDensity;
    int32 GrassMaxDensity;
    float GrassMinHeight;
    float GrassMaxHeight;
    float GrassMinWidth;
    float GrassMaxWidth;
    float GrassDensityNoiseScale;
    int32 GrassBladeSegments;
    bool bTwoSidedGrass;
    float TextureScale;

    bool  bBendCaveFaces = true;
    int32 CavePatchSubdiv = 2;
    float CavePatchFlatDot = 0.99f;

    // Seed-derived constants. These were being recomputed with Hash2D/Hash3D on every
// density sample — about 20 hashes per voxel for values that depend only on Seed.
    struct FCaveOffsets
    {
        float Entrance[2];
        float Macro[3];
        float T1[6];
        float T2[6];
        float Chamber[2][3];
    };
    FCaveOffsets CaveOff;
    float InvCubeSize = 0.01f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Caves")
    FCaveRoughnessSettings CaveRoughness;

    FCaveRoughOffsets RoughOff;

    float   GetCaveRoughDepth(float VoxX, float VoxY, float VoxZ, const FVector3f& Normal,
        float MacroWeight, float MicroWeight) const;
    FVector ApplyCaveRoughness(const FVector& WorldPos, const FVector3f& Normal,
        float MacroWeight, float MicroWeight) const;

    // Must be called once after all plain fields are filled in.
    void PrepareDerived();

    FSmoothVertex GetSmoothVertexEx(int32 VertX, int32 VertY, int32 VertZ,
        int32 VoxX, int32 VoxY, int32 VoxZ,
        const FLocalHeightGrid& HeightGrid,
        const FChunkNeighborhood& Neighborhood,
        const FIntVector& ChunkCoord,
        FCaveSmoothCache* CaveCache) const;

    bool GetCaveVertex(int32 VertX, int32 VertY, int32 VertZ, const FIntVector& ChunkCoord,
        FCaveSmoothCache* CaveCache, FSmoothVertex& Out) const;

    float GetHeightAtWorldCorner(int32 WorldX, int32 WorldY) const;
    float GetInterpolatedHeightLocal(float LocalX, float LocalY, const FLocalHeightGrid& HeightGrid) const;

    // Carve-time surface height for one voxel column: the min of its 4 corner heights.
    // This is the exact expression GenerateChunkData uses, factored out so the mesher
    // and the generator can never drift apart.
    float GetSurfaceHeightLocal(int32 LocalX, int32 LocalY, const FLocalHeightGrid& HeightGrid) const;
    int32 GetGroundLevelLocal(int32 LocalX, int32 LocalY, const FLocalHeightGrid& HeightGrid) const;

    // Signed carve field. > 0 means carved out (air). Sign is bit-identical to the old
    // IsInsideCave, so existing worlds regenerate unchanged.
    float GetCaveDensityAt(float WorldX, float WorldY, float WorldZ, float SurfaceHeight) const;
    float GetCaveSmoothFieldAt(float VX, float VY, float VZ, float SurfaceHeight) const;   // NEW
    float GetCaveDensity(int32 WorldX, int32 WorldY, int32 WorldZ, float SurfaceHeight) const;
    bool IsInsideCave(int32 WorldX, int32 WorldY, int32 WorldZ, float SurfaceHeight) const;

    FVector GetSmoothVertexLocal(int32 VertX, int32 VertY, int32 VertZ, int32 VoxX, int32 VoxY, int32 VoxZ, const FLocalHeightGrid& HeightGrid, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord, FCaveSmoothCache* CaveCache = nullptr) const;
    FVector GetSmoothNormalLocal(int32 VertX, int32 VertY, const FLocalHeightGrid& HeightGrid) const;
    float GetNeighborTopHeightLocal(int32 LocalX, int32 LocalY, int32 LocalZ, const FVector& VertexLocalPos, const FChunkNeighborhood& Neighborhood, const FLocalHeightGrid& HeightGrid) const;
    FLinearColor GetStylizedColorForVoxel(const FVector& WorldPos, EVoxelType VoxelType) const;
    void AppendVoxelFacesLocal(int32 lx, int32 ly, int32 lz, UE::Geometry::FDynamicMesh3& Mesh, FTriIDArray& OutTriIDs, const FLocalHeightGrid& HeightGrid, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord, FCaveSmoothCache* CaveCache = nullptr) const;
    void AppendGrassBladesLocal(int32 lx, int32 ly, int32 lz, UE::Geometry::FDynamicMesh3& Mesh, FTriIDArray& OutTriIDs, const FLocalHeightGrid& HeightGrid, const FChunkNeighborhood& Neighborhood, const FIntVector& ChunkCoord) const;
};

struct FCaveSmoothCache
{
    void Init(const FTerrainGenConfig* InConfig, const FLocalHeightGrid* InHeights,
        const FChunkNeighborhood* InNeighborhood, const FIntVector& InChunkCoord);

    bool IsReady() const { return bReady; }


    bool IsCellSmoothSurface(int32 cx, int32 cy, int32 cz);

    int32 SmoothCell[3] = { MIN_int32, MIN_int32, MIN_int32 };
    bool  bSmoothCellResult = false;

    bool  IsCellExpectedAir(int32 cx, int32 cy, int32 cz);   // public
    bool  GetVertexOffset(int32 vx, int32 vy, int32 vz, FVector3f& OutOffset, FVector3f& OutNormal, int8* OutMerge = nullptr);
    bool  GetBaseOffset(int32 vx, int32 vy, int32 vz, FVector3f& OutOffset, FVector3f& OutNormal, int8& OutMerge);

    bool IsCellEdited(int32 cx, int32 cy, int32 cz);
    bool AirReachesSurface(int32 vx, int32 vy, int32 vz, int32 qx, int32 qy, int32 qz);

    float CornerHeight(int32 x, int32 y);
    float ColumnSurface(int32 cx, int32 cy);
    int32 ColumnGround(int32 cx, int32 cy);

    TArray<int8>  BaseMergeCache;
    TArray<int8>  VertMergeCache;
    int32         RingW = 0;
    TArray<float> RingHeight;
    TArray<uint8> RingHeightValid;

private:
    static constexpr int32 SLAB_COUNT = 8;   // power of two; index is z & 7

    float GetCellDensity(int32 cx, int32 cy, int32 cz);
    float VertexSurfaceHeight(int32 vx, int32 vy) const;

    TArray<FVector3f> BaseNormalCache;
    TArray<FVector3f> VertNormalCache;


    const FTerrainGenConfig* Config = nullptr;
    const FLocalHeightGrid* Heights = nullptr;
    const FChunkNeighborhood* Neighborhood = nullptr;
    FIntVector ChunkCoord = FIntVector::ZeroValue;
    bool bReady = false;

    int32 CS = 32;
    int32 CellW = 36;   // CS + 4 : cells  cx in [-2 .. CS+1]
    int32 BaseW = 35;   // CS + 3 : stage-1 verts  vx in [-1 .. CS+1]
    int32 VertW = 33;   // CS + 1 : final verts    vx in [ 0 .. CS  ]

    TArray<int32> CellSlabZ;
    TArray<float> CellDensityCache;
    TArray<uint8> CellValid;

    TArray<int32>     BaseSlabZ;
    TArray<FVector3f> BaseOffsetCache;
    TArray<uint8>     BaseState;   // 0 unknown, 1 no vertex, 2 have vertex

    TArray<int32>     VertSlabZ;
    TArray<FVector3f> VertOffsetCache;
    TArray<uint8>     VertState;
};

UCLASS()
class GROUNDUP_API ASmoothVoxelTerrain : public AActor
{
    GENERATED_BODY()

public:
    struct FVoxelChunk
    {
        FIntVector Coord;
        EChunkState State = EChunkState::Unloaded;

        bool bGeneratingGrass = false;
        bool bGrassGenerated = false;
        bool bWaterGenerated = false;

        TSharedPtr<TArray<EVoxelType>, ESPMode::ThreadSafe> VoxelData;
        TSharedPtr<TArray<float>, ESPMode::ThreadSafe> HeightMap;
        TSharedPtr<TArray<FColumnCache>, ESPMode::ThreadSafe> Columns;

        // Exclusive upper bound of any solid voxel in this chunk. The mesher's z loop
        // stops here instead of walking all MaxHeight planes of guaranteed air.
        int32 MaxSolidZ = 0;

        void BuildEditContext(FVoxelEditContext& OutCtx, ASmoothVoxelTerrain* TerrainOwner);
        void AddVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, UE::Geometry::FDynamicMesh3& Mesh, UE::Geometry::FDynamicMesh3* GrassMesh, ASmoothVoxelTerrain* TerrainOwner, FVoxelEditContext& Ctx);

        TMap<int32, FTriIDArray> VoxelTriangles;
        TMap<int32, FTriIDArray> GrassVoxelTriangles;

        UDynamicMeshComponent* MeshComponent = nullptr;
        UDynamicMeshComponent* GrassMeshComponent = nullptr;
        UDynamicMeshComponent* WaterMeshComponent = nullptr;

        void UpdateVoxel(int32 LocalX, int32 LocalY, int32 LocalZ, EVoxelType NewType, ASmoothVoxelTerrain* TerrainOwner);
        void UpdateVoxelMesh(int32 LocalX, int32 LocalY, int32 LocalZ, EVoxelType NewType, ASmoothVoxelTerrain* TerrainOwner);
        void RemoveVoxelFaces(int32 LocalX, int32 LocalY, int32 LocalZ, UE::Geometry::FDynamicMesh3& Mesh, UE::Geometry::FDynamicMesh3* GrassMesh, ASmoothVoxelTerrain* TerrainOwner);
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

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural Generation")
    int32 MaxCollisionUpdatesPerFrame = 2;

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
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    bool bSmoothTerrain = true;

    UPROPERTY(EditAnywhere, Category = "Terrain|Caves")
    bool bBendCaveFaces = true;

    // Uniform, by necessity: a per-patch count cracks the mesh along shared edges.
    UPROPERTY(EditAnywhere, Category = "Terrain|Caves", meta = (ClampMin = "1", ClampMax = "4"))
    int32 CavePatchSubdiv = 2;

    // Edges whose endpoint normals agree within this dot are left straight, so flat
    // stretches of wall collapse back to a single quad. Pure function of the edge.
    UPROPERTY(EditAnywhere, Category = "Terrain|Caves", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float CavePatchFlatDot = 0.99f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Caves")
    FCaveRoughnessSettings CaveRoughness;

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

    FTerrainGenConfig GetTerrainConfig() const;

private:
    FVoxelChunk* GetChunk(const FIntVector& Coord);
    const FVoxelChunk* GetChunk(const FIntVector& Coord) const;

    TMap<FIntVector, TSharedPtr<FVoxelChunk>> Chunks;

    UPROPERTY(VisibleAnywhere)
    USceneComponent* RootSceneComponent = nullptr;

    void GenerateChunks();
    FIntVector WorldToChunkCoord(const FVector& WorldPos) const;
    void WorldToLocalVoxel(const FVector& WorldPos, const FIntVector& ChunkCoord, int32& OutX, int32& OutY, int32& OutZ) const;
    FVector ChunkCoordToWorldOrigin(const FIntVector& ChunkCoord) const;

    TArray<TWeakObjectPtr<UDynamicMeshComponent>> PendingCollisionUpdates;
    void MarkCollisionDirty(UDynamicMeshComponent* Comp);
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

    UDynamicMeshComponent* AcquireMeshComponent(int32 MeshType);
    void ReleaseMeshComponent(UDynamicMeshComponent* Comp, int32 MeshType);
};