#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TreeGenerator.h" 
#include "GeneratedTree.generated.h"

class UDynamicMeshComponent;
class UMaterialInterface;

UCLASS()
class GROUNDUP_API AGeneratedTree : public AActor
{
    GENERATED_BODY()

public:
    AGeneratedTree();

protected:
    virtual void BeginPlay() override;
    virtual void OnConstruction(const FTransform& Transform) override;

public:
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Tree")
    UDynamicMeshComponent* MeshComponent;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Settings")
    int32 Seed = 0;

    // --- HARMONIOUS RADIUS SYSTEM ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Settings|Radius & Shape")
    float TrunkRadius = 16.0f; // Absolute base radius of the tree

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Settings|Radius & Shape", meta = (ClampMin = "0.1", ClampMax = "1.0"))
    float BranchRadiusScale = 0.5f; // Child base radius = Parent radius at spawn point * this scale (Guarantees natural joints)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Settings|Radius & Shape", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float GlobalTaper = 0.1f; // Every branch tapers to this % of its own base radius (e.g., 0.1 = 10%)

    // --- TRUNK BASE (Roots) ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Settings|Trunk Base")
    float TrunkFlare = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Settings|Trunk Base", meta = (ClampMin = "0.01", ClampMax = "1.0"))
    float TrunkFlareHeight = 0.15f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Settings|Trunk Base")
    int32 TrunkRidgeFrequency = 5;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Settings|Trunk Base", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float TrunkRidgeIntensity = 0.15f;

    // --- DYNAMIC RESOLUTION ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Settings|Resolution", meta = (ClampMin = "4", ClampMax = "64"))
    int32 BaseRadialResolution = 16; // Trunk uses this. Twigs automatically scale down to 3 or 4 sides to save polys.

    // --- HIERARCHICAL BRANCHING ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Settings|Branching")
    TArray<FTreeItLevelParams> BranchLevels;

    // --- LEAVES ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Settings|Leaves")
    float LeafSize = 60.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Settings|Leaves")
    int32 LeafCards = 3;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Settings|Materials")
    UMaterialInterface* TrunkMaterial;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Settings|Materials")
    UMaterialInterface* LeavesMaterial;
};