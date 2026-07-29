#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "TreeGenerator.generated.h"

class UDynamicMeshComponent;
class UMaterialInterface;

USTRUCT(BlueprintType)
struct FTreeItLevelParams
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Shape")
    float Length = 200.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Shape")
    float LengthVariance = 30.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Shape", meta = (ClampMin = "1"))
    int32 Segments = 6;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Shape", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float Jitter = 0.05f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Branching")
    int32 BranchesSpawned = 4;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Branching")
    float BranchAngle = 45.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Branching")
    float GravityBend = 0.05f;

    // --- NEW Puffy Leaf Parameter ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree Leaves")
    int32 LeavesSpawned = 0; // Number of leaf clusters distributed across the body of this branch
};

UCLASS()
class GROUNDUP_API UTreeGenerator : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, Category = "Tree Generator")
    static void GenerateTreeIt(
        UDynamicMeshComponent* DynamicMeshComponent,
        int32 Seed,
        float TrunkRadius,
        float BranchRadiusScale,
        float GlobalTaper,
        float TrunkFlare,
        float TrunkFlareHeight,
        int32 TrunkRidgeFrequency,
        float TrunkRidgeIntensity,
        int32 BaseRadialResolution,
        TArray<FTreeItLevelParams> BranchLevels,
        float LeafLength,
        float LeafWidthScale,
        int32 LeafCards,
        float LeafPitch,
        float LeafPitchVariance,
        float LeafGravityBend,
        UMaterialInterface* BarkMaterial = nullptr,
        UMaterialInterface* LeafMaterial = nullptr
    );
};