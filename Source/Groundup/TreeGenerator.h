// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "TreeGenerator.generated.h"

class UDynamicMeshComponent;
class UMaterialInterface;   // <-- added

UCLASS()
class GROUNDUP_API UTreeGenerator : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    /**
     * Generates a random oak tree mesh on the given DynamicMeshComponent.
     * @param DynamicMeshComponent The component to populate.
     * @param Seed                 Random seed for reproducibility.
     * @param TrunkHeight          Height of the main trunk.
     * @param TrunkRadius          Radius at the base of the trunk.
     * @param BranchLevels         How many generations of branches to create.
     * @param LeafClusterRadius    Size of the leaf clusters at branch tips.
     * @param BarkMaterial         (Optional) Material to apply to the trunk and branches (slot 0).
     * @param LeafMaterial         (Optional) Material to apply to the leaf clusters (slot 1).
     */
    UFUNCTION(BlueprintCallable, Category = "Tree Generator")
    static void GenerateOakTree(
        UDynamicMeshComponent* DynamicMeshComponent,
        int32 Seed = 0,
        float TrunkHeight = 200.0f,
        float TrunkRadius = 20.0f,
        int32 BranchLevels = 3,
        float LeafClusterRadius = 50.0f,
        UMaterialInterface* BarkMaterial = nullptr,
        UMaterialInterface* LeafMaterial = nullptr
    );
};