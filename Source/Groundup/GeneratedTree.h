// GeneratedTree.h
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GeneratedTree.generated.h"

class UDynamicMeshComponent;

UCLASS()
class GROUNDUP_API AGeneratedTree : public AActor
{
    GENERATED_BODY()

public:
    AGeneratedTree();

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    UDynamicMeshComponent* MeshComponent;

    // Configurable generation settings
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree")
    int32 Seed = 0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree")
    float TrunkHeight = 200.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree")
    float TrunkRadius = 20.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree")
    int32 BranchLevels = 3;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tree")
    float LeafClusterRadius = 50.0f;

protected:
    virtual void BeginPlay() override;
};