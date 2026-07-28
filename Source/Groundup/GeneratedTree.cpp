// GeneratedTree.cpp
#include "GeneratedTree.h"
#include "Components/DynamicMeshComponent.h"
#include "TreeGenerator.h"

AGeneratedTree::AGeneratedTree()
{
    MeshComponent = CreateDefaultSubobject<UDynamicMeshComponent>(TEXT("MeshComponent"));
    RootComponent = MeshComponent;
}

void AGeneratedTree::BeginPlay()
{
    Super::BeginPlay();
    UTreeGenerator::GenerateOakTree(MeshComponent, Seed, TrunkHeight, TrunkRadius, BranchLevels, LeafClusterRadius);
}