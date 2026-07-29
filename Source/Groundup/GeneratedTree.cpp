#include "GeneratedTree.h"
#include "Components/DynamicMeshComponent.h"
#include "TreeGenerator.h"

AGeneratedTree::AGeneratedTree()
{
    PrimaryActorTick.bCanEverTick = false;

    MeshComponent = CreateDefaultSubobject<UDynamicMeshComponent>(TEXT("MeshComponent"));
    RootComponent = MeshComponent;

    // The array is now strictly used for structural growth (Length, Angles, Density)
    // The width/radius is now gracefully handled by the global properties above!

    FTreeItLevelParams Trunk;
    Trunk.Length = 300.f; Trunk.Segments = 10; Trunk.BranchesSpawned = 5;
    Trunk.GravityBend = 0.0f; Trunk.Jitter = 0.1f;
    BranchLevels.Add(Trunk);

    FTreeItLevelParams Branches;
    Branches.Length = 160.f; Branches.Segments = 7; Branches.BranchesSpawned = 3;
    Branches.BranchAngle = 55.f; Branches.GravityBend = 0.05f; Branches.Jitter = 0.2f;
    BranchLevels.Add(Branches);

    FTreeItLevelParams Twigs;
    Twigs.Length = 60.f; Twigs.Segments = 4; Twigs.BranchesSpawned = 0;
    Twigs.BranchAngle = 40.f; Twigs.GravityBend = 0.08f; Twigs.Jitter = 0.3f;
    BranchLevels.Add(Twigs);
}

void AGeneratedTree::BeginPlay()
{
    Super::BeginPlay();

    UTreeGenerator::GenerateTreeIt(
        MeshComponent, Seed, TrunkRadius, BranchRadiusScale, GlobalTaper,
        TrunkFlare, TrunkFlareHeight, TrunkRidgeFrequency, TrunkRidgeIntensity, BaseRadialResolution,
        BranchLevels, LeafSize, LeafCards, TrunkMaterial, LeavesMaterial
    );
}

void AGeneratedTree::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);

    UTreeGenerator::GenerateTreeIt(
        MeshComponent, Seed, TrunkRadius, BranchRadiusScale, GlobalTaper,
        TrunkFlare, TrunkFlareHeight, TrunkRidgeFrequency, TrunkRidgeIntensity, BaseRadialResolution,
        BranchLevels, LeafSize, LeafCards, TrunkMaterial, LeavesMaterial
    );
}