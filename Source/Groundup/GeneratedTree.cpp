#include "GeneratedTree.h"
#include "Components/DynamicMeshComponent.h"
#include "TreeGenerator.h"

AGeneratedTree::AGeneratedTree()
{
    PrimaryActorTick.bCanEverTick = false;

    // 1. Trunk Component (Normal settings, standard shadowing)
    TrunkMeshComponent = CreateDefaultSubobject<UDynamicMeshComponent>(TEXT("TrunkMeshComponent"));
    RootComponent = TrunkMeshComponent;

    // 2. Leaves Component (NO SHADOWS for massive performance gain and zero self-shadowing)
    LeavesMeshComponent = CreateDefaultSubobject<UDynamicMeshComponent>(TEXT("LeavesMeshComponent"));
    LeavesMeshComponent->SetupAttachment(RootComponent);
    LeavesMeshComponent->SetCastShadow(false);

    // 3. Proxy Component (INVISIBLE but CASTS SHADOWS to preserve ground silhouette)
    ProxyMeshComponent = CreateDefaultSubobject<UDynamicMeshComponent>(TEXT("ProxyMeshComponent"));
    ProxyMeshComponent->SetupAttachment(RootComponent);
    ProxyMeshComponent->SetVisibility(false);
    ProxyMeshComponent->bCastHiddenShadow = true;

    // --- The Default Array now handles BODY LEAVES (LeavesSpawned) ---

    FTreeItLevelParams Trunk;
    Trunk.Length = 300.f; Trunk.Segments = 10; Trunk.BranchesSpawned = 5;
    Trunk.GravityBend = 0.0f; Trunk.Jitter = 0.1f;
    Trunk.LeavesSpawned = 0; // Trunks usually don't have leaves
    BranchLevels.Add(Trunk);

    FTreeItLevelParams Branches;
    Branches.Length = 160.f; Branches.Segments = 7; Branches.BranchesSpawned = 3;
    Branches.BranchAngle = 55.f; Branches.GravityBend = 0.05f; Branches.Jitter = 0.2f;
    Branches.LeavesSpawned = 10; // Fluffs up the main branches nicely
    BranchLevels.Add(Branches);

    FTreeItLevelParams Twigs;
    Twigs.Length = 60.f; Twigs.Segments = 4; Twigs.BranchesSpawned = 0;
    Twigs.BranchAngle = 40.f; Twigs.GravityBend = 0.08f; Twigs.Jitter = 0.3f;
    Twigs.LeavesSpawned = 15; // Makes the twig network incredibly puffy
    BranchLevels.Add(Twigs);
}

void AGeneratedTree::BeginPlay()
{
    Super::BeginPlay();

    UTreeGenerator::GenerateTreeIt(
        TrunkMeshComponent, LeavesMeshComponent, ProxyMeshComponent,
        Seed, TrunkRadius, BranchRadiusScale, GlobalTaper,
        TrunkFlare, TrunkFlareHeight, TrunkRidgeFrequency, TrunkRidgeIntensity, BaseRadialResolution,
        BranchLevels, LeafLength, LeafWidthScale, LeafCards, LeafPitch, LeafPitchVariance, LeafGravityBend,
        TrunkMaterial, LeavesMaterial, ShadowMaterial
    );
}

void AGeneratedTree::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);

    UTreeGenerator::GenerateTreeIt(
        TrunkMeshComponent, LeavesMeshComponent, ProxyMeshComponent,
        Seed, TrunkRadius, BranchRadiusScale, GlobalTaper,
        TrunkFlare, TrunkFlareHeight, TrunkRidgeFrequency, TrunkRidgeIntensity, BaseRadialResolution,
        BranchLevels, LeafLength, LeafWidthScale, LeafCards, LeafPitch, LeafPitchVariance, LeafGravityBend,
        TrunkMaterial, LeavesMaterial, ShadowMaterial
    );
}