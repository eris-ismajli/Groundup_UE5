#include "TreeGenerator.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "DynamicMesh/MeshNormals.h"
#include "Components/DynamicMeshComponent.h"
#include "Materials/MaterialInterface.h"
#include "Engine/Engine.h"

using namespace UE::Geometry;

struct FBranchNode
{
    FVector3d Pos;
    FVector3d Dir;
    FVector3d X;
    FVector3d Y;
    float Radius;
};

// ----------------------------------------------------------------------
// Extrude Spline 
// ----------------------------------------------------------------------
static void ExtrudeSpline(FDynamicMesh3& Mesh, const TArray<FBranchNode>& Nodes, int32 Slices,
    float UVScale, FDynamicMeshMaterialAttribute* MatAttr, FDynamicMeshUVOverlay* UVOverlay,
    int32 RidgeFreq, float RidgeIntensity)
{
    int32 NumNodes = Nodes.Num();
    if (NumNodes < 2) return;

    TArray<TArray<int32>> Rings;
    Rings.SetNum(NumNodes);
    TArray<TArray<int32>> UVRings;
    UVRings.SetNum(NumNodes);

    float CurrentV = 0.0f;

    for (int32 i = 0; i < NumNodes; i++)
    {
        const FBranchNode& Node = Nodes[i];
        Rings[i].SetNum(Slices);
        UVRings[i].SetNum(Slices + 1);

        if (i > 0) CurrentV += FVector3d::Distance(Nodes[i].Pos, Nodes[i - 1].Pos) * UVScale;

        for (int32 s = 0; s < Slices; s++)
        {
            double Angle = (double)s / (double)Slices * 2.0 * PI;
            float RadiusOffset = 0.0f;

            if (RidgeFreq > 0 && RidgeIntensity > 0.0f)
            {
                RadiusOffset = FMath::Cos(Angle * RidgeFreq) * (Node.Radius * RidgeIntensity);
            }

            float FinalRadius = FMath::Max(0.1f, Node.Radius + RadiusOffset);

            FVector3d Offset = Node.X * FMath::Cos(Angle) + Node.Y * FMath::Sin(Angle);
            Rings[i][s] = Mesh.AppendVertex(Node.Pos + Offset * FinalRadius);
        }

        for (int32 s = 0; s <= Slices; s++)
        {
            float U = (float)s / (float)Slices;
            UVRings[i][s] = UVOverlay ? UVOverlay->AppendElement(FVector2f(U, CurrentV)) : -1;
        }
    }

    for (int32 i = 0; i < NumNodes - 1; i++)
    {
        for (int32 s = 0; s < Slices; s++)
        {
            int32 NextS = (s + 1) % Slices;
            int32 A = Rings[i][s];
            int32 B = Rings[i][NextS];
            int32 C = Rings[i + 1][s];
            int32 D = Rings[i + 1][NextS];

            int32 Tri1 = Mesh.AppendTriangle(A, B, C);
            int32 Tri2 = Mesh.AppendTriangle(B, D, C);

            if (MatAttr)
            {
                if (Tri1 >= 0) MatAttr->SetValue(Tri1, 0);
                if (Tri2 >= 0) MatAttr->SetValue(Tri2, 0);
            }

            if (UVOverlay && Tri1 >= 0 && Tri2 >= 0)
            {
                int32 uvA = UVRings[i][s];
                int32 uvB = UVRings[i][s + 1];
                int32 uvC = UVRings[i + 1][s];
                int32 uvD = UVRings[i + 1][s + 1];

                UVOverlay->SetTriangle(Tri1, FIndex3i(uvA, uvB, uvC));
                UVOverlay->SetTriangle(Tri2, FIndex3i(uvB, uvD, uvC));
            }
        }
    }
}

// ----------------------------------------------------------------------
// Leaf Spawning (Now dynamic surface snapping!)
// ----------------------------------------------------------------------
static void AddLeafCards(FDynamicMesh3& Mesh, FRandomStream& Rand, const FVector3d& SplineCenter,
    const FVector3d& BranchDir, const FVector3d& BranchX, const FVector3d& BranchY, float LocalRadius,
    float Length, float WidthScale, int32 NumCards, float Pitch, float PitchVar, float GravityBend,
    FDynamicMeshMaterialAttribute* MatAttr, FDynamicMeshUVOverlay* UVOverlay)
{
    float HalfWidth = (Length * WidthScale) * 0.5f;

    for (int32 i = 0; i < NumCards; ++i)
    {
        double Angle = ((double)i / NumCards) * 2.0 * PI + Rand.FRandRange(-0.5, 0.5);
        FVector3d Outward = BranchX * FMath::Cos(Angle) + BranchY * FMath::Sin(Angle);

        // FIX: Pushes the Anchor to the bark surface! (0.7 multiplier embeds the stem slightly so there are no floating gaps)
        FVector3d SurfaceAnchor = SplineCenter + Outward * (LocalRadius * 0.7f);

        float RandomPitch = FMath::DegreesToRadians(Pitch + Rand.FRandRange(-PitchVar, PitchVar));
        FVector3d LeafForward = (Outward * FMath::Cos(RandomPitch) + BranchDir * FMath::Sin(RandomPitch)).GetSafeNormal();

        FVector3d WorldDown(0, 0, -1);
        LeafForward = FMath::Lerp(LeafForward, WorldDown, GravityBend).GetSafeNormal();

        FVector3d UpRef = FVector3d(0, 0, 1);
        if (FMath::Abs(LeafForward.Z) > 0.98f)
        {
            UpRef = BranchX;
        }
        FVector3d LeafRight = LeafForward.Cross(UpRef).GetSafeNormal();

        FVector3d V0 = SurfaceAnchor - LeafRight * HalfWidth;                     // Bottom Left (Stem)
        FVector3d V1 = SurfaceAnchor + LeafRight * HalfWidth;                     // Bottom Right (Stem)
        FVector3d V2 = SurfaceAnchor + LeafRight * HalfWidth + LeafForward * Length; // Top Right (Tip)
        FVector3d V3 = SurfaceAnchor - LeafRight * HalfWidth + LeafForward * Length; // Top Left (Tip)

        int32 Vert0 = Mesh.AppendVertex(V0);
        int32 Vert1 = Mesh.AppendVertex(V1);
        int32 Vert2 = Mesh.AppendVertex(V2);
        int32 Vert3 = Mesh.AppendVertex(V3);

        int32 Tri1 = Mesh.AppendTriangle(Vert0, Vert1, Vert2);
        int32 Tri2 = Mesh.AppendTriangle(Vert0, Vert2, Vert3);

        if (MatAttr)
        {
            if (Tri1 >= 0) MatAttr->SetValue(Tri1, 1);
            if (Tri2 >= 0) MatAttr->SetValue(Tri2, 1);
        }

        if (UVOverlay && Tri1 >= 0 && Tri2 >= 0)
        {
            int32 UV0 = UVOverlay->AppendElement(FVector2f(0.f, 1.f));
            int32 UV1 = UVOverlay->AppendElement(FVector2f(1.f, 1.f));
            int32 UV2 = UVOverlay->AppendElement(FVector2f(1.f, 0.f));
            int32 UV3 = UVOverlay->AppendElement(FVector2f(0.f, 0.f));

            UVOverlay->SetTriangle(Tri1, FIndex3i(UV0, UV1, UV2));
            UVOverlay->SetTriangle(Tri2, FIndex3i(UV0, UV2, UV3));
        }
    }
}

// ----------------------------------------------------------------------
// Core Algorithmic Generation Engine
// ----------------------------------------------------------------------
static void GenerateBranchTreeItLevel(
    FDynamicMesh3& Mesh, FRandomStream& Rand, int32 Level,
    const FVector3d& StartPos, const FVector3d& StartDir,
    float StartRadius, float GlobalTrunkRadius, float BranchRadiusScale, float GlobalTaper,
    float TrunkFlare, float TrunkFlareHeight, int32 RidgeFreq, float RidgeIntensity, int32 BaseRes,
    const TArray<FTreeItLevelParams>& Levels, float LeafLength, float LeafWidthScale, int32 LeafCards,
    float LeafPitch, float LeafPitchVar, float LeafGravity,
    FDynamicMeshMaterialAttribute* MatAttr, FDynamicMeshUVOverlay* UVOverlay)
{
    if (Level >= Levels.Num()) return;
    const FTreeItLevelParams& Params = Levels[Level];

    TArray<FBranchNode> Nodes;
    FVector3d CurrentPos = StartPos;
    FVector3d CurrentDir = StartDir.GetSafeNormal();
    FVector3d CurrentX, CurrentY;
    CurrentDir.FindBestAxisVectors(CurrentX, CurrentY);

    float ActualLength = FMath::Max(Params.Length + Rand.FRandRange(-Params.LengthVariance, Params.LengthVariance), 5.0f);
    int32 Segs = FMath::Max(1, Params.Segments);
    float SegmentLength = ActualLength / (float)Segs;

    for (int32 i = 0; i <= Segs; ++i)
    {
        float t = (float)i / Segs;
        FBranchNode Node;

        Node.Pos = CurrentPos;
        Node.Dir = CurrentDir;
        Node.X = CurrentX;
        Node.Y = CurrentY;

        float BaseRadius = FMath::Lerp(StartRadius, StartRadius * FMath::Clamp(GlobalTaper, 0.f, 1.f), t);

        if (Level == 0 && TrunkFlare > 0.0f && TrunkFlareHeight > 0.01f)
        {
            float FlareAlpha = FMath::Clamp(1.0f - (t / TrunkFlareHeight), 0.0f, 1.0f);
            FlareAlpha = FlareAlpha * FlareAlpha;
            BaseRadius += (StartRadius * TrunkFlare) * FlareAlpha;
        }

        Node.Radius = BaseRadius;
        Nodes.Add(Node);

        if (i < Segs)
        {
            FVector3d JitterOffset(Rand.FRandRange(-1.f, 1.f), Rand.FRandRange(-1.f, 1.f), Rand.FRandRange(-1.f, 1.f));
            FVector3d WanderingDir = (CurrentDir + JitterOffset * Params.Jitter).GetSafeNormal();

            FVector3d GravityOffset(0, 0, -Params.GravityBend);
            FVector3d NextDir = (WanderingDir + GravityOffset).GetSafeNormal();

            FQuat Rotation = FQuat::FindBetweenNormals(FVector(CurrentDir), FVector(NextDir));
            CurrentX = FVector3d(Rotation.RotateVector(FVector(CurrentX)));
            CurrentY = FVector3d(Rotation.RotateVector(FVector(CurrentY)));

            CurrentDir = NextDir;
            CurrentPos += CurrentDir * SegmentLength;
        }
    }

    float RadiusRatio = FMath::Clamp(StartRadius / GlobalTrunkRadius, 0.1f, 1.0f);
    int32 AlgorithmicResolution = FMath::Max(3, FMath::RoundToInt(BaseRes * RadiusRatio));

    int32 AppliedRidges = (Level == 0) ? RidgeFreq : 0;
    float AppliedRidgeInt = (Level == 0) ? RidgeIntensity : 0.0f;

    ExtrudeSpline(Mesh, Nodes, AlgorithmicResolution, 0.01f, MatAttr, UVOverlay, AppliedRidges, AppliedRidgeInt);


    // 1. --- SPAWN CHILD BRANCHES ---
    if (Level + 1 < Levels.Num())
    {
        for (int32 i = 0; i < Params.BranchesSpawned; ++i)
        {
            float SpawnT = Rand.FRandRange(0.25f, 0.95f);
            float FloatIdx = SpawnT * Segs;
            int32 NodeIdx = FMath::Clamp(FMath::FloorToInt(FloatIdx), 0, Segs - 1);
            float Alpha = FloatIdx - NodeIdx;

            FVector3d ChildPos = FMath::Lerp(Nodes[NodeIdx].Pos, Nodes[NodeIdx + 1].Pos, Alpha);
            FVector3d ParentDir = FMath::Lerp(Nodes[NodeIdx].Dir, Nodes[NodeIdx + 1].Dir, Alpha).GetSafeNormal();
            float ParentRadiusAtSpawn = FMath::Lerp(Nodes[NodeIdx].Radius, Nodes[NodeIdx + 1].Radius, Alpha);
            float ChildStartRadius = ParentRadiusAtSpawn * BranchRadiusScale;

            FVector3d PX, PY;
            ParentDir.FindBestAxisVectors(PX, PY);
            double Roll = Rand.FRandRange(0.0, 2.0 * PI);
            FVector3d OutwardDir = PX * FMath::Cos(Roll) + PY * FMath::Sin(Roll);

            float RadAngle = FMath::DegreesToRadians(Params.BranchAngle + Rand.FRandRange(-15.f, 15.f));
            FVector3d ChildDir = (ParentDir * FMath::Cos(RadAngle) + OutwardDir * FMath::Sin(RadAngle)).GetSafeNormal();

            GenerateBranchTreeItLevel(
                Mesh, Rand, Level + 1, ChildPos, ChildDir,
                ChildStartRadius, GlobalTrunkRadius, BranchRadiusScale, GlobalTaper,
                TrunkFlare, TrunkFlareHeight, RidgeFreq, RidgeIntensity, BaseRes,
                Levels, LeafLength, LeafWidthScale, LeafCards, LeafPitch, LeafPitchVar, LeafGravity, MatAttr, UVOverlay
            );
        }
    }

    // 2. --- SPAWN BODY LEAVES (The "Puffy" implementation!) ---
    for (int32 i = 0; i < Params.LeavesSpawned; ++i)
    {
        // Pick a random spot along the body of the branch
        float SpawnT = Rand.FRandRange(0.1f, 0.95f);
        float FloatIdx = SpawnT * Segs;
        int32 NodeIdx = FMath::Clamp(FMath::FloorToInt(FloatIdx), 0, Segs - 1);
        float Alpha = FloatIdx - NodeIdx;

        FVector3d LeafCenterPos = FMath::Lerp(Nodes[NodeIdx].Pos, Nodes[NodeIdx + 1].Pos, Alpha);
        FVector3d ParentDir = FMath::Lerp(Nodes[NodeIdx].Dir, Nodes[NodeIdx + 1].Dir, Alpha).GetSafeNormal();
        float ParentRadiusAtSpawn = FMath::Lerp(Nodes[NodeIdx].Radius, Nodes[NodeIdx + 1].Radius, Alpha);

        FVector3d PX, PY;
        ParentDir.FindBestAxisVectors(PX, PY);

        AddLeafCards(Mesh, Rand, LeafCenterPos, ParentDir, PX, PY, ParentRadiusAtSpawn,
            LeafLength, LeafWidthScale, LeafCards, LeafPitch, LeafPitchVar, LeafGravity, MatAttr, UVOverlay);
    }

    // 3. --- SPAWN TIP LEAVES ---
    bool bIsLastLevel = (Level == Levels.Num() - 1);
    if (bIsLastLevel)
    {
        // Cap off the very end of the twig with a beautiful leaf cluster
        AddLeafCards(Mesh, Rand, Nodes.Last().Pos, Nodes.Last().Dir, Nodes.Last().X, Nodes.Last().Y, Nodes.Last().Radius,
            LeafLength, LeafWidthScale, LeafCards, LeafPitch, LeafPitchVar, LeafGravity, MatAttr, UVOverlay);
    }
}

void UTreeGenerator::GenerateTreeIt(UDynamicMeshComponent* DynamicMeshComponent,
    int32 Seed, float TrunkRadius, float BranchRadiusScale, float GlobalTaper,
    float TrunkFlare, float TrunkFlareHeight, int32 TrunkRidgeFrequency, float TrunkRidgeIntensity, int32 BaseRadialResolution,
    TArray<FTreeItLevelParams> BranchLevels, float LeafLength, float LeafWidthScale, int32 LeafCards,
    float LeafPitch, float LeafPitchVariance, float LeafGravityBend,
    UMaterialInterface* BarkMaterial, UMaterialInterface* LeafMaterial)
{
    if (!DynamicMeshComponent || BranchLevels.Num() == 0) return;

    FRandomStream Rand(Seed);
    FDynamicMesh3 Mesh;

    Mesh.EnableAttributes();
    FDynamicMeshAttributeSet* Attr = Mesh.Attributes();
    Attr->SetNumUVLayers(1);
    Attr->EnableMaterialID();
    Attr->SetNumNormalLayers(1);

    FDynamicMeshUVOverlay* UVOverlay = Attr->GetUVLayer(0);
    FDynamicMeshMaterialAttribute* MaterialIDAttribute = Attr->GetMaterialID();

    FVector3d RootPos(0, 0, 0);
    FVector3d RootDir(0, 0, 1);

    GenerateBranchTreeItLevel(
        Mesh, Rand, 0, RootPos, RootDir,
        TrunkRadius, TrunkRadius, BranchRadiusScale, GlobalTaper,
        TrunkFlare, TrunkFlareHeight, TrunkRidgeFrequency, TrunkRidgeIntensity, BaseRadialResolution,
        BranchLevels, LeafLength, LeafWidthScale, LeafCards, LeafPitch, LeafPitchVariance, LeafGravityBend,
        MaterialIDAttribute, UVOverlay
    );

    FMeshNormals::QuickComputeVertexNormals(Mesh);

    DynamicMeshComponent->SetMesh(MoveTemp(Mesh));

    if (BarkMaterial) DynamicMeshComponent->SetMaterial(0, BarkMaterial);
    if (LeafMaterial) DynamicMeshComponent->SetMaterial(1, LeafMaterial);

    DynamicMeshComponent->NotifyMeshUpdated();
}