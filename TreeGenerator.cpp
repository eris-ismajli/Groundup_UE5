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

struct FLeafTriangle
{
    int32 TriID;
    FVector3d ClumpCenter;
};

static void ExtrudeSpline(FDynamicMesh3& TrunkMesh, const TArray<FBranchNode>& Nodes, int32 Slices,
    float UVScale, int32 RidgeFreq, float RidgeIntensity)
{
    FDynamicMeshUVOverlay* UVOverlay = TrunkMesh.Attributes() ? TrunkMesh.Attributes()->GetUVLayer(0) : nullptr;

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
            Rings[i][s] = TrunkMesh.AppendVertex(Node.Pos + Offset * FinalRadius);
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

            int32 Tri1 = TrunkMesh.AppendTriangle(A, B, C);
            int32 Tri2 = TrunkMesh.AppendTriangle(B, D, C);

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

static void AddLeafCards(FDynamicMesh3& LeavesMesh, FRandomStream& Rand, const FVector3d& SplineCenter,
    const FVector3d& BranchDir, const FVector3d& BranchX, const FVector3d& BranchY, float LocalRadius,
    float Length, float WidthScale, int32 NumCards, float Pitch, float PitchVar, float GravityBend,
    TArray<FLeafTriangle>& OutLeafTriangles)
{
    FDynamicMeshUVOverlay* UVOverlay = LeavesMesh.Attributes() ? LeavesMesh.Attributes()->GetUVLayer(0) : nullptr;

    float ClumpRadius = Length;
    float BaseCardLength = Length * 0.7f;
    float BaseCardHalfLength = BaseCardLength * 0.5f;
    float BaseCardHalfWidth = (BaseCardLength * WidthScale) * 0.5f;

    FVector3d ClumpCenter = SplineCenter;

    auto AddQuad = [&](const FVector3d& Center, const FVector3d& Right, const FVector3d& Up, float HalfW, float HalfH)
        {
            FVector3d V0 = Center - Right * HalfW - Up * HalfH;
            FVector3d V1 = Center + Right * HalfW - Up * HalfH;
            FVector3d V2 = Center + Right * HalfW + Up * HalfH;
            FVector3d V3 = Center - Right * HalfW + Up * HalfH;

            int32 Vert0 = LeavesMesh.AppendVertex(V0);
            int32 Vert1 = LeavesMesh.AppendVertex(V1);
            int32 Vert2 = LeavesMesh.AppendVertex(V2);
            int32 Vert3 = LeavesMesh.AppendVertex(V3);

            int32 Tri1 = LeavesMesh.AppendTriangle(Vert0, Vert1, Vert2);
            int32 Tri2 = LeavesMesh.AppendTriangle(Vert0, Vert2, Vert3);

            if (Tri1 >= 0) OutLeafTriangles.Add(FLeafTriangle{ Tri1, ClumpCenter });
            if (Tri2 >= 0) OutLeafTriangles.Add(FLeafTriangle{ Tri2, ClumpCenter });

            if (UVOverlay && Tri1 >= 0 && Tri2 >= 0)
            {
                int32 UV0 = UVOverlay->AppendElement(FVector2f(0.f, 1.f));
                int32 UV1 = UVOverlay->AppendElement(FVector2f(1.f, 1.f));
                int32 UV2 = UVOverlay->AppendElement(FVector2f(1.f, 0.f));
                int32 UV3 = UVOverlay->AppendElement(FVector2f(0.f, 0.f));

                UVOverlay->SetTriangle(Tri1, FIndex3i(UV0, UV1, UV2));
                UVOverlay->SetTriangle(Tri2, FIndex3i(UV0, UV2, UV3));
            }
        };

    // Use a Fibonacci Sphere algorithm for completely uniform, gapless distribution across the surface
    float GoldenRatio = (1.0f + FMath::Sqrt(5.0f)) / 2.0f;
    float AngleIncrement = PI * 2.0f * GoldenRatio;

    // Pre-calculate the maximum allowed distance so cards NEVER stick out of the clump radius
    float MaxDist = FMath::Max(0.0f, ClumpRadius - BaseCardHalfLength * 1.15f);

    for (int32 i = 0; i < NumCards; ++i)
    {
        // 1. Calculate Fibonacci sphere coordinates for perfectly uniform angular placement
        float t = (float)i / (float)NumCards;
        float Z = 1.0f - (t * 2.0f); // Z maps from 1 to -1
        float RadiusAtZ = FMath::Sqrt(FMath::Max(0.0f, 1.0f - Z * Z));
        float Theta = AngleIncrement * i;

        FVector3d SphereDir(RadiusAtZ * FMath::Cos(Theta), RadiusAtZ * FMath::Sin(Theta), Z);

        // Add tiny positional jitter to hide the math, but keep the structural uniformity
        FVector3d Jitter(Rand.FRandRange(-0.05f, 0.05f), Rand.FRandRange(-0.05f, 0.05f), Rand.FRandRange(-0.05f, 0.05f));
        FVector3d Outward = (SphereDir + Jitter).GetSafeNormal();

        // Push leaves towards the outer shell for a solid outline, with just enough inner density
        float DepthFrac = FMath::Pow(Rand.FRand(), 0.35f);
        float Dist = FMath::Lerp(MaxDist * 0.3f, MaxDist, DepthFrac);

        FVector3d Offset = Outward * Dist;
        Offset.Z *= (1.0f - GravityBend * 0.3f);
        FVector3d CardCenter = ClumpCenter + Offset;

        // 2. Align perfectly to the surface normal to round out the puff
        FVector3d UpRef(0, 0, 1);
        if (FMath::Abs(Outward.Z) > 0.98f) { UpRef = BranchX; }
        FVector3d TangentRight = Outward.Cross(UpRef).GetSafeNormal();
        FVector3d TangentUp = TangentRight.Cross(Outward).GetSafeNormal();

        float RandomPitch = FMath::DegreesToRadians(Pitch + Rand.FRandRange(-PitchVar, PitchVar));
        FVector3d TiltedOutward = (Outward * FMath::Cos(RandomPitch) + TangentUp * FMath::Sin(RandomPitch)).GetSafeNormal();
        FVector3d TiltedUp = TangentRight.Cross(TiltedOutward).GetSafeNormal();

        float RollAngle = Rand.FRandRange(0.0f, 2.0f * PI);
        FVector3d FinalRight = TangentRight * FMath::Cos(RollAngle) + TiltedUp * FMath::Sin(RollAngle);
        FVector3d FinalUp = TiltedUp * FMath::Cos(RollAngle) - TangentRight * FMath::Sin(RollAngle);
        FVector3d FinalNormal = FinalRight.Cross(FinalUp).GetSafeNormal();

        if (GravityBend > 0.001f)
        {
            FVector3d WorldDown(0, 0, -1);
            FinalUp = FMath::Lerp(FinalUp, WorldDown, GravityBend * 0.8f).GetSafeNormal();
            FinalRight = FinalUp.Cross(FinalNormal).GetSafeNormal();
            FinalNormal = FinalRight.Cross(FinalUp).GetSafeNormal();
        }

        // Tighter scale limits so random huge leaves don't break the silhouette
        float ScaleJitter = Rand.FRandRange(0.85f, 1.15f);
        float CurHalfW = BaseCardHalfWidth * ScaleJitter;
        float CurHalfH = BaseCardHalfLength * ScaleJitter;

        // 3. Generate Tri-plane Tuft
        float Cos60 = 0.5f;
        float Sin60 = 0.8660254f;

        AddQuad(CardCenter, FinalRight, FinalUp, CurHalfW, CurHalfH);

        FVector3d Right2 = FinalRight * Cos60 - FinalNormal * Sin60;
        AddQuad(CardCenter, Right2, FinalUp, CurHalfW, CurHalfH);

        FVector3d Right3 = FinalRight * (-Cos60) - FinalNormal * Sin60;
        AddQuad(CardCenter, Right3, FinalUp, CurHalfW, CurHalfH);
    }
}

static void AddShadowProxy(FDynamicMesh3& ProxyMesh, const FVector3d& ClumpCenter, float ProxySize)
{
    // --- Generate a subdivided icosahedron (80 triangles) for a perfectly round silhouette ---

    float t = (1.0f + FMath::Sqrt(5.0f)) / 2.0f;

    // Base 12 vertices of a regular icosahedron (normalised)
    TArray<FVector3d> BaseVerts = {
        FVector3d(-1,  t,  0), FVector3d(1,  t,  0), FVector3d(-1, -t,  0), FVector3d(1, -t,  0),
        FVector3d(0, -1,  t), FVector3d(0,  1,  t), FVector3d(0, -1, -t), FVector3d(0,  1, -t),
        FVector3d(t,  0, -1), FVector3d(t,  0,  1), FVector3d(-t,  0, -1), FVector3d(-t,  0,  1)
    };
    for (FVector3d& v : BaseVerts) v.Normalize();

    // 20 triangles of the base icosahedron
    TArray<FIndex3i> BaseTris = {
        {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
        {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
        {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
        {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
    };

    // We'll collect unique vertices (midpoints) as we subdivide
    TMap<uint64, int32> MidpointMap;
    auto GetMidpoint = [&](int32 v1, int32 v2, const TArray<FVector3d>& Verts, TArray<FVector3d>& OutVerts)
        {
            uint64 Key = (uint64)FMath::Min(v1, v2) | ((uint64)FMath::Max(v1, v2) << 32);
            if (int32* Found = MidpointMap.Find(Key))
            {
                return *Found;
            }
            FVector3d Mid = (Verts[v1] + Verts[v2]) * 0.5;
            Mid.Normalize(); // keep on sphere surface
            int32 NewIdx = OutVerts.Add(Mid);
            MidpointMap.Add(Key, NewIdx);
            return NewIdx;
        };

    // Subdivide once: each triangle -> 4 triangles
    TArray<FVector3d> SphereVerts = BaseVerts; // copy base
    TArray<FIndex3i> SubTris;
    for (const FIndex3i& Tri : BaseTris)
    {
        int32 a = GetMidpoint(Tri.A, Tri.B, BaseVerts, SphereVerts);
        int32 b = GetMidpoint(Tri.B, Tri.C, BaseVerts, SphereVerts);
        int32 c = GetMidpoint(Tri.C, Tri.A, BaseVerts, SphereVerts);

        SubTris.Add(FIndex3i(Tri.A, a, c));
        SubTris.Add(FIndex3i(Tri.B, b, a));
        SubTris.Add(FIndex3i(Tri.C, c, b));
        SubTris.Add(FIndex3i(a, b, c));
    }

    // Add vertices to the mesh (scaled to ProxySize, centered on ClumpCenter)
    ProxyMesh.EnableVertexNormals(FVector3f::ZeroVector);
    TArray<int32> VertIDs;
    VertIDs.SetNum(SphereVerts.Num());
    for (int32 i = 0; i < SphereVerts.Num(); ++i)
    {
        FVector3d Pos = ClumpCenter + SphereVerts[i] * ProxySize;
        VertIDs[i] = ProxyMesh.AppendVertex(Pos);
        ProxyMesh.SetVertexNormal(VertIDs[i], FVector3f(SphereVerts[i])); // smooth normals
    }

    // Add triangles
    TArray<int32> TriIDs;
    for (const FIndex3i& Tri : SubTris)
    {
        int32 TriID = ProxyMesh.AppendTriangle(VertIDs[Tri.A], VertIDs[Tri.B], VertIDs[Tri.C]);
        if (TriID >= 0)
        {
            TriIDs.Add(TriID);
        }
    }

    // UVs (unchanged logic, simply applied to all new triangles)
    if (ProxyMesh.Attributes())
    {
        FDynamicMeshUVOverlay* UV0 = ProxyMesh.Attributes()->GetUVLayer(0);
        FDynamicMeshUVOverlay* UV1 = ProxyMesh.Attributes()->GetUVLayer(1);
        FDynamicMeshUVOverlay* UV2 = ProxyMesh.Attributes()->GetUVLayer(2);

        for (int32 TriID : TriIDs)
        {
            if (UV0)
            {
                int32 U = UV0->AppendElement(FVector2f(0.5f, 0.5f));
                UV0->SetTriangle(TriID, FIndex3i(U, U, U));
            }
            if (UV1 && UV2)
            {
                int32 CenterXY_Idx = UV1->AppendElement(FVector2f(ClumpCenter.X, ClumpCenter.Y));
                UV1->SetTriangle(TriID, FIndex3i(CenterXY_Idx, CenterXY_Idx, CenterXY_Idx));

                int32 CenterZSize_Idx = UV2->AppendElement(FVector2f(ClumpCenter.Z, ProxySize));
                UV2->SetTriangle(TriID, FIndex3i(CenterZSize_Idx, CenterZSize_Idx, CenterZSize_Idx));
            }
        }
    }
}

static void GenerateBranchTreeItLevel(
    FDynamicMesh3& TrunkMesh, FDynamicMesh3& LeavesMesh, FDynamicMesh3& ProxyMesh, FRandomStream& Rand, int32 Level,
    const FVector3d& StartPos, const FVector3d& StartDir,
    float StartRadius, float GlobalTrunkRadius, float BranchRadiusScale, float GlobalTaper,
    float TrunkFlare, float TrunkFlareHeight, int32 RidgeFreq, float RidgeIntensity, int32 BaseRes,
    const TArray<FTreeItLevelParams>& Levels, float LeafLength, float LeafWidthScale, int32 LeafCards,
    float LeafPitch, float LeafPitchVar, float LeafGravity,
    TArray<FLeafTriangle>& OutLeafTriangles)
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

    ExtrudeSpline(TrunkMesh, Nodes, AlgorithmicResolution, 0.01f, AppliedRidges, AppliedRidgeInt);

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
                TrunkMesh, LeavesMesh, ProxyMesh, Rand, Level + 1, ChildPos, ChildDir,
                ChildStartRadius, GlobalTrunkRadius, BranchRadiusScale, GlobalTaper,
                TrunkFlare, TrunkFlareHeight, RidgeFreq, RidgeIntensity, BaseRes,
                Levels, LeafLength, LeafWidthScale, LeafCards, LeafPitch, LeafPitchVar, LeafGravity,
                OutLeafTriangles
            );
        }
    }

    for (int32 i = 0; i < Params.LeavesSpawned; ++i)
    {
        float SpawnT = Rand.FRandRange(0.1f, 0.95f);
        float FloatIdx = SpawnT * Segs;
        int32 NodeIdx = FMath::Clamp(FMath::FloorToInt(FloatIdx), 0, Segs - 1);
        float Alpha = FloatIdx - NodeIdx;

        FVector3d LeafCenterPos = FMath::Lerp(Nodes[NodeIdx].Pos, Nodes[NodeIdx + 1].Pos, Alpha);
        FVector3d ParentDir = FMath::Lerp(Nodes[NodeIdx].Dir, Nodes[NodeIdx + 1].Dir, Alpha).GetSafeNormal();
        float ParentRadiusAtSpawn = FMath::Lerp(Nodes[NodeIdx].Radius, Nodes[NodeIdx + 1].Radius, Alpha);

        FVector3d PX, PY;
        ParentDir.FindBestAxisVectors(PX, PY);

        AddLeafCards(LeavesMesh, Rand, LeafCenterPos, ParentDir, PX, PY, ParentRadiusAtSpawn,
            LeafLength, LeafWidthScale, LeafCards, LeafPitch, LeafPitchVar, LeafGravity, OutLeafTriangles);

        AddShadowProxy(ProxyMesh, LeafCenterPos, LeafLength);
    }

    if (Level == Levels.Num() - 1)
    {
        AddLeafCards(LeavesMesh, Rand, Nodes.Last().Pos, Nodes.Last().Dir, Nodes.Last().X, Nodes.Last().Y, Nodes.Last().Radius,
            LeafLength, LeafWidthScale, LeafCards, LeafPitch, LeafPitchVar, LeafGravity, OutLeafTriangles);

        AddShadowProxy(ProxyMesh, Nodes.Last().Pos, LeafLength);
    }
}

void UTreeGenerator::GenerateTreeIt(
    UDynamicMeshComponent* TrunkComponent, UDynamicMeshComponent* LeavesComponent, UDynamicMeshComponent* ProxyComponent,
    int32 Seed, float TrunkRadius, float BranchRadiusScale, float GlobalTaper,
    float TrunkFlare, float TrunkFlareHeight, int32 TrunkRidgeFrequency, float TrunkRidgeIntensity, int32 BaseRadialResolution,
    TArray<FTreeItLevelParams> BranchLevels, float LeafLength, float LeafWidthScale, int32 LeafCards,
    float LeafPitch, float LeafPitchVariance, float LeafGravityBend,
    UMaterialInterface* BarkMaterial, UMaterialInterface* LeafMaterial, UMaterialInterface* ShadowMaterial)
{
    if (!TrunkComponent || !LeavesComponent || !ProxyComponent || BranchLevels.Num() == 0) return;

    FRandomStream Rand(Seed);

    FDynamicMesh3 TrunkMesh;
    TrunkMesh.EnableAttributes();
    TrunkMesh.Attributes()->SetNumNormalLayers(1);
    TrunkMesh.Attributes()->SetNumUVLayers(1);

    FDynamicMesh3 LeavesMesh;
    LeavesMesh.EnableAttributes();
    LeavesMesh.Attributes()->SetNumNormalLayers(1);
    LeavesMesh.Attributes()->SetNumUVLayers(1);

    FDynamicMesh3 ProxyMesh;
    ProxyMesh.EnableAttributes();
    ProxyMesh.Attributes()->SetNumNormalLayers(1);
    ProxyMesh.Attributes()->SetNumUVLayers(3);

    FVector3d RootPos(0, 0, 0);
    FVector3d RootDir(0, 0, 1);

    TArray<FLeafTriangle> LeafTriangles;

    GenerateBranchTreeItLevel(
        TrunkMesh, LeavesMesh, ProxyMesh, Rand, 0, RootPos, RootDir,
        TrunkRadius, TrunkRadius, BranchRadiusScale, GlobalTaper,
        TrunkFlare, TrunkFlareHeight, TrunkRidgeFrequency, TrunkRidgeIntensity, BaseRadialResolution,
        BranchLevels, LeafLength, LeafWidthScale, LeafCards, LeafPitch, LeafPitchVariance, LeafGravityBend,
        LeafTriangles
    );

    FMeshNormals::QuickComputeVertexNormals(TrunkMesh);
    FMeshNormals::QuickComputeVertexNormals(ProxyMesh);
    FMeshNormals::QuickComputeVertexNormals(LeavesMesh);

    FDynamicMeshNormalOverlay* NormalOverlay = LeavesMesh.Attributes()->GetNormalLayer(0);
    if (NormalOverlay)
    {
        for (const FLeafTriangle& LeafTri : LeafTriangles)
        {
            FIndex3i TriVerts = LeavesMesh.GetTriangle(LeafTri.TriID);
            FVector3d V0 = LeavesMesh.GetVertex(TriVerts.A);
            FVector3d V1 = LeavesMesh.GetVertex(TriVerts.B);
            FVector3d V2 = LeavesMesh.GetVertex(TriVerts.C);

            FVector3f N0 = (FVector3f)(V0 - LeafTri.ClumpCenter).GetSafeNormal();
            FVector3f N1 = (FVector3f)(V1 - LeafTri.ClumpCenter).GetSafeNormal();
            FVector3f N2 = (FVector3f)(V2 - LeafTri.ClumpCenter).GetSafeNormal();

            int32 Elem0 = NormalOverlay->AppendElement(N0);
            int32 Elem1 = NormalOverlay->AppendElement(N1);
            int32 Elem2 = NormalOverlay->AppendElement(N2);

            NormalOverlay->SetTriangle(LeafTri.TriID, FIndex3i(Elem0, Elem1, Elem2));
        }
    }

    TrunkComponent->SetMesh(MoveTemp(TrunkMesh));
    LeavesComponent->SetMesh(MoveTemp(LeavesMesh));
    ProxyComponent->SetMesh(MoveTemp(ProxyMesh));

    if (BarkMaterial) TrunkComponent->SetMaterial(0, BarkMaterial);
    if (LeafMaterial) LeavesComponent->SetMaterial(0, LeafMaterial);
    if (ShadowMaterial) ProxyComponent->SetMaterial(0, ShadowMaterial);

    TrunkComponent->NotifyMeshUpdated();
    LeavesComponent->NotifyMeshUpdated();
    ProxyComponent->NotifyMeshUpdated();
}