// Fill out your copyright notice in the Description page of Project Settings.

#include "TreeGenerator.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "DynamicMesh/MeshNormals.h"
#include "Components/DynamicMeshComponent.h"
#include "Materials/MaterialInterface.h"
#include "Engine/Engine.h"

using namespace UE::Geometry;

// ----------------------------------------------------------------------
// Helper: ridged trunk cylinder (bark, material ID = 0)
// ----------------------------------------------------------------------
static void AddRidgedCylinder(FDynamicMesh3& Mesh, FRandomStream& Rand,
    const FVector3d& Start, const FVector3d& End,
    float RadiusStart, float RadiusEnd,
    int32 Slices = 12, int32 HeightSegments = 6,
    float NoiseStrength = 0.25f,
    FDynamicMeshMaterialAttribute* MaterialIDAttribute = nullptr,
    FDynamicMeshUVOverlay* UVOverlay = nullptr)
{
    FVector3d Dir = End - Start;
    double Height = Dir.Length();
    if (Height < KINDA_SMALL_NUMBER) return;
    Dir /= Height;

    FVector3d Z = Dir;
    FVector3d X, Y;
    Z.FindBestAxisVectors(X, Y);

    TArray<TArray<int32>> Rings;
    Rings.SetNum(HeightSegments + 1);

    for (int32 h = 0; h <= HeightSegments; h++)
    {
        double t = (double)h / (double)HeightSegments;
        FVector3d RingCenter = Start + Dir * (t * Height);
        double BaseRadius = FMath::Lerp(RadiusStart, RadiusEnd, t);

        Rings[h].SetNum(Slices);
        for (int32 s = 0; s < Slices; s++)
        {
            double Angle = (double)s / (double)Slices * 2.0 * PI;
            FVector3d RadialDir = X * FMath::Cos(Angle) + Y * FMath::Sin(Angle);
            double R = BaseRadius * (1.0 + Rand.FRandRange(-NoiseStrength, NoiseStrength));
            FVector3d VertexPos = RingCenter + RadialDir * R;

            int32 VertID = Mesh.AppendVertex(VertexPos);
            Rings[h][s] = VertID;

            if (UVOverlay)
            {
                float U = (float)s / (float)Slices;
                float V = (float)t;
                UVOverlay->AppendElement(FVector2f(U, V));
            }
        }
    }

    for (int32 h = 0; h < HeightSegments; h++)
    {
        for (int32 s = 0; s < Slices; s++)
        {
            int32 NextS = (s + 1) % Slices;
            int32 A = Rings[h][s];
            int32 B = Rings[h][NextS];
            int32 C = Rings[h + 1][s];
            int32 D = Rings[h + 1][NextS];

            int32 Tri1 = Mesh.AppendTriangle(A, B, C);
            int32 Tri2 = Mesh.AppendTriangle(B, D, C);

            if (MaterialIDAttribute)
            {
                MaterialIDAttribute->SetValue(Tri1, 0); // bark
                MaterialIDAttribute->SetValue(Tri2, 0);
            }
        }
    }
}

// ----------------------------------------------------------------------
// Helper: smooth branch cylinder (bark, material ID = 0)
// ----------------------------------------------------------------------
static void AddBranchCylinder(FDynamicMesh3& Mesh,
    const FVector3d& Start, const FVector3d& End,
    float RadiusStart, float RadiusEnd, int32 Slices = 8,
    FDynamicMeshMaterialAttribute* MaterialIDAttribute = nullptr,
    FDynamicMeshUVOverlay* UVOverlay = nullptr)
{
    FVector3d Dir = End - Start;
    double Height = Dir.Length();
    if (Height < KINDA_SMALL_NUMBER) return;
    Dir /= Height;

    FVector3d Z = Dir;
    FVector3d X, Y;
    Z.FindBestAxisVectors(X, Y);

    TArray<int32> Ring0, Ring1;
    for (int32 i = 0; i < Slices; i++)
    {
        double Angle = (double)i / (double)Slices * 2.0 * PI;
        FVector3d Offset = (X * FMath::Cos(Angle) + Y * FMath::Sin(Angle));
        Ring0.Add(Mesh.AppendVertex(Start + Offset * RadiusStart));
        Ring1.Add(Mesh.AppendVertex(End + Offset * RadiusEnd));

        if (UVOverlay)
        {
            float U = (float)i / (float)Slices;
            UVOverlay->AppendElement(FVector2f(U, 0.0f)); // for Ring0
            UVOverlay->AppendElement(FVector2f(U, 1.0f)); // for Ring1
        }
    }

    for (int32 i = 0; i < Slices; i++)
    {
        int32 Next = (i + 1) % Slices;
        int32 Tri1 = Mesh.AppendTriangle(Ring0[i], Ring1[Next], Ring1[i]);
        int32 Tri2 = Mesh.AppendTriangle(Ring0[i], Ring0[Next], Ring1[Next]);

        if (MaterialIDAttribute)
        {
            MaterialIDAttribute->SetValue(Tri1, 0);
            MaterialIDAttribute->SetValue(Tri2, 0);
        }
    }
}

// ----------------------------------------------------------------------
// Helper: leaf cluster sphere (leaves, material ID = 1)
// ----------------------------------------------------------------------
static void AddLeafSphere(FDynamicMesh3& Mesh, const FVector3d& Center, float Radius, int32 Segments = 8,
    FDynamicMeshMaterialAttribute* MaterialIDAttribute = nullptr,
    FDynamicMeshUVOverlay* UVOverlay = nullptr)
{
    int32 StartVert = Mesh.VertexCount();
    int32 LatSteps = Segments;
    int32 LongSteps = Segments * 2;

    for (int32 j = 0; j <= LatSteps; j++)
    {
        double Phi = PI * (double)j / (double)LatSteps;
        double SinPhi = FMath::Sin(Phi);
        double CosPhi = FMath::Cos(Phi);
        for (int32 i = 0; i <= LongSteps; i++)
        {
            double Theta = 2.0 * PI * (double)i / (double)LongSteps;
            double SinTheta = FMath::Sin(Theta);
            double CosTheta = FMath::Cos(Theta);
            FVector3d Pos = Center + Radius * FVector3d(SinPhi * CosTheta, SinPhi * SinTheta, CosPhi);
            Mesh.AppendVertex(Pos);

            if (UVOverlay)
            {
                float U = (float)i / (float)LongSteps;
                float V = (float)j / (float)LatSteps;
                UVOverlay->AppendElement(FVector2f(U, V));
            }
        }
    }

    int32 Columns = LongSteps + 1;
    for (int32 j = 0; j < LatSteps; j++)
    {
        for (int32 i = 0; i < LongSteps; i++)
        {
            int32 A = StartVert + j * Columns + i;
            int32 B = A + 1;
            int32 C = A + Columns;
            int32 D = C + 1;

            int32 Tri1 = Mesh.AppendTriangle(A, B, C);
            int32 Tri2 = Mesh.AppendTriangle(B, D, C);

            if (MaterialIDAttribute)
            {
                MaterialIDAttribute->SetValue(Tri1, 1); // leaves
                MaterialIDAttribute->SetValue(Tri2, 1);
            }
        }
    }
}

// ----------------------------------------------------------------------
// Recursive branch generation
// ----------------------------------------------------------------------
static void GenerateBranches(FDynamicMesh3& Mesh, FRandomStream& Rand,
    const FVector3d& Start, const FVector3d& Direction,
    float Length, float Radius, int32 Level, int32 MaxLevel,
    float LeafRadius,
    FDynamicMeshMaterialAttribute* MaterialIDAttribute,
    FDynamicMeshUVOverlay* UVOverlay)
{
    if (Level > MaxLevel || Length < 5.0f || Radius < 1.0f) return;

    FVector3d End = Start + Direction * Length;
    float EndRadius = Radius * 0.7f;
    AddBranchCylinder(Mesh, Start, End, Radius, EndRadius, 8, MaterialIDAttribute, UVOverlay);

    if (Level == MaxLevel)
    {
        AddLeafSphere(Mesh, End, LeafRadius, 8, MaterialIDAttribute, UVOverlay);
        return;
    }

    int32 NumBranches = Rand.RandRange(2, 4);

    // Main continuation
    {
        FVector3d MainDir = Direction.RotateAngleAxis(Rand.RandRange(-20.0f, 20.0f),
            FVector3d::UpVector.Cross(Direction).GetSafeNormal());
        MainDir = MainDir.RotateAngleAxis(Rand.RandRange(-15.0f, 15.0f), Direction);
        float MainLength = Length * Rand.FRandRange(0.6f, 0.9f);
        GenerateBranches(Mesh, Rand, End, MainDir, MainLength, EndRadius, Level + 1, MaxLevel, LeafRadius,
            MaterialIDAttribute, UVOverlay);
    }

    // Side branches
    for (int32 i = 0; i < NumBranches - 1; i++)
    {
        double Angle = 360.0 * (double)i / (double)(NumBranches - 1) + Rand.FRandRange(-20.0, 20.0);
        FVector3d BranchDir = Direction.RotateAngleAxis(Rand.RandRange(30.0f, 70.0f),
            FVector3d(0, 0, 1).Cross(Direction).GetSafeNormal());
        BranchDir = BranchDir.RotateAngleAxis(Angle, Direction);
        BranchDir.Normalize();
        float BranchLength = Length * Rand.FRandRange(0.3f, 0.6f);
        GenerateBranches(Mesh, Rand, End, BranchDir, BranchLength, EndRadius, Level + 1, MaxLevel, LeafRadius,
            MaterialIDAttribute, UVOverlay);
    }
}

// ----------------------------------------------------------------------
// Main generation function
// ----------------------------------------------------------------------
void UTreeGenerator::GenerateOakTree(UDynamicMeshComponent* DynamicMeshComponent,
    int32 Seed, float TrunkHeight, float TrunkRadius,
    int32 BranchLevels, float LeafClusterRadius,
    UMaterialInterface* BarkMaterial,
    UMaterialInterface* LeafMaterial)
{
    if (!DynamicMeshComponent)
    {
        UE_LOG(LogTemp, Warning, TEXT("GenerateOakTree: Invalid DynamicMeshComponent."));
        return;
    }

    FRandomStream Rand(Seed);
    FDynamicMesh3 Mesh;

    // --- Attribute setup (matching your terrain pattern) ---
    Mesh.EnableAttributes();
    FDynamicMeshAttributeSet* Attr = Mesh.Attributes();
    Attr->SetNumUVLayers(1);               // one UV channel for now
    Attr->EnableMaterialID();
    Attr->SetNumNormalLayers(1);           // for normals

    FDynamicMeshUVOverlay* UVOverlay = Attr->GetUVLayer(0);
    auto* MaterialIDAttribute = Attr->GetMaterialID();

    // --- Trunk (ridged, original gentle ridges) ---
    FVector3d TrunkStart(0, 0, 0);
    FVector3d TrunkEnd(0, 0, TrunkHeight);
    float TrunkEndRadius = TrunkRadius * 0.6f;
    AddRidgedCylinder(Mesh, Rand, TrunkStart, TrunkEnd, TrunkRadius, TrunkEndRadius,
        12, 6, 0.25f, MaterialIDAttribute, UVOverlay);

    // --- Branches and leaves ---
    FVector3d TopOfTrunk = TrunkEnd;
    FVector3d Up(0, 0, 1);
    GenerateBranches(Mesh, Rand, TopOfTrunk, Up,
        TrunkHeight * 0.4f, TrunkEndRadius, 1, BranchLevels, LeafClusterRadius,
        MaterialIDAttribute, UVOverlay);

    // Extra leaf cluster directly at the top
    AddLeafSphere(Mesh, TopOfTrunk, LeafClusterRadius * 0.8f, 8, MaterialIDAttribute, UVOverlay);

    // --- Normals ---
    FMeshNormals::QuickComputeVertexNormals(Mesh);

    // --- Apply mesh to component ---
    DynamicMeshComponent->SetMesh(MoveTemp(Mesh));

    // --- Assign materials (slot 0 = bark, slot 1 = leaves) ---
    if (BarkMaterial)  DynamicMeshComponent->SetMaterial(0, BarkMaterial);
    if (LeafMaterial)  DynamicMeshComponent->SetMaterial(1, LeafMaterial);

    DynamicMeshComponent->NotifyMeshUpdated();
}