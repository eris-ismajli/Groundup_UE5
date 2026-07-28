// Fill out your copyright notice in the Description page of Project Settings.

#include "TreeGenerator.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "DynamicMesh/MeshNormals.h"          // <-- added for normal computation
#include "Components/DynamicMeshComponent.h"
#include "Engine/Engine.h"

using namespace UE::Geometry;

// Helper: generate a cylinder between two points (outward facing)
static void AddCylinder(FDynamicMesh3& Mesh, const FVector3d& Start, const FVector3d& End,
    float RadiusStart, float RadiusEnd, int32 Slices = 8)
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
    }

    for (int32 i = 0; i < Slices; i++)
    {
        int32 Next = (i + 1) % Slices;
        // Outward winding (flipped)
        Mesh.AppendTriangle(Ring0[i], Ring1[Next], Ring1[i]);
        Mesh.AppendTriangle(Ring0[i], Ring0[Next], Ring1[Next]);
    }
}

// Helper: add a sphere (outward facing)
static void AddSphere(FDynamicMesh3& Mesh, const FVector3d& Center, float Radius, int32 Segments = 6)
{
    int32 StartVert = Mesh.VertexCount();
    for (int32 j = 0; j <= Segments; j++)
    {
        double Phi = PI * (double)j / (double)Segments;
        double SinPhi = FMath::Sin(Phi);
        double CosPhi = FMath::Cos(Phi);
        for (int32 i = 0; i <= Segments * 2; i++)
        {
            double Theta = 2.0 * PI * (double)i / (double)(Segments * 2);
            double SinTheta = FMath::Sin(Theta);
            double CosTheta = FMath::Cos(Theta);
            FVector3d Pos = Center + Radius * FVector3d(SinPhi * CosTheta, SinPhi * SinTheta, CosPhi);
            Mesh.AppendVertex(Pos);
        }
    }

    int32 Columns = Segments * 2 + 1;
    for (int32 j = 0; j < Segments; j++)
    {
        for (int32 i = 0; i < Segments * 2; i++)
        {
            int32 A = StartVert + j * Columns + i;
            int32 B = A + 1;
            int32 C = A + Columns;
            int32 D = C + 1;
            // Outward facing triangles
            Mesh.AppendTriangle(A, B, C);
            Mesh.AppendTriangle(B, D, C);
        }
    }
}

// Recursive branch generation
static void GenerateBranches(FDynamicMesh3& Mesh, FRandomStream& Rand,
    const FVector3d& Start, const FVector3d& Direction,
    float Length, float Radius, int32 Level, int32 MaxLevel,
    float LeafRadius)
{
    if (Level > MaxLevel || Length < 5.0f || Radius < 1.0f) return;

    FVector3d End = Start + Direction * Length;
    // Taper slightly
    float EndRadius = Radius * 0.7f;
    AddCylinder(Mesh, Start, End, Radius, EndRadius, 8);

    // At the final level, place a leaf cluster instead of further branches
    if (Level == MaxLevel)
    {
        AddSphere(Mesh, End, LeafRadius, 8);
        return;
    }

    // Number of sub-branches (2-4 for oak-like)
    int32 NumBranches = Rand.RandRange(2, 4);
    // Main continuation branch (slightly off the original direction)
    {
        FVector3d MainDir = Direction.RotateAngleAxis(Rand.RandRange(-20.0f, 20.0f),
            FVector3d::UpVector.Cross(Direction).GetSafeNormal());
        MainDir = MainDir.RotateAngleAxis(Rand.RandRange(-15.0f, 15.0f), Direction);
        float MainLength = Length * Rand.FRandRange(0.6f, 0.9f);
        GenerateBranches(Mesh, Rand, End, MainDir, MainLength, EndRadius, Level + 1, MaxLevel, LeafRadius);
    }

    // Side branches
    for (int32 i = 0; i < NumBranches - 1; i++)
    {
        // Spread around the main axis
        double Angle = 360.0 * (double)i / (double)(NumBranches - 1) + Rand.FRandRange(-20.0, 20.0);
        FVector3d BranchDir = Direction.RotateAngleAxis(Rand.RandRange(30.0f, 70.0f),
            FVector3d(0, 0, 1).Cross(Direction).GetSafeNormal());
        BranchDir = BranchDir.RotateAngleAxis(Angle, Direction);
        BranchDir.Normalize();
        float BranchLength = Length * Rand.FRandRange(0.3f, 0.6f);
        GenerateBranches(Mesh, Rand, End, BranchDir, BranchLength, EndRadius, Level + 1, MaxLevel, LeafRadius);
    }
}

void UTreeGenerator::GenerateOakTree(UDynamicMeshComponent* DynamicMeshComponent,
    int32 Seed, float TrunkHeight, float TrunkRadius,
    int32 BranchLevels, float LeafClusterRadius)
{
    if (!DynamicMeshComponent)
    {
        UE_LOG(LogTemp, Warning, TEXT("GenerateOakTree: Invalid DynamicMeshComponent."));
        return;
    }

    FRandomStream Rand(Seed);
    FDynamicMesh3 Mesh;

    // Enable attributes (normals, etc.)
    Mesh.EnableAttributes();
    // Allocate a normal layer (UE5.7 compatible)
    Mesh.Attributes()->SetNumNormalLayers(1);

    // Trunk from origin upward
    FVector3d Start(0, 0, 0);
    FVector3d TrunkDir(0, 0, 1);
    float TrunkEndRadius = TrunkRadius * 0.6f;
    AddCylinder(Mesh, Start, Start + TrunkDir * TrunkHeight, TrunkRadius, TrunkEndRadius, 10);

    // Recursive branches starting at top of trunk
    GenerateBranches(Mesh, Rand, Start + TrunkDir * TrunkHeight, TrunkDir,
        TrunkHeight * 0.4f, TrunkEndRadius, 1, BranchLevels, LeafClusterRadius);

    // Also add a few leaves directly at the top
    AddSphere(Mesh, Start + TrunkDir * TrunkHeight, LeafClusterRadius * 0.8f, 8);

    // Compute proper vertex normals so the tree is lit correctly
    FMeshNormals::QuickComputeVertexNormals(Mesh);

    // Assign to component
    DynamicMeshComponent->SetMesh(MoveTemp(Mesh));
    DynamicMeshComponent->NotifyMeshUpdated();
}