// Copyright Epic Games, Inc. All Rights Reserved.

#include "GroundupCharacter.h"
#include "Animation/AnimInstance.h"
#include "Camera/CameraComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "EnhancedInputComponent.h"
#include "InputActionValue.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "SmoothVoxelTerrain.h"
#include "Groundup.h"
#include "DrawDebugHelpers.h"

AGroundupCharacter::AGroundupCharacter()
{
	GetCapsuleComponent()->InitCapsuleSize(55.f, 96.0f);

	FirstPersonMesh = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("First Person Mesh"));
	FirstPersonMesh->SetupAttachment(GetMesh());
	FirstPersonMesh->SetOnlyOwnerSee(true);
	FirstPersonMesh->FirstPersonPrimitiveType = EFirstPersonPrimitiveType::FirstPerson;
	FirstPersonMesh->SetCollisionProfileName(FName("NoCollision"));

	FirstPersonCameraComponent = CreateDefaultSubobject<UCameraComponent>(TEXT("First Person Camera"));
	FirstPersonCameraComponent->SetupAttachment(FirstPersonMesh, FName("head"));
	FirstPersonCameraComponent->SetRelativeLocationAndRotation(FVector(-2.8f, 5.89f, 0.0f), FRotator(0.0f, 90.0f, -90.0f));
	FirstPersonCameraComponent->bUsePawnControlRotation = true;
	FirstPersonCameraComponent->bEnableFirstPersonFieldOfView = true;
	FirstPersonCameraComponent->bEnableFirstPersonScale = true;
	FirstPersonCameraComponent->FirstPersonFieldOfView = 70.0f;
	FirstPersonCameraComponent->FirstPersonScale = 0.6f;

	GetMesh()->SetOwnerNoSee(true);
	GetMesh()->FirstPersonPrimitiveType = EFirstPersonPrimitiveType::WorldSpaceRepresentation;

	GetCapsuleComponent()->SetCapsuleSize(34.0f, 96.0f);

	GetCharacterMovement()->BrakingDecelerationFalling = 1500.0f;
	GetCharacterMovement()->AirControl = 0.5f;
}

void AGroundupCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	if (UEnhancedInputComponent* EnhancedInputComponent = Cast<UEnhancedInputComponent>(PlayerInputComponent))
	{
		EnhancedInputComponent->BindAction(JumpAction, ETriggerEvent::Started, this, &AGroundupCharacter::DoJumpStart);
		EnhancedInputComponent->BindAction(JumpAction, ETriggerEvent::Completed, this, &AGroundupCharacter::DoJumpEnd);

		EnhancedInputComponent->BindAction(MoveAction, ETriggerEvent::Triggered, this, &AGroundupCharacter::MoveInput);

		EnhancedInputComponent->BindAction(LookAction, ETriggerEvent::Triggered, this, &AGroundupCharacter::LookInput);
		EnhancedInputComponent->BindAction(MouseLookAction, ETriggerEvent::Triggered, this, &AGroundupCharacter::LookInput);
	}
	else
	{
		UE_LOG(LogGroundup, Error, TEXT("'%s' Failed to find an Enhanced Input Component!"), *GetNameSafe(this));
	}
}

void AGroundupCharacter::ExecutePlaceVoxel(ASmoothVoxelTerrain* HitTerrain, FHitResult& HitResult)
{
	if (!HitTerrain) return;
	FVector PlaceLocation = HitResult.ImpactPoint + HitResult.ImpactNormal * (HitTerrain->CubeSize * 0.5f);
	HitTerrain->PlaceVoxel(PlaceLocation);
}

void AGroundupCharacter::ExecuteBreakVoxel(ASmoothVoxelTerrain* HitTerrain, FHitResult& HitResult)
{
    if (!HitTerrain) return;

    const float CubeSize = HitTerrain->CubeSize;
    const FVector ImpactPoint = HitResult.ImpactPoint;
    const FVector ImpactNormal = HitResult.ImpactNormal.GetSafeNormal();

    // Convert impact point into terrain local coordinates
    const FVector LocalPos = HitTerrain->GetActorTransform().InverseTransformPosition(ImpactPoint);

    int32 TargetVoxelX = 0, TargetVoxelY = 0, TargetVoxelZ = 0;
    EVoxelType TargetVoxelType = EVoxelType::Air;
    bool bFoundSolid = false;

    auto CheckVoxel = [&](int32 X, int32 Y, int32 Z) -> bool
        {
            EVoxelType Type = HitTerrain->GetVoxelAtWorld(X, Y, Z);
            if (Type != EVoxelType::Air)
            {
                TargetVoxelX = X;
                TargetVoxelY = Y;
                TargetVoxelZ = Z;
                TargetVoxelType = Type;
                bFoundSolid = true;
                return true;
            }
            return false;
        };

    const float AbsX = FMath::Abs(ImpactNormal.X);
    const float AbsY = FMath::Abs(ImpactNormal.Y);
    const float AbsZ = FMath::Abs(ImpactNormal.Z);

    // -------------------------------------------------------------------------
    // 1. SMALL INWARD NUDGE (Normal-aligned only, NO RayDir drift)
    // A 15% step stays firmly inside this voxel on both triangles.
    // -------------------------------------------------------------------------
    const FVector InwardNudge = -ImpactNormal * (CubeSize * 0.15f);
    const FVector NudgeLocal = HitTerrain->GetActorTransform().InverseTransformPosition(ImpactPoint + InwardNudge);

    const int32 NudgeX = FMath::FloorToInt(NudgeLocal.X / CubeSize);
    const int32 NudgeY = FMath::FloorToInt(NudgeLocal.Y / CubeSize);
    const int32 NudgeZ = FMath::FloorToInt(NudgeLocal.Z / CubeSize);

    if (CheckVoxel(NudgeX, NudgeY, NudgeZ))
    {
        // Resolved immediately via normal penetration
    }
    // -------------------------------------------------------------------------
    // 2. AXIS-LOCKED PROJECTION (Prevents horizontal/diagonal drift)
    // -------------------------------------------------------------------------
    else if (AbsZ >= AbsX && AbsZ >= AbsY)
    {
        // Ground, slopes, hills, or ceilings:
        // Lock X and Y to the hit column so Triangle 2 cannot drift into the block behind it.
        const int32 ColX = FMath::FloorToInt(LocalPos.X / CubeSize);
        const int32 ColY = FMath::FloorToInt(LocalPos.Y / CubeSize);
        const int32 StartZ = FMath::FloorToInt(LocalPos.Z / CubeSize);

        if (ImpactNormal.Z > 0.0f)
        {
            // Upward face / slope: search down the column for the ground voxel
            for (int32 dz = 0; dz >= -6; --dz)
            {
                if (CheckVoxel(ColX, ColY, StartZ + dz)) break;
            }
        }
        else
        {
            // Ceiling / overhang: search up the column into the ceiling rock
            for (int32 dz = 0; dz <= 6; ++dz)
            {
                if (CheckVoxel(ColX, ColY, StartZ + dz)) break;
            }
        }
    }
    else if (AbsX >= AbsY)
    {
        // East/West wall: lock Y and Z, search along X into the wall
        const int32 RowY = FMath::FloorToInt(LocalPos.Y / CubeSize);
        const int32 RowZ = FMath::FloorToInt(LocalPos.Z / CubeSize);
        const int32 StartX = FMath::FloorToInt(LocalPos.X / CubeSize);
        const int32 StepX = (ImpactNormal.X > 0.0f) ? -1 : 1;

        for (int32 dx = 0; dx != StepX * 4; dx += StepX)
        {
            if (CheckVoxel(StartX + dx, RowY, RowZ)) break;
        }
    }
    else
    {
        // North/South wall: lock X and Z, search along Y into the wall
        const int32 RowX = FMath::FloorToInt(LocalPos.X / CubeSize);
        const int32 RowZ = FMath::FloorToInt(LocalPos.Z / CubeSize);
        const int32 StartY = FMath::FloorToInt(LocalPos.Y / CubeSize);
        const int32 StepY = (ImpactNormal.Y > 0.0f) ? -1 : 1;

        for (int32 dy = 0; dy != StepY * 4; dy += StepY)
        {
            if (CheckVoxel(RowX, StartY + dy, RowZ)) break;
        }
    }

    // -------------------------------------------------------------------------
    // 3. REMOVE RESOLVED VOXEL AT EXACT CENTER
    // -------------------------------------------------------------------------
    if (bFoundSolid && TargetVoxelType != EVoxelType::Air)
    {
        const FVector LocalCenter(
            (TargetVoxelX + 0.5f) * CubeSize,
            (TargetVoxelY + 0.5f) * CubeSize,
            (TargetVoxelZ + 0.5f) * CubeSize
        );
        const FVector WorldCenter = HitTerrain->GetActorTransform().TransformPosition(LocalCenter);

        if (bShowVoxelDebug && GetWorld())
        {
            DrawDebugSphere(GetWorld(), WorldCenter, 12.0f, 8, FColor::Green, false, VoxelDebugLife);
            DrawDebugString(GetWorld(), ImpactPoint + FVector(0, 0, 30),
                FString::Printf(TEXT("Break Voxel (%d, %d, %d) Type: %d"),
                    TargetVoxelX, TargetVoxelY, TargetVoxelZ, (int32)TargetVoxelType),
                nullptr, FColor::White, VoxelDebugLife);
        }

        HitTerrain->RemoveVoxel(WorldCenter);
    }
}

void AGroundupCharacter::ExecuteHighlightVoxel(ASmoothVoxelTerrain* HitTerrain, FHitResult& HitResult)
{
	if (!HitTerrain) return;
	FVector AdjustedPoint = HitResult.ImpactPoint - HitResult.ImpactNormal * HitTerrain->CubeSize * 0.1f;
}

void AGroundupCharacter::HandleVoxelInteraction(const EVoxelInteractionAction Action)
{
    if (!FirstPersonCameraComponent) return;

    FVector Start = FirstPersonCameraComponent->GetComponentLocation();
    FVector End = Start + (FirstPersonCameraComponent->GetForwardVector() * 1000.0f);

    FHitResult HitResult;
    FCollisionQueryParams QueryParams;
    QueryParams.AddIgnoredActor(this);
    QueryParams.bTraceComplex = true;
    QueryParams.bReturnFaceIndex = true;

    if (GetWorld()->LineTraceSingleByChannel(HitResult, Start, End, ECC_Visibility, QueryParams))
    {
        ASmoothVoxelTerrain* HitTerrain = Cast<ASmoothVoxelTerrain>(HitResult.GetActor());
        if (HitTerrain)
        {
            switch (Action)
            {
            case EVoxelInteractionAction::Place:
                ExecutePlaceVoxel(HitTerrain, HitResult);
                break;
            case EVoxelInteractionAction::Break:
                ExecuteBreakVoxel(HitTerrain, HitResult);
                break;
            case EVoxelInteractionAction::Hover:
                ExecuteHighlightVoxel(HitTerrain, HitResult);
                break;
            default:
                UE_LOG(LogTemp, Error, TEXT("Undefined voxel interaction action."));
            }
        }
    }
}

void AGroundupCharacter::MoveInput(const FInputActionValue& Value)
{
	FVector2D MovementVector = Value.Get<FVector2D>();
	DoMove(MovementVector.X, MovementVector.Y);
}

void AGroundupCharacter::LookInput(const FInputActionValue& Value)
{
	FVector2D LookAxisVector = Value.Get<FVector2D>();
	DoAim(LookAxisVector.X, LookAxisVector.Y);
}

void AGroundupCharacter::DoAim(float Yaw, float Pitch)
{
	if (GetController())
	{
		AddControllerYawInput(Yaw);
		AddControllerPitchInput(Pitch);
	}
}

void AGroundupCharacter::DoMove(float Right, float Forward)
{
	if (GetController())
	{
		AddMovementInput(GetActorRightVector(), Right);
		AddMovementInput(GetActorForwardVector(), Forward);
	}
}

void AGroundupCharacter::DoJumpStart() { Jump(); }
void AGroundupCharacter::DoJumpEnd() { StopJumping(); }
void AGroundupCharacter::RemoveVoxel() { HandleVoxelInteraction(EVoxelInteractionAction::Break); }
void AGroundupCharacter::PlaceVoxel() { HandleVoxelInteraction(EVoxelInteractionAction::Place); }
void AGroundupCharacter::HoverVoxel() { HandleVoxelInteraction(EVoxelInteractionAction::Hover); }