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
	// Set size for collision capsule
	GetCapsuleComponent()->InitCapsuleSize(55.f, 96.0f);

	// Create the first person mesh that will be viewed only by this character's owner
	FirstPersonMesh = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("First Person Mesh"));

	FirstPersonMesh->SetupAttachment(GetMesh());
	FirstPersonMesh->SetOnlyOwnerSee(true);
	FirstPersonMesh->FirstPersonPrimitiveType = EFirstPersonPrimitiveType::FirstPerson;
	FirstPersonMesh->SetCollisionProfileName(FName("NoCollision"));

	// Create the Camera Component	
	FirstPersonCameraComponent = CreateDefaultSubobject<UCameraComponent>(TEXT("First Person Camera"));
	FirstPersonCameraComponent->SetupAttachment(FirstPersonMesh, FName("head"));
	FirstPersonCameraComponent->SetRelativeLocationAndRotation(FVector(-2.8f, 5.89f, 0.0f), FRotator(0.0f, 90.0f, -90.0f));
	FirstPersonCameraComponent->bUsePawnControlRotation = true;
	FirstPersonCameraComponent->bEnableFirstPersonFieldOfView = true;
	FirstPersonCameraComponent->bEnableFirstPersonScale = true;
	FirstPersonCameraComponent->FirstPersonFieldOfView = 70.0f;
	FirstPersonCameraComponent->FirstPersonScale = 0.6f;

	// configure the character comps
	GetMesh()->SetOwnerNoSee(true);
	GetMesh()->FirstPersonPrimitiveType = EFirstPersonPrimitiveType::WorldSpaceRepresentation;

	GetCapsuleComponent()->SetCapsuleSize(34.0f, 96.0f);

	// Configure character movement
	GetCharacterMovement()->BrakingDecelerationFalling = 1500.0f;
	GetCharacterMovement()->AirControl = 0.5f;
}

void AGroundupCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	// Set up action bindings
	if (UEnhancedInputComponent* EnhancedInputComponent = Cast<UEnhancedInputComponent>(PlayerInputComponent))
	{
		// Jumping
		EnhancedInputComponent->BindAction(JumpAction, ETriggerEvent::Started, this, &AGroundupCharacter::DoJumpStart);
		EnhancedInputComponent->BindAction(JumpAction, ETriggerEvent::Completed, this, &AGroundupCharacter::DoJumpEnd);

		// Moving
		EnhancedInputComponent->BindAction(MoveAction, ETriggerEvent::Triggered, this, &AGroundupCharacter::MoveInput);

		// Looking/Aiming
		EnhancedInputComponent->BindAction(LookAction, ETriggerEvent::Triggered, this, &AGroundupCharacter::LookInput);
		EnhancedInputComponent->BindAction(MouseLookAction, ETriggerEvent::Triggered, this, &AGroundupCharacter::LookInput);

	}
	else
	{
		UE_LOG(LogGroundup, Error, TEXT("'%s' Failed to find an Enhanced Input Component! This template is built to use the Enhanced Input system. If you intend to use the legacy system, then you will need to update this C++ file."), *GetNameSafe(this));
	}
}


void AGroundupCharacter::ExecutePlaceVoxel(ASmoothVoxelTerrain* HitTerrain, FHitResult& HitResult)
{
	FVector PlaceLocation = HitResult.ImpactPoint + HitResult.ImpactNormal * (HitTerrain->CubeSize * 0.5f);

	// Debug visualization for placement
	if (bShowVoxelDebug && GetWorld())
	{
		// Impact point and normal
		DrawDebugSphere(GetWorld(), HitResult.ImpactPoint, 12.0f, 12, FColor::Red, false, VoxelDebugLife);
		DrawDebugLine(GetWorld(), HitResult.ImpactPoint,
			HitResult.ImpactPoint + HitResult.ImpactNormal * 80.0f,
			FColor::Yellow, false, VoxelDebugLife, 0, 2.0f);

		// Placement location
		DrawDebugSphere(GetWorld(), PlaceLocation, 8.0f, 8, FColor::Orange, false, VoxelDebugLife);

		// Try to find the voxel that will be placed
		int32 VoxelX, VoxelY, VoxelZ;
		EVoxelType VoxelType;
		if (HitTerrain->GetVoxelAtWorldPoint(PlaceLocation, VoxelX, VoxelY, VoxelZ, &VoxelType))
		{
			FVector WorldMin = HitTerrain->GetActorTransform().TransformPosition(FVector(
				VoxelX * HitTerrain->CubeSize,
				VoxelY * HitTerrain->CubeSize,
				VoxelZ * HitTerrain->CubeSize));

			FVector WorldMax = HitTerrain->GetActorTransform().TransformPosition(FVector(
				(VoxelX + 1) * HitTerrain->CubeSize,
				(VoxelY + 1) * HitTerrain->CubeSize,
				(VoxelZ + 1) * HitTerrain->CubeSize));

			FVector BoxCenter = (WorldMin + WorldMax) * 0.5f;
			FVector BoxExtent = (WorldMax - WorldMin) * 0.5f;

			DrawDebugBox(GetWorld(), BoxCenter, BoxExtent, FColor::Green, false, VoxelDebugLife);
			DrawDebugSphere(GetWorld(), BoxCenter, 10.0f, 8, FColor::Green, false, VoxelDebugLife);
			DrawDebugString(GetWorld(), HitResult.ImpactPoint + FVector(0, 0, 30),
				FString::Printf(TEXT("Place Voxel (%d, %d, %d)"), VoxelX, VoxelY, VoxelZ),
				nullptr, FColor::White, VoxelDebugLife);
		}
	}

	HitTerrain->PlaceVoxel(PlaceLocation);
}

void AGroundupCharacter::ExecuteBreakVoxel(ASmoothVoxelTerrain* HitTerrain, FHitResult& HitResult)
{
	const float CubeSize = HitTerrain->CubeSize;

	// Nudge the impact point inward along the hit normal to avoid boundary ambiguity
	FVector AdjustedPoint = HitResult.ImpactPoint - HitResult.ImpactNormal * CubeSize * 0.01f;

	// Debug visualization for removal
	if (bShowVoxelDebug && GetWorld())
	{
		DrawDebugSphere(GetWorld(), HitResult.ImpactPoint, 12.0f, 12, FColor::Red, false, VoxelDebugLife);
		DrawDebugLine(GetWorld(), HitResult.ImpactPoint,
			HitResult.ImpactPoint + HitResult.ImpactNormal * 80.0f,
			FColor::Yellow, false, VoxelDebugLife, 0, 2.0f);
		DrawDebugSphere(GetWorld(), AdjustedPoint, 8.0f, 8, FColor::Orange, false, VoxelDebugLife);
	}

	int32 VoxelX, VoxelY, VoxelZ;
	EVoxelType VoxelType;

	if (HitTerrain->GetVoxelAtWorldPoint(HitResult.ImpactPoint, VoxelX, VoxelY, VoxelZ, &VoxelType))
	{
		bool bAutoAssisted = false;
		bool bIsTopFace = (HitResult.ImpactNormal.Z > 0.7f);

		// Auto-assist only when clicking the top face AND hitting air
		if (bIsTopFace && VoxelType == EVoxelType::Air)
		{
			int32 BelowX = VoxelX;
			int32 BelowY = VoxelY;
			int32 BelowZ = VoxelZ - 1;

			EVoxelType BelowType = HitTerrain->GetVoxelAtWorld(BelowX, BelowY, BelowZ);
			if (BelowType != EVoxelType::Air)
			{
				VoxelX = BelowX;
				VoxelY = BelowY;
				VoxelZ = BelowZ;
				VoxelType = BelowType;
				bAutoAssisted = true;
			}
			else
			{
				if (bShowVoxelDebug && GetWorld())
				{
					DrawDebugString(GetWorld(), HitResult.ImpactPoint,
						TEXT("Air below air - no break"),
						nullptr, FColor::Magenta, VoxelDebugLife);
				}
				return;
			}
		}

		// Only break if we now have a solid voxel
		if (VoxelType != EVoxelType::Air)
		{
			FVector LocalCenter(
				VoxelX * CubeSize + CubeSize * 0.5f,
				VoxelY * CubeSize + CubeSize * 0.5f,
				VoxelZ * CubeSize + CubeSize * 0.5f
			);
			FVector WorldCenter = HitTerrain->GetActorTransform().TransformPosition(LocalCenter);

			// Debug visualization of the voxel being removed
			if (bShowVoxelDebug && GetWorld())
			{
				FVector WorldMin = HitTerrain->GetActorTransform().TransformPosition(FVector(
					VoxelX * CubeSize,
					VoxelY * CubeSize,
					VoxelZ * CubeSize));

				FVector WorldMax = HitTerrain->GetActorTransform().TransformPosition(FVector(
					(VoxelX + 1) * CubeSize,
					(VoxelY + 1) * CubeSize,
					(VoxelZ + 1) * CubeSize));

				FVector BoxCenter = (WorldMin + WorldMax) * 0.5f;
				FVector BoxExtent = (WorldMax - WorldMin) * 0.5f;

				FColor BoxColor = bAutoAssisted ? FColor::Green : FColor::Cyan;

				DrawDebugBox(GetWorld(), BoxCenter, BoxExtent, BoxColor, false, VoxelDebugLife);
				DrawDebugSphere(GetWorld(), WorldCenter, 10.0f, 8, BoxColor, false, VoxelDebugLife);

				DrawDebugString(GetWorld(), HitResult.ImpactPoint + FVector(0, 0, 30),
					FString::Printf(TEXT("Break Voxel (%d, %d, %d) Type: %d%s"),
						VoxelX, VoxelY, VoxelZ,
						(int32)VoxelType,
						bAutoAssisted ? TEXT(" (auto-assisted below)") : TEXT("")),
					nullptr, FColor::White, VoxelDebugLife);
			}

			HitTerrain->RemoveVoxel(WorldCenter);
		}
	}
}

void AGroundupCharacter::ExecuteHighlightVoxel(ASmoothVoxelTerrain* HitTerrain, FHitResult& HitResult) {
	FVector AdjustedPoint = HitResult.ImpactPoint - HitResult.ImpactNormal * HitTerrain->CubeSize * 0.1f;
	// call highlight voxel method
}

void AGroundupCharacter::HandleVoxelInteraction(const EVoxelInteractionAction Action) {
	if (!FirstPersonCameraComponent) return;

	FVector Start = FirstPersonCameraComponent->GetComponentLocation();
	FVector End = Start + (FirstPersonCameraComponent->GetForwardVector() * 1000.0f);

	FHitResult HitResult;
	FCollisionQueryParams QueryParams;
	QueryParams.AddIgnoredActor(this);

	if (GetWorld()->LineTraceSingleByChannel(HitResult, Start, End, ECC_Visibility, QueryParams))
	{
		ASmoothVoxelTerrain* HitTerrain = Cast<ASmoothVoxelTerrain>(HitResult.GetActor());
		if (HitTerrain)
		{
			switch (Action) {
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

void AGroundupCharacter::DoJumpStart()
{
	Jump();
}

void AGroundupCharacter::DoJumpEnd()
{
	StopJumping();
}

void AGroundupCharacter::RemoveVoxel()
{
	HandleVoxelInteraction(EVoxelInteractionAction::Break);
}

void AGroundupCharacter::PlaceVoxel()
{
	HandleVoxelInteraction(EVoxelInteractionAction::Place);
}

void AGroundupCharacter::HoverVoxel()
{
	HandleVoxelInteraction(EVoxelInteractionAction::Hover);
}