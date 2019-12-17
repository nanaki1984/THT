// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

#include "THTCharacter.h"
#include "HeadMountedDisplayFunctionLibrary.h"
#include "Camera/CameraComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/InputComponent.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/Controller.h"
#include "Camera/PCCameraComponent.h"

//////////////////////////////////////////////////////////////////////////
// ATHTCharacter

ATHTCharacter::ATHTCharacter()
{
	// Set size for collision capsule
	GetCapsuleComponent()->InitCapsuleSize(42.f, 96.0f);

	// set our turn rates for input
	BaseTurnRate = 45.f;
	BaseLookUpRate = 45.f;

	// Don't rotate when the controller rotates. Let that just affect the camera.
	bUseControllerRotationPitch = false;
	bUseControllerRotationYaw = false;
	bUseControllerRotationRoll = false;

	// Configure character movement
	GetCharacterMovement()->bOrientRotationToMovement = true; // Character moves in the direction of input...	
	GetCharacterMovement()->RotationRate = FRotator(0.0f, 540.0f, 0.0f); // ...at this rotation rate
	GetCharacterMovement()->JumpZVelocity = 600.f;
	GetCharacterMovement()->AirControl = 0.2f;

    CameraBehaviour = CreateDefaultSubobject<UPCCameraComponent>(TEXT("CameraBehaviour"));

	// Create a follow camera
	CameraComponent = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraComponent"));
    //CameraComponent->bUsePawnControlRotation = false; // Camera does not rotate relative to arm

	// Note: The skeletal mesh and anim blueprint references on the Mesh component (inherited from Character) 
	// are set in the derived blueprint asset named MyCharacter (to avoid direct content references in C++)
}

void ATHTCharacter::Tick(float DeltaSeconds)
{
    CameraBehaviour->UpdateCameraTransform(DeltaSeconds);

    Super::Tick(DeltaSeconds);
}

//////////////////////////////////////////////////////////////////////////
// Input

void ATHTCharacter::SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent)
{
	// Set up gameplay key bindings
	check(PlayerInputComponent);
	PlayerInputComponent->BindAction("Jump", IE_Pressed, this, &ACharacter::Jump);
	PlayerInputComponent->BindAction("Jump", IE_Released, this, &ACharacter::StopJumping);

	PlayerInputComponent->BindAxis("MoveForward", this, &ATHTCharacter::MoveForward);
	PlayerInputComponent->BindAxis("MoveRight", this, &ATHTCharacter::MoveRight);

	// We have 2 versions of the rotation bindings to handle different kinds of devices differently
	// "turn" handles devices that provide an absolute delta, such as a mouse.
	// "turnrate" is for devices that we choose to treat as a rate of change, such as an analog joystick
	//PlayerInputComponent->BindAxis("Turn", this, &APawn::AddControllerYawInput);
    PlayerInputComponent->BindAxis("TurnRate", this, &ATHTCharacter::TurnAtRate);
	//PlayerInputComponent->BindAxis("LookUp", this, &APawn::AddControllerPitchInput);
    PlayerInputComponent->BindAxis("LookUpRate", this, &ATHTCharacter::LookUpAtRate);

	// handle touch devices
	PlayerInputComponent->BindTouch(IE_Pressed, this, &ATHTCharacter::TouchStarted);
	PlayerInputComponent->BindTouch(IE_Released, this, &ATHTCharacter::TouchStopped);

	// VR headset functionality
	PlayerInputComponent->BindAction("ResetVR", IE_Pressed, this, &ATHTCharacter::OnResetVR);
}


void ATHTCharacter::OnResetVR()
{
	UHeadMountedDisplayFunctionLibrary::ResetOrientationAndPosition();
}

void ATHTCharacter::TouchStarted(ETouchIndex::Type FingerIndex, FVector Location)
{
	Jump();
}

void ATHTCharacter::TouchStopped(ETouchIndex::Type FingerIndex, FVector Location)
{
	StopJumping();
}

void ATHTCharacter::TurnAtRate(float Rate)
{
	// calculate delta for this frame from the rate information
	//AddControllerYawInput(Rate * BaseTurnRate * GetWorld()->GetDeltaSeconds());
    CameraBehaviour->InputYaw = Rate;
}

void ATHTCharacter::LookUpAtRate(float Rate)
{
	// calculate delta for this frame from the rate information
	//AddControllerPitchInput(Rate * BaseLookUpRate * GetWorld()->GetDeltaSeconds());
    CameraBehaviour->InputPitch = -Rate;
}

void ATHTCharacter::MoveForward(float Value)
{
	if ((Controller != NULL) && (Value != 0.0f))
	{
		// find out which way is forward
		const FRotator Rotation = CameraComponent->GetComponentRotation();// Controller->GetControlRotation();
		const FRotator YawRotation(0, Rotation.Yaw, 0);

		// get forward vector
		const FVector Direction = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::X);
		AddMovementInput(Direction, Value);
	}
}

void ATHTCharacter::MoveRight(float Value)
{
	if ( (Controller != NULL) && (Value != 0.0f) )
	{
		// find out which way is right
        const FRotator Rotation = CameraComponent->GetComponentRotation();// Controller->GetControlRotation();
		const FRotator YawRotation(0, Rotation.Yaw, 0);
	
		// get right vector 
		const FVector Direction = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::Y);
		// add movement in that direction
		AddMovementInput(Direction, Value);
	}
}
