// Fill out your copyright notice in the Description page of Project Settings.

#include "Vehicles/BaseVehicle.h"
#include "Engine/World.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/WidgetComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/Character.h"
#include "Animation/AnimInstance.h"
#include "Vehicles/BaseVehiclePhysics.h"
#include "Vehicles/BaseVehicleCamera.h"

ABaseVehicle::ABaseVehicle()
    : CurrentDriver(nullptr)
    , CurrentPassenger(nullptr)
    , bLightsOn(false)
    , bKinematic(false)
    , bBroken(false)
{
	PrimaryActorTick.bCanEverTick = true;

    Physics = CreateDefaultSubobject<UBaseVehiclePhysics>(TEXT("Physics"));
    Camera = CreateDefaultSubobject<UBaseVehicleCamera>(TEXT("Camera"));
}

void ABaseVehicle::UpdateBrokenStatus()
{
    bool bNewBroken = false;

    if (bNewBroken != bBroken)
    {
        bBroken = bNewBroken;
        if (bBroken)
        {
            AccForceBeforeBroken = Physics->GetAccelerationForce();
            RevAccForceBeforeBroken = Physics->GetReverseAccelerationForce();

            Physics->SetAccelerationForce(.0f);
            Physics->SetReverseAccelerationForce(.0f);
        }
        else
        {
            Physics->SetAccelerationForce(AccForceBeforeBroken);
            Physics->SetReverseAccelerationForce(RevAccForceBeforeBroken);
        }
    }
}

void ABaseVehicle::StartDriving(APawn* Driver)
{
    check(!CurrentDriver);
    CurrentDriver = Driver;

    GetWorld()->GetFirstPlayerController()->Possess(this);

    CurrentDriver->MoveIgnoreActorAdd(this);
    MoveIgnoreActorAdd(CurrentDriver);

    Physics->SetCurrentDriver(CurrentDriver);
    Physics->GetChassis()->SetSimulatePhysics(false); // Force Kinematic to prevent physics exploding...
    Physics->SetForceKinematic(false);

    ReceiveStartDriving(CurrentDriver, true);
}

bool ABaseVehicle::CanEndDriving() const
{
    if (CurrentDriver)
    {
        if (FMath::Abs(Physics->GetCurrentNormalizedSpeed()) > .02f)
            return false;

        return true;
    }

    return false;
}

void ABaseVehicle::BeginEndDriving()
{
    if (bLightsOn)
        ToggleLights();

    if (GetWorld()->GetFirstPlayerController())
        GetWorld()->GetFirstPlayerController()->Possess(CurrentDriver);
}

void ABaseVehicle::EndDriving()
{
    check(CurrentDriver);
    APawn* PreviousDriver = CurrentDriver;
    CurrentDriver = nullptr;

    Physics->SetForceKinematic(true);
    Physics->SetCurrentDriver(nullptr);

    PreviousDriver->MoveIgnoreActorRemove(this);
    MoveIgnoreActorRemove(PreviousDriver);

    ReceiveEndDriving(PreviousDriver);

    if (GetWorld()->GetFirstPlayerController() && GetWorld()->GetFirstPlayerController()->GetPawn() != PreviousDriver)
        GetWorld()->GetFirstPlayerController()->Possess(PreviousDriver);
}

bool ABaseVehicle::GetCurrentDriverCanDrive() const
{
    return true;
}

void ABaseVehicle::PutPassengerInside(APawn* Passenger)
{
    check(!CurrentPassenger);
    CurrentPassenger = Passenger;

    CurrentPassenger->MoveIgnoreActorAdd(this);
    MoveIgnoreActorAdd(CurrentPassenger);

    Physics->SetCurrentPassenger(CurrentPassenger);

    ReceivePassengerEnter(Passenger);
}

bool ABaseVehicle::CanDropOutPassenger()
{
    if (CurrentPassenger)
    {
        if (FMath::Abs(Physics->GetCurrentNormalizedSpeed()) > .02f)
            return false;

        return true;
    }

    return false;
}

APawn* ABaseVehicle::DropOutPassenger()
{
    check(CurrentPassenger);
    APawn* PreviousPassenger = CurrentPassenger;
    CurrentPassenger = nullptr;

    Physics->SetCurrentPassenger(nullptr);

    PreviousPassenger->MoveIgnoreActorRemove(this);
    MoveIgnoreActorRemove(PreviousPassenger);

    ReceivePassengerExit(PreviousPassenger);

    return PreviousPassenger;
}

void ABaseVehicle::ToggleLights()
{
    if (bLightsOn)
    {
        bLightsOn = false;
        ReceiveLightsOff();
    }
    else
    {
        if (CurrentDriver)// && CurrentDriverInterface->CanDrive())
        {
            bLightsOn = true;
            ReceiveLightsOn();
        }
    }
}

float ABaseVehicle::PlayAnimMontage(UAnimMontage* AnimMontage, float InPlayRate, FName StartSectionName)
{
    UAnimInstance * AnimInstance = (SkeletalMesh)? SkeletalMesh->GetAnimInstance() : nullptr;
    if (AnimMontage && AnimInstance)
    {
        float const Duration = AnimInstance->Montage_Play(AnimMontage, InPlayRate);

        if (Duration > 0.f)
        {
            // Start at a given Section.
            if (StartSectionName != NAME_None)
            {
                AnimInstance->Montage_JumpToSection(StartSectionName, AnimMontage);
            }

            return Duration;
        }
    }

    return 0.f;
}

void ABaseVehicle::StopAnimMontage(UAnimMontage* AnimMontage)
{
	UAnimInstance * AnimInstance = (SkeletalMesh)? SkeletalMesh->GetAnimInstance() : nullptr;
	UAnimMontage * MontageToStop = (AnimMontage)? AnimMontage : GetCurrentMontage();
	bool bShouldStopMontage =  AnimInstance && MontageToStop && !AnimInstance->Montage_GetIsStopped(MontageToStop);

	if ( bShouldStopMontage )
	{
		AnimInstance->Montage_Stop(MontageToStop->BlendOut.GetBlendTime(), MontageToStop);
	}
}

UAnimMontage* ABaseVehicle::GetCurrentMontage()
{
	UAnimInstance * AnimInstance = (SkeletalMesh)? SkeletalMesh->GetAnimInstance() : nullptr;
	if ( AnimInstance )
	{
		return AnimInstance->GetCurrentActiveMontage();
	}

	return nullptr;
}

void ABaseVehicle::BeginPlay()
{
    AccForceBeforeBroken = Physics->GetAccelerationForce();
    RevAccForceBeforeBroken = Physics->GetReverseAccelerationForce();

    auto& FrontWheelsPositions = Physics->GetFrontWheelsSettings().Positions;
    auto& BackWheelsPositions = Physics->GetBackWheelsSettings().Positions;

    FrontWheels.SetNum(FrontWheelsPositions.Num(), false);
    BackWheels.SetNum(BackWheelsPositions.Num(), false);

    TInlineComponentArray<USceneComponent*> Nodes;
    GetComponents(Nodes);
    for (auto Node : Nodes)
    {
        bool bIsFrontWheel = Node->ComponentHasTag("FrontWheel"),
             bIsBackWheel  = Node->ComponentHasTag("BackWheel");
        if (bIsFrontWheel || bIsBackWheel)
        {
            FVector NodeLoc = Node->GetSocketTransform(NAME_None, RTS_Actor).GetLocation();

            int32 BestIndex = -1,
                  Index = 0,
                  Count = bIsFrontWheel
                    ? FrontWheelsPositions.Num()
                    : BackWheelsPositions.Num();

            float BestSqDist = TNumericLimits<float>::Max();
            for (; Index < Count; ++Index)
            {
                float SqDist = FVector::DistSquared(NodeLoc, bIsFrontWheel
                    ? FrontWheelsPositions[Index]
                    : BackWheelsPositions[Index]);

                if (SqDist < BestSqDist)
                {
                    BestIndex = Index;
                    BestSqDist = SqDist;
                }
            }

            if (bIsFrontWheel)
                FrontWheels[BestIndex] = Node;
            else
                BackWheels[BestIndex] = Node;
        }
    }

    Super::BeginPlay();

    SkeletalMesh = Cast<USkeletalMeshComponent>(Physics->GetChassis());

    bLightsOn = false;
    ReceiveLightsOff();
}

void ABaseVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (CanEndDriving())
        EndDriving();

    Super::EndPlay(EndPlayReason);
}

void ABaseVehicle::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    PlayerInputComponent->BindAxis("MoveRight");
    PlayerInputComponent->BindAxis("VehicleBrake");
    PlayerInputComponent->BindAxis("VehicleThrottle");
    PlayerInputComponent->BindAxis("Turn");
    PlayerInputComponent->BindAxis("LookUp");

    Super::SetupPlayerInputComponent(PlayerInputComponent);
}

void ABaseVehicle::SetVehicleKinematic(bool bNewKinematic)
{
    if (bKinematic != bNewKinematic)
    {
        Physics->SetForceKinematic(bNewKinematic);
        Physics->GetChassis()->SetSimulatePhysics(!bNewKinematic);

        if (SkeletalMesh)
        {
            SkeletalMesh->PhysicsTransformUpdateMode = bNewKinematic
                ? EPhysicsTransformUpdateMode::ComponentTransformIsKinematic
                : EPhysicsTransformUpdateMode::SimulationUpatesComponentTransform;
        }

        bKinematic = bNewKinematic;
    }
}

void ABaseVehicle::PossessedBy(AController* NewController)
{
	APlayerController* pc = Cast<APlayerController>(NewController);

	if (pc) // need to check this to prevent crashes in playgrounds where aya is already spawned in the level
	{
	}

    Super::PossessedBy(NewController);
}

void ABaseVehicle::UnPossessed()
{
    Super::UnPossessed();
}

void ABaseVehicle::Tick(float DeltaTime)
{
    UpdateBrokenStatus();

    auto& FrontWheelsState = Physics->GetFrontWheelsState();
    for (int32 Index = 0, Count = FrontWheels.Num(); Index < Count; ++Index)
    {
        if (FrontWheels[Index])
        {
            auto& Wheel = FrontWheelsState.Wheels[Index];
            FrontWheels[Index]->SetWorldLocationAndRotation(Wheel.WorldPosition, Wheel.WorldRotation);
            if (Wheel.bOnTheLeft)
                FrontWheels[Index]->AddRelativeRotation(FRotator(.0f, .0f, 180.0f));
        }
    }

    auto& BackWheelsStates = Physics->GetBackWheelsState();
    for (int32 Index = 0, Count = BackWheels.Num(); Index < Count; ++Index)
    {
        if (BackWheels[Index])
        {
            auto& Wheel = BackWheelsStates.Wheels[Index];
            BackWheels[Index]->SetWorldLocationAndRotation(Wheel.WorldPosition, Wheel.WorldRotation);
            if (Wheel.bOnTheLeft)
                BackWheels[Index]->AddRelativeRotation(FRotator(.0f, .0f, 180.0f));
        }
    }

    if (Controller) // ToDo: wait also for player to complete animation
    {
        float SteeringInput = GetInputAxisValue("MoveRight");
        float BrakeInput = GetInputAxisValue("VehicleBrake");
        float ThrottleInput = GetInputAxisValue("VehicleThrottle");

        if (FMath::Abs(SteeringInput) > KINDA_SMALL_NUMBER
            || FMath::Abs(BrakeInput) > KINDA_SMALL_NUMBER
            || FMath::Abs(ThrottleInput) > KINDA_SMALL_NUMBER)
        {
            if (false)//CurrentDriverInterface && !CurrentDriverInterface->CanDrive())
                SteeringInput = BrakeInput = ThrottleInput = .0f;
        }

        Physics->InputSteering = SteeringInput;

        Physics->InputBrake = FMath::Max(.0f, -BrakeInput);
        Physics->InputThrottle = FMath::Max(.0f, ThrottleInput);

        Camera->InputYaw = GetInputAxisValue("Turn");
        Camera->InputPitch = -GetInputAxisValue("LookUp");
    }
    else
    {
        Physics->InputSteering = .0f;
        Physics->InputBrake = .0f;
        Physics->InputThrottle = .0f;
    }

    Super::Tick(DeltaTime);
}
