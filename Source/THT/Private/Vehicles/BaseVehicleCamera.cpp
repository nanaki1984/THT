// Fill out your copyright notice in the Description page of Project Settings.

#include "Vehicles/BaseVehicleCamera.h"
#include "GameFramework/Actor.h"
#include "Camera/CameraComponent.h"
#include "Vehicles/BaseVehiclePhysics.h"
#include "Vehicles/BaseVehicle.h"
#include "DrawDebugHelpers.h"

UBaseVehicleCamera::UBaseVehicleCamera()
{/*
	PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.TickGroup = TG_DuringPhysics;// TG_PrePhysics;// TG_PostPhysics;// TG_PostUpdateWork;
    PrimaryComponentTick.EndTickGroup = TG_DuringPhysics;// TG_PrePhysics;// TG_PostPhysics;// TG_PostUpdateWork;*/
    PrimaryComponentTick.bCanEverTick = false;
}

void UBaseVehicleCamera::BeginPlay()
{
    Camera  = Cast<UCameraComponent>(GetOwner()->GetComponentByClass(UCameraComponent::StaticClass()));
    Physics = Cast<UBaseVehiclePhysics>(GetOwner()->GetComponentByClass(UBaseVehiclePhysics::StaticClass()));
    if (!ensureMsgf(Camera && Physics, TEXT("BaseVehicleCamera w/out Camera Component or Base Vehicle Component!")))
    {
        PrimaryComponentTick.SetTickFunctionEnable(false);
        return;
    }

    //PrimaryComponentTick.AddPrerequisite(Physics, Physics->PrimaryComponentTick);

    ensureMsgf(Data, TEXT("BaseVehicleCamera w/out Data!"));

    PreviousGripForce = .0f;
    PreviousFov = Data ? Data->MinFov : Camera->FieldOfView;
    PreviousReverseLerp = .0f;
    
    const FTransform& Tr = Physics->GetChassisTransform();// GetOwner()->GetTransform();
    PreviousWorldPivot = Tr.TransformPosition(Data ? Data->Pivot : FVector::ZeroVector);
    PreviousBasis = Physics->GetBasisBasedOnWheels();// Tr.GetRotation();

    PreviousYawPitch = FRotator::ZeroRotator;
    WantedYawPitch = FRotator::ZeroRotator;

    PreviousSourceTargetDistance = Data ? FVector::Distance(Data->SourceOffset, Data->TargetOffset) : .0f;

    Super::BeginPlay();
}

void UBaseVehicleCamera::SetupTransitionFromCamera(const FMinimalViewInfo& PreviousViewInfo)
{
    UpdateCameraTransform(.0f);

    FQuat RToViewInfo = PreviousCamRotation.Inverse() * PreviousViewInfo.Rotation.Quaternion();
    //PreviousYawPitch = WantedYawPitch = RToViewInfo.Rotator();
    InterpolationOutYawPitch = RToViewInfo.Rotator();
    PreviousYawPitch = WantedYawPitch = FRotator::ZeroRotator; // Reset camera to default

    InterpolationOutSource = PreviousViewInfo.Rotation.UnrotateVector(PreviousViewInfo.Location - PreviousWorldPivot);
    InterpolationOutFov = PreviousViewInfo.FOV;
    InterpolationTime = .0f;

    bInterpolating = true;
}

//void UBaseVehicleCamera::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
void UBaseVehicleCamera::UpdateCameraTransform(float DeltaTime)
{
	//Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    FTransform Tr = Physics->GetChassisTransform();
    PreviousBasis = FQuat::Slerp(PreviousBasis, Physics->GetBasisBasedOnWheels(), DeltaTime * Data->BasisInterpolation);
    Tr.SetRotation(PreviousBasis);

    FVector Forward = PreviousBasis.GetForwardVector();
    FVector Right   = PreviousBasis.GetRightVector();

    FVector PlaneForward = FVector::VectorPlaneProject(Forward, FVector::UpVector).GetSafeNormal();
    FVector PlaneRight   = FVector::VectorPlaneProject(Right, FVector::UpVector).GetSafeNormal();

    FQuat VehicleToPlane = FQuat::FindBetweenNormals(Forward, PlaneForward);
    FQuat VehicleToRight = FQuat::FindBetweenNormals(Right, PlaneRight);

    PreviousCamRotation = FQuat::Slerp(FQuat::Identity, VehicleToRight, Data->RollNormalize) * FQuat::Slerp(FQuat::Identity, VehicleToPlane, Data->PitchNormalize) * PreviousBasis;

    float ReverseLerp = FMath::InterpEaseInOut(.0f, 1.f, FMath::Max(.0f, -Physics->GetCurrentNormalizedSpeed()), Data->ReverseInterpolationExp);
    PreviousReverseLerp = FMath::Lerp(PreviousReverseLerp, ReverseLerp, DeltaTime * Data->ReverseInterpolation);
    FVector Target   = FMath::Lerp(Data->TargetOffset, Data->ReverseTargetOffset, PreviousReverseLerp),
            Source   = FMath::Lerp(Data->SourceOffset, Data->ReverseSourceOffset, PreviousReverseLerp),
            ToSource = Source - Target;

    bool bWasInterpolating = bInterpolating;
    float InterpPercentage;
    if (bInterpolating)
    {
        InterpolationTime += DeltaTime;
/*
        float S = FMath::Clamp(InterpolationTime / Data->TransitionParams.BlendTime, .0f, 1.f);
        InterpPercentage = Data->TransitionParams.GetBlendAlpha(S);

        Source = FMath::Lerp(InterpolationOutSource, Source, InterpPercentage);

        float ToSourceLen = ToSource.Size();
        ToSource *= (1.f / ToSourceLen);
        FQuat R = FQuat::Slerp(
            FQuat::Identity,
            FQuat::FindBetweenNormals(FVector::ForwardVector, -ToSource),
            InterpPercentage);

        ToSource = -R.RotateVector(FVector::ForwardVector) * ToSourceLen;
        Target = Source - ToSource;
*/
        bInterpolating = false;// (S < 1.f);
    }

    float NormalizedSpeed = FMath::Abs(Physics->GetCurrentNormalizedSpeed());

    WantedYawPitch.Add(Data->InputSpeed * InputPitch * DeltaTime, Data->InputSpeed * InputYaw * DeltaTime, .0f);
    WantedYawPitch.Pitch = FMath::Clamp(WantedYawPitch.Pitch, -45.f, 30.f);
    WantedYawPitch.Yaw = FRotator::NormalizeAxis(WantedYawPitch.Yaw);
    WantedYawPitch *= FMath::Pow(1.f - NormalizedSpeed * Data->InputDamping, Data->InputDampingSpeed * DeltaTime);
    PreviousYawPitch = FMath::Lerp(PreviousYawPitch, WantedYawPitch, Data->InputInterpolation * DeltaTime);

    float Grip = Data->GripFeedbackStrength * Physics->GetNetForceOnAxis(Right) * 1e-8f * FMath::SmoothStep(.0f, .2f, NormalizedSpeed);
    PreviousGripForce = FMath::Lerp(PreviousGripForce, Grip, FMath::Min(1.f, DeltaTime * Data->GripFeedbackInterpolation));

    FVector NewLocalPivot = Tr.InverseTransformPositionNoScale(PreviousWorldPivot),
            InertiaDelta  = Physics->GetLocalVelocity() * Data->PivotInertiaStrength * 0.016f;

    InertiaDelta.Z *= Data->PivotHeightInertiaStrength;
    FVector WantedLocalPivot = Data->Pivot - InertiaDelta;

    float NewLocalPivotZ = NewLocalPivot.Z;
    NewLocalPivot = FMath::Lerp(NewLocalPivot, WantedLocalPivot, DeltaTime * Data->PivotInterpolation);
    NewLocalPivot.Z = FMath::Lerp(NewLocalPivotZ, WantedLocalPivot.Z, DeltaTime * Data->PivotHeightInterpolation);

    PreviousWorldPivot = Tr.TransformPositionNoScale(NewLocalPivot);

    FQuat YawPitchQuat = bWasInterpolating
        ? FMath::Lerp(InterpolationOutYawPitch, PreviousYawPitch, InterpPercentage).Quaternion()
        : PreviousYawPitch.Quaternion();

    FQuat CamRotation = PreviousCamRotation * YawPitchQuat;
    FVector CamSource = PreviousWorldPivot + CamRotation.RotateVector(Target + FQuat(FVector::UpVector, -PreviousGripForce) * ToSource);
    FVector CamTarget = PreviousWorldPivot + CamRotation.RotateVector(Target);

    FHitResult Hit;
    FCollisionQueryParams Params;
    Params.bTraceComplex = false;// true;
    Params.AddIgnoredActor(GetOwner());

    ABaseVehicle* Vehicle = Cast<ABaseVehicle>(GetOwner());
    if (Vehicle)
    {
        if (Vehicle->GetCurrentDriver())
            Params.AddIgnoredActor(Vehicle->GetCurrentDriver());
        if (Vehicle->GetCurrentPassenger())
            Params.AddIgnoredActor(Vehicle->GetCurrentPassenger());
    }

    if (GetWorld()->LineTraceSingleByChannel(Hit, CamTarget, CamSource, Data->CollisionChannel, Params))
        CamSource = Hit.Location;

    ToSource = CamSource - CamTarget;
    ToSource = ToSource.GetClampedToMaxSize(PreviousSourceTargetDistance + Data->DistanceSpeed * DeltaTime); // ToDo: interpolate only if not occluded or distance is greaterequal
    PreviousSourceTargetDistance = ToSource.Size();

    FVector UpVector = FQuat(ToSource.GetSafeNormal(), -Data->GripBankFeedbackStrength * PreviousGripForce) * FVector::UpVector;
    Camera->SetWorldLocationAndRotation(CamTarget + ToSource, FRotationMatrix::MakeFromXZ(-ToSource, UpVector).ToQuat());

    float Fov = FMath::Lerp(Data->MinFov, Data->MaxFov, FMath::Abs(NormalizedSpeed));
    PreviousFov = FMath::Lerp(PreviousFov, Fov, FMath::Min(1.f, DeltaTime * Data->FovInterpolation));

    Camera->FieldOfView = bWasInterpolating
        ? FMath::Lerp(InterpolationOutFov, PreviousFov, InterpPercentage)
        : PreviousFov;
}
