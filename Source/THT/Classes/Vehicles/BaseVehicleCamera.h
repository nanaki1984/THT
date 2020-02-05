// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Engine/DataAsset.h"
#include "BaseVehicleCamera.generated.h"

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class THT_API UBaseVehicleCamera : public UActorComponent
{
	GENERATED_BODY()

protected:

    class UCameraComponent* Camera;
    class UBaseVehiclePhysics* Physics;

    FVector PreviousWorldPivot;
    FQuat PreviousBasis;
    FQuat PreviousCamRotation;
    float PreviousGripForce;
    float PreviousFov;
    float PreviousReverseLerp;

    FRotator PreviousYawPitch;
    FRotator WantedYawPitch;

    float PreviousSourceTargetDistance;

    FVector InterpolationOutSource;
    FRotator InterpolationOutYawPitch;
    float InterpolationOutFov;
    float InterpolationTime;

    uint32 bInterpolating : 1;

public:

	UPROPERTY(Transient, EditInstanceOnly, BlueprintReadWrite)
	/** Between -1 and 1 */
	float InputYaw;
	UPROPERTY(Transient, EditInstanceOnly, BlueprintReadWrite)
	/** Between -1 and 1 */
	float InputPitch;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    class UVehicleCameraData* Data;

    UBaseVehicleCamera();

    virtual void BeginPlay() override;
	//virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    void SetupTransitionFromCamera(const struct FMinimalViewInfo& PreviousViewInfo);

    void UpdateCameraTransform(float DeltaTime);
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class THT_API UVehicleCameraData : public UDataAsset
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, Category="Basis")
    float BasisInterpolation;
    UPROPERTY(EditAnywhere, Category="Basis")
    float RollNormalize;
    UPROPERTY(EditAnywhere, Category="Basis")
    float PitchNormalize;

    UPROPERTY(EditAnywhere, Category="Offsets")
    FVector Pivot;
    UPROPERTY(EditAnywhere, Category="Offsets")
    FVector SourceOffset;
    UPROPERTY(EditAnywhere, Category="Offsets")
    FVector TargetOffset;
    UPROPERTY(EditAnywhere, Category="Offsets")
    FVector ReverseSourceOffset;
    UPROPERTY(EditAnywhere, Category="Offsets")
    FVector ReverseTargetOffset;

    UPROPERTY(EditAnywhere, Category="Reverse")
    float ReverseInterpolation;
    UPROPERTY(EditAnywhere, Category="Reverse")
    float ReverseInterpolationExp;

    UPROPERTY(EditAnywhere, Category="Input")
    float InputSpeed;
    UPROPERTY(EditAnywhere, Category="Input")
    float InputDamping;
    UPROPERTY(EditAnywhere, Category="Input")
    float InputDampingSpeed;
    UPROPERTY(EditAnywhere, Category="Input")
    float InputInterpolation;

    UPROPERTY(EditAnywhere, Category="Grip")
    float GripFeedbackStrength;
    UPROPERTY(EditAnywhere, Category="Grip")
    float GripFeedbackInterpolation;
    UPROPERTY(EditAnywhere, Category="Grip")
    float GripBankFeedbackStrength;

    UPROPERTY(EditAnywhere, Category="Inertia")
    float PivotInertiaStrength;
    UPROPERTY(EditAnywhere, Category="Inertia")
    float PivotHeightInertiaStrength;
    UPROPERTY(EditAnywhere, Category="Inertia")
    float PivotInterpolation;
    UPROPERTY(EditAnywhere, Category="Inertia")
    float PivotHeightInterpolation;

    UPROPERTY(EditAnywhere, Category="FOV")
    float MinFov;
    UPROPERTY(EditAnywhere, Category="FOV")
    float MaxFov;
    UPROPERTY(EditAnywhere, Category="FOV")
    float FovInterpolation;

    UPROPERTY(EditAnywhere, Category="Collisions")
    float DistanceSpeed;
    UPROPERTY(EditAnywhere, Category="Collisions")
    TEnumAsByte<ECollisionChannel> CollisionChannel;
};
