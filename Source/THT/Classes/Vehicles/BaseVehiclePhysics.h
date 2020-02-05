// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "BaseVehiclePhysics.generated.h"

USTRUCT(BlueprintType)
struct FVehicleWheels
{
    GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	/** domain between 0 and +90 degrees, normalized to [0..1], negated for negative angles */
	class UCurveFloat* Grip;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (UIMin = 0, ClampMin = 0, UIMax = 1, ClampMax = 1))
	/** Between 0 and 1 */
	float Traction;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (UIMin = 0, ClampMin = 0, UIMax = 1, ClampMax = 1))
	/** Between 0 and 1 */
	float ReverseTraction;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (UIMin = 0, ClampMin = 0, UIMax = 1, ClampMax = 1))
	/** Between 0 and 1 */
	float Brake;

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	class UCurveFloat* TractionScaleByNormalizedSpeed;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
	class UCurveFloat* BrakeScaleByNormalizedSpeed;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (UIMin = 0, ClampMin = 0))
	float SpringStrength;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (UIMin = 0, ClampMin = 0))
	float SpringDamping;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (UIMin = 0, ClampMin = 0))
	float SpringRestLength;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (UIMin = 0, ClampMin = 0, UIMax = 1, ClampMax = 1))
	float SpringMaxCompression;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (UIMin = 0, ClampMin = 0, UIMax = 0.3, ClampMax = 0.3))
	float BetaPenaltyImpulses;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (UIMin = 0, ClampMin = 0))
	float Radius;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (UIMin = 0, ClampMin = 0))
	float Width;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    TArray<FVector> Positions;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    TArray<float> ForceMultipliers;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    uint32 bSteering : 1;
};

/** Struct to contain wheel state, only for physics substepping */
struct FVehicleSingleWheelStateProxy
{
    FVector WorldPosition;
    FVector WorldVelocity;
	FVector WorldForces;
    FVector WorldImpulses;
    FVector ContactPoint;
    FVector ContactNormal;

    FQuat LocalRotation;

    FVector PreviousLocalPosition0;
    FVector PreviousLocalPosition1;
    FVector PreviousWorldImpulse;

    float LocalRollAngle;
    float PreviousDeltaTime;
    float NormalForce;

    uint32 bNeedsImpulse : 1;
    uint32 bOnTheLeft    : 1;
    uint32 bOnGround     : 1;

	/** Actor hit by the trace. */
	TWeakObjectPtr<class AActor> HitActor;

	/** PrimitiveComponent hit by the trace. */
	TWeakObjectPtr<class UPrimitiveComponent> HitComponent;

    void Reset(const FTransform& ChassisTransform, const FVector& LocalPosition);
};

/** Struct to contain all wheels states, only for physics substepping */
struct FVehicleWheelStateProxy
{
    TArray<FVehicleSingleWheelStateProxy> Wheels;
};

USTRUCT(BlueprintType)
struct FVehicleSingleWheelState
{
    GENERATED_BODY()

    UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
	FVector WorldPosition;

    UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
	FRotator WorldRotation;

    UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
	FVector WorldVelocity;

    UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
	FVector ContactPoint;

    UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
	FVector ContactNormal;

    FVector WorldForces;
    float   NormalForce;

    UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
	uint32 bOnTheLeft : 1;
    UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
	uint32 bOnGround : 1;

	UPROPERTY()
	/** Actor hit by the trace. */
	TWeakObjectPtr<class AActor> HitActor;

	UPROPERTY()
	/** PrimitiveComponent hit by the trace. */
	TWeakObjectPtr<class UPrimitiveComponent> HitComponent;

    void CopyFrom(const FTransform& ChassisTransform, const FVehicleSingleWheelStateProxy& Proxy);
};

USTRUCT(BlueprintType)
struct FVehicleWheelState
{
    GENERATED_BODY()

	UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
    TArray<FVehicleSingleWheelState> Wheels;
};

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class THT_API UBaseVehiclePhysics
    : public UActorComponent
{
	GENERATED_BODY()

protected:

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Speed", meta = (UIMin = 0, ClampMin = 0))
	float MaxSpeed;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Speed", meta = (UIMin = 0, ClampMin = 0))
	float MaxReverseSpeed;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Speed", meta = (UIMin = 0, ClampMin = 0))
	float ReverseCheckSpeedThreshold;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Steering", meta = (UIMin = 0, ClampMin = 0, UIMax = 90, ClampMax = 90))
	float MaxSteeringAngle;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Steering", meta = (UIMin = 0, ClampMin = 0, UIMax = 1, ClampMax = 1))
	float SteeringAngleSpeedReduction;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Steering", meta = (UIMin = 0, ClampMin = 0, UIMax = 1, ClampMax = 1))
	float MaxSteeringLag;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Steering", meta = (UIMin = 0, ClampMin = 0, UIMax = 1, ClampMax = 1))
	float MinSteeringLag;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traction Forces", meta = (UIMin = 0, ClampMin = 0))
	float AccelerationForce;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traction Forces", meta = (UIMin = 0, ClampMin = 0))
	float ReverseAccelerationForce;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traction Forces", meta = (UIMin = 0, ClampMin = 0))
	float BrakeForce;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grip", meta = (UIMin = 0, ClampMin = 0))
	float GripStrength;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Grip", meta = (UIMin = 0, ClampMin = 0))
	float GripThreshold;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Gravity", meta = (UIMin = 1, ClampMin = 1, UIMax = 4, ClampMax = 4))
	float GravityScale;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Friction Forces", meta = (UIMin = 0, ClampMin = 0))
	float TiresStaticFrictionSqSpeedThreshold;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Friction Forces", meta = (UIMin = 0, ClampMin = 0, UIMax = 1, ClampMax = 1))
	float TiresStaticFriction;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Friction Forces", meta = (UIMin = 0, ClampMin = 0, UIMax = 1, ClampMax = 1))
	float TiresDynamicFriction;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drag", meta = (UIMin = 0, ClampMin = 0, UIMax = 1, ClampMax = 1))
	float DragCoefficient;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Roll Correction", meta = (UIMin = 0, ClampMin = 0, UIMax = 90, ClampMax = 90))
	float MinRollAngleForCorrection;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Roll Correction", meta = (UIMin = 1, ClampMin = 1, UIMax = 4, ClampMax = 4))
	float RollCorrectionStrength;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Wheels")
    TEnumAsByte<ECollisionChannel> WheelsCollisionChannel;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Wheels")
    FCollisionResponseContainer WheelsCollisionResponce;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Wheels")
	FVehicleWheels FrontWheels;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Wheels")
	FVehicleWheels BackWheels;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damage", meta = (UIMin = 0, ClampMin = 0))
	float DamageScaler;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damage", meta = (UIMin = 0, ClampMin = 0, UIMax = 4, ClampMax = 4))
	float DamageCurvePower;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damage", meta = (UIMin = 0, ClampMin = 0))
	float DamageMinNDotV;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damage", meta = (UIMin = 0, ClampMin = 0, UIMax = 1, ClampMax = 1))
	float DamageSteeringLagMultiplier;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damage", meta = (UIMin = 0, ClampMin = 0, UIMax = 1, ClampMax = 1))
	float DamageGripMultiplier;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damage", meta = (UIMin = 0, ClampMin = 0, UIMax = 1, ClampMax = 1))
	float DamageSpeedMultiplier;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damage", meta = (UIMin = 0, ClampMin = 0))
	float DamageSpringsRandomOffset;

    UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
	FVehicleWheelState FrontWheelsState;
	UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
	FVehicleWheelState BackWheelsState;

	UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
	float CurrentSpeed;
	UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
	float CurrentNormalizedSpeed;
	UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
	float CurrentSteering;

    FVehicleWheelStateProxy FrontWheelsStateProxy;
    FVehicleWheelStateProxy BackWheelsStateProxy;

    FCalculateCustomPhysics VehiclePhysicsCallback;

    FTransform ChassisTransform;
    FVector LocalVelocity;

    FVector CentripetalCenter;
    float CentripetalRadius;
    float ForceBrakeLerp;

    /** Chassis (Root component of owner) */
    class UPrimitiveComponent* Chassis;
    class UBaseVehicleCamera* Camera;

    class UDamageComponent* Damage;

    class APawn* CurrentDriver;
    class UPrimitiveComponent* CurrentDriverRoot;

    class APawn* CurrentPassenger;
    class UPrimitiveComponent* CurrentPassengerRoot;

    UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
    uint32 bVehicleOnGround : 1;
	UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
	uint32 bInReverse : 1;
	UPROPERTY(Transient, VisibleInstanceOnly, BlueprintReadOnly)
	uint32 bBraking : 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flags")
	uint32 bForceKinematic : 1;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Flags")
	uint32 bReceivesDamage : 1;

    uint32 bForceBrake : 1;

    bool GetSimpleWheelCollision(FHitResult& Hit, const FVector& SpringStart, const FVector& SpringEnd);
    bool GetWheelCollision(FHitResult& Hit, const FVector& SpringStart, const FVector& SpringEnd, const FQuat& Rotation, float TireRadius, float TireWidth);
    bool UpdateWheels(const FVehicleWheels& Wheels, FVehicleWheelStateProxy& State, float DeltaTime, struct FBodyInstance* Body, float NormalizedDamage, bool bInverseSteering = false);
    void UpdatePhysics(float DeltaTime, struct FBodyInstance* Body);

    UFUNCTION()
    void OnChassisHit(class UPrimitiveComponent* HitComponent, AActor* OtherActor, class UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit);

public:
	UPROPERTY(Transient, EditInstanceOnly, BlueprintReadWrite)
	/** Between -1 and 1 */
	float InputSteering;
	UPROPERTY(Transient, EditInstanceOnly, BlueprintReadWrite)
	/** Between 0 and 1 */
	float InputThrottle;
	UPROPERTY(Transient, EditInstanceOnly, BlueprintReadWrite)
	/** Between 0 and 1 */
	float InputBrake;

    UBaseVehiclePhysics();
    
	FORCEINLINE class UPrimitiveComponent* GetChassis() const { return Chassis; }
    FORCEINLINE const FTransform& GetChassisTransform() const { return ChassisTransform; }
    FORCEINLINE const FVector& GetLocalVelocity() const { return LocalVelocity; }

	FORCEINLINE float GetMaxSpeed() const { return MaxSpeed; }
	FORCEINLINE float GetMaxReverseSpeed() const { return MaxReverseSpeed; }
	FORCEINLINE float GetMaxSteeringAngle() const { return MaxSteeringAngle; }

	FORCEINLINE void SetMaxSpeed(float NewMaxSpeed) { MaxSpeed = FMath::Max(.0f, NewMaxSpeed); }
	FORCEINLINE void SetMaxReverseSpeed(float NewMaxReverseSpeed) { MaxReverseSpeed = FMath::Max(.0f, NewMaxReverseSpeed); }
	FORCEINLINE void SetMaxSteeringAngle(float NewMaxSteeringAngle) { MaxSteeringAngle = FMath::Max(.0f, NewMaxSteeringAngle); }

    FORCEINLINE const FVehicleWheels& GetFrontWheelsSettings() const { return FrontWheels; }
    FORCEINLINE const FVehicleWheels& GetBackWheelsSettings() const { return BackWheels; }
    FORCEINLINE const FVehicleWheelState& GetFrontWheelsState() const { return FrontWheelsState; }
    FORCEINLINE const FVehicleWheelState& GetBackWheelsState() const { return BackWheelsState; }

    FORCEINLINE float GetAccelerationForce() const { return AccelerationForce; }
    FORCEINLINE float GetReverseAccelerationForce() const { return ReverseAccelerationForce; }

    FORCEINLINE void SetAccelerationForce(float NewAccelerationForce) { AccelerationForce = NewAccelerationForce; }
    FORCEINLINE void SetReverseAccelerationForce(float NewReverseAccelerationForce) { ReverseAccelerationForce = NewReverseAccelerationForce; }

    FORCEINLINE bool GetForceKinematic() const { return bForceKinematic; }
    FORCEINLINE void SetForceKinematic(bool bNewForceKinematic) { bForceKinematic = bNewForceKinematic; }

    FORCEINLINE float GetNetForceOnAxis(const FVector& Axis) const
    {
        float Forces = .0f;
        for (auto& Wheel : FrontWheelsState.Wheels)
            Forces += FVector::DotProduct(Wheel.WorldForces, Axis);
        for (auto& Wheel : BackWheelsState.Wheels)
            Forces += FVector::DotProduct(Wheel.WorldForces, Axis);
        return Forces;
    }

    UFUNCTION(BlueprintCallable)
    FORCEINLINE float GetNetNormalForce() const
    {
        float NetNormalForce = .0f;
        for (auto& Wheel : FrontWheelsState.Wheels)
            NetNormalForce += Wheel.NormalForce;
        for (auto& Wheel : BackWheelsState.Wheels)
            NetNormalForce += Wheel.NormalForce;
        return NetNormalForce;
    }

    FORCEINLINE float GetCurrentSpeed() const { return CurrentSpeed; }
    FORCEINLINE float GetCurrentNormalizedSpeed() const { return CurrentNormalizedSpeed; }
    FORCEINLINE float GetCurrentSteering() const { return CurrentSteering; }
    FORCEINLINE bool IsOnGround() const { return bVehicleOnGround; }

    UFUNCTION(BlueprintCallable)
    FORCEINLINE float GetThrottle() const { return bInReverse ? FMath::Max(.0f, InputBrake) : FMath::Max(.0f, InputThrottle); }
    UFUNCTION(BlueprintCallable)
    FORCEINLINE float GetBrake() const { return bInReverse ? FMath::Max(.0f, InputThrottle) : FMath::Max(.0f, InputBrake); }

    FORCEINLINE bool InReverse() const { return bInReverse; }
    FORCEINLINE bool Braking() const { return bBraking; }

    FQuat GetBasisBasedOnWheels() const;

    FORCEINLINE class APawn* GetCurrentDriver() const { return CurrentDriver; }
    void SetCurrentDriver(class APawn* InCurrentDriver);

    FORCEINLINE class APawn* GetCurrentPassenger() const { return CurrentPassenger; }
    void SetCurrentPassenger(class APawn* InCurrentPassenger);

    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
};
