// Fill out your copyright notice in the Description page of Project Settings.

#include "Vehicles/BaseVehiclePhysics.h"
#include "EngineUtils.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Components/PrimitiveComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Curves/CurveFloat.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "Vehicles/BaseVehicleCamera.h"
#include "Engine/EngineTypes.h"
#include "PhysicalMaterials/PhysicalMaterial.h"

DEFINE_LOG_CATEGORY_STATIC(LogVehicles, Verbose, All);

void FVehicleSingleWheelStateProxy::Reset(const FTransform& ChassisTransform, const FVector& LocalPosition)
{
    WorldPosition = ChassisTransform.TransformPositionNoScale(LocalPosition);
    WorldVelocity = FVector::ZeroVector;
    WorldForces   = FVector::ZeroVector;
    WorldImpulses = FVector::ZeroVector;
    ContactPoint  = FVector::ZeroVector;
    ContactNormal = FVector::ZeroVector;

    LocalRotation = FQuat::Identity;

    PreviousLocalPosition0 = LocalPosition;
    PreviousLocalPosition1 = LocalPosition;
    PreviousWorldImpulse   = FVector::ZeroVector;

    LocalRollAngle    = .0f;
    PreviousDeltaTime = 1.f / 60.0f; // ToDo: get substep delta time
    NormalForce       = .0f;

    bNeedsImpulse = false;
    bOnTheLeft    = LocalPosition.Y < .0f;
    bOnGround     = false;

	HitActor = nullptr;
	HitComponent = nullptr;
}

void FVehicleSingleWheelState::CopyFrom(const FTransform& ChassisTransform, const FVehicleSingleWheelStateProxy& Proxy)
{
    WorldPosition = ChassisTransform.TransformPositionNoScale(Proxy.PreviousLocalPosition0);
    WorldRotation = (ChassisTransform.GetRotation() * Proxy.LocalRotation).Rotator();

    WorldVelocity = Proxy.WorldVelocity;
    ContactPoint  = Proxy.ContactPoint;
    ContactNormal = Proxy.ContactNormal;
    WorldForces   = Proxy.WorldForces;
    NormalForce   = Proxy.NormalForce;
    bOnTheLeft    = Proxy.bOnTheLeft;
    bOnGround     = Proxy.bOnGround;
	HitActor	  = Proxy.HitActor;
	HitComponent  = Proxy.HitComponent;
}

UBaseVehiclePhysics::UBaseVehiclePhysics()
{
	PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.TickGroup = TG_PostPhysics;
    PrimaryComponentTick.EndTickGroup = TG_PostPhysics;

    VehiclePhysicsCallback.BindUObject(this, &UBaseVehiclePhysics::UpdatePhysics);
}

bool UBaseVehiclePhysics::GetSimpleWheelCollision(FHitResult& Hit, const FVector& SpringStart, const FVector& SpringEnd)
{
    UWorld* World = GetWorld();

    FCollisionQueryParams Params;
    Params.bTraceComplex = false;// true;
    Params.AddIgnoredActor(GetOwner());
    Params.AddIgnoredActors(Chassis->MoveIgnoreActors);

    return World->LineTraceSingleByChannel(Hit, SpringStart, SpringEnd, WheelsCollisionChannel, Params, WheelsCollisionResponce);
}

bool UBaseVehiclePhysics::GetWheelCollision(FHitResult& Hit, const FVector& SpringStart, const FVector& SpringEnd, const FQuat& Rotation, float TireRadius, float TireWidth)
{
    check(TireWidth > KINDA_SMALL_NUMBER);

    UWorld* World = GetWorld();

    FCollisionQueryParams Params;
    Params.bTraceComplex = false;// true;
    Params.AddIgnoredActor(GetOwner());
    Params.AddIgnoredActors(Chassis->MoveIgnoreActors);

    FCollisionShape Sphere = FCollisionShape::MakeSphere(TireRadius);
    FVector Right = Rotation.GetRightVector();
    const float HalfTireWidth = TireWidth * .5f;

    Hit.Reset();
    while (World->SweepSingleByChannel(Hit, SpringStart, SpringEnd, Rotation, WheelsCollisionChannel, Sphere, Params, WheelsCollisionResponce))
    {
        FVector V = Hit.ImpactPoint - Hit.Location;
        float Dot = FVector::DotProduct(V, Right);

        if (FMath::Abs(Hit.ImpactNormal.Z) > KINDA_SMALL_NUMBER && FMath::Abs(Dot) <= HalfTireWidth) // ToDo: Slope Test
        {
            V -= Dot * Right;
            V.Normalize();

            Hit.ImpactPoint = Hit.Location + V * TireRadius;

            return true;
        }

        if (!Hit.Actor.IsValid()) // ToDo: brush meshes!
            break;

        Params.AddIgnoredActor(Hit.Actor.Get());
        Hit.Reset();
    }

    FVector Up = Rotation.GetUpVector();
    if (GetSimpleWheelCollision(Hit, SpringStart, SpringEnd - Up * TireRadius))
    {
        Hit.Location = Hit.ImpactPoint + Up * TireRadius;
        return true;
    }

    return false;
}

bool UBaseVehiclePhysics::UpdateWheels(const FVehicleWheels& Wheels, FVehicleWheelStateProxy& States, float DeltaTime, FBodyInstance* Body, float NormalizedDamage, bool bInverseSteering)
{
    UWorld* World = GetOwner()->GetWorld();

    bool bAnyWheelsOnGround = false;
    bool bUseTireWidth = (Wheels.Width > KINDA_SMALL_NUMBER);

    FHitResult Hit;

    FVector ChassisUp = ChassisTransform.GetUnitAxis(EAxis::Z);
    FVector ChassisCoM = Body->GetCOMPosition();

    FVector ChassisInvI = Body->GetBodyInertiaTensor();
    ChassisInvI.X = 1.f / ChassisInvI.X;
    ChassisInvI.Y = 1.f / ChassisInvI.Y;
    ChassisInvI.Z = 1.f / ChassisInvI.Z;

    float ChassisMass = Body->GetBodyMass(),
          ChassisInvM = 1.f / ChassisMass,
          OoDeltaTime = 1.f / DeltaTime;

    const float GripSqThreshold = GripThreshold * GripThreshold;

    int32 Index = 0;
    for (auto& LocalPos : Wheels.Positions)
    {
        float ForceMul = Wheels.ForceMultipliers[Index];
        FVehicleSingleWheelStateProxy& State = States.Wheels[Index++];

        State.PreviousWorldImpulse = State.WorldImpulses;
        State.WorldImpulses = FVector::ZeroVector;

        FQuat WorldRot = Wheels.bSteering
            ? FQuat(FVector::UpVector, FMath::DegreesToRadians(MaxSteeringAngle * FMath::Lerp(1.f, SteeringAngleSpeedReduction, FMath::Abs(CurrentNormalizedSpeed)) * CurrentSteering * (bInverseSteering ? -1.f : 1.f)))
            : FQuat::Identity;
        WorldRot = ChassisTransform.GetRotation() * WorldRot;

        FVector LocalSpringStart = LocalPos + FVector::UpVector * Wheels.SpringRestLength;

        FVector SpringStart = ChassisTransform.TransformPositionNoScale(LocalSpringStart);
        FVector SpringEnd   = ChassisTransform.TransformPositionNoScale(LocalPos - FVector::UpVector * (bUseTireWidth ? .0f : Wheels.Radius));

        bool bAppliedStaticFriction = false;

        bool bOnGround = State.bOnGround = (bUseTireWidth
            ? GetWheelCollision(Hit, SpringStart, SpringEnd, WorldRot, Wheels.Radius, Wheels.Width)
            : GetSimpleWheelCollision(Hit, SpringStart, SpringEnd));
        bAnyWheelsOnGround |= bOnGround;

		State.HitActor = Hit.Actor;
		State.HitComponent = Hit.Component;

        FVector WorldPosition = bOnGround
            ? (bUseTireWidth ? Hit.Location : Hit.ImpactPoint + ChassisUp * Wheels.Radius)
            : ChassisTransform.TransformPositionNoScale(State.PreviousLocalPosition0);
        FVector PreviousWorldVelocity = State.WorldVelocity;
        FVector WorldVelocity = State.WorldVelocity = Body->GetUnrealWorldVelocityAtPoint(bOnGround ? Hit.ImpactPoint : WorldPosition);

        float SpringLength = FMath::Max(.0f, bOnGround
            ? (bUseTireWidth ? FVector::DotProduct(SpringStart - WorldPosition, ChassisUp) : Hit.Distance - Wheels.Radius)
            : FVector::DotProduct(LocalSpringStart - State.PreviousLocalPosition0, FVector::UpVector));

        State.bNeedsImpulse = false;// FVector::DotProduct(State.PreviousWorldImpulse, State.PreviousWorldImpulse) > KINDA_SMALL_NUMBER;
        FVector R = ChassisTransform.InverseTransformVectorNoScale(WorldPosition - ChassisCoM);

        float LocalRollAngle = State.LocalRollAngle;

        float SpringOffset = (1.f + FMath::Sin(LocalRollAngle / PI)) * .5f * NormalizedDamage * DamageSpringsRandomOffset;
        float SpringForces = Wheels.SpringStrength * FMath::Min(1.f, GravityScale) * (Wheels.SpringRestLength - SpringLength - SpringOffset);
        if (bOnGround)
        {
            SpringForces -= FVector::DotProduct(WorldVelocity, ChassisUp) * Wheels.SpringDamping;

            float SpringMinLen = Wheels.SpringRestLength * Wheels.SpringMaxCompression;
            if (SpringLength < SpringMinLen)
            {
                FVector V = FVector::CrossProduct(FVector::CrossProduct(R, FVector::UpVector) * ChassisInvI, R);
                float OoK = 1.f / (ChassisInvM + FVector::DotProduct(V, FVector::UpVector)); // All computations are in local space but it's fine

                const float Threshold = 2.f;
                float Penalty = Wheels.BetaPenaltyImpulses * FMath::Max(.0f, SpringMinLen - SpringLength - Threshold) * OoDeltaTime;

                State.WorldImpulses += FMath::Max(.0f, -FVector::DotProduct(WorldVelocity, ChassisUp) + Penalty) * OoK * ChassisUp;
                State.bNeedsImpulse = true;

                //DrawDebugLine(World, WorldPosition, WorldPosition + State.WorldImpulses * .01f, FColor::Red, false, -1.f, 0, 1.f);
            }

            FVector LocalProjVel = ChassisTransform.InverseTransformVectorNoScale(FVector::VectorPlaneProject(WorldVelocity, ChassisUp));
            float LocalProjVelMag = LocalProjVel.SizeSquared();
            if (LocalProjVelMag <= TiresStaticFrictionSqSpeedThreshold && LocalProjVelMag > .0f)
            {
                LocalProjVelMag = FMath::Sqrt(LocalProjVelMag);
                LocalProjVel *= (1.f / LocalProjVelMag);

                FVector V = FVector::CrossProduct(FVector::CrossProduct(R, LocalProjVel) * ChassisInvI, LocalProjVel);
                float K = ((1.f / Body->GetBodyMass()) + FVector::DotProduct(V, LocalProjVel));

                float DeltaVelocity = TiresStaticFriction * FMath::Max(.0f, SpringForces) * K * DeltaTime;
                if (DeltaVelocity >= LocalProjVelMag)
                {
                    State.WorldImpulses += -ChassisTransform.TransformVectorNoScale(LocalProjVel) * (LocalProjVelMag / K);
                    State.bNeedsImpulse = true;

                    bAppliedStaticFriction = true;
                }
            }
        }
        else
        { // spring forces are inside the system, no need to apply forces on chassis, just use verlet on local wheel position
            FVector X0 = State.PreviousLocalPosition0,
                    X1 = State.PreviousLocalPosition1,
                    DX = (X0 - X1).ProjectOnTo(FVector::UpVector);

            SpringForces += FVector::DotProduct(DX * OoDeltaTime, FVector::UpVector) * Wheels.SpringDamping;
            SpringForces *= ChassisInvM;

            FVector NewLocalPos = X0
                + DX * (DeltaTime / State.PreviousDeltaTime)
                - .5f * (SpringForces * FVector::UpVector) * DeltaTime * (DeltaTime + State.PreviousDeltaTime);
            NewLocalPos.Z = FMath::Clamp(NewLocalPos.Z, LocalPos.Z, LocalSpringStart.Z);

            WorldPosition = ChassisTransform.TransformPositionNoScale(NewLocalPos);

            SpringForces = .0f;
        }

        //DrawDebugLine(World, WorldPosition, WorldPosition + (SpringForces * .01f) * FVector::UpVector, FColor::Red, false, -1.f, 1, 0.f);

        State.PreviousLocalPosition1 = State.PreviousLocalPosition0;
        State.PreviousLocalPosition0 = ChassisTransform.InverseTransformPositionNoScale(WorldPosition);
        State.WorldPosition = WorldPosition;

        FVector WorldForces = ChassisUp * SpringForces;

        //DrawDebugLine(World, WorldPosition, WorldPosition + WorldForces * .01f, FColor::Red, false, -1.f, 1, 0.f);

        float N = .0f;
        if (bOnGround)
        {
            State.ContactPoint  = Hit.ImpactPoint;
            State.ContactNormal = Hit.ImpactNormal;

            //DrawDebugLine(World, Hit.ImpactPoint, Hit.ImpactPoint + Hit.ImpactNormal * 100.f, FColor::Red, false, -1.f, 0, 1.f);

            FVector Forward = WorldRot.RotateVector(FVector::ForwardVector);
            Forward = FVector::VectorPlaneProject(Forward, Hit.ImpactNormal);
            Forward.Normalize();

            FVector Right = FVector::CrossProduct(Forward, Hit.ImpactNormal);

            LocalRollAngle += FVector::DotProduct(WorldVelocity, Forward) * DeltaTime / Wheels.Radius;

            N = FMath::Max(.0f, FVector::DotProduct(Hit.ImpactNormal, WorldForces)) / FMath::Min(1.f, GravityScale);

            FVector TractionForce;
            if (bInReverse)
            {
                float TractionMult = Wheels.TractionScaleByNormalizedSpeed
                    ? Wheels.TractionScaleByNormalizedSpeed->GetFloatValue(FMath::Max(.0f, -CurrentNormalizedSpeed))
                    : 1.f;
                TractionForce = FMath::Min(.0f, -InputBrake) * TractionMult * ReverseAccelerationForce * Wheels.ReverseTraction * N * Forward;

                if (bBraking)
                {
                    float BrakeMult = Wheels.BrakeScaleByNormalizedSpeed
                        ? Wheels.BrakeScaleByNormalizedSpeed->GetFloatValue(FMath::Max(.0f, -CurrentNormalizedSpeed))
                        : 1.f;
                    TractionForce += FMath::Max(.0f, InputThrottle) * BrakeMult * BrakeForce * Wheels.ReverseTraction * N * Forward;
                }
            }
            else
            {
                float TractionMult = Wheels.TractionScaleByNormalizedSpeed
                    ? Wheels.TractionScaleByNormalizedSpeed->GetFloatValue(FMath::Max(.0f, CurrentNormalizedSpeed))
                    : 1.f;
                TractionForce = FMath::Max(.0f, InputThrottle) * TractionMult * AccelerationForce * Wheels.Traction * N * Forward;

                if (bBraking)
                {
                    float BrakeMult = Wheels.BrakeScaleByNormalizedSpeed
                        ? Wheels.BrakeScaleByNormalizedSpeed->GetFloatValue(FMath::Max(.0f, CurrentNormalizedSpeed))
                        : 1.f;
                    TractionForce += FMath::Min(.0f, -InputBrake) * BrakeMult * BrakeForce * Wheels.Traction * N * Forward;
                }
            }

            //DrawDebugLine(World, WorldPosition, WorldPosition + TractionForce * .01f, FColor::Blue, false, -1.f, 1, 0.f);
            WorldForces += TractionForce * ForceMul;

            FVector ProjVel = FVector::VectorPlaneProject(WorldVelocity, Hit.ImpactNormal),
                    NormVel = ProjVel;

            float SqProjVel  = FVector::DotProduct(ProjVel, ProjVel),
                  ProjVelMag = .0f,
                  GripMult   = ForceMul;

            if (SqProjVel < KINDA_SMALL_NUMBER)
            {
                NormVel = Forward;
                ProjVel = FVector::ZeroVector;
            }
            else
            {
                ProjVelMag = FMath::Sqrt(SqProjVel);

                NormVel /= ProjVelMag;
                ProjVel = NormVel;

                //GripMult *= FMath::Min(1.f, SqProjVel / GripSqThreshold);
                GripMult *= FMath::InterpEaseOut(.0f, 1.f, FMath::Min(1.f, SqProjVel / GripSqThreshold), 2.f);
            }

            GripMult *= FMath::Lerp(1.f, DamageGripMultiplier, NormalizedDamage);

            float Angle = 2.f * FMath::Atan2(FVector::DotProduct(Right, NormVel), FMath::Abs(FVector::DotProduct(Forward, NormVel))) / PI;
            float Sign = -1.f;
            if (Angle < 0.0f)
            {
                Angle = -Angle;
                Sign = 1.f;
            }

            FVector GripForce = (Sign * Wheels.Grip->GetFloatValue(FMath::Min(1.f, Angle)) * N * GripMult * GripStrength) * Right;
            //DrawDebugLine(World, WorldPosition, WorldPosition + GripForce * .01f, FColor::Green, false, -1.f, 1, 0.f);
            WorldForces += GripForce;

            // Tire dynamic friction
            if (!bAppliedStaticFriction)
            {
                float FrictionForce = TiresDynamicFriction * N;
                WorldForces -= ProjVel * FrictionForce;
            }

            //DrawDebugLine(World, WorldPosition, WorldPosition + WorldForces * .01f, FColor::Green, false, -1.f, 1, 0.f);
        }
        else
        {
            State.ContactPoint  = FVector::ZeroVector;
            State.ContactNormal = FVector::ZeroVector;

            LocalRollAngle += (bInReverse
                ? FMath::Min(.0f, InputThrottle - InputBrake)
                : FMath::Max(.0f, InputThrottle - InputBrake)) * (MaxSpeed / Wheels.Radius) * DeltaTime;
        }

        State.LocalRotation = ChassisTransform.GetRotation().Inverse() * (WorldRot * FQuat(FVector::RightVector, LocalRollAngle));
        State.WorldForces = WorldForces;
        State.LocalRollAngle = LocalRollAngle;
        State.PreviousDeltaTime = DeltaTime;
        State.NormalForce = N;
    }

    return bAnyWheelsOnGround;
}

void UBaseVehiclePhysics::UpdatePhysics(float DeltaTime, FBodyInstance* Body)
{
    check(Chassis);

    float NormalizedDamage = .0f;
    //if (Damage)
    //{
    //    NormalizedDamage = (Damage->GetMaxHealth() - Damage->GetCurrentHealth()) / Damage->GetMaxHealth();
    //    NormalizedDamage = FMath::Pow(FMath::Clamp(NormalizedDamage, .0f, 1.f), DamageCurvePower);
    //}

    float PrevSpeed = CurrentSpeed;

    FVector Forward  = ChassisTransform.GetUnitAxis(EAxis::X),
            WorldVel = Body->GetUnrealWorldVelocity();

    CurrentSpeed = FVector::DotProduct(Forward, WorldVel);

    if (CurrentSpeed > MaxSpeed)
        Body->AddImpulse(Forward * (MaxSpeed - CurrentSpeed), true);
    else if (CurrentSpeed < -MaxReverseSpeed)
        Body->AddImpulse(-Forward * (MaxReverseSpeed + CurrentSpeed), true);

    CurrentNormalizedSpeed = FMath::Clamp(CurrentSpeed / (CurrentSpeed < .0f ? MaxReverseSpeed : MaxSpeed), -1.f, 1.f);

    float SteeringLagBasedOnSpeed = FMath::Lerp(MaxSteeringLag, MinSteeringLag, FMath::Abs(CurrentNormalizedSpeed));
    SteeringLagBasedOnSpeed *= (1.f + DamageSteeringLagMultiplier * NormalizedDamage);
    CurrentSteering = FMath::Lerp(CurrentSteering, InputSteering, 1.f - FMath::Pow(SteeringLagBasedOnSpeed, DeltaTime));

    if (bInReverse)
    {
        if (FMath::Abs(CurrentSpeed) <= ReverseCheckSpeedThreshold && InputThrottle > .0f)
            bInReverse = false;
    }
    else
    {
        if (FMath::Abs(CurrentSpeed) <= ReverseCheckSpeedThreshold && InputBrake > .0f)
            bInReverse = true;
    }

    if (bForceBrake)
        ForceBrakeLerp = FMath::FInterpConstantTo(ForceBrakeLerp, 1.f, DeltaTime, 12.f);
    else
        ForceBrakeLerp = FMath::FInterpConstantTo(ForceBrakeLerp, .0f, DeltaTime, 8.f);
    bForceBrake = false;

    if (ForceBrakeLerp > .0f)
    {
        if (bInReverse)
        {
            InputThrottle = FMath::Lerp(InputThrottle, 1.f, ForceBrakeLerp);
            InputBrake = FMath::Lerp(InputBrake, .0f, ForceBrakeLerp);
        }
        else
        {
            InputThrottle = FMath::Lerp(InputThrottle, .0f, ForceBrakeLerp);
            InputBrake = FMath::Lerp(InputBrake, 1.f, ForceBrakeLerp);
        }
    }
    bBraking = bInReverse ? (InputThrottle > .0f) : (InputBrake > .0f);

    // Compute & apply wheels forces
    bVehicleOnGround = false;
    bVehicleOnGround |= UpdateWheels(FrontWheels, FrontWheelsStateProxy, DeltaTime, Body, NormalizedDamage);
    bVehicleOnGround |= UpdateWheels(BackWheels, BackWheelsStateProxy, DeltaTime, Body, NormalizedDamage, true);

    FVector ChassisUp = ChassisTransform.GetUnitAxis(EAxis::Z), Right = ChassisTransform.GetUnitAxis(EAxis::Y), WheelsMidPoint = FVector::ZeroVector;
    int32 WheelsCount = 0;

    float NetNormalForce = .0f, NetCentripetalForce = .0f;

    for (auto& Wheel : FrontWheelsStateProxy.Wheels)
    {
        Body->AddForceAtPosition(Wheel.WorldForces, Wheel.ContactPoint, false);
        NetNormalForce += FVector::DotProduct(Wheel.WorldForces, ChassisUp);
        NetCentripetalForce += FVector::DotProduct(Wheel.WorldForces, Right);

        WheelsMidPoint += Wheel.ContactPoint;
        ++WheelsCount;

        if (Wheel.bNeedsImpulse)
            Body->AddImpulseAtPosition(Wheel.WorldImpulses/* - Wheel.PreviousWorldImpulse*/, Wheel.ContactPoint);
    }

    for (auto& Wheel : BackWheelsStateProxy.Wheels)
    {
        Body->AddForceAtPosition(Wheel.WorldForces, Wheel.ContactPoint, false);
        NetNormalForce += FVector::DotProduct(Wheel.WorldForces, ChassisUp);
        NetCentripetalForce += FVector::DotProduct(Wheel.WorldForces, Right);

        WheelsMidPoint += Wheel.ContactPoint;
        ++WheelsCount;

        if (Wheel.bNeedsImpulse)
            Body->AddImpulseAtPosition(Wheel.WorldImpulses/* - Wheel.PreviousWorldImpulse*/, Wheel.ContactPoint);
    }

    WheelsMidPoint /= (float)WheelsCount;

    // Apply drag force
    float SqrRadius = Chassis->Bounds.SphereRadius;
    SqrRadius *= SqrRadius;
    float VelMag = WorldVel.SizeSquared();
    float DragForce = .000002f * SqrRadius * VelMag * DragCoefficient;

    VelMag = FMath::Sqrt(VelMag);
    DragForce = FMath::Min(DragForce, Chassis->GetMass() * VelMag / DeltaTime);
    Body->AddForce(-(WorldVel / VelMag) * DragForce, false);

    // Apply extra gravity
    float ExtraGravity = FMath::Max(.0f, GravityScale - 1.f);
    if (ExtraGravity > .0f)
        Body->AddForce(ExtraGravity * GetOwner()->GetWorld()->GetGravityZ() * FVector::UpVector, false, true);

    // Keep vehicle from knocking over
    FVector PlaneRight = FVector::VectorPlaneProject(Right, FVector::UpVector).GetSafeNormal();
    FVector Axis = FVector::CrossProduct(PlaneRight, Right);

    float Sin = FMath::Abs(FVector::DotProduct(Axis, Forward));
    float TwistAngle = .0f;
    if (Sin > KINDA_SMALL_NUMBER)
    {
        TwistAngle = FMath::Asin(Sin);
        Axis /= Sin;
    }

    const float TwistAngleThreshold = FMath::DegreesToRadians(MinRollAngleForCorrection);
    if (TwistAngle > TwistAngleThreshold)
    {
        //FVector AngImpulse = -Chassis->ScaleByMomentOfInertia(Axis * (TwistAngle - TwistAngleThreshold) * RollCorrectionStrength * 100.f * DeltaTime);
        //Chassis->AddAngularImpulseInRadians(AngImpulse);
        FVector AngImpulse = -(Axis * (TwistAngle - TwistAngleThreshold) * RollCorrectionStrength * 100.f * DeltaTime);
        Body->AddAngularImpulseInRadians(AngImpulse, true);
    }

    // Compute centripetal acceleration data
    CentripetalRadius = (CurrentSpeed * CurrentSpeed) * Body->GetBodyMass() / NetCentripetalForce;
    CentripetalCenter = WheelsMidPoint + Right * CentripetalRadius;
    CentripetalRadius = FMath::Abs(CentripetalRadius);
}

FQuat UBaseVehiclePhysics::GetBasisBasedOnWheels() const
{
    if (!HasBegunPlay())
        return GetOwner()->GetTransform().GetRotation();

    FVector Front0 = FrontWheelsState.Wheels[0].WorldPosition - BackWheelsState.Wheels[0].WorldPosition,
            Front1 = FrontWheelsState.Wheels[1].WorldPosition - BackWheelsState.Wheels[1].WorldPosition;

    FVector Right0 = FrontWheelsState.Wheels[0].WorldPosition - FrontWheelsState.Wheels[1].WorldPosition,
            Right1 = BackWheelsState.Wheels[0].WorldPosition - BackWheelsState.Wheels[1].WorldPosition;

    Front0 = (Front0 + Front1).GetSafeNormal();
    Right0 = (Right0 + Right1).GetSafeNormal();

    FMatrix R;
    R.SetAxis(0, Front0);
    R.SetAxis(1, Right0);
    R.SetAxis(2, FVector::CrossProduct(Front0, Right0).GetSafeNormal());

    return R.ToQuat();
}

void UBaseVehiclePhysics::SetCurrentDriver(APawn* InCurrentDriver)
{
    CurrentDriver = InCurrentDriver;
    if (CurrentDriver)
        CurrentDriverRoot = Cast<UPrimitiveComponent>(CurrentDriver->GetRootComponent());
    else
        CurrentDriverRoot = nullptr;
}

void UBaseVehiclePhysics::SetCurrentPassenger(class APawn* InCurrentPassenger)
{
    CurrentPassenger = InCurrentPassenger;
    if (CurrentPassenger)
        CurrentPassengerRoot = Cast<UPrimitiveComponent>(CurrentPassenger->GetRootComponent());
    else
        CurrentPassengerRoot = nullptr;
}

void UBaseVehiclePhysics::OnChassisHit(class UPrimitiveComponent* HitComponent, AActor* OtherActor, class UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
    if (bReceivesDamage && Damage &&
        (!OtherActor || !OtherActor->IsA(APawn::StaticClass()))) // ToDo: disable damage from actors by tag
    {
        FVector V = HitComponent->GetPhysicsLinearVelocityAtPoint(Hit.ImpactPoint) - OtherComp->GetPhysicsLinearVelocityAtPoint(Hit.ImpactPoint);

        FVector N = Hit.ImpactNormal;
        N.Z = .0f; // Don't mind top & bottom hits

        float NV = FVector::DotProduct(V, N);
        //if (NV > DamageMinNDotV)
        //    Damage->ApplyDamage(FDamagePackage{ (NV - DamageMinNDotV) * DamageScaler, .0f, .0f }, NormalImpulse, OtherActor);
    }

    if (!bForceKinematic && !Chassis->IsSimulatingPhysics())
        Chassis->SetSimulatePhysics(true);
}

void UBaseVehiclePhysics::BeginPlay()
{
    Chassis = Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent());
    if (!ensureMsgf(Chassis, TEXT("BaseVehiclePhysics w/out Root Primitive Component!")))
    {
        PrimaryComponentTick.SetTickFunctionEnable(false);
        return;
    }

    Camera = Cast<UBaseVehicleCamera>(GetOwner()->GetComponentByClass(UBaseVehicleCamera::StaticClass()));
    if (!ensureMsgf(Camera, TEXT("BaseVehiclePhysics w/out Camera Component!")))
    {
        PrimaryComponentTick.SetTickFunctionEnable(false);
        return;
    }

    //Damage = Cast<UDamageComponent>(GetOwner()->GetComponentByClass(UDamageComponent::StaticClass()));

    Chassis->OnComponentHit.AddDynamic(this, &UBaseVehiclePhysics::OnChassisHit);

    ChassisTransform = Chassis->GetComponentTransform();
    LocalVelocity = FVector::ZeroVector;

    int32 Count = FrontWheels.Positions.Num();
    FrontWheelsState.Wheels.SetNum(Count);
    FrontWheelsStateProxy.Wheels.SetNum(Count);
    for (int32 Index = 0; Index < Count; ++Index)
    {
        FVehicleSingleWheelStateProxy& State = FrontWheelsStateProxy.Wheels[Index];
        State.Reset(ChassisTransform, FrontWheels.Positions[Index]);
        FrontWheelsState.Wheels[Index].CopyFrom(ChassisTransform, State);
    }

    Count = BackWheels.Positions.Num();
    BackWheelsState.Wheels.SetNum(Count);
    BackWheelsStateProxy.Wheels.SetNum(Count);
    for (int32 Index = 0; Index < Count; ++Index)
    {
        FVehicleSingleWheelStateProxy& State = BackWheelsStateProxy.Wheels[Index];
        State.Reset(ChassisTransform, BackWheels.Positions[Index]);
        BackWheelsState.Wheels[Index].CopyFrom(ChassisTransform, State);
    }

    CurrentSpeed = .0f;
    CurrentNormalizedSpeed = .0f;

    InputSteering = InputThrottle = InputBrake = .0f;

    bInReverse = false;
    bBraking = false;
    bVehicleOnGround = false;

    Super::BeginPlay();
}

void UBaseVehiclePhysics::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    bool bAnyInputs = (InputThrottle > KINDA_SMALL_NUMBER)
        || (InputBrake > KINDA_SMALL_NUMBER)
        || FMath::Abs(InputSteering) > KINDA_SMALL_NUMBER;

    if (Chassis->IsSimulatingPhysics())
    { // Check for no inputs & low speed
        FVector LinVel = Chassis->GetPhysicsLinearVelocity(),
                AngVel = Chassis->GetPhysicsAngularVelocityInDegrees();

        bool bNotSimulate =
            !bAnyInputs
            && LinVel.SizeSquared() < 4.f
            && AngVel.SizeSquared() < 4.f;

        if (bNotSimulate)
        {
            bInReverse = false;
            bBraking = false;

            Chassis->SetSimulatePhysics(false);
        }
        else
            Chassis->GetBodyInstance()->AddCustomPhysics(VehiclePhysicsCallback);
    }
    else
    { // Check for inputs
        if (!bForceKinematic && bAnyInputs)
            Chassis->SetSimulatePhysics(true);
    }

    ChassisTransform = Chassis->GetComponentTransform();
    LocalVelocity = ChassisTransform.InverseTransformVectorNoScale(Chassis->GetPhysicsLinearVelocity());

    int32 Index = 0;
    for (auto& Wheel : FrontWheelsState.Wheels)
        Wheel.CopyFrom(ChassisTransform, FrontWheelsStateProxy.Wheels[Index++]);
    Index = 0;
    for (auto& Wheel : BackWheelsState.Wheels)
        Wheel.CopyFrom(ChassisTransform, BackWheelsStateProxy.Wheels[Index++]);

    Camera->UpdateCameraTransform(DeltaTime);

    // Evade characters (brake)
    const float PredictionTime = .8f;
    const float CharaRadiusMult = 1.33f;

    FVector WorldLoc = ChassisTransform.GetLocation();
    FVector WorldVel = Chassis->GetPhysicsLinearVelocity();

    bForceBrake = false;

    for (TActorIterator<ACharacter> It(GetWorld()); It; ++It)
    {
        if (!It->GetActorEnableCollision())
            continue;

        if (Chassis->MoveIgnoreActors.Contains(Cast<AActor>(*It)))
            continue;

        FBoxSphereBounds CharaBounds = It->GetMesh()->Bounds;
        CharaBounds.SphereRadius *= CharaRadiusMult;

        FVector FromChara = WorldLoc - CharaBounds.Origin;
        float Dist = FromChara.Size();
        if (Dist > (MaxSpeed * PredictionTime))
            continue; // Too distant

        Dist = FVector::Dist(CharaBounds.Origin, CentripetalCenter) + CharaBounds.SphereRadius - CentripetalRadius;
        if (Dist < .0f)
            continue; // Can't be reached

        FVector Delta = (It->GetVelocity() - WorldVel) * PredictionTime;
        FVector Start = CharaBounds.Origin;

        if (FVector::DotProduct(Delta, FromChara) <= .0f)
            continue; // Going away from

        UCapsuleComponent* Capsule = It->GetCapsuleComponent();
        FHitResult Hit;
        if (Chassis->SweepComponent(
            Hit,
            Start, Start + Delta,
            FQuat::Identity,
            FCollisionShape::MakeSphere(CharaBounds.SphereRadius),
            false))
        {
            bForceBrake = true;
            ForceBrakeLerp = FMath::Max(.5f, ForceBrakeLerp);
            break;
        }
    }

    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UBaseVehiclePhysics::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Chassis->OnComponentHit.RemoveDynamic(this, &UBaseVehiclePhysics::OnChassisHit);

    Super::EndPlay(EndPlayReason);
}
