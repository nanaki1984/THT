#include "Camera/PCCameraComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Camera/CameraComponent.h"
#include "Components/CapsuleComponent.h"
#include "Camera/PCCameraData.h"
#include "DrawDebugHelpers.h"

UPCCameraComponent::UPCCameraComponent()
    : CameraComponent(nullptr)
{
    PrimaryComponentTick.bCanEverTick = false;
}

void UPCCameraComponent::BeginPlay()
{
    CameraComponent = Cast<UCameraComponent>(GetOwner()->GetComponentByClass(UCameraComponent::StaticClass()));
    MovementComponent = Cast<UCharacterMovementComponent>(GetOwner()->GetComponentByClass(UCharacterMovementComponent::StaticClass()));
    CapsuleComponent = Cast<UCapsuleComponent>(GetOwner()->GetComponentByClass(UCapsuleComponent::StaticClass()));
    if (!ensureMsgf(CameraComponent && MovementComponent && CapsuleComponent, TEXT("Missing components!")))
    {
        PrimaryComponentTick.SetTickFunctionEnable(false);
        return;
    }

    LastValidFloor = MovementComponent->CurrentFloor;

    UPCCameraData* BestData = nullptr;
    for (UPCCameraData* Data : Cameras)
    {
        Data->Initialize();

        if (!BestData || BestData->Weight < Data->Weight)
            BestData = Data;
    }

    check(BestData);
    FPCCameraRotationData BestDataInterp;
    BestData->GetInterpolatedData(BestData->DefaultPitch, BestDataInterp);

    PreviousYawPitch = WantedYawPitch = FRotator(BestData->DefaultPitch, .0f, .0f);

    PreviousSourceTargetDistance = BestDataInterp.MaxDistance;
    PreviousFov = BestDataInterp.FieldOfView;

    PrevPivotOffset = FVector::ZeroVector;
    PrevPivotHeight = GetOwner()->GetActorLocation().Z + BestData->PivotBase.Z + BestDataInterp.PivotOffset.Z;

    Super::BeginPlay();
}

void UPCCameraComponent::UpdateCameraTransform(float DeltaTime)
{
    WantedYawPitch.Add(InputSpeed.X * InputPitch * DeltaTime, InputSpeed.Y * InputYaw * DeltaTime, .0f);
    WantedYawPitch.Pitch = FMath::Clamp(WantedYawPitch.Pitch, -90.f, 90.f);
    WantedYawPitch.Yaw = FRotator::NormalizeAxis(WantedYawPitch.Yaw);

    FPCCameraRotationData Data;
    FVector PivotBase;
    float DataWeight;

    int32 Index = 0, Count = Cameras.Num();
    bool bFirstCamFound = false;
    for (; Index < Count; ++Index)
    {
        UPCCameraData* Camera = Cameras[Index];
        if (Camera->Weight > KINDA_SMALL_NUMBER)
        {
            bFirstCamFound = true;

            Camera->GetInterpolatedData(WantedYawPitch.Pitch, Data);

            PivotBase = Camera->PivotBase;
            DataWeight = Camera->Weight;
            break;
        }
    }

    if (!bFirstCamFound)
        return;

    FPCCameraRotationData OtherData;
    for (; Index < Count; ++Index)
    {
        UPCCameraData* Camera = Cameras[Index];
        if (Camera->Weight > KINDA_SMALL_NUMBER)
        {
            Camera->GetInterpolatedData(WantedYawPitch.Pitch, OtherData);
            FPCCameraRotationData::Mix(Data, PivotBase, DataWeight, OtherData, Camera->PivotBase, Camera->Weight);
        }
    }

    WantedYawPitch.Pitch = Data.Angle;
    PreviousYawPitch = FMath::Lerp(PreviousYawPitch, WantedYawPitch, InputInterpolation * DeltaTime);

    FVector Pivot = PivotBase + Data.PivotOffset;
    FQuat YawPitchQuat = PreviousYawPitch.Quaternion();
    FQuat YawQuat = FRotator(.0f, PreviousYawPitch.Yaw, .0f).Quaternion();

    FVector NewPivotOffset = YawQuat.UnrotateVector(MovementComponent->Velocity * LookAheadTime);
    NewPivotOffset *= LookAheadScale;

    float BackToZeroInterp = LookAheadBackToZeroInterpBase + FMath::Sqrt(InputPitch * InputPitch + InputYaw * InputYaw) * LookAheadBackToZeroInterp;

    float PrevHeightOffset = PrevPivotOffset.Z, NewHeightOffset = FMath::Min(.0f, NewPivotOffset.Z);
    PrevPivotOffset.Z = NewPivotOffset.Z = .0f;

    if (NewPivotOffset.SizeSquared2D() > KINDA_SMALL_NUMBER)
    {
        PrevPivotOffset.X = FMath::FInterpConstantTo(PrevPivotOffset.X, NewPivotOffset.X, DeltaTime, LookAheadLinearSpeed * LookAheadScale.X);
        PrevPivotOffset.Y = FMath::FInterpConstantTo(PrevPivotOffset.Y, NewPivotOffset.Y, DeltaTime, LookAheadLinearSpeed * LookAheadScale.Y);
    }
    else
    {
        if (BackToZeroInterp > .0f)
            PrevPivotOffset = FMath::VInterpTo(PrevPivotOffset, FVector::ZeroVector, DeltaTime, BackToZeroInterp);
    }

    if (NewHeightOffset < -KINDA_SMALL_NUMBER)
        PrevPivotOffset.Z = FMath::FInterpConstantTo(PrevHeightOffset, NewHeightOffset, DeltaTime, LookAheadLinearSpeed * LookAheadScale.Z);
    else
        PrevPivotOffset.Z = FMath::FInterpTo(PrevHeightOffset, .0f, DeltaTime, LookAheadBackToZeroInterp);

    FVector ActorPos = GetOwner()->GetActorLocation();

    float TargetPivotHeight = ActorPos.Z + Pivot.Z;
    if (LastValidFloor.IsWalkableFloor())
    {
        float DistToFloor = FVector::DotProduct(MovementComponent->GetActorFeetLocation() - LastValidFloor.HitResult.ImpactPoint, FVector::UpVector) - 2.15f;
        if (DistToFloor >= .0f)
            TargetPivotHeight -= DistToFloor * DistanceToFloorPercentage;
    }
    if (MovementComponent->CurrentFloor.IsWalkableFloor())
        LastValidFloor = MovementComponent->CurrentFloor;

    if (TargetPivotHeight <= PrevPivotHeight)
        PrevPivotHeight = TargetPivotHeight;
    else
        PrevPivotHeight = FMath::FInterpTo(PrevPivotHeight, TargetPivotHeight, DeltaTime, HeightInterp);

    FVector CamTarget = ActorPos + YawQuat.RotateVector(FVector(Pivot.X, Pivot.Y, .0f) + PrevPivotOffset);
    CamTarget.Z = PrevPivotHeight + PrevPivotOffset.Z;
    //DrawDebugSphere(GetWorld(), CamTarget, 20.f, 8, FColor::Red);
    FVector CamSource = CamTarget - YawPitchQuat.GetForwardVector() * Data.MaxDistance;

    FHitResult Hit;
    FCollisionQueryParams Params;
    Params.bTraceComplex = false;
    Params.AddIgnoredActor(GetOwner());

    bool bOccluded = GetWorld()->SweepSingleByChannel(Hit, ActorPos, CamSource, FQuat::Identity, CollisionChannel, FCollisionShape::MakeSphere(2.f), Params);
    if (bOccluded)
        CamSource = Hit.Location;

    FVector ActorToSource = CamSource - ActorPos;
    float SourceTargetDist = ActorToSource.Size();
    if (!bOccluded)
        ActorToSource = ActorToSource.GetClampedToMaxSize(PreviousSourceTargetDistance + DistanceSpeed * DeltaTime);
    PreviousSourceTargetDistance = ActorToSource.Size();

    CameraComponent->SetWorldLocationAndRotation(ActorPos + ActorToSource, FRotationMatrix::MakeFromXZ(CamTarget - CamSource, FVector::UpVector).ToQuat());

    PreviousFov = FMath::Lerp(PreviousFov, Data.FieldOfView, FMath::Min(1.f, DeltaTime * FovSpeed));
    CameraComponent->FieldOfView = PreviousFov;
}
