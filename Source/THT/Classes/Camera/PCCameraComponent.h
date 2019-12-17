#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "PCCameraComponent.generated.h"

class UCameraComponent;
class UCapsuleComponent;
class UPCCameraData;

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class THT_API UPCCameraComponent : public UActorComponent
{
    GENERATED_BODY()

protected:

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector2D InputSpeed;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float InputInterpolation;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    TEnumAsByte<ECollisionChannel> CollisionChannel;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float DistanceSpeed;
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float FovSpeed;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float LookAheadTime;
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float LookAheadLinearSpeed;
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float LookAheadBackToZeroInterpBase;
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float LookAheadBackToZeroInterp;
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FVector LookAheadScale;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float HeightInterp;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
    float DistanceToFloorPercentage;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    TArray<UPCCameraData*> Cameras;

    UCameraComponent* CameraComponent;
    UCharacterMovementComponent* MovementComponent;
    UCapsuleComponent* CapsuleComponent;
    FFindFloorResult LastValidFloor;

    FRotator PreviousYawPitch;
    FRotator WantedYawPitch;
    float PreviousSourceTargetDistance;
    float PreviousFov;

    FVector GroundReference;
    FVector PrevPivotOffset;
    float PrevPivotHeight;

    virtual void BeginPlay() override;

public:

    UPROPERTY(Transient, EditInstanceOnly, BlueprintReadWrite)
    /** Between -1 and 1 */
    float InputYaw;

    UPROPERTY(Transient, EditInstanceOnly, BlueprintReadWrite)
    /** Between -1 and 1 */
    float InputPitch;

    UPCCameraComponent();

    void UpdateCameraTransform(float DeltaTime);
};
