// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Camera/PlayerCameraManager.h"
#include "BaseVehicle.generated.h"

UCLASS()
class THT_API ABaseVehicle
    : public APawn
{
	GENERATED_BODY()

protected:
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    class UBaseVehiclePhysics* Physics;
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    class UBaseVehicleCamera* Camera;

    class USkeletalMeshComponent* SkeletalMesh;

    TInlineComponentArray<class USceneComponent*, 2> FrontWheels;
    TInlineComponentArray<class USceneComponent*, 2> BackWheels;

    int32 RootSwithNodeIndex;

    float AccForceBeforeBroken;
    float RevAccForceBeforeBroken;

    APawn* CurrentDriver;
    APawn* CurrentPassenger;

    UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly)
    uint32 bLightsOn : 1;
    UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly)
    uint32 bKinematic : 1;
    UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly)
    uint32 bBroken : 1;

    void UpdateBrokenStatus();

    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "Start Driving"))
    void ReceiveStartDriving(APawn* InCurrentDriver, bool bCanDrive);
    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "End Driving"))
    void ReceiveEndDriving(APawn* PreviousDriver);
    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "Passenger Enter"))
    void ReceivePassengerEnter(APawn* InCurrentPassenger);
    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "Passenger Exit"))
    void ReceivePassengerExit(APawn* PreviousPassenger);

    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "Lights On"))
    void ReceiveLightsOn();
    UFUNCTION(BlueprintImplementableEvent, meta = (DisplayName = "Lights Off"))
    void ReceiveLightsOff();

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Driver", meta = (MakeEditWidget = true))
	/** Driver local space offset */
	FVector DriverOffset;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Passenger", meta = (MakeEditWidget = true))
	/** Driver local space offset */
	FVector PassengerOffset;

    ABaseVehicle();

    FORCEINLINE class UBaseVehiclePhysics* GetPhysics() const { return Physics; }
    FORCEINLINE class UBaseVehicleCamera* GetCamera() const { return Camera; }

    FORCEINLINE bool AreLightsOn() const { return bLightsOn; }
    FORCEINLINE bool IsKinematic() const { return bKinematic; }
    FORCEINLINE bool IsBroken() const { return bBroken; }

    UFUNCTION(BlueprintCallable)
    void StartDriving(APawn* Driver);
    UFUNCTION(BlueprintCallable)
    bool CanEndDriving() const;
    UFUNCTION(BlueprintCallable)
    void BeginEndDriving();
    UFUNCTION(BlueprintCallable)
    void EndDriving();

    UFUNCTION(BlueprintCallable)
    void SetVehicleKinematic(bool bNewKinematic);

    UFUNCTION(BlueprintCallable)
    bool HasDriver() const { return CurrentDriver; }
    UFUNCTION(BlueprintCallable)
    bool GetCurrentDriverCanDrive() const;

    UFUNCTION(BlueprintCallable)
    APawn* GetCurrentDriver() const { return CurrentDriver; }
    UFUNCTION(BlueprintCallable)
    APawn* GetCurrentPassenger() const { return CurrentPassenger; }

    UFUNCTION(BlueprintCallable)
    void PutPassengerInside(APawn* Passenger);
    UFUNCTION(BlueprintCallable)
    bool CanDropOutPassenger();
    UFUNCTION(BlueprintCallable)
    APawn* DropOutPassenger();

    UFUNCTION(BlueprintCallable)
    void ToggleLights();

	/** Play Animation Montage on the character mesh **/
	UFUNCTION(BlueprintCallable, Category = "Vehicle|Animation")
	virtual float PlayAnimMontage(class UAnimMontage* AnimMontage, float InPlayRate = 1.f, FName StartSectionName = NAME_None);
	/** Stop Animation Montage. If nullptr, it will stop what's currently active. The Blend Out Time is taken from the montage asset that is being stopped. **/
	UFUNCTION(BlueprintCallable, Category = "Vehicle|Animation")
	virtual void StopAnimMontage(class UAnimMontage* AnimMontage = nullptr);
	/** Return current playing Montage **/
	UFUNCTION(BlueprintCallable, Category = "Vehicle|Animation")
	class UAnimMontage* GetCurrentMontage();

    virtual void PossessedBy(AController* NewController) override;
    virtual void UnPossessed() override;

    virtual void Tick(float DeltaTime) override;
};
