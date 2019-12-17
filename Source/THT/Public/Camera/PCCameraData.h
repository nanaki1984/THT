#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "PCCameraData.generated.h"

USTRUCT(BlueprintType)
struct THT_API FPCCameraRotationData
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere)
    float Angle;

    UPROPERTY(EditAnywhere)
    float MaxDistance;

    UPROPERTY(EditAnywhere)
    float FieldOfView;

    UPROPERTY(EditAnywhere)
    FVector PivotOffset;

    bool operator<(const FPCCameraRotationData& RHS) const;

    static void Mix(
        FPCCameraRotationData& InOutData, FVector& InOutPivotBase, float& InOutDataWeight,
        const FPCCameraRotationData& OtherData, const FVector& OtherPivotBase, float OtherWeight);
};

UCLASS(BlueprintType)
class THT_API UPCCameraData : public UDataAsset
{
    GENERATED_BODY()

public:

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector PivotBase;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FPCCameraRotationData> Data;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float DefaultPitch;

    UPROPERTY(Transient, EditInstanceOnly, BlueprintReadWrite, meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float Weight;

    UPCCameraData();

    void Initialize();
    void GetInterpolatedData(float Angle, FPCCameraRotationData& OutData);
};
