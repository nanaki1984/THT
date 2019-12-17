#include "Camera/PCCameraData.h"

bool FPCCameraRotationData::operator<(const FPCCameraRotationData& RHS) const
{
    return Angle < RHS.Angle;
}

void FPCCameraRotationData::Mix(
    FPCCameraRotationData& InOutData, FVector& InOutPivotBase, float& InOutDataWeight,
    const FPCCameraRotationData& OtherData, const FVector& OtherPivotBase, float OtherWeight)
{
    float TotalWeight = InOutDataWeight + OtherWeight, S = 1.f - (InOutDataWeight / TotalWeight);

    InOutData.Angle = FMath::Lerp(InOutData.Angle, OtherData.Angle, S);
    InOutData.MaxDistance = FMath::Lerp(InOutData.MaxDistance, OtherData.MaxDistance, S);
    InOutData.FieldOfView = FMath::Lerp(InOutData.FieldOfView, OtherData.FieldOfView, S);
    InOutData.PivotOffset = FMath::Lerp(InOutData.PivotOffset, OtherData.PivotOffset, S);

    InOutPivotBase = FMath::Lerp(InOutPivotBase, OtherPivotBase, S);

    InOutDataWeight = FMath::Lerp(InOutDataWeight, OtherWeight, S);
}

UPCCameraData::UPCCameraData()
    : PivotBase(.0f, .0f, 200.0f)
    , Weight(1.f)
{ }

void UPCCameraData::Initialize()
{
    Data.Sort();
}

void UPCCameraData::GetInterpolatedData(float Angle, FPCCameraRotationData& OutData)
{
    OutData.Angle = Angle;

    int32 UpperIndex = FMath::Min(Algo::UpperBound(Data, OutData), Data.Num() - 1);
    int32 LowerIndex = FMath::Max(0, UpperIndex - 1);

    FPCCameraRotationData& LowerData = Data[LowerIndex];
    FPCCameraRotationData& UpperData = Data[UpperIndex];

    float S = UpperData.Angle - LowerData.Angle;
    if (FMath::Abs(S) > .0f)
        S = FMath::Clamp((Angle - LowerData.Angle) / S, .0f, 1.f);

    OutData.Angle = FMath::Lerp(LowerData.Angle, UpperData.Angle, S);
    OutData.MaxDistance = FMath::Lerp(LowerData.MaxDistance, UpperData.MaxDistance, S);
    OutData.FieldOfView = FMath::Lerp(LowerData.FieldOfView, UpperData.FieldOfView, S);
    OutData.PivotOffset = FMath::Lerp(LowerData.PivotOffset, UpperData.PivotOffset, S);
}
