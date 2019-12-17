#pragma once

#include "CoreMinimal.h"
#include "RandomLevel.generated.h"

UENUM(BlueprintType)
enum class ETileType : uint8
{
	Blocked = 0,
	AnyOther,
};

struct THT_API FRandomLevel
{
public:
	const int32 kBaseLevelSize = 128;
	const int32 kBaseLevelTilesCount = kBaseLevelSize * kBaseLevelSize;

private:
	int32 CountAliveNeighbours(int32 Index) const;
	void DoSingleStep(int32 BirthLimit, int32 DeathLimit);
    void FloodFill(int32 FirstIndex, TFunction<void(int32, int32, int32)>&& Callback);

public:
	TArray<ETileType> Tiles;
    TArray<ETileType> NextTiles;

	void Generate(float InitialChance, int32 BirthLimit, int32 DeathLimit, int32 Steps);
    bool QualityCheck(float MinimumAreaCovered);
};
