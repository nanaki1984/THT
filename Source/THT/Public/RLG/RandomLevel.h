#pragma once

#include "CoreMinimal.h"

struct THT_API FRandomLevel
{
private:
    int32 LevelSize;
    int32 LevelTilesCount;

    TBitArray<FDefaultBitArrayAllocator> Tiles;
    TBitArray<FDefaultBitArrayAllocator> NextTiles;

    int32 CountAliveNeighbours(int32 Index) const;
	void DoSingleStep(int32 BirthLimit, int32 DeathLimit);
    void FloodFill(int32 FirstIndex, TFunction<void(int32, int32, int32)>&& Callback);

public:

    FORCEINLINE int32 GetLevelSize() const { return LevelSize; }
    FORCEINLINE bool GetTileIsValid(int32 Index) const { return Tiles[Index]; }

	void Generate(int32 InLevelSize, float InitialChance, int32 BirthLimit, int32 DeathLimit, int32 Steps);
    bool QualityCheck(float MinimumAreaCovered);
};
