#include "RLG/RandomLevel.h"

int32 FRandomLevel::CountAliveNeighbours(int32 Index) const
{
	int32 X0 = Index % LevelSize,
		  Y0 = Index / LevelSize;

	int32 Counter = 0;
	for (int32 I = -1; I <= 1; ++I)
	{
		for (int32 J = -1; J <= 1; ++J)
		{
			if (0 == I && 0 == J)
				continue;

			int32 X = X0 + I,
				  Y = Y0 + J;
            if (X < 0 || X >= LevelSize || Y < 0 || Y >= LevelSize)
                continue;

            if (Tiles[Y * LevelSize + X])
                ++Counter;
		}
	}

    return Counter;
}

void FRandomLevel::DoSingleStep(int32 BirthLimit, int32 DeathLimit)
{
    for (int32 Index = 0; Index < LevelTilesCount; ++Index)
    {
        int32 AliveCounter = CountAliveNeighbours(Index);

        if (Tiles[Index])
        {
            if (AliveCounter < DeathLimit)
                NextTiles[Index] = false;
            else
                NextTiles[Index] = true;
        }
        else
        {
            if (AliveCounter > BirthLimit)
                NextTiles[Index] = true;
            else
                NextTiles[Index] = false;
        }
    }

    Swap(Tiles, NextTiles);
}

void FRandomLevel::FloodFill(int32 FirstIndex, TFunction<void(int32, int32, int32)>&& Callback)
{
    TBitArray<FDefaultBitArrayAllocator> CellsVisited;
    CellsVisited.Init(false, LevelTilesCount);

    TArray<int32> CellsQueue;
    CellsQueue.Push(FirstIndex);
    bool ValidType = Tiles[FirstIndex];

    while (CellsQueue.Num() > 0)
    {
        int32 CellIndex = CellsQueue.Pop(false);
        if (CellsVisited[CellIndex])
            continue;
        CellsVisited[CellIndex] = true;

        if (Tiles[CellIndex] == ValidType)
        {
            int32 X = CellIndex % LevelSize,
                  Y = CellIndex / LevelSize;

            Callback(CellIndex, X, Y);

            if (Y > 0) // Up
                CellsQueue.Push(CellIndex - LevelSize);
            if (Y < (LevelSize - 1)) // Down
                CellsQueue.Push(CellIndex + LevelSize);
            if (X > 0) // Left
                CellsQueue.Push(CellIndex - 1);
            if (X < (LevelSize - 1)) // Right
                CellsQueue.Push(CellIndex + 1);
        }
    }
}

void FRandomLevel::Generate(int32 InLevelSize, float InitialChance, int32 BirthLimit, int32 DeathLimit, int32 Steps)
{
    LevelSize = FMath::Max(8, InLevelSize);
    LevelTilesCount = LevelSize * LevelSize;

    Tiles.Init(false, LevelTilesCount);
    NextTiles.Init(false, LevelTilesCount);

    for (int32 Index = 0; Index < LevelTilesCount; ++Index)
		Tiles[Index] = FMath::FRand() <= InitialChance;

    for (int32 Step = 0; Step < Steps; ++Step)
        DoSingleStep(BirthLimit, DeathLimit);
}

bool FRandomLevel::QualityCheck(float MinimumAreaCovered)
{
    int32 ValidCellsCount = 0;

    TBitArray<FDefaultBitArrayAllocator> CellsValidity;
    CellsValidity.Init(false, LevelTilesCount);

    int32 FirstIndex = 0;
    while (!Tiles[FirstIndex] && FirstIndex < LevelTilesCount)
        ++FirstIndex;

    FloodFill(FirstIndex, [&CellsValidity, &ValidCellsCount](int32 CellIndex, int32 X, int32 Y)
    {
        CellsValidity[CellIndex] = true;
        ++ValidCellsCount;
    });

    int32 Last = (LevelSize - 1);
    TArray<int32> TilesToBeUnlocked;
    for (int32 Index = 0; Index < LevelTilesCount; ++Index)
    {
        if (!CellsValidity[Index]) // Reset unreachable tiles
            Tiles[Index] = false;

        int32 X = Index % LevelSize,
              Y = Index / LevelSize;

        if (Tiles[Index])
        { // Look at left/right and up/down neighbours, if both blocked, unblock them
            if (Y > 0 && Y < Last)
            {
                if (!Tiles[Index - LevelSize]
                 && !Tiles[Index + LevelSize])
                {
                    TilesToBeUnlocked.Add(Index - LevelSize);
                    TilesToBeUnlocked.Add(Index + LevelSize);
                }
            }

            if (X > 0 && X < Last)
            {
                if (!Tiles[Index - 1]
                 && !Tiles[Index + 1])
                {
                    TilesToBeUnlocked.Add(Index - 1);
                    TilesToBeUnlocked.Add(Index + 1);
                }
            }
        }
        else
        { // Check if three neightbours are not blocked, just unblock it
            int32 Counter = 0;
            if (Y > 0 && Tiles[Index - LevelSize])
                ++Counter;
            if (Y < Last && Tiles[Index + LevelSize])
                ++Counter;
            if (X > 0 && Tiles[Index - 1])
                ++Counter;
            if (X < Last && Tiles[Index + 1])
                ++Counter;

            if (Counter >= 3)
                TilesToBeUnlocked.Add(Index);
        }
    }

    for (int32 Index : TilesToBeUnlocked)
    {
        if (Tiles[Index])
            continue;

        Tiles[Index] = true;
        ++ValidCellsCount;
    }
    TilesToBeUnlocked.SetNum(0, false);

    // Remove holes smaller than 5 cells
    CellsValidity.Init(false, LevelTilesCount);
    for (int32 Index = 0; Index < LevelTilesCount; ++Index)
    {
        if (Tiles[Index])
            continue;

        if (!CellsValidity[Index])
        {
            int32 AreaSize = 0;
            TArray<int32, TInlineAllocator<4>> FoundCells;
            FloodFill(Index, [&CellsValidity, &AreaSize, &FoundCells](int32 CellIndex, int32 X, int32 Y)
            {
                CellsValidity[CellIndex] = true;
                ++AreaSize;

                if (FoundCells.Num() < 4)
                    FoundCells.Add(CellIndex);
            });

            if (AreaSize <= 4)
                TilesToBeUnlocked.Insert(FoundCells.GetData(), FoundCells.Num(), TilesToBeUnlocked.Num());
        }
    }

    for (int32 Index : TilesToBeUnlocked)
    {
        if (Tiles[Index])
            continue;

        Tiles[Index] = true;
        ++ValidCellsCount;
    }

    float AreaCovered = ValidCellsCount / (float)LevelTilesCount;
    if (AreaCovered < MinimumAreaCovered)
        return false;

    return true;
}
