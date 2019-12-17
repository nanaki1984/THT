#include "RLG/RandomLevel.h"

int32 FRandomLevel::CountAliveNeighbours(int32 Index) const
{
	int32 X0 = Index % kBaseLevelSize,
		  Y0 = Index / kBaseLevelSize;

	int32 Counter = 0;
	for (int32 I = -1; I <= 1; ++I)
	{
		for (int32 J = -1; J <= 1; ++J)
		{
			if (0 == I && 0 == J)
				continue;

			int32 X = X0 + I,
				  Y = Y0 + J;
            if (X < 0 || X >= kBaseLevelSize || Y < 0 || Y >= kBaseLevelSize)
                continue;

            if (Tiles[Y * kBaseLevelSize + X] > ETileType::Blocked)
                ++Counter;
		}
	}

    return Counter;
}

void FRandomLevel::DoSingleStep(int32 BirthLimit, int32 DeathLimit)
{
    for (int32 Index = 0; Index < kBaseLevelTilesCount; ++Index)
    {
        int32 AliveCounter = CountAliveNeighbours(Index);

        ETileType CurrTile = Tiles[Index];
        if (CurrTile > ETileType::Blocked)
        {
            if (AliveCounter < DeathLimit)
                NextTiles[Index] = ETileType::Blocked;
            else
                NextTiles[Index] = CurrTile;
        }
        else
        {
            if (AliveCounter > BirthLimit)
                NextTiles[Index] = ETileType::AnyOther;
            else
                NextTiles[Index] = CurrTile;
        }
    }

    Swap(Tiles, NextTiles);
}

void FRandomLevel::FloodFill(int32 FirstIndex, TFunction<void(int32, int32, int32)>&& Callback)
{
    TBitArray<FDefaultBitArrayAllocator> CellsVisited;
    CellsVisited.Init(false, kBaseLevelTilesCount);

    TArray<int32> CellsQueue;
    CellsQueue.Push(FirstIndex);
    ETileType ValidType = Tiles[FirstIndex];

    while (CellsQueue.Num() > 0)
    {
        int32 CellIndex = CellsQueue.Pop(false);
        if (CellsVisited[CellIndex])
            continue;
        CellsVisited[CellIndex] = true;

        if (Tiles[CellIndex] == ValidType)
        {
            int32 X = CellIndex % kBaseLevelSize,
                  Y = CellIndex / kBaseLevelSize;

            Callback(CellIndex, X, Y);

            if (Y > 0) // Up
                CellsQueue.Push(CellIndex - kBaseLevelSize);
            if (Y < (kBaseLevelSize - 1)) // Down
                CellsQueue.Push(CellIndex + kBaseLevelSize);
            if (X > 0) // Left
                CellsQueue.Push(CellIndex - 1);
            if (X < (kBaseLevelSize - 1)) // Right
                CellsQueue.Push(CellIndex + 1);
        }
    }
}

void FRandomLevel::Generate(float InitialChance, int32 BirthLimit, int32 DeathLimit, int32 Steps)
{
	Tiles.SetNumUninitialized(kBaseLevelTilesCount);
    NextTiles.SetNumUninitialized(kBaseLevelTilesCount);

    for (int32 Index = 0; Index < kBaseLevelTilesCount; ++Index)
		Tiles[Index] = FMath::FRand() <= InitialChance
			? ETileType::AnyOther
			: ETileType::Blocked;

    for (int32 Step = 0; Step < Steps; ++Step)
        DoSingleStep(BirthLimit, DeathLimit);

    NextTiles.SetNum(0);
}

bool FRandomLevel::QualityCheck(float MinimumAreaCovered)
{
    int32 ValidCellsCount = 0;

    TBitArray<FDefaultBitArrayAllocator> CellsValidity;
    CellsValidity.Init(false, kBaseLevelTilesCount);

    int32 FirstIndex = 0;
    while (Tiles[FirstIndex] == ETileType::Blocked && FirstIndex < kBaseLevelTilesCount)
        ++FirstIndex;

    FloodFill(FirstIndex, [&CellsValidity, &ValidCellsCount](int32 CellIndex, int32 X, int32 Y)
    {
        CellsValidity[CellIndex] = true;
        ++ValidCellsCount;
    });

    int32 Last = (kBaseLevelSize - 1);
    TArray<int32> TilesToBeUnlocked;
    for (int32 Index = 0; Index < kBaseLevelTilesCount; ++Index)
    {
        if (!CellsValidity[Index]) // Reset unreachable tiles
            Tiles[Index] = ETileType::Blocked;

        int32 X = Index % kBaseLevelSize,
              Y = Index / kBaseLevelSize;

        if (ETileType::AnyOther == Tiles[Index])
        { // Look at left/right and up/down neighbours, if both blocked, unblock them
            if (Y > 0 && Y < Last)
            {
                if (ETileType::Blocked == Tiles[Index - kBaseLevelSize]
                 && ETileType::Blocked == Tiles[Index + kBaseLevelSize])
                {
                    TilesToBeUnlocked.Add(Index - kBaseLevelSize);
                    TilesToBeUnlocked.Add(Index + kBaseLevelSize);
                }
            }

            if (X > 0 && X < Last)
            {
                if (ETileType::Blocked == Tiles[Index - 1]
                 && ETileType::Blocked == Tiles[Index + 1])
                {
                    TilesToBeUnlocked.Add(Index - 1);
                    TilesToBeUnlocked.Add(Index + 1);
                }
            }
        }
        else
        { // Check if three neightbours are not blocked, just unblock it
            int32 Counter = 0;
            if (Y > 0 && Tiles[Index - kBaseLevelSize] > ETileType::Blocked)
                ++Counter;
            if (Y < Last && Tiles[Index + kBaseLevelSize] > ETileType::Blocked)
                ++Counter;
            if (X > 0 && Tiles[Index - 1] > ETileType::Blocked)
                ++Counter;
            if (X < Last && Tiles[Index + 1] > ETileType::Blocked)
                ++Counter;

            if (Counter >= 3)
                TilesToBeUnlocked.Add(Index);
        }
    }

    for (int32 Index : TilesToBeUnlocked)
    {
        if (Tiles[Index] > ETileType::Blocked)
            continue;

        Tiles[Index] = ETileType::AnyOther;
        ++ValidCellsCount;
    }
    TilesToBeUnlocked.SetNum(0, false);

    // Remove holes smaller than 5 cells
    CellsValidity.Init(false, kBaseLevelTilesCount);
    for (int32 Index = 0; Index < kBaseLevelTilesCount; ++Index)
    {
        if (Tiles[Index] > ETileType::Blocked)
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
        if (Tiles[Index] > ETileType::Blocked)
            continue;

        Tiles[Index] = ETileType::AnyOther;
        ++ValidCellsCount;
    }

    float AreaCovered = ValidCellsCount / (float)kBaseLevelTilesCount;
    if (AreaCovered < MinimumAreaCovered)
        return false;

    return true;
}
