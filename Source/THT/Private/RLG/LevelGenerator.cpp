#include "RLG/LevelGenerator.h"
#include "RLG/PathGenerator.h"

ALevelGenerator::ALevelGenerator()
    : InitialChance(.4f)
    , BirthLimit(4)
    , DeathLimit(3)
    , Steps(6)
    , MinimumAreaCovered(.6f)
    , LevelSize(0)
{
	PrimaryActorTick.bCanEverTick = false;
}

void ALevelGenerator::BeginPlay()
{
	Super::BeginPlay();
}

bool ALevelGenerator::GenerateNewTiles(int32 MaxTriesCount)
{
    FRandomLevel RandomLevel;

    int32 Counter = 0;
    bool bSucceded = false;
    do
    {
        RandomLevel.Generate(InitialChance, BirthLimit, DeathLimit, Steps);
        bSucceded = RandomLevel.QualityCheck(MinimumAreaCovered);
        ++Counter;
    } while (!bSucceded && Counter <= MaxTriesCount);

    if (bSucceded)
    {
        LevelSize = RandomLevel.kBaseLevelSize;
        Tiles = MoveTemp(RandomLevel.Tiles);
        TileFlags.SetNum(Tiles.Num());
        TileDistances.SetNum(Tiles.Num());

        int32 MinDistNE = TNumericLimits<int32>::Max(),
              MinDistSE = TNumericLimits<int32>::Max(),
              MinDistSW = TNumericLimits<int32>::Max(),
              MinDistNW = TNumericLimits<int32>::Max();

        int32 LastTile      = LevelSize - 1,
              LastTileIndex = LevelSize * LevelSize - 1;
        for (int32 Index = 0; Index <= LastTileIndex; ++Index)
        {
            int32 X = Index % LevelSize, Y = Index / LevelSize;

            if (ETileType::Blocked == Tiles[Index])
            {
                TileFlags[Index] = 0;
                continue;
            }

            int32 DistNE = (LastTile - X) + Y,
                  DistSE = (LastTile - X) + (LastTile - Y),
                  DistSW = X + (LastTile - Y),
                  DistNW = X + Y;

            if (DistNE < MinDistNE)
            {
                CornerNE = FIntVector(X, Y, 0);
                MinDistNE = DistNE;
            }
            if (DistSE < MinDistSE)
            {
                CornerSE = FIntVector(X, Y, 0);
                MinDistSE = DistSE;
            }
            if (DistSW < MinDistSW)
            {
                CornerSW = FIntVector(X, Y, 0);
                MinDistSW = DistSW;
            }
            if (DistNW < MinDistNW)
            {
                CornerNW = FIntVector(X, Y, 0);
                MinDistNW = DistNW;
            }

            TileFlags[Index] = (1 << (int32)ECellFlags::Floor) |
                               (1 << (int32)ECellFlags::Ceiling);

            if (0 == Y || ETileType::Blocked == Tiles[Index - LevelSize])
                TileFlags[Index] |= (1 << (int32)ECellFlags::WWall);
            if (LastTile == Y || ETileType::Blocked == Tiles[Index + LevelSize])
                TileFlags[Index] |= (1 << (int32)ECellFlags::EWall);
            if (0 == X || ETileType::Blocked == Tiles[Index - 1])
                TileFlags[Index] |= (1 << (int32)ECellFlags::SWall);
            if (LastTile == X || ETileType::Blocked == Tiles[Index + 1])
                TileFlags[Index] |= (1 << (int32)ECellFlags::NWall);
        }

        FPathGenerator PathGen;
        PathGen.Setup(Tiles.GetData(), LevelSize);

        TArray<FIntVector> Path;
        FIntVector Center0, Center1;
        verify(PathGen.GeneratePath(CornerNW, CornerSE, Path));
        Center0 = Path[Path.Num() >> 1];
        verify(PathGen.GeneratePath(CornerSW, CornerNE, Path));
        Center1 = Path[Path.Num() >> 1];
        verify(PathGen.GeneratePath(Center0, Center1, Path));
        LevelCenter = Path[Path.Num() >> 1];

        TArray<FIntVector> ExitDoorPositions;
        TArray<FIntVector> TreasurePosList;
        for (int32 Index = 0; Index <= LastTileIndex; ++Index)
        {
            if (ETileType::Blocked == Tiles[Index])
            {
                TileDistances[Index] = FIntVector::ZeroValue;
                continue;
            }

            int32 X = Index % LevelSize, Y = Index / LevelSize;

            verify(PathGen.GeneratePath(LevelCenter, FIntVector(X, Y, 0), Path));
            FIntVector& Distances = TileDistances[Index];
            Distances.X = FMath::Abs(LevelCenter.X - X) + FMath::Abs(LevelCenter.Y - Y);
            int32 CenterDist = Distances.Y = Path.Num();

            int32 MinWallDist = TNumericLimits<int32>::Max();
            for (int32 OtherIndex = 0; OtherIndex <= LastTileIndex; ++OtherIndex)
            {
                if (Index == OtherIndex || Tiles[OtherIndex] > ETileType::Blocked)
                    continue;

                int32 OtherX = OtherIndex % LevelSize, OtherY = OtherIndex / LevelSize;
                int32 WallDist = FMath::Abs(X - OtherX) + FMath::Abs(Y - OtherY);
                if (WallDist < MinWallDist)
                    MinWallDist = WallDist;
            }

            Distances.Z = MinWallDist;

            if (ExitDoor.TileIsValid(Distances))
                ExitDoorPositions.Add(FIntVector(X, Y, 0));

            if (Treasures.TileIsValid(Distances))
                TreasurePosList.Add(FIntVector(X, Y, 0));
        }

        check(ExitDoorPositions.Num() > 0);
        ExitDoorPosition = ExitDoorPositions[FMath::RandRange(0, ExitDoorPositions.Num() - 1)];

        int32 LastIndex = TreasurePosList.Num() - 1;
		for (int32 i = 0; i <= LastIndex; ++i)
		{
			int32 Index = FMath::RandRange(i, LastIndex);
			if (i != Index)
                TreasurePosList.Swap(i, Index);
		}

        for (int32 i = 0; i < TreasuresCount; ++i)
            TreasurePositions.Add(TreasurePosList[i]);

        // ToDo: players needs a map of the level, with fog of war to keep the discovery reward intact

        return true;
    }

    return false;
}

ETileType ALevelGenerator::GetTileAt(int32 X, int32 Y) const
{
    if (X >= 0 && X < LevelSize && Y >= 0 && Y < LevelSize)
        return Tiles[Y * LevelSize + X];
    return ETileType::Blocked;
}

bool ALevelGenerator::HasTileFlag(int32 X, int32 Y, ECellFlags Flag) const
{
    if (X >= 0 && X < LevelSize && Y >= 0 && Y < LevelSize)
        return TileFlags[Y * LevelSize + X] & (1 << (int32)Flag);
    return false;
}

const FIntVector& ALevelGenerator::GetTileDistances(int32 X, int32 Y) const
{
    if (X >= 0 && X < LevelSize && Y >= 0 && Y < LevelSize)
        return TileDistances[Y * LevelSize + X];
    return FIntVector::ZeroValue;
}
