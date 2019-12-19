#include "RLG/LevelGenerator.h"
#include "RLG/PathGenerator.h"

ALevelGenerator::ALevelGenerator()
    : InitialLevelWidth(80)
    , InitialChance(.4f)
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
        RandomLevel.Generate(InitialLevelWidth, InitialChance, BirthLimit, DeathLimit, Steps);
        bSucceded = RandomLevel.QualityCheck(MinimumAreaCovered);
        ++Counter;
    } while (!bSucceded && Counter <= MaxTriesCount);

    if (bSucceded)
    {
        LevelSize = RandomLevel.GetLevelSize();
        Tiles.SetNum(LevelSize * LevelSize);
/*
        Tiles = MoveTemp(RandomLevel.Tiles);
        TileFlags.SetNum(Tiles.Num());
        TileDistances.SetNum(Tiles.Num());
*/
        int32 LastTile      = LevelSize - 1,
              LastTileIndex = LevelSize * LevelSize - 1;

        TArray<int32> TileCosts;
        for (int32 Index = 0; Index < LastTileIndex; ++Index)
        {
            if (RandomLevel.GetTileIsValid(Index))
            {
                Tiles[Index].Type = ETileType::AnyOther;
                TileCosts.Add(1);
            }
            else
            {
                Tiles[Index].Type = ETileType::Blocked;
                TileCosts.Add(0);
            }
        }

        int32 MinDistNE = TNumericLimits<int32>::Max(),
              MinDistSE = TNumericLimits<int32>::Max(),
              MinDistSW = TNumericLimits<int32>::Max(),
              MinDistNW = TNumericLimits<int32>::Max();

        for (int32 Index = 0; Index <= LastTileIndex; ++Index)
        {
            int32 X = Index % LevelSize,
                  Y = Index / LevelSize;

            if (ETileType::Blocked == Tiles[Index].Type)
            {
                Tiles[Index].Flags = 0;
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

            int32 Flags = (1 << (int32)ECellFlags::Floor) |
                          (1 << (int32)ECellFlags::Ceiling);

            if (0 == Y || ETileType::Blocked == Tiles[Index - LevelSize].Type)
                Flags |= (1 << (int32)ECellFlags::WWall);
            if (LastTile == Y || ETileType::Blocked == Tiles[Index + LevelSize].Type)
                Flags |= (1 << (int32)ECellFlags::EWall);
            if (0 == X || ETileType::Blocked == Tiles[Index - 1].Type)
                Flags |= (1 << (int32)ECellFlags::SWall);
            if (LastTile == X || ETileType::Blocked == Tiles[Index + 1].Type)
                Flags |= (1 << (int32)ECellFlags::NWall);

            Tiles[Index].Flags = Flags;
        }

        FPathGenerator PathGen;
        PathGen.Setup(TileCosts, LevelSize);

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
            if (ETileType::Blocked == Tiles[Index].Type)
            {
                Tiles[Index].Distances = FIntVector::ZeroValue;
                continue;
            }

            int32 X = Index % LevelSize,
                  Y = Index / LevelSize;

            verify(PathGen.GeneratePath(LevelCenter, FIntVector(X, Y, 0), Path));
            FIntVector& Distances = Tiles[Index].Distances;
            int32 CenterDist = Distances.X = Path.Num();

            int32 MinWallDist = TNumericLimits<int32>::Max();
            for (int32 OtherIndex = 0; OtherIndex <= LastTileIndex; ++OtherIndex)
            {
                if (Index == OtherIndex || Tiles[OtherIndex].Type > ETileType::Blocked)
                    continue;

                int32 OtherX = OtherIndex % LevelSize, OtherY = OtherIndex / LevelSize;

                int32 WallDist = FMath::Abs(X - OtherX) + FMath::Abs(Y - OtherY) - 1;
                if (WallDist < MinWallDist)
                    MinWallDist = WallDist;
            }

            Distances.Y = MinWallDist;
        }

        for (int32 Index = 0; Index <= LastTileIndex; ++Index)
        {
            if (ETileType::Blocked == Tiles[Index].Type)
                continue;

            FIntVector& Distances = Tiles[Index].Distances;

            int32 X   = Index % LevelSize,
                  Y   = Index / LevelSize,
                  Rad = 8;

            int32 MaxWallDist = 0;
            for (int32 J = -Rad; J <= Rad; ++J)
            {
                for (int32 K = -Rad; K <= Rad; ++K)
                {
                    if (FMath::Abs(J) + FMath::Abs(K) > Rad)
                        continue;

                    int32 XX = X + J, YY = Y + K;
                    if (XX >= 0 && XX <= LastTile && YY >= 0 && YY <= LastTile)
                    {
                        int32 OtherIndex = YY * LevelSize + XX;
                        if (Tiles[OtherIndex].Type > ETileType::Blocked)
                            MaxWallDist = FMath::Max(MaxWallDist, Tiles[OtherIndex].Distances.Y);
                    }
                }
            }

            Distances.Z = MaxWallDist;

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
            if (i < TreasurePosList.Num())
                TreasurePositions.Add(TreasurePosList[i]);

        // ToDo: players needs a map of the level, with fog of war to keep the discovery reward intact

        return true;
    }

    return false;
}

ETileType ALevelGenerator::GetTileAt(int32 X, int32 Y) const
{
    if (X >= 0 && X < LevelSize && Y >= 0 && Y < LevelSize)
        return Tiles[Y * LevelSize + X].Type;
    return ETileType::Blocked;
}

bool ALevelGenerator::HasTileFlag(int32 X, int32 Y, ECellFlags Flag) const
{
    if (X >= 0 && X < LevelSize && Y >= 0 && Y < LevelSize)
        return Tiles[Y * LevelSize + X].Flags & (1 << (int32)Flag);
    return false;
}

const FIntVector& ALevelGenerator::GetTileDistances(int32 X, int32 Y) const
{
    if (X >= 0 && X < LevelSize && Y >= 0 && Y < LevelSize)
        return Tiles[Y * LevelSize + X].Distances;
    return FIntVector::ZeroValue;
}
