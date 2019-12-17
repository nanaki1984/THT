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

        FIntVector Corner0, Corner1;
        int32 MinDist0 = TNumericLimits<int32>::Max(),
              MinDist1 = TNumericLimits<int32>::Max();

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

            int32 Dist0 = X + Y;
            int32 Dist1 = (LastTile - X) + (LastTile - Y);

            if (Dist0 < MinDist0)
            {
                Corner0 = FIntVector(X, Y, 0);
                MinDist0 = Dist0;
            }

            if (Dist1 < MinDist1)
            {
                Corner1 = FIntVector(X, Y, 0);
                MinDist1 = Dist1;
            }

            TileFlags[Index] = (1 << (int32)ECellFlags::Floor) |
                               (1 << (int32)ECellFlags::Ceiling);

            if (0 == Y || ETileType::Blocked == Tiles[Index - LevelSize])
                TileFlags[Index] |= (1 << (int32)ECellFlags::NWall);
            if (LastTile == Y || ETileType::Blocked == Tiles[Index + LevelSize])
                TileFlags[Index] |= (1 << (int32)ECellFlags::SWall);
            if (0 == X || ETileType::Blocked == Tiles[Index - 1])
                TileFlags[Index] |= (1 << (int32)ECellFlags::WWall);
            if (LastTile == X || ETileType::Blocked == Tiles[Index + 1])
                TileFlags[Index] |= (1 << (int32)ECellFlags::EWall);
        }

        FPathGenerator PathGen;
        PathGen.Setup(Tiles.GetData(), LevelSize);

        TArray<FIntVector> Path;
        if (PathGen.GeneratePath(Corner0, Corner1, Path))
            LevelCenter = Path[Path.Num() >> 1];

		// ToDo: compute nearest distance to wall and distance from center for every cell

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
