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

            for (auto& PlacingData : Objects)
            {
                if (PlacingData.Value.TileIsValid(Distances))
                {
                    Tiles[Index].AddClass(PlacingData.Key);
                    TilesByClass.FindOrAdd(PlacingData.Key).Add(Index);
                }
            }
        }

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

bool ALevelGenerator::GetRandomPositionsByClass(FName Class, int32 Count, int32 MinDistance, TArray<FIntVector>& OutPositions)
{
    TArray<int32>* SubTiles = TilesByClass.Find(Class);
    if (!SubTiles || 0 == SubTiles->Num())
        return false;

    // Shuffle indices of tiles of requested class
    int32 TilesIndicesNum = SubTiles->Num();

    TArray<int32> Indices;
    Indices.SetNumUninitialized(TilesIndicesNum);
    for (int32 Index = 0; Index < TilesIndicesNum; ++Index)
        Indices[Index] = Index;
    for (int32 Index = 0; Index < TilesIndicesNum - 1; ++Index)
    {
        int32 OtherIndex = FMath::RandRange(Index + 1, TilesIndicesNum - 1);
        Indices.Swap(Index, OtherIndex);
    }

    int32 OutStartIndex = OutPositions.Num();
    for (int32 Index = 0; Index < Count; ++Index)
    {
        int32 NextTileIndex = INDEX_NONE,
              IndexOffset   = FMath::RandRange(0, TilesIndicesNum - 1),
              TriesCount    = 0;
        while (INDEX_NONE == NextTileIndex)
        {
            int32 TileIndex = (*SubTiles)[Indices[IndexOffset]];

            int32 TileX = TileIndex % LevelSize,
                  TileY = TileIndex / LevelSize;

            bool bTileIsValid = true;
            for (int32 I = 0; I < Index; ++I)
            {
                int32 Dist = FMath::Abs(OutPositions[OutStartIndex + I].X - TileX) + FMath::Abs(OutPositions[OutStartIndex + I].Y - TileY);
                if (Dist < MinDistance)
                {
                    bTileIsValid = false;
                    break;
                }
            }

            if (bTileIsValid)
            {
                NextTileIndex = TileIndex;
                Indices[IndexOffset] = Indices[--TilesIndicesNum];
            }
            else
            {
                IndexOffset = (IndexOffset + 1) % TilesIndicesNum;

                if (++TriesCount == TilesIndicesNum)
                {
                    if (--MinDistance <= 0)
                        return false;

                    TriesCount  = 0;
                }
            }
        }
        check(NextTileIndex != INDEX_NONE);
        OutPositions.Add(FIntVector(NextTileIndex % LevelSize, NextTileIndex / LevelSize, NextTileIndex));
    }

    for (int32 Index = 0; Index < Count; ++Index)
    {
        FIntVector& TilePos = OutPositions[OutStartIndex + Index];

        SubTiles->Remove(TilePos.Z);
        Tiles[TilePos.Z].RemoveClass(Class);

        TilePos.Z = 0;
    }

    return true;
}
