#include "PathGenerator.h"

void FPathGenerator::Setup(ETileType* InTiles, int32 InLevelSize)
{
    Tiles = InTiles;
    check(Tiles);
    LevelSize = InLevelSize;
    check(LevelSize > 0);
    Points.SetNum(LevelSize * LevelSize);

    OpenSet.Reserve(LevelSize);
    CloseSet.Init(false, LevelSize * LevelSize);
    VisitedSet.Init(false, LevelSize * LevelSize);
}

bool FPathGenerator::GeneratePath(const FIntVector& Start, const FIntVector& End, TArray<FIntVector>& OutPath)
{
    OutPath.SetNum(0, false);

    if (Start.X == End.X && Start.Y == End.Y)
    {
        OutPath.Insert(FIntVector(Start.X, Start.Y, 0), 0);
        return true;
    }

    OpenSet.Reset();
    CloseSet.SetRange(0, LevelSize * LevelSize, false);
    VisitedSet.SetRange(0, LevelSize * LevelSize, false);

    int32 Offset = Start.X + Start.Y * LevelSize;
    Points[Offset] = FPathPoint(Start.X, Start.Y);
    VisitedSet[Offset] = true;

    auto LessFunc = [](const FPathPoint& A, const FPathPoint& B) { return A.F < B.F; };
    FPathPoint* Current = Points.GetData() + Offset;
    OpenSet.HeapPush(Current, LessFunc);

    FIntVector NeighbourOffsets[4] =
    { {  0, -1, 0 }
    , {  0,  1, 0 }
    , {  1,  0, 0 }
    , { -1,  0, 0 } };

    while (OpenSet.Num() > 0)
    {
        OpenSet.HeapPop(Current, LessFunc, false);

        Offset = Current->X + Current->Y * LevelSize;
        CloseSet[Offset] = true;

        if (Current->X == End.X && Current->Y == End.Y)
        {
            while (Current)
            {
                OutPath.Insert(FIntVector(Current->X, Current->Y, 0), 0);
                Current = Current->Parent;
            }

            return true;
        }

        for (FIntVector NeighbourOffset : NeighbourOffsets)
        {
            int32 X = Current->X + NeighbourOffset.X;
            int32 Y = Current->Y + NeighbourOffset.Y;
            if (X >= 0 && X < LevelSize && Y >= 0 && Y < LevelSize)
            {
                Offset = X + Y * LevelSize;
                if (CloseSet[Offset])
                    continue;
                if (ETileType::Blocked == Tiles[Offset])
                    continue;

                int32 Manhattan = FMath::Abs(End.X - X) + FMath::Abs(End.Y - Y);
                FPathPoint NewPathPoint(X, Y, Current, 1, Manhattan);

                if (!VisitedSet[Offset] || NewPathPoint.G < Points[Offset].G)
                {
                    Points[Offset] = NewPathPoint;
                    VisitedSet[Offset] = true;
                    OpenSet.HeapPush(Points.GetData() + Offset, LessFunc);
                }
            }
        }
    }

    return false;
}
