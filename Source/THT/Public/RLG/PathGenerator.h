#pragma once

#include "CoreMinimal.h"
#include "RandomLevel.h"

struct THT_API FPathGenerator
{
private:
    struct FPathPoint
    {
        int32 X;
        int32 Y;

        int32 G;
        int32 F;

        FPathPoint* Parent;

        FPathPoint()
        { }

        FPathPoint(int32 InX, int32 InY)
            : X(InX)
            , Y(InY)
            , G(0)
            , F(0)
            , Parent(nullptr)
        { }

        FPathPoint(int32 InX, int32 InY, FPathPoint* InParent, int32 Cost, int32 Heuristic)
            : X(InX)
            , Y(InY)
            , G(InParent->G + Cost)
            , F(InParent->G + Cost + Heuristic)
            , Parent(InParent)
        { }
    };

    ETileType* Tiles;
    int32 LevelSize;
    TArray<FPathPoint> Points;

public:
    void Setup(ETileType* InTiles, int32 InLevelSize = 64);
    bool GeneratePath(const FIntVector& Start, const FIntVector& End, TArray<FIntVector>& OutPath);
};
