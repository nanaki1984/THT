#pragma once

#include "CoreMinimal.h"

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

    TArray<int32> TileCosts;
    int32 LevelSize;
    TArray<FPathPoint> Points;

    TArray<FPathPoint*> OpenSet;
    TBitArray<FDefaultBitArrayAllocator> CloseSet;
    TBitArray<FDefaultBitArrayAllocator> VisitedSet;

public:
    void Setup(TArray<int32>& InTileCosts, int32 InLevelSize = 64);
    bool GeneratePath(const FIntVector& Start, const FIntVector& End, TArray<FIntVector>& OutPath);
};
