// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RLG/RandomLevel.h"
#include "LevelGenerator.generated.h"

UENUM(BlueprintType)
enum class ETileType : uint8
{
    Blocked = 0,
    AnyOther,
};

UENUM(BlueprintType, Meta = (Bitflags))
enum class ECellFlags : uint8
{
    Floor = 0,
    Ceiling,
    NWall,
    EWall,
    SWall,
    WWall,
};

USTRUCT(BlueprintType)
struct FPlacingData
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere)
    int32 MinDistanceFromCenter;
    UPROPERTY(EditAnywhere)
    int32 MaxDistanceFromCenter;
    UPROPERTY(EditAnywhere)
    int32 MinNearestWallDistance;
    UPROPERTY(EditAnywhere)
    int32 MaxNearestWallDistance;
    UPROPERTY(EditAnywhere)
    int32 MinFarthestWallDistance;
    UPROPERTY(EditAnywhere)
    int32 MaxFarthestWallDistance;

    FPlacingData()
        : MinDistanceFromCenter(0)
        , MaxDistanceFromCenter(255)
        , MinNearestWallDistance(0)
        , MaxNearestWallDistance(255)
        , MinFarthestWallDistance(0)
        , MaxFarthestWallDistance(255)
    { }

    FORCEINLINE bool TileIsValid(const FIntVector& Distances)
    {
        return Distances.X >= MinDistanceFromCenter
            && Distances.X <= MaxDistanceFromCenter
            && Distances.Y >= MinNearestWallDistance
            && Distances.Y <= MaxNearestWallDistance
            && Distances.Z >= MinFarthestWallDistance
            && Distances.Z <= MaxFarthestWallDistance;
    }
};

USTRUCT(BlueprintType)
struct FTileData
{
	GENERATED_BODY()

private:
    TArray<FName, TInlineAllocator<4>> Classes;

public:
    UPROPERTY(BlueprintReadOnly)
    ETileType Type;
    UPROPERTY(BlueprintReadOnly, meta = (Bitmask, BitmaskEnum = "ECellFlags"))
    int32 Flags;
    UPROPERTY(BlueprintReadOnly)
    FIntVector Distances;

    FORCEINLINE void AddClass(FName ClassName) { Classes.AddUnique(ClassName); }
    FORCEINLINE void RemoveClass(FName ClassName) { Classes.Remove(ClassName); }

    FORCEINLINE bool HasClass(FName ClassName) const { return Classes.Contains(ClassName); }
    FORCEINLINE bool HasFlag(ECellFlags Flag) const { return Flags & (1 << (int32)Flag); }
};

UCLASS()
class THT_API ALevelGenerator : public AActor
{
	GENERATED_BODY()

protected:
    UPROPERTY(Category=CellularAutomata, EditAnywhere, BlueprintReadOnly)
    int32 InitialLevelWidth;
    UPROPERTY(Category=CellularAutomata, EditAnywhere, BlueprintReadOnly)
    float InitialChance;
    UPROPERTY(Category=CellularAutomata, EditAnywhere, BlueprintReadOnly)
    int32 BirthLimit;
    UPROPERTY(Category=CellularAutomata, EditAnywhere, BlueprintReadOnly)
    int32 DeathLimit;
    UPROPERTY(Category=CellularAutomata, EditAnywhere, BlueprintReadOnly)
    int32 Steps;
    UPROPERTY(Category=CellularAutomata, EditAnywhere, BlueprintReadOnly)
    float MinimumAreaCovered;

    UPROPERTY(Category = Placing, EditAnywhere, BlueprintReadOnly)
    TMap<FName, FPlacingData> Objects;

    UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly)
    int32 LevelSize;

    UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly)
    FIntVector CornerNE;
    UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly)
    FIntVector CornerSE;
    UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly)
    FIntVector CornerSW;
    UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly)
    FIntVector CornerNW;

    UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly)
    FIntVector LevelCenter;

    TArray<FTileData> Tiles;
    TMap<FName, TArray<int32>> TilesByClass;

    virtual void BeginPlay() override;

public:
    ALevelGenerator();

    FORCEINLINE int32 GetLevelSize() const { return LevelSize; }

    UFUNCTION(BlueprintCallable)
    bool GenerateNewTiles(int32 MaxTriesCount = 8);

    UFUNCTION(BlueprintCallable)
    ETileType GetTileAt(int32 X, int32 Y) const;
    UFUNCTION(BlueprintCallable)
    bool HasTileFlag(int32 X, int32 Y, ECellFlags Flag) const;
    UFUNCTION(BlueprintCallable)
    const FIntVector& GetTileDistances(int32 X, int32 Y) const;

    UFUNCTION(BlueprintCallable)
    bool GetRandomPositionsByClass(FName Class, int32 Count, int32 MinDistance, TArray<FIntVector>& OutPositions);
};
