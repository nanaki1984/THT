// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RLG/RandomLevel.h"
#include "LevelGenerator.generated.h"

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
    int32 MinWallsDistance;
    UPROPERTY(EditAnywhere)
    int32 MaxWallsDistance;
    UPROPERTY(EditAnywhere)
    int32 MinDistanceFromCenter;
    UPROPERTY(EditAnywhere)
    int32 MaxDistanceFromCenter;

    FORCEINLINE bool TileIsValid(const FIntVector& Distances)
    {
        return Distances.Z >= MinWallsDistance
            && Distances.Z <= MaxWallsDistance
            && Distances.Y >= MinDistanceFromCenter
            && Distances.Y <= MaxDistanceFromCenter;
    }
};

UCLASS()
class THT_API ALevelGenerator : public AActor
{
	GENERATED_BODY()

protected:
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
    int32 TreasuresCount;
    UPROPERTY(Category = Placing, EditAnywhere, BlueprintReadOnly)
    FPlacingData ExitDoor;
    UPROPERTY(Category = Placing, EditAnywhere, BlueprintReadOnly)
    FPlacingData Treasures;

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
    UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly)
    FIntVector ExitDoorPosition;
    UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly)
    TArray<FIntVector> TreasurePositions;

    TArray<ETileType> Tiles;
    TArray<int32> TileFlags;
    TArray<FIntVector> TileDistances;

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
};
