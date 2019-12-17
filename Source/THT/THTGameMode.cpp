// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

#include "THTGameMode.h"
#include "THTCharacter.h"
#include "UObject/ConstructorHelpers.h"

ATHTGameMode::ATHTGameMode()
{
	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/ThirdPersonCPP/Blueprints/ThirdPersonCharacter"));
	if (PlayerPawnBPClass.Class != NULL)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}
