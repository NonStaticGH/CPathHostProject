// Copyright Epic Games, Inc. All Rights Reserved.

#include "CPath_UE5HostProjectGameMode.h"
#include "CPath_UE5HostProjectCharacter.h"
#include "Kismet/GameplayStatics.h"
#include "CPathVolume.h"
#include "UObject/ConstructorHelpers.h"

ACPath_UE5HostProjectGameMode::ACPath_UE5HostProjectGameMode()
{
	// set default pawn class to our Blueprinted character
	//static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter"));
	//if (PlayerPawnBPClass.Class != NULL)
	//{
	//	DefaultPawnClass = PlayerPawnBPClass.Class;
	//}
}

void ACPath_UE5HostProjectGameMode::BeginPlay()
{
	UGameplayStatics::GetActorOfClass(this, ACPathVolume::StaticClass());
	UE_LOG(LogTemp, Warning, TEXT("GAMEMODE ENTERED"));
}
