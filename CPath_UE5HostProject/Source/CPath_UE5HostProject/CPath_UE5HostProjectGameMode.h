// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "CPath_UE5HostProjectGameMode.generated.h"

UCLASS(minimalapi)
class ACPath_UE5HostProjectGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	ACPath_UE5HostProjectGameMode();

	virtual void BeginPlay() override;

};



