// // Copyright Dominik Trautman. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "CPathNode.h"
#include <memory>
#include "NewThreadingTest.generated.h"

UCLASS()
class CPATH_UE5HOSTPROJECT_API ANewThreadingTest : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ANewThreadingTest();

	UPROPERTY(VisibleAnywhere)
		int UnfinishedRequests = 0;
		
	UPROPERTY(EditAnywhere)
		float TickInterval = 0;


	UFUNCTION()
	void OnPathFound(FCPathResult& PathResult);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	class ACPathVolume* VolumeRef = nullptr;
};
