// // Copyright Dominik Trautman. All Rights Reserved.


#include "NewThreadingTest.h"
#include "CPathCore.h"
#include "CPathVolume.h"

// Sets default values
ANewThreadingTest::ANewThreadingTest()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	
}



void ANewThreadingTest::OnPathFound(FCPathResult& PathResult)
{
	UE_LOG(LogTemp, Warning, TEXT("TEST: ReceivedPath with result: %d"), (int)PathResult.FailReason);
	UnfinishedRequests--;
}

// Called when the game starts or when spawned
void ANewThreadingTest::BeginPlay()
{
	Super::BeginPlay();
	SetActorTickInterval(TickInterval);
}

// Called every frame
void ANewThreadingTest::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (!VolumeRef)
	{
		TArray<AActor*> Overlapping;
		GetOverlappingActors(Overlapping, ACPathVolume::StaticClass());
		if (Overlapping.Num() > 0)
			VolumeRef = Cast<ACPathVolume>(Overlapping[0]);
	}
	else
	{
		auto FunctionName = GET_FUNCTION_NAME_CHECKED(ANewThreadingTest, OnPathFound);
		FVector End = GetActorLocation() + FVector(2000, 0, 0);
		ACPathCore::GetInstance(GetWorld())->FindPathAsync(this, FunctionName, VolumeRef, GetActorLocation(), End, 2);
		UnfinishedRequests++;
	}
	
}

