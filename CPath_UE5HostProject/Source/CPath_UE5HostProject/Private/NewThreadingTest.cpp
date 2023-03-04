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


uint32 ANewThreadingTest::RequestsSent = 0;
uint32 ANewThreadingTest::RequestsReceived = 0;

void ANewThreadingTest::OnPathFound(FCPathResult& PathResult)
{
#ifdef LOG_PATHFINDERS
	UE_LOG(LogTemp, Warning, TEXT("TEST: ReceivedPath with result: %d"), (int)PathResult.FailReason);
#endif
	
	UnfinishedRequests--;
	RequestsReceived++;
}

// Called when the game starts or when spawned
void ANewThreadingTest::BeginPlay()
{
	Super::BeginPlay();
	SetActorTickInterval(TickInterval);
	RequestsSent = 0;
	RequestsReceived = 0;
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
		VolumeRef->FindPathAsync(this, FunctionName, GetActorLocation(), End, 2);
		UnfinishedRequests++;
		RequestsSent++;
	}
	RequestsSentBP = RequestsSent;
	RequestsReceivedBP = RequestsReceived;
}

