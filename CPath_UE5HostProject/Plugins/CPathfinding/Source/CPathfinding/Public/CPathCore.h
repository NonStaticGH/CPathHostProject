// // Copyright Dominik Trautman. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <vector>
#include "Containers/Queue.h"
#include "CPathfindingThread.h"
#include "CPathCore.generated.h"

/**
 *
 */


class ACPathVolume;

UCLASS()
class CPATHFINDING_API ACPathCore : public AActor
{
	GENERATED_BODY()

	friend class FCPathfindingThread;
public:

	~ACPathCore();

	
	static ACPathCore* GetInstance(UWorld* World);
	static bool DoesInstanceExist();
	static void EnableNewInstanceCreation();

	virtual void Tick(float DeltaSeconds) override;
	virtual void BeginPlay() override;
	virtual void BeginDestroy() override;
	virtual void EndPlay(EEndPlayReason::Type EndPlayReason) override;

	// This is called by CPathVolumes before Octrees are deleted
	// BeginDestroy() is latent and may ba called AFTER Octree is deleted, causing a crash
	// so this is necessary
	void StopAndDeleteThreads();

	// Using this directly is unsafe, please use the FindPathAsync function in ACPathVolume class.
	void AssignAsyncRequest(FCPathRequest& Request);

	

protected:
	ACPathCore();
	static void PrintCoreMessage(FString Message);

private:
	int ExpectedThreadCount;
	std::vector<FCPathfindingThread*> Threads;

	TQueue<std::pair<FCPathResult*, PathResultDelegate>, EQueueMode::Mpsc> OutputQueue;


	FCPathfindingThread* CreateThread(int ThreadIndex);



	static ACPathCore* Instance;
	static bool WasInstanceCreated;

};







