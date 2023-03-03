// // Copyright Dominik Trautman. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <memory>
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

	virtual void Tick(float DeltaSeconds) override;
	virtual void BeginPlay() override;
	virtual void BeginDestroy() override;

	// This is called by CPathVolumes before Octrees are deleted
	// BeginDestroy() is latent and may ba called AFTER Octree is deleted, causing a crash
	// so this is necessary
	void StopAndDeleteThreads();

	void FindPathAsync(UObject* CallingObject, const FName& InFunctionName,
						ACPathVolume* VolumeRef, FVector Start, FVector End, 
						uint32 SmoothingPasses = 1, int32 UserData = 0, float TimeLimit = 0.2, 
						bool RequestRawPath = false, bool RequestUserPath = true);

protected:
	ACPathCore();

private:
	int ExpectedThreadCount;
	std::vector<FCPathfindingThread*> Threads;

	TQueue<std::pair<FCPathResult*, PathResultDelegate>, EQueueMode::Mpsc> OutputQueue;


	FCPathfindingThread* CreateThread(int ThreadIndex);

	inline void AssignAsyncRequest(FCPathRequest& Request);

	static ACPathCore* Instance;

};







