// // Copyright Dominik Trautman. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Core/Public/HAL/Runnable.h"
#include "Core/Public/HAL/RunnableThread.h"
#include "CPathNode.h"
#include <atomic>
#include "HAL/Event.h"


// The class used to perform pathfinding on its own thread
class CPATHFINDING_API FCPathfindingThread : public FRunnable
{
public:
	FCPathfindingThread(class ACPathCore* Producer, int Index);
	~FCPathfindingThread();

	virtual bool Init();

	virtual uint32 Run();

	virtual void Stop();

	virtual void Exit();



	// -----These public members are accessed from main game thread!!---

	void EnsureCompletion();

	bool IsWorking();

	void WakeUp();

	bool IsThreadValid();

	// How much tasks are assigned for this thread.
	// Includes the one that it's currently working on.
	int GetTaskCount();

	void AssignTask(FCPathRequest& FindPathRequest);

	void PrintThreadMessage(FString Message);
	
	int ThreadIndex;
	
	// -----------------------------------------------------------------

private:
	TQueue<FCPathRequest, EQueueMode::Spsc> InputQueue;

	std::atomic_bool KillRequested = false;
	std::atomic_bool IsDoingWork = false;
	std::atomic_int CurrentTaskCount = 0;
	int TasksSubmited = 0;
	int TasksAssigned = 0;
	
	class ACPathCore* CoreRef = nullptr;
	FRunnableThread* Thread = nullptr;
	class CPathAStar* AStar = nullptr;

	FCriticalSection Mutex;
	FEvent* Semaphore = nullptr;

	FString ThreadName;

	void SubmitResult(FCPathResult* Result, PathResultDelegate Delegate);

	// Returns false if volume is not valid before/after waiting
	bool WaitForVolume(class ACPathVolume* Volume);





};