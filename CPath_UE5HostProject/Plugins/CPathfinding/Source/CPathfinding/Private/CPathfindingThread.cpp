// // Copyright Dominik Trautman. All Rights Reserved.


#include "CPathfindingThread.h"
#include "CPathVolume.h"
#include "CPathFindPath.h"
#include "Engine/World.h"
#include "GenericPlatform/GenericPlatformProcess.h"
#include "CPathCore.h"


FCPathfindingThread::FCPathfindingThread(ACPathCore* Producer, int Index)
{
	CoreRef = Producer;
	ThreadIndex = Index;
	Semaphore = FGenericPlatformProcess::GetSynchEventFromPool();
	ThreadName = FString::Printf(TEXT("CPathfindingThread %d"), Index);
	Thread = FRunnableThread::Create(this, *ThreadName);
	AStar = new CPathAStar();
}

FCPathfindingThread::~FCPathfindingThread()
{
	PrintThreadMessage(FString("Destructor"));
	if (Semaphore)
	{
		FGenericPlatformProcess::ReturnSynchEventToPool(Semaphore);
		Semaphore = nullptr;
	}
	if (Thread)
	{
		delete Thread;
		Thread = nullptr;
	}	
	if (AStar)
	{
		delete AStar;
		AStar = nullptr;
	}
}

bool FCPathfindingThread::Init()
{
	InputQueue.Empty();
	return true;
}

uint32 FCPathfindingThread::Run()
{
	PrintThreadMessage(FString("Working"));
	while (!KillRequested.load())
	{
		// Getting/waiting for new request
		if (InputQueue.IsEmpty())
		{
			IsDoingWork = false;
			PrintThreadMessage(FString::Printf(TEXT("WaitingForTask. CurrentTaskCount= %d, TasksSubmited= %d, TasksAssigned= %d"), CurrentTaskCount.load(), TasksSubmited, TasksAssigned));
			Semaphore->Wait();
			if (KillRequested)
				return 0;
			IsDoingWork = true;
		}
		FCPathRequest Request;
		if (!InputQueue.Dequeue(Request))
			continue;

		// After volume is generated and valid, performing FindPath call
		if (WaitForVolume(Request.VolumeRef))
		{
			Request.VolumeRef->PathfindersRunning++;

			// State of the volume could have changed during incrementing the atomic variable
			// So it's necessary to check it again before doing any pathfinding
			if (WaitForVolume(Request.VolumeRef))
			{
				// This is deleted in CPathCore::Tick
				FCPathResult* Result = new FCPathResult();

				Result->FailReason = AStar->FindPath(Request.VolumeRef, Result, Request.Start, Request.End,
					Request.SmoothingPasses, Request.UserData, Request.TimeLimit,
					Request.RequestRawPath, Request.RequestUserPath);
					
				Request.VolumeRef->PathfindersRunning--;

				// Thread could be stopped during pathfinding
				// In this case we dont have a proper result
				if (KillRequested)
				{
					delete Result;
					return 0;
				}

				SubmitResult(Result, Request.OnPathFound);
			}
			else
			{
				Request.VolumeRef->PathfindersRunning--;
				CurrentTaskCount--;
			}
		}
		else
		{
			CurrentTaskCount--;
		}
	
	}
	return 0;
}

void FCPathfindingThread::Stop()
{
	PrintThreadMessage(FString("Stop"));
	KillRequested.store(true);
	AStar->bStop.store(true);
	InputQueue.Empty();
	WakeUp();
}

void FCPathfindingThread::Exit()
{
	PrintThreadMessage(FString("Exit"));
}



void FCPathfindingThread::EnsureCompletion()
{	
	PrintThreadMessage(FString("Ensuring Completion"));
	Stop();
	if (Thread)
		Thread->WaitForCompletion(); 
}


bool FCPathfindingThread::IsWorking()
{
	return IsDoingWork.load();
}

void FCPathfindingThread::WakeUp()
{
	if(Semaphore)
		Semaphore->Trigger();
}

bool FCPathfindingThread::IsThreadValid()
{
	return (bool)Thread;
}

int FCPathfindingThread::GetTaskCount()
{
	return CurrentTaskCount.load();
}

void FCPathfindingThread::AssignTask(FCPathRequest& FindPathRequest)
{
	InputQueue.Enqueue(FindPathRequest);
	CurrentTaskCount++;
	TasksAssigned++;
	WakeUp();
}

void FCPathfindingThread::PrintThreadMessage(FString Message)
{
#ifdef LOG_PATHFINDERS
	UE_LOG(LogTemp, Warning, TEXT("%s: %s"), *ThreadName, *Message);
#endif
}

void FCPathfindingThread::SubmitResult(FCPathResult* Result, PathResultDelegate Delegate)
{
	checkf(IsValid(CoreRef), TEXT("CPATH - PathfindingThread SubmitResult:::CoreRef not valid!"));
	CoreRef->OutputQueue.Enqueue(std::pair<FCPathResult*, PathResultDelegate>(Result, Delegate));
	CurrentTaskCount--;
	TasksSubmited++;
}

bool FCPathfindingThread::WaitForVolume(ACPathVolume* Volume)
{
	if (IsValid(Volume))
	{
		if (Volume->GeneratorsRunning.load() > 0 || !Volume->InitialGenerationCompleteAtom.load())
		{
			if (Volume->GenerationFinishedSemaphore)
			{
				Volume->PathfindersWaiting++;
				PrintThreadMessage(FString("WaitingForVolume"));
				Volume->GenerationFinishedSemaphore->Wait();
				if (IsValid(Volume))
				{
					Volume->PathfindersWaiting--;
					return true;
				}
			}
			// false cause generation ongoing and semaphore invalid
			return false;
		}
		// true here cause volume is valid and not generating
		return true;
	}
	return false;
	
}
