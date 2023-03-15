// // Copyright Dominik Trautman. All Rights Reserved.


#include "CPathCore.h"
#include "CPathFindPath.h"
#include "Delegates/Delegate.h"

#include "CPathfindingThread.h"

// Sets default values
ACPathCore::ACPathCore()
{
	PrimaryActorTick.bCanEverTick = true;
}

ACPathCore* ACPathCore::Instance = nullptr;
bool ACPathCore::WasInstanceCreated = false;

ACPathCore* ACPathCore::GetInstance(UWorld* World)
{
	if (!IsValid(Instance) && !WasInstanceCreated)
	{
		PrintCoreMessage(FString("Spawning Instance"));
		Instance = World->SpawnActor<ACPathCore>();
		WasInstanceCreated = true;
	}
	return Instance;
}

bool ACPathCore::DoesInstanceExist()
{
	return (bool)Instance;
}

void ACPathCore::EnableNewInstanceCreation()
{
	WasInstanceCreated = false;
}

// Called when the game starts or when spawned
void ACPathCore::BeginPlay()
{
	Super::BeginPlay();
	ExpectedThreadCount = FPlatformMisc::NumberOfCores() - 1;
	//ExpectedThreadCount = 1;
	for (int i = 0; i < ExpectedThreadCount; i++)
	{
		Threads.push_back(CreateThread(i));
	}
}

void ACPathCore::BeginDestroy()
{
	StopAndDeleteThreads();
	Instance = nullptr;
	Super::BeginDestroy();
}

void ACPathCore::EndPlay(EEndPlayReason::Type EndPlayReason)
{
	StopAndDeleteThreads();
	Instance = nullptr;
	Super::EndPlay(EndPlayReason);
}

void ACPathCore::StopAndDeleteThreads()
{
	if (Threads.size() > 0)
	{
		PrintCoreMessage(FString("Deleting threads"));
	}
	while (Threads.size() > 0)
	{
		auto Thread = Threads.back();
		Thread->EnsureCompletion();
		delete Thread;
		Threads.pop_back();
	}

	
}


void ACPathCore::PrintCoreMessage(FString Message)
{
#ifdef LOG_PATHFINDERS
	if(LOG_PATHFINDERS > 1)
		UE_LOG(LogTemp, Warning, TEXT("CORE: %s"), *Message);
#endif
}

// Called every frame
void ACPathCore::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	while (!OutputQueue.IsEmpty())
	{
		std::pair<FCPathResult*, PathResultDelegate> Result;
		OutputQueue.Dequeue(Result);

		if (Result.second.IsBound())
		{
			//auto UPtr = std::unique_ptr<FCPathResult>(Result.first);
			Result.second.Execute(*Result.first);
		}
		else
		{
			PrintCoreMessage(FString("Tick - DELEGATE wasn't bound"));
		}
		delete Result.first;
	}
}

ACPathCore::~ACPathCore()
{
	PrintCoreMessage(FString("Destructor"));
	StopAndDeleteThreads();
}


FCPathfindingThread* ACPathCore::CreateThread(int ThreadIndex)
{
	FCPathfindingThread* FRunnableInstance = new FCPathfindingThread(this, ThreadIndex);
	
	return FRunnableInstance;
}

inline void ACPathCore::AssignAsyncRequest(FCPathRequest& Request)
{
	int LeastBusyThread = 0;
	int LeastTaskCount = MAX_int32;

	for (int i = 0; i < Threads.size(); i++)
	{
		checkf(Threads[i], TEXT("CPATH - CPathCore AssignAsyncRequest:::Thread was not valid!"));

		if (Threads[i]->IsThreadValid())
		{
			int TaskCount = Threads[i]->GetTaskCount();
			if (TaskCount < LeastTaskCount)
			{
				LeastTaskCount = TaskCount;
				LeastBusyThread = i;
				if (TaskCount == 0)
					break;
			}
		}
		else
		{
			delete Threads[i];
			Threads[i] = CreateThread(i);
			checkf(Threads[i]->IsThreadValid(), TEXT("CPATH - CPathCore AssignAsyncRequest:::Thread died unexpectedly and a new one couldn't be created. (this has never triggered for me)"));
			LeastBusyThread = i;
			LeastTaskCount = 0;
			break;
		}
	}
	Threads[LeastBusyThread]->AssignTask(Request);
}


