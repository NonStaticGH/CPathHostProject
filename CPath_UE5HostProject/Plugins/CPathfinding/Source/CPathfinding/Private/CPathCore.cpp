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

// Called when the game starts or when spawned
void ACPathCore::BeginPlay()
{
	Super::BeginPlay();
	ExpectedThreadCount = FPlatformMisc::NumberOfCores() - 1;

	for (int i = 0; i < ExpectedThreadCount; i++)
	{
		Threads.push_back(CreateThread(i));
	}
}

void ACPathCore::BeginDestroy()
{
	Super::BeginDestroy();
	StopAndDeleteThreads();
	Instance = nullptr;
}

void ACPathCore::StopAndDeleteThreads()
{
	if (Threads.size() > 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("CORE: DeletingThreads"));
	}
	while (Threads.size() > 0)
	{
		auto Thread = Threads.back();
		Thread->EnsureCompletion();
		delete Thread;
		Threads.pop_back();
	}
}

void ACPathCore::FindPathAsync(UObject* CallingObject, const FName& InFunctionName, ACPathVolume* VolumeRef, FVector Start, FVector End, uint32 SmoothingPasses, int32 UserData, float TimeLimit, bool RequestRawPath, bool RequestUserPath)
{
	FCPathRequest Request;
	Request.OnPathFound.BindUFunction(CallingObject, InFunctionName);
	Request.VolumeRef = VolumeRef;
	Request.Start = Start;
	Request.End = End;
	Request.SmoothingPasses = SmoothingPasses;
	Request.UserData = UserData;
	Request.TimeLimit = TimeLimit;
	Request.RequestRawPath = RequestRawPath;
	Request.RequestUserPath = RequestUserPath;

	AssignAsyncRequest(Request);
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
#if LOG_PATHFINDERS
			UE_LOG(LogTemp, Warning, TEXT("ACPathCore::Tick - DELEGATE wasn't bound"));
#endif
		}
		delete Result.first;
	}
}

ACPathCore::~ACPathCore()
{
	UE_LOG(LogTemp, Warning, TEXT("CORE: Destructor"));
	StopAndDeleteThreads();
}

ACPathCore* ACPathCore::GetInstance(UWorld* World)
{
	if (!IsValid(Instance))
	{
		UE_LOG(LogTemp, Warning, TEXT("CORE: Spawning"));
		Instance = World->SpawnActor<ACPathCore>();
	}
	return Instance;
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
			checkf(Threads[i]->IsThreadValid(), TEXT("CPATH - CPathCore AssignAsyncRequest:::Couldn't create a new thread!"));
			LeastBusyThread = i;
			LeastTaskCount = 0;
			break;
		}
	}
	Threads[LeastBusyThread]->AssignTask(Request);
}


