// Copyright Dominik Trautman. All Rights Reserved.

#include "CPathAsyncFindPath.h"
#include "CPathAStar.h"
#include "CPathVolume.h"
#include <thread>

UCPathAsyncFindPath* UCPathAsyncFindPath::FindPathAsync(ACPathVolume* Volume, FVector StartLocation, FVector EndLocation, int SmoothingPasses, float TimeLimit)
{
#if WITH_EDITOR
    checkf(IsValid(Volume), TEXT("CPATH - FindPathAsync:::Volume was invalid"));
#endif

    UCPathAsyncFindPath* Instance = NewObject<UCPathAsyncFindPath>();
    Instance->VolumeRef = Volume;
    Instance->RunnableFindPath = new FCPathRunnableFindPath(Instance);
    Instance->PathStart = StartLocation;
    Instance->PathEnd = EndLocation;
    Instance->Smoothing = SmoothingPasses;
    Instance->SearchTimeLimit = TimeLimit;
    Instance->RegisterWithGameInstance(Volume->GetGameInstance());
    
    return Instance;
}

void UCPathAsyncFindPath::Activate()
{
    if (!IsValid(VolumeRef))
    {
        Failure.Broadcast(UserPath, false);
        SetReadyToDestroy();
        RemoveFromRoot();
    }
    else
    {
        CurrentThread = FRunnableThread::Create(RunnableFindPath, TEXT("AStar Pathfinding Thread"));
        VolumeRef->GetWorld()->GetTimerManager().SetTimer(CheckThreadTimerHandle, this, &UCPathAsyncFindPath::CheckThreadStatus, 1.f / 30.f, true);
    }
}

void UCPathAsyncFindPath::BeginDestroy()
{


    Super::BeginDestroy();
    if (CurrentThread)
    {
        CurrentThread->Suspend(true);
        if (RunnableFindPath && RunnableFindPath->AStar)
        {
            RunnableFindPath->AStar->bStop = true;
        }
        CurrentThread->Suspend(false);
        CurrentThread->WaitForCompletion();
        CurrentThread->Kill();
    }

       
    delete RunnableFindPath;
}

void UCPathAsyncFindPath::CheckThreadStatus()
{
    if (ThreadResponse >= 0)
    {
        if (ThreadResponse == 1)
        {
            Success.Broadcast(UserPath, true);
        }
        else
            Failure.Broadcast(UserPath, false);

        if (IsValid(VolumeRef))
        {
            VolumeRef->GetWorld()->GetTimerManager().ClearTimer(CheckThreadTimerHandle);
        }
        
        SetReadyToDestroy();
        RemoveFromRoot();
    }    
}



FCPathRunnableFindPath::FCPathRunnableFindPath(UCPathAsyncFindPath* AsyncNode)
{
    AsyncActionRef = AsyncNode;
}

bool FCPathRunnableFindPath::Init()
{
    AStar = new CPathAStar();
    return true;
}

uint32 FCPathRunnableFindPath::Run()
{
    // Waiting for the volume to finish generating
    while(AsyncActionRef->VolumeRef->GeneratorsRunning.load() > 0 && !AStar->bStop)
        std::this_thread::sleep_for(std::chrono::milliseconds(25));          

    // Preventing further generation while we search for a path
    bIncreasedPathfRunning = true;
    AsyncActionRef->VolumeRef->PathfindersRunning++;
    
    auto FoundPath = AStar->FindPath(AsyncActionRef->VolumeRef, AsyncActionRef->PathStart, AsyncActionRef->PathEnd, AsyncActionRef->Smoothing, AsyncActionRef->SearchTimeLimit);
    if (FoundPath)
    {
        AStar->TransformToUserPath(FoundPath, AsyncActionRef->UserPath);        
        AsyncActionRef->ThreadResponse.store(1);
    }
    else
    {
        AsyncActionRef->ThreadResponse.store(0);
    }
       
    if (bIncreasedPathfRunning)
        AsyncActionRef->VolumeRef->PathfindersRunning--;
    bIncreasedPathfRunning = false;
    return 0;
}

void FCPathRunnableFindPath::Stop()
{
    // Preventing a potential deadlock if the process is killed without waiting
    if(bIncreasedPathfRunning)
        AsyncActionRef->VolumeRef->PathfindersRunning--;
    bIncreasedPathfRunning = false;

    delete AStar;
    AStar = nullptr;
    //UE_LOG(LogTemp, Warning, TEXT("pathfinder stopped"));
}

void FCPathRunnableFindPath::Exit()
{
    delete AStar;
    AStar = nullptr;
   // UE_LOG(LogTemp, Warning, TEXT("pathfinder exit"));
}
