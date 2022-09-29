// Copyright Dominik Trautman. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "CPathNode.h"
#include "Core/Public/HAL/Runnable.h"
#include "Core/Public/HAL/RunnableThread.h"
#include "Kismet/BlueprintAsyncActionBase.h"
#include <atomic>
#include "CPathAsyncFindPath.generated.h"


/**
 Accessing this class through ANY MEANS other than delegates is UNSAFE.
 */


DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FResponseDelegate, const TArray<FCPathNode>&, Path, bool, TestBool);


UCLASS()
class CPATHFINDING_API UCPathAsyncFindPath : public UBlueprintAsyncActionBase
{
	GENERATED_BODY()

public:
	UPROPERTY(BlueprintAssignable)
		FResponseDelegate Success;

	UPROPERTY(BlueprintAssignable)
		FResponseDelegate Failure;

	
	// On success, returns a path from Start to End location. Both start and end must be inside the given Volume.
	// If start or end is unreachable (or time limit was exceeded) returns nothing.
	// SmoothingPasses - During a smoothing pass, every other node is potentially removed, as long as there is an empty space to the next one.
	// With SmoothingPasses=0, the path will be very jagged since the graph is Discrete.
	// With SmoothingPasses > 2 there is a potential loss of data, especially if a custom Cost function is used.
	UFUNCTION(BlueprintCallable, Category=CPath, meta = (BlueprintInternalUseOnly = "true"))
		static UCPathAsyncFindPath* FindPathAsync(class ACPathVolume* Volume, FVector StartLocation, FVector EndLocation, int SmoothingPasses=2, float TimeLimit=0.2f);

	virtual void Activate() override;
	virtual void BeginDestroy() override;
	class ACPathVolume* VolumeRef;

	FVector PathStart, PathEnd;
	uint32 Smoothing;
	float SearchTimeLimit;

	// 1 = finished Success
	// 0 = finished failed
	std::atomic_int ThreadResponse = -1;
	FTimerHandle CheckThreadTimerHandle;
	void CheckThreadStatus();

	TArray<FCPathNode> UserPath;
	TArray<CPathAStarNode> RawPathNodes;

private:

	
	class FCPathRunnableFindPath* RunnableFindPath = nullptr;
	FRunnableThread* CurrentThread = nullptr;
	
};

class FCPathRunnableFindPath : public FRunnable
{
public:
	FCPathRunnableFindPath(class UCPathAsyncFindPath* AsyncNode);

	virtual bool Init();

	virtual uint32 Run();

	virtual void Stop();

	virtual void Exit();

	//bool StopThread = false;

	class CPathAStar* AStar = nullptr;
private:
	bool bIncreasedPathfRunning = false;

	class UCPathAsyncFindPath* AsyncActionRef = nullptr;
};
