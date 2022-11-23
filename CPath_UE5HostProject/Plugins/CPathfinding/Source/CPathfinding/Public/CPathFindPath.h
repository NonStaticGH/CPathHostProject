// Copyright Dominik Trautman. Published in 2022. All Rights Reserved.

#pragma once


#include "CoreMinimal.h"
#include "CPathNode.h"
#include "Core/Public/HAL/Runnable.h"
#include "Core/Public/HAL/RunnableThread.h"
#include "Kismet/BlueprintAsyncActionBase.h"
#include <atomic>
#include <CPathNode.h>
#include <vector>
#include <memory>
#include <CPathDefines.h>
#include "CPathFindPath.generated.h"


class ACPathVolume;

/**
The class for pathfinding, used in UCPathAsyncFindPath. Can also be used on game thread to get the path instantly.
*/
class CPathAStar
{
public:
	CPathAStar();
	CPathAStar(ACPathVolume* VolumeRef, FVector Start, FVector End, uint32 SmoothingPasses = 1, int32 UserData = 0, float TimeLimit = 1.f / 200.f);

	~CPathAStar();

	// Can be called from main thread, but can freeze the game if you increase TimeLimit.
	CPathAStarNode* FindPath(ACPathVolume* VolumeRef, FVector Start, FVector End, uint32 SmoothingPasses = 1, int32 UserData = 0, float TimeLimit = 1.f / 200.f, TArray<CPathAStarNode>* RawNodes = nullptr);

	// Uses cached data in this class, only working if all the arguments were passed via constructor.
	// Returns true on success, result is in UserPath
	bool FindPath();

	// Set this to true to interrupt pathfinding. FindPath returns an empty array.
	bool bStop = false;

	// Removes nodes in (nearly)straight sections, transforms to Blueprint exposed struct, optionally reverses it so that the path is from start to end and returns raw nodes.
	void TransformToUserPath(CPathAStarNode* PathEndNode, TArray<FCPathNode>& UserPath, bool bReverse = true);

	// This is used by FindPath if it failed null.
	ECPathfindingFailReason FailReason = None;

	// The final usable path
	TArray<FCPathNode> UserPath;

	// The path before preprocessing
	TArray<CPathAStarNode> RawPathNodes;

	// Cached FindPath parameters
	FVector PathStart, PathEnd;
	uint32 Smoothing = 2; 
	int32 UsrData = 0;
	float SearchTimeLimit = 1.f / 200.f;

	// Used in removing nodes that lay on the same line. The biger the number, the more nodes will be removed, but the path potentially loses data.
	float LineAngleToleranceDegrees = 3;

protected:

	ACPathVolume* Volume;

	inline float EucDistance(CPathAStarNode& Node, FVector TargetWorldLocation) const;

	inline void CalcFitness(CPathAStarNode& Node);

	// Nodes that were consumed from priority queue
	// This is emptied whenever FindPath is called
	std::vector<std::unique_ptr<CPathAStarNode>> ProcessedNodes;

private:
	FVector TargetLocation;
	// Sweeps from Start to End using the tracing shape from volume. Returns true if no obstacles
	inline bool CanSkip(FVector Start, FVector End);

	// Iterates over the path from end to start, removing every other node if CanSkip returns true
	inline void SmoothenPath(CPathAStarNode* PathEndNode);

	friend class UCPathAsyncFindPath;
	friend class FCPathRunnableFindPath;

};


DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FResponseDelegate, const TArray<FCPathNode>&, Path, TEnumAsByte<ECPathfindingFailReason>, FailReason);

/**
 Accessing this class through ANY MEANS other than delegates is UNSAFE.
 */
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
	UFUNCTION(BlueprintCallable, Category = CPath, meta = (BlueprintInternalUseOnly = "true"))
		static UCPathAsyncFindPath* FindPathAsync(class ACPathVolume* Volume, FVector StartLocation, FVector EndLocation, int SmoothingPasses = 2, int32 UserData = 0, float TimeLimit = 0.2f);

	virtual void Activate() override;
	virtual void BeginDestroy() override;

	// 1 = finished Success
	// 0 = finished failed
	std::atomic_int ThreadResponse = -1;
	FTimerHandle CheckThreadTimerHandle;
	void CheckThreadStatus();

private:

	// Thread objects
	CPathAStar* AStar = nullptr;
	class FCPathRunnableFindPath* RunnableFindPath = nullptr;
	FRunnableThread* CurrentThread = nullptr;

	friend class FCPathRunnableFindPath;
};

// The class used to perform pathfinding on its own thread
class CPATHFINDING_API FCPathRunnableFindPath : public FRunnable
{
public:
	FCPathRunnableFindPath(class UCPathAsyncFindPath* AsyncNode);

	virtual bool Init();

	virtual uint32 Run();

	virtual void Stop();

	virtual void Exit();

	//bool StopThread = false;

private:
	float SleepCounter = 0;

	bool bIncreasedPathfRunning = false;

	class UCPathAsyncFindPath* AsyncActionRef = nullptr;
};
