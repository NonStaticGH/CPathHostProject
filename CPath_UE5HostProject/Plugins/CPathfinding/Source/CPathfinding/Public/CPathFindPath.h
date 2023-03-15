// Copyright Dominik Trautman. Published in 2022. All Rights Reserved.

#pragma once


#include "CoreMinimal.h"
#include "CPathNode.h"
#include "Core/Public/HAL/Runnable.h"
#include "Core/Public/HAL/RunnableThread.h"
#include "Kismet/BlueprintAsyncActionBase.h"
#include <atomic>
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

	~CPathAStar();

	static CPathAStar* GetInstance(UWorld* World);
	static void DeleteInstance(UWorld* World);


	// Can be called from main thread, but can freeze the game if you increase TimeLimit.
	ECPathfindingFailReason FindPath(ACPathVolume* VolumeRef, FCPathResult* Result, FVector Start, FVector End, uint32 SmoothingPasses = 2, int32 UserData = 0, float TimeLimit = 0.15f, bool RequestRawPath = false, bool RequestUserPath = true);

	// Set this to true to interrupt pathfinding. FindPath returns an empty array.
	// This is set to false at the beginning of each FindPath call!
	std::atomic_bool bStop = false;


		// Used in removing nodes that lay on the same line. The biger the number, the more nodes will be removed, but the path potentially loses data.
	float LineAngleToleranceDegrees = 3;

protected:

	inline float EucDistance(CPathAStarNode& Node, FVector TargetWorldLocation) const;

	inline void CalcFitness(CPathAStarNode& Node);


private:
	FVector TargetLocation; 
	ACPathVolume* CurrentVolumeRef;

	// Sweeps from Start to End using the tracing shape from volume. Returns true if no obstacles
	inline bool CanSkip(FVector Start, FVector End);

	// Iterates over the path from end to start, removing every other node if CanSkip returns true
	inline void SmoothenPath(CPathAStarNode* PathEndNode);

	// Removes nodes in (nearly)straight sections, transforms to Blueprint exposed struct, optionally reverses it so that the path is from start to end and returns raw nodes.
	void TransformToUserPath(CPathAStarNode* PathEndNode, TArray<FCPathNode>& UserPath, bool bReverse = true);

	friend class UCPathAsyncFindPath;
	friend class FCPathRunnableFindPath;

	static CPathAStar* GlobalInstance;

};


DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FResponseDelegate, const TArray<FCPathNode>&, Path, TEnumAsByte<ECPathfindingFailReason>, FailReason);

/**
 This is the class that creates the FindPathAsync node in Blueprints
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

	FCPathRequest Request;

	// On success, returns a path from Start to End location. Both start and end must be inside the given Volume.
	// If start or end is unreachable (or time limit was exceeded), returns nothing.
	// SmoothingPasses - During a smoothing pass, every other node is potentially removed, as long as there is an empty space to the next one.
	// With SmoothingPasses=0, the path will be very jagged since the graph is Discrete.
	// With SmoothingPasses > 2 there is a potential loss of data, especially if the CalcFitness method has been overriden
	UFUNCTION(BlueprintCallable, Category = CPath, meta = (BlueprintInternalUseOnly = "true"))
		static UCPathAsyncFindPath* FindPathAsync(class ACPathVolume* Volume, FVector StartLocation, FVector EndLocation, int SmoothingPasses = 2, int32 UserData = 0, float TimeLimit = 0.2f);

	UFUNCTION()
		void OnPathFound(FCPathResult& PathResult);

	virtual void Activate() override;
	virtual void BeginDestroy() override;
};

