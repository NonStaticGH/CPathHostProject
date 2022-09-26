// Copyright Dominik Trautman. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include <CPathNode.h>
#include <vector>
#include <memory>


/**
 * 
 */

class ACPathVolume;

class CPathAStar
{
public:
	CPathAStar();
	~CPathAStar();

	// Can be called from main thread, but can freeze the game if you increase TimeLimit.
	CPathAStarNode* FindPath(ACPathVolume* VolumeRef, FVector Start, FVector End, uint32 SmoothingPasses =1, float TimeLimit =1.f/200.f, TArray<CPathAStarNode>* RawNodes = nullptr);

	void DrawPath(const TArray<FCPathNode>& Path) const;

	// Set this to true to interrupt pathfinding. FindPath returns an empty array.
	bool bStop = false;

	// Removes nodes in (nearly)straight sections, transforms to Blueprint exposed struct, optionally reverses it so that the path is from start to end and returns raw nodes.
	void TransformToUserPath(CPathAStarNode* PathEnd, TArray<FCPathNode>& UserPath, bool bReverse = true);

protected:
	ACPathVolume* Volume;
	FVector TargetLocation;


	float SearchTimeLimit = 1.f / 200.f;
	

	inline float EucDistance(CPathAStarNode& Node, FVector TargetWorldLocation) const;

	inline void CalcFitness(CPathAStarNode& Node);

	// Sweeps from Start to End using the tracing shape from volume. Returns true if no obstacles
	inline bool CanSkip(FVector Start, FVector End);

	// Iterates over the path from end to start, removing every other node if CanSkip returns true
	void SmoothenPath(CPathAStarNode* PathEnd);

	// Used in removing nodes that lay on the same line. The biger the number, the more nodes will be removed, but the path potentially loses data.
	float LineAngleToleranceDegrees = 3;

	// Nodes that were consumed from priority queue
	// This is emptied whenever FindPath is called
	std::vector<std::unique_ptr<CPathAStarNode>> ProcessedNodes;
	

	//std::priority_queue<CPathAStarNode, std::vector<CPathAStarNode>, std::greater<CPathAStarNode>> pq;
};
