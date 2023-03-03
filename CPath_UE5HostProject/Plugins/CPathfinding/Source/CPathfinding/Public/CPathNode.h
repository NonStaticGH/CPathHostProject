// Copyright Dominik Trautman. Published in 2022. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "CPathDefines.h"
#include "CPathNode.generated.h"

/**
 *
 */


 // Internal class used while generating path
class CPATHFINDING_API CPathAStarNode
{
public:
	CPathAStarNode();
	CPathAStarNode(uint32 ID)
		:
		TreeID(ID)
	{}
	CPathAStarNode(uint32 ID, uint32 Data)
		:
		TreeID(ID),
		TreeUserData(Data)
	{}

	uint32 TreeID = 0xFFFFFFFF;

	// Data from Octree that you may modify by overriding `RecheckOctreeAtDepth`
	// and access from `CalcFitness`
	uint32 TreeUserData = 0;

	// We want to find a node with minimum fitness, this way distance doesnt have to be inverted
	float FitnessResult = 9999999999.f;
	float DistanceSoFar = 0;

	// This is NOT always valid. 
	CPathAStarNode* PreviousNode = nullptr;

	FVector WorldLocation;

	// ------ Operators for containers ----------------------------------------
	bool operator <(const CPathAStarNode& Rhs) const
	{
		return FitnessResult < Rhs.FitnessResult;
	}

	bool operator >(const CPathAStarNode& Rhs) const
	{
		return FitnessResult > Rhs.FitnessResult;
	}

	bool operator ==(const CPathAStarNode& Rhs) const
	{
		return TreeID == Rhs.TreeID;
	}

	struct Hash
	{
		size_t operator()(const CPathAStarNode& Node) const
		{
			return Node.TreeID;
		}
	};

	~CPathAStarNode();
};


USTRUCT(BlueprintType)
struct CPATHFINDING_API FCPathNode
{
	GENERATED_BODY()

		FCPathNode() {}
	FCPathNode(FVector Location)
		:
		WorldLocation(Location)
	{}


	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = CPath)
		FVector WorldLocation;

	// Normalized vector pointing to next node. ZeroVector on last node.
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = CPath)
		FVector Normal = FVector(0, 0, 0);


};

// Data returned by FindPath call
USTRUCT()
struct CPATHFINDING_API FCPathResult
{
	GENERATED_BODY()

	friend class FCPathfindingThread;

	ECPathfindingFailReason FailReason = Unknown;
	float SearchDuration = 0;

	// THE final usable path.
	TArray<FCPathNode> UserPath;
	float UserPathLength = 0;

	// The raw path with Octree data before any preprocessing. By default this is empty. 
	// To get this data, set RequestRawPath to true in the FindPath call
	TArray<CPathAStarNode> RawPathNodes;
	float RawPathLength = 0;
};


DECLARE_DELEGATE_OneParam(PathResultDelegate, FCPathResult&);


// Struct used to save parameters for a FindPath call
struct CPATHFINDING_API FCPathRequest
{
	PathResultDelegate OnPathFound;
	class ACPathVolume* VolumeRef;
	FVector Start;
	FVector End;
	uint32 SmoothingPasses;
	int32 UserData;
	float TimeLimit;
	FCPathResult* Result;
	bool RequestRawPath;
	bool RequestUserPath;
};


