// Copyright Dominik Trautman. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "CPathNode.generated.h"

/**
 * 
 */

// Internal class used while generating path
class CPathAStarNode
{
public:
	CPathAStarNode();
	CPathAStarNode(uint32 ID)
		:
		TreeID(ID)
	{

	}

	uint32 TreeID = 0xFFFFFFFF;

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
struct FCPathNode
{
	GENERATED_BODY()

	FCPathNode(){}
	FCPathNode(FVector Location)
		:
		WorldLocation(Location)
	{}


    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category=CPath)
        FVector WorldLocation;

	// Normalized vector pointing to next node. ZeroVector on last node.
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = CPath)
		FVector Normal = FVector(0, 0, 0);


};