// Copyright Dominik Trautman. Published in 2022. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 *
 */



 // The Octree representation
class CPATHFINDING_API CPathOctree
{
public:
	CPathOctree();


	CPathOctree* Children = nullptr;

	uint32 Data = 0;


	inline void SetIsFree(bool IsFree)
	{
		Data &= 0xFFFFFFFE;
		Data |= (uint32)IsFree;
	}

	inline bool GetIsFree() const
	{
		return Data << 31;
	}

	~CPathOctree()
	{
		delete[] Children;
	};
};

// Class used to remember data needed to draw a debug voxel 
class CPATHFINDING_API CPathVoxelDrawData
{

public:

	CPathVoxelDrawData()
	{}

	CPathVoxelDrawData(FVector WorldLocation, float VoxelExtent, bool IsFree)
		:
		Location(WorldLocation),
		Extent(VoxelExtent),
		Free(IsFree)
	{}

	FVector Location;
	float Extent;
	bool Free = false;

};

