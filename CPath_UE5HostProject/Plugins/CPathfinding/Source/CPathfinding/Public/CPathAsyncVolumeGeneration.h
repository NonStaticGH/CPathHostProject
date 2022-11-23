// Copyright Dominik Trautman. Published in 2022. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Core/Public/HAL/Runnable.h"
#include "Core/Public/HAL/RunnableThread.h"

class ACPathVolume;
class CPathOctree;




class CPATHFINDING_API FCPathAsyncVolumeGenerator : public FRunnable
{


public:
	// Geneated trees in range Start(inclusive) - End(not inclusive). If Obstacles = true, it takes from Volume->TreesToRegenerate, if not, it takes from Volume->Octrees (default)
	FCPathAsyncVolumeGenerator(ACPathVolume* Volume, uint32 StartIndex, uint32 EndIndex, uint8 ThreadID, FString ThreadName, bool Obstacles = false);

	// Not used for now
	FCPathAsyncVolumeGenerator(ACPathVolume* Volume);

	~FCPathAsyncVolumeGenerator();

	virtual bool Init();

	virtual uint32 Run();

	virtual void Stop();

	virtual void Exit();

	// The main generating function, generated/regenerates the whole octree at given index
	void RefreshTree(uint32 OuterIndex);

	bool bStop = false;
	bool bObstacles = false;

	FRunnableThread* ThreadRef = nullptr;

	uint8 GenThreadID;

	FString Name = "";

	uint32 OctreeCountAtDepth[4] = { 0, 0, 0, 0 };


protected:

	ACPathVolume* VolumeRef;

	uint32 FirstIndex = 0;
	uint32 LastIndex = 0;

	bool bIncreasedGenRunning = false;

	// Gets called by RefreshTree. Returns true if ANY child is free
	bool RefreshTreeRec(CPathOctree* OctreeRef, uint32 Depth, FVector TreeLocation);


public:

};
