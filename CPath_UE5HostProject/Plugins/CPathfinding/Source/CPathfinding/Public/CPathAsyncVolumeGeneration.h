// Copyright Dominik Trautman. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"

class ACPathVolume;
class CPathOctree;

UENUM()
enum EAgentShape
{
	Capsule  =3, 
	Box      =2,
	Sphere   =0
};



class FCPathAsyncVolumeGenerator : public FRunnable
{

	
public:	
	// Geneated trees in range Start(inclusive) - End(not inclusive). If Obstacles = true, it takes from Volume->TreesToRegenerate, if not, it takes from Volume->Octrees (default)
	FCPathAsyncVolumeGenerator(ACPathVolume* Volume, uint32 StartIndex, uint32 EndIndex, bool Obstacles = false);

	// Not used for now
	FCPathAsyncVolumeGenerator(ACPathVolume* Volume);

	~FCPathAsyncVolumeGenerator();

	virtual bool Init();

	virtual uint32 Run();

	virtual void Stop();

	virtual void Exit();

	// Retraces the octree from TreeID downwards. Pass optional arguments to make it a bit faster. When OctreeRef is passed, TreeID is ignored
	void RefreshTree(uint32 OuterIndex);

	bool bStop = false;
	bool bObstacles = false;

	FRunnableThread* ThreadRef = nullptr;

protected:

	ACPathVolume* VolumeRef;

	uint32 FirstIndex = 0;
	uint32 LastIndex = 0;

	bool bIncreasedGenRunning = false;

	// Gets called by RefreshTree. Returns true if ANY child is free
	bool RefreshTreeRec(CPathOctree* OctreeRef, uint32 Depth, FVector TreeLocation);


public: 

};
