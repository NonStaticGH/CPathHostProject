// Copyright Dominik Trautman. Published in 2022. All Rights Reserved.


#include "CPathAsyncVolumeGeneration.h"
#include "CPathVolume.h"
#include "Engine/World.h"
#include <thread>

FCPathAsyncVolumeGenerator::FCPathAsyncVolumeGenerator(ACPathVolume* Volume, uint32 StartIndex, uint32 EndIndex, uint8 ThreadID, FString ThreadName, bool Obstacles)
	:
	FCPathAsyncVolumeGenerator(Volume)
{
	bObstacles = Obstacles;
	FirstIndex = StartIndex;
	LastIndex = EndIndex;
	GenThreadID = ThreadID;
	Name = ThreadName;
}

// Sets default values
FCPathAsyncVolumeGenerator::FCPathAsyncVolumeGenerator(ACPathVolume* Volume)
{
	VolumeRef = Volume;

}

FCPathAsyncVolumeGenerator::~FCPathAsyncVolumeGenerator()
{
	bStop = true;
	if (ThreadRef)
		ThreadRef->Kill(true);
}

bool FCPathAsyncVolumeGenerator::Init()
{
	return true;
}

uint32 FCPathAsyncVolumeGenerator::Run()
{
	bIncreasedGenRunning = true;
	VolumeRef->GeneratorsRunning++;

	// Waiting for pathfinders to finish.
	// Generators have priority over pathfinders, so we block further pathfinders from starting by incrementing GeneratorsRunning first	
	while (VolumeRef->PathfindersRunning.load() > 0 && !bStop)
		std::this_thread::sleep_for(std::chrono::milliseconds(25));

#ifdef LOG_GENERATORS
	auto GenerationStart = TIMENOW;
#endif

	if (LastIndex > 0)
	{
		if (bObstacles)
		{
			auto StartIter = VolumeRef->TreesToRegenerate.begin();
			for (uint32 i = 0; i < FirstIndex; i++)
				StartIter++;

			auto EndIter = StartIter;
			for (uint32 i = FirstIndex; i < LastIndex; i++)
				EndIter++;

			for (auto Iter = StartIter; Iter != EndIter && !bStop; Iter++)
			{
				RefreshTree(*Iter);
			}
		}
		else
		{
			for (uint32 OuterIndex = FirstIndex; OuterIndex < LastIndex && !bStop; OuterIndex++)
			{
				RefreshTree(OuterIndex);
			}
		}
	}

#ifdef LOG_GENERATORS
	auto GenerationTime = TIMEDIFF(GenerationStart, TIMENOW);

	int NodeCount = 0;
	for (int i = 0; i <= VolumeRef->OctreeDepth; i++)
	{
		NodeCount += OctreeCountAtDepth[i];
	}

	UE_LOG(LogTemp, Warning, TEXT("%s generated %d nodes in %lfms"), *Name, NodeCount, GenerationTime);
#endif

	if (bIncreasedGenRunning)
		VolumeRef->GeneratorsRunning--;
	bIncreasedGenRunning = false;
	return 0;
}

void FCPathAsyncVolumeGenerator::Stop()
{

	// Preventing a potential deadlock if the process is killed without waiting
	if (bIncreasedGenRunning)
		VolumeRef->GeneratorsRunning--;

	bIncreasedGenRunning = false;
}

void FCPathAsyncVolumeGenerator::Exit()
{

}

void FCPathAsyncVolumeGenerator::RefreshTree(uint32 OuterIndex)
{
	CPathOctree* OctreeRef = &VolumeRef->Octrees[OuterIndex];
	if (!OctreeRef)
	{
		return;
	}
	RefreshTreeRec(OctreeRef, 0, VolumeRef->WorldLocationFromTreeID(OuterIndex));
}

bool FCPathAsyncVolumeGenerator::RefreshTreeRec(CPathOctree* OctreeRef, uint32 Depth, FVector TreeLocation)
{

	bool IsFree = VolumeRef->RecheckOctreeAtDepth(OctreeRef, TreeLocation, Depth);

	OctreeCountAtDepth[Depth]++;

	if (IsFree)
	{
		delete[] OctreeRef->Children;
		OctreeRef->Children = nullptr;
		return true;
	}
	else if (++Depth <= (uint32)VolumeRef->OctreeDepth)
	{
		float HalfSize = VolumeRef->GetVoxelSizeByDepth(Depth) / 2.f;

		if (!OctreeRef->Children)
			OctreeRef->Children = new CPathOctree[8];
		uint8 FreeChildren = 0;
		// Checking children
		for (uint32 ChildIndex = 0; ChildIndex < 8; ChildIndex++)
		{
			FVector Location = TreeLocation + VolumeRef->LookupTable_ChildPositionOffsetMaskByIndex[ChildIndex] * HalfSize;
			FreeChildren += RefreshTreeRec(&OctreeRef->Children[ChildIndex], Depth, Location);
		}

		if (FreeChildren)
		{
			return true;
		}
		else
		{
			delete[] OctreeRef->Children;
			OctreeRef->Children = nullptr;
			return false;
		}

	}
	return false;
}


