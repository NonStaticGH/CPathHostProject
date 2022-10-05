// Copyright Dominik Trautman. All Rights Reserved.


#include "CPathAsyncVolumeGeneration.h"
#include "CPathVolume.h"
#include "Engine/World.h"
#include <thread>

FCPathAsyncVolumeGenerator::FCPathAsyncVolumeGenerator(ACPathVolume* Volume, uint32 StartIndex, uint32 EndIndex, bool Obstacles)
	:
	FCPathAsyncVolumeGenerator(Volume)
{
	bObstacles = Obstacles;
	FirstIndex = StartIndex;
	LastIndex = EndIndex;
	//UE_LOG(LogTemp, Warning, TEXT("CONSTRUCTOR, %d %d"), LastIndex, FirstIndex);
}

// Sets default values
FCPathAsyncVolumeGenerator::FCPathAsyncVolumeGenerator(ACPathVolume* Volume)
{
	VolumeRef = Volume;

}

FCPathAsyncVolumeGenerator::~FCPathAsyncVolumeGenerator()
{
	//UE_LOG(LogTemp, Warning, TEXT("Thread %d - Destroying"), LastIndex);
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
	//std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	while (VolumeRef->PathfindersRunning.load() > 0 && !bStop)
		std::this_thread::sleep_for(std::chrono::milliseconds(25));

	auto GenerationStart = TIMENOW;


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

	auto b = TIMEDIFF(GenerationStart, TIMENOW);

	//UE_LOG(LogTemp, Warning, TEXT("Thread %d - bStop= %d"), LastIndex, bStop);

	
	if (bIncreasedGenRunning)
		VolumeRef->GeneratorsRunning--;
	bIncreasedGenRunning = false;
	return 0;
}

void FCPathAsyncVolumeGenerator::Stop()
{
	
	// Preventing a potential deadlock if the process is killed without waiting
	if(bIncreasedGenRunning)
		VolumeRef->GeneratorsRunning--;

	bIncreasedGenRunning = false;
	//UE_LOG(LogTemp, Warning, TEXT("Generator %d stopped"), LastIndex);
}

void FCPathAsyncVolumeGenerator::Exit()
{
	//UE_LOG(LogTemp, Warning, TEXT("Generator %d Exit"), LastIndex);
	// Setting thread ref to null so that it can be collected by CleanFinishedThreads in volume
	ThreadRef = nullptr;
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

	bool IsFree = VolumeRef->CheckAndUpdateTree(OctreeRef, TreeLocation, Depth);

	VolumeRef->VoxelCountAtDepth[Depth]++;

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


