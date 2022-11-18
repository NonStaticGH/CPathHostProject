// Copyright Dominik Trautman. Published in 2022. All Rights Reserved.

#include "CPathVolume.h"

#include "DrawDebugHelpers.h"
#include "Components/BoxComponent.h"
#include <queue>
#include <deque>
#include <list>
#include <unordered_set>
#include "CPathDynamicObstacle.h"
#include "CPathNode.h"
#include "TimerManager.h"
#include "Engine/Selection.h"
#include "GenericPlatform/GenericPlatformAtomics.h"





ACPathVolume::ACPathVolume()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	VolumeBox = CreateDefaultSubobject<UBoxComponent>("VolumeBox");
	RootComponent = VolumeBox;
	VolumeBox->InitBoxExtent(FVector(VoxelSize));

#if WITH_EDITOR


#endif
	DepthsToDraw = { true, true, true, true };


	FVector Location = GetActorLocation() - VolumeBox->GetScaledBoxExtent() + VoxelSize;
	DrawDebugBox(GetWorld(), Location, FVector(VoxelSize), FColor::White, true);

}

void ACPathVolume::DebugDrawNeighbours(FVector WorldLocation)
{
	uint32 LeafID;
	if (FindLeafByWorldLocation(WorldLocation, LeafID))
	{
		DrawDebugBox(GetWorld(), WorldLocationFromTreeID(LeafID), FVector(GetVoxelSizeByDepth(ExtractDepth(LeafID)) / 2.f), FColor::Emerald, false, 5, 10, DebugBoxesThickness*1.3);
		auto Neighbours = FindNeighbourLeafs(LeafID, true);

		for (auto N : Neighbours)
		{
			DrawDebugBox(GetWorld(), WorldLocationFromTreeID(N), FVector(GetVoxelSizeByDepth(ExtractDepth(N)) / 2.f), FColor::Yellow, false, 5, 0U, DebugBoxesThickness*1.4);
		}
	}
}

bool ACPathVolume::DrawDebugVoxel(uint32 TreeID, bool DrawIfNotLeaf, float Duration, FColor Color, CPathVoxelDrawData* OutDrawData)
{

	uint32 Depth;
	float Thickness = DebugBoxesThickness;
	auto Tree = FindTreeByID(TreeID, Depth);
	if (Tree->Children && !DrawIfNotLeaf)
		return false;
	bool IsFree = Tree->GetIsFree();
	if (IsFree)
	{
		if (!DrawFree)
			return false;
	}
	else
	{
		if (!DrawOccupied)
			return false;
		if (Color == FColor::Green)
		{
			Color = FColor::Red;
			Thickness *= 1.5;
		}
	}


	bool Persistent = false;
	if (Duration < 0)
		Persistent = true;

	if (DepthsToDraw[Depth])
	{
		float Extent = GetVoxelSizeByDepth(ExtractDepth(TreeID)) / 2.f;
		FVector Location = WorldLocationFromTreeID(TreeID);
		DrawDebugBox(GetWorld(), Location, FVector(Extent), Color, Persistent, Duration, 0U, Thickness);
		if (OutDrawData)
		{
			OutDrawData->Extent = Extent;
			OutDrawData->Free = IsFree;
			OutDrawData->Location = Location;
		}

		return true;
	}

	return false;
}

void ACPathVolume::DrawDebugVoxel(const CPathVoxelDrawData& DrawData, float Duration) const
{
	float Thickness = DebugBoxesThickness;
	FColor Color = FColor::Green;
	if (!DrawData.Free)
	{
		Color = FColor::Red;
		Thickness *= 1.5;
	}

	bool Persistent = false;
	if (Duration < 0)
		Persistent = true;

	DrawDebugBox(GetWorld(), DrawData.Location, FVector(DrawData.Extent), Color, Persistent, Duration, 0U, Thickness);
}

void ACPathVolume::DrawDebugNodesAroundLocation(FVector WorldLocation, int VoxelLimit, float Duration)
{
	// We dont want to get new data while its generating
	if (GeneratorsRunning.load())
	{
		for (auto Data : PreviousDrawAroundLocationData)
		{
			DrawDebugVoxel(Data, Duration);
		}
		return;
	}
	PreviousDrawAroundLocationData.clear();

	uint32 OriginTreeID = 0xFFFFFFFF;
	CPathOctree* OriginTree = FindLeafByWorldLocation(WorldLocation, OriginTreeID, false);
	if (!OriginTree)
		return;

	std::list<uint32> IndexList;
	std::unordered_set<uint32> VisitedIndexes;

	CPathAStarNode StartNode(OriginTreeID);
	StartNode.FitnessResult = 0;


	// Ordered by neighbours, first come first served
	IndexList.push_back(StartNode.TreeID);



	while (!IndexList.empty() && VoxelLimit > 0)
	{
		uint32 CurrID = IndexList.front();
		IndexList.pop_front();
		CPathVoxelDrawData DrawData;
		if (DrawDebugVoxel(CurrID, true, Duration, FColor::Green, &DrawData))
		{
			VoxelLimit--;
			PreviousDrawAroundLocationData.push_back(DrawData);
		}


		std::vector<uint32> Neighbours = FindNeighbourLeafs(CurrID, !DrawOccupied);
		for (uint32 NewTreeID : Neighbours)
		{

			// We dont want to redraw nodes
			if (!VisitedIndexes.count(NewTreeID))
			{
				IndexList.push_back(NewTreeID);
				VisitedIndexes.insert(NewTreeID);
			}
		}
	}
}

void ACPathVolume::DrawDebugPath(const TArray<FCPathNode>& Path, float Duration, bool DrawPoints, FColor Color)
{
	bool Persistent = Duration < 0;
	for (int i = 0; i < Path.Num() - 1; i++)
	{
		DrawDebugLine(GetWorld(), Path[i].WorldLocation, Path[i + 1].WorldLocation, Color, Persistent, Duration, 0U, DebugPathThickness);
		if (DrawPoints)
			DrawDebugPoint(GetWorld(), Path[i].WorldLocation, 10, FColor::Cyan, Persistent, Duration);
	}
}


void ACPathVolume::BeginPlay()
{
	Super::BeginPlay();

	VolumeBox->SetCollisionResponseToChannel(TraceChannel, ECR_Ignore);

	if (GenerateOnBeginPlay)
		GenerateGraph();
}

bool ACPathVolume::GenerateGraph()
{
	GenerationStarted = true;
	PrintGenerationTime = true;

	UBoxComponent* tempBox = Cast<UBoxComponent>(GetRootComponent());
	tempBox->UpdateOverlaps();


	float Divider = VoxelSize * FMath::Pow(2.f, OctreeDepth);

	NodeCount[0] = FMath::CeilToInt(VolumeBox->GetScaledBoxExtent().X * 2.0 / Divider);
	NodeCount[1] = FMath::CeilToInt(VolumeBox->GetScaledBoxExtent().Y * 2.0 / Divider);
	NodeCount[2] = FMath::CeilToInt(VolumeBox->GetScaledBoxExtent().Z * 2.0 / Divider);

	checkf(OctreeDepth <= MAX_DEPTH && OctreeDepth >= 0, TEXT("CPATH - Graph Generation:::OctreeDepth must be within 0 and MAX_DEPTH"));
	//checkf(AgentShape == ECollisionShapeType::Capsule || AgentShape == ECollisionShapeType::Sphere || AgentShape == ECollisionShapeType::Box, TEXT("CPATH - Graph Generation:::Agent shape must be Capsule, Sphere or Box"));


	for (int i = 0; i <= OctreeDepth; i++)
	{

		LookupTable_VoxelSizeByDepth[i] = VoxelSize * FMath::Pow(2.f, OctreeDepth - i);
		TraceShapesByDepth.emplace_back();
		TraceShapesByDepth.back().push_back(FCollisionShape::MakeBox(FVector(GetVoxelSizeByDepth(i) / 2.f)));

		float CurrSize = GetVoxelSizeByDepth(i);
		if (AgentRadius * 2 > CurrSize || AgentHalfHeight * 2 > CurrSize)
		{
			switch (AgentShape)
			{
			case Capsule:
				TraceShapesByDepth.back().push_back(FCollisionShape::MakeCapsule(AgentRadius, AgentHalfHeight));
			case Box:
				TraceShapesByDepth.back().push_back(FCollisionShape::MakeBox(FVector(AgentRadius, AgentRadius, AgentHalfHeight)));
			case Sphere:
				TraceShapesByDepth.back().push_back(FCollisionShape::MakeSphere(AgentRadius));
			default:
				break;
			}
		}
	}

	StartPosition = GetActorLocation() - VolumeBox->GetScaledBoxExtent() + GetVoxelSizeByDepth(0) / 2;

	uint32 OuterNodeCount = NodeCount[0] * NodeCount[1] * NodeCount[2];
	checkf(OuterNodeCount < DEPTH_0_LIMIT, TEXT("CPATH - Graph Generation:::Depth 0 is too dense, increase OctreeDepth and/or voxel size, or decrease volume area."));
	Octrees = new CPathOctree[OuterNodeCount];

	// If we use all logical threads in the system, the rest of the game
	// will have no computing power to work with. From my small test sample
	// Using hyper threads barely increased performance so its not worth it
	/*if (FPlatformMisc::NumberOfCoresIncludingHyperthreads() > FPlatformMisc::NumberOfCores())
		ThreadCount = FPlatformMisc::NumberOfCores() + (FPlatformMisc::NumberOfCoresIncludingHyperthreads() - FPlatformMisc::NumberOfCores()) / 3;
	else
		ThreadCount = FPlatformMisc::NumberOfCores() - 1;*/
	if (MaxGenerationThreads <= 0)
		MaxGenerationThreads = FPlatformMisc::NumberOfCores() - 1;

	MaxGenerationThreads = FMath::Min(MaxGenerationThreads, 31);

	uint32 NodesPerThread = OuterNodeCount / MaxGenerationThreads;

	for (int i = 0; i < 64; i++)
	{
		ThreadIDs[1] = false;
	}


	for (int CurrentThread = 0; CurrentThread < MaxGenerationThreads; CurrentThread++)
	{
		uint32 LastIndex = NodesPerThread * (CurrentThread + 1);
		if (CurrentThread == MaxGenerationThreads - 1)
			LastIndex += OuterNodeCount % MaxGenerationThreads;

		int ThreadID = GetFreeThreadID();
		FString ThreadName = "CPathGenerator Initial, ID: ";
		ThreadName.AppendInt(ThreadID);
		GeneratorThreads.push_back(std::make_unique<FCPathAsyncVolumeGenerator>(this, NodesPerThread * CurrentThread, LastIndex, ThreadID, ThreadName));
		GeneratorThreads.back()->ThreadRef = FRunnableThread::Create(GeneratorThreads.back().get(), *ThreadName);
		if (GeneratorThreads.back()->ThreadRef)
		{
			ThreadIDs[ThreadID] = true;
		}
		else
		{
			GeneratorThreads.pop_back();
		}

	}
	OuterIndexesPerThread = 5 * (5 + OctreeDepth) * FMath::Pow(8.f, MAX_DEPTH - OctreeDepth);
	// Setting timer for dynamic generation and garbage collection
	GetWorld()->GetTimerManager().SetTimer(GenerationTimerHandle, this, &ACPathVolume::InitialGenerationUpdate, 1.f / 60.f, true);
	return true;
}

void ACPathVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void ACPathVolume::BeginDestroy()
{
	Super::BeginDestroy();

	GeneratorThreads.clear();
	delete[] Octrees;
}


inline FVector ACPathVolume::WorldLocationToLocalCoordsInt3(FVector WorldLocation) const
{
	FVector RelativePos = WorldLocation - StartPosition;
	RelativePos = RelativePos / GetVoxelSizeByDepth(0);
	return FVector(FMath::RoundToFloat(RelativePos.X),
		FMath::RoundToFloat(RelativePos.Y),
		FMath::RoundToFloat(RelativePos.Z));

}

inline int ACPathVolume::WorldLocationToIndex(FVector WorldLocation) const
{
	FVector XYZ = WorldLocationToLocalCoordsInt3(WorldLocation);
	return LocalCoordsInt3ToIndex(XYZ);
}

inline bool ACPathVolume::IsInBounds(FVector XYZ) const
{
	if (XYZ.X < 0 || XYZ.X >= NodeCount[0])
		return false;

	if (XYZ.Y < 0 || XYZ.Y >= NodeCount[1])
		return false;

	if (XYZ.Z < 0 || XYZ.Z >= NodeCount[2])
		return false;

	return true;
}

inline float ACPathVolume::LocalCoordsInt3ToIndex(FVector V) const
{
	return (V.X * (NodeCount[1] * NodeCount[2])) + (V.Y * NodeCount[2]) + V.Z;
}

inline float ACPathVolume::GetVoxelSizeByDepth(int Depth) const
{
#if WITH_EDITOR
	checkf(Depth <= OctreeDepth, TEXT("CPATH - Graph Generation:::DEPTH was higher than OctreeDepth"));
#endif

	return LookupTable_VoxelSizeByDepth[Depth];
}

inline uint32 ACPathVolume::CreateTreeID(uint32 Index, uint32 Depth) const
{
#if WITH_EDITOR
	checkf(Depth <= MAX_DEPTH, TEXT("CPATH - Graph Generation:::DEPTH can be up to MAX_DEPTH"));
#endif		
	Index |= Depth << DEPTH_0_BITS;

	return Index;
}

inline uint32 ACPathVolume::ExtractOuterIndex(uint32 TreeID) const
{
	return TreeID & DEPTH_0_MASK;
}

inline void ACPathVolume::ReplaceDepth(uint32& TreeID, uint32 NewDepth)
{
#if WITH_EDITOR
	checkf(NewDepth <= MAX_DEPTH, TEXT("CPATH - Graph Generation:::DEPTH can be up to MAX_DEPTH"));
#endif

	TreeID &= ~DEPTH_MASK;
	TreeID |= NewDepth << DEPTH_0_BITS;
}

inline uint32 ACPathVolume::ExtractDepth(uint32 TreeID) const
{
	return (TreeID & DEPTH_MASK) >> DEPTH_0_BITS;
}

inline uint32 ACPathVolume::ExtractChildIndex(uint32 TreeID, uint32 Depth) const
{
#if WITH_EDITOR
	checkf(Depth <= MAX_DEPTH && Depth > 0, TEXT("CPATH - Graph Generation:::DEPTH can be up to MAX_DEPTH"));
#endif
	uint32 DepthOffset = (Depth - 1) * 3 + DEPTH_0_BITS + 2;
	uint32 Mask = 0x00000007 << DepthOffset;

	return (TreeID & Mask) >> DepthOffset;
}

inline void ACPathVolume::AddChildIndex(uint32& TreeID, uint32 Depth, uint32 ChildIndex)
{
#if WITH_EDITOR
	checkf(Depth <= MAX_DEPTH && Depth > 0, TEXT("CPATH - Graph Generation:::DEPTH can be up to MAX_DEPTH"));
	checkf(ChildIndex < 8, TEXT("CPATH - Graph Generation:::Child Index can be up to 7"));
#endif

	ChildIndex <<= (Depth - 1) * 3 + DEPTH_0_BITS + 2;

	TreeID |= ChildIndex;
}

inline FVector ACPathVolume::WorldLocationFromTreeID(uint32 TreeID) const
{
	uint32 OuterIndex = ExtractOuterIndex(TreeID);
	uint32 Depth = ExtractDepth(TreeID);

	FVector CurrPosition = StartPosition + GetVoxelSizeByDepth(0) * LocalCoordsInt3FromOuterIndex(OuterIndex);

	for (uint32 CurrDepth = 1; CurrDepth <= Depth; CurrDepth++)
	{
		CurrPosition += GetVoxelSizeByDepth(CurrDepth) * 0.5f * LookupTable_ChildPositionOffsetMaskByIndex[ExtractChildIndex(TreeID, CurrDepth)];
	}

	return CurrPosition;
}

inline FVector ACPathVolume::LocalCoordsInt3FromOuterIndex(uint32 OuterIndex) const
{
	uint32 X = OuterIndex / (NodeCount[1] * NodeCount[2]);
	OuterIndex -= X * NodeCount[1] * NodeCount[2];
	return FVector(X, OuterIndex / NodeCount[2], OuterIndex % NodeCount[2]);
}

inline void ACPathVolume::ReplaceChildIndex(uint32& TreeID, uint32 Depth, uint32 ChildIndex)
{
#if WITH_EDITOR
	checkf(Depth <= MAX_DEPTH && Depth > 0, TEXT("CPATH - Graph Generation:::DEPTH can be up to MAX_DEPTH"));
	checkf(ChildIndex < 8, TEXT("CPATH - Graph Generation:::Child Index can be up to 7"));
#endif

	uint32 DepthOffset = (Depth - 1) * 3 + DEPTH_0_BITS + 2;

	// Clearing previous child index
	TreeID &= ~(0x00000007 << DepthOffset);

	ChildIndex <<= DepthOffset;
	TreeID |= ChildIndex;
}

inline void ACPathVolume::ReplaceChildIndexAndDepth(uint32& TreeID, uint32 Depth, uint32 ChildIndex)
{
#if WITH_EDITOR
	checkf(Depth <= MAX_DEPTH && Depth > 0, TEXT("CPATH - Graph Generation:::DEPTH can be up to MAX_DEPTH"));
	checkf(ChildIndex < 8, TEXT("CPATH - Graph Generation:::Child Index can be up to 7"));
#endif

	uint32 DepthOffset = (Depth - 1) * 3 + DEPTH_0_BITS + 2;

	// Clearing previous child index
	TreeID &= ~(0x00000007 << DepthOffset);

	ChildIndex <<= DepthOffset;
	TreeID |= ChildIndex;
	ReplaceDepth(TreeID, Depth);
}

inline void ACPathVolume::GetAllSubtrees(uint32 TreeID, std::vector<uint32>& Container)
{
	uint32 Depth = 0;
	CPathOctree* Tree = FindTreeByID(TreeID, Depth);
	GetAllSubtreesRec(TreeID, Tree, Container, Depth);
}

void ACPathVolume::GetAllSubtreesRec(uint32 TreeID, CPathOctree* Tree, std::vector<uint32>& Container, uint32 Depth)
{
	if (Tree->Children)
	{
		Depth++;
		for (uint32 ChildID = 0; ChildID < 8; ChildID++)
		{
			uint32 ID = TreeID;
			ReplaceChildIndexAndDepth(ID, Depth, ChildID);
			GetAllSubtreesRec(ID, &Tree->Children[ChildID], Container, Depth);
			Container.push_back(ID);
		}
	}
}

inline CPathOctree* ACPathVolume::FindTreeByID(uint32 TreeID)
{
	uint32 Depth = ExtractDepth(TreeID);
	CPathOctree* CurrTree = &Octrees[ExtractOuterIndex(TreeID)];


	for (uint32 CurrDepth = 1; CurrDepth <= Depth; CurrDepth++)
	{
		// Child not found, returning the deepest found parent
		if (!CurrTree->Children)
		{
			break;
		}

		CurrTree = &CurrTree->Children[ExtractChildIndex(TreeID, CurrDepth)];
	}
	return CurrTree;
}

CPathOctree* ACPathVolume::FindTreeByID(uint32 TreeID, uint32& DepthReached)
{
	uint32 Depth = ExtractDepth(TreeID);
	CPathOctree* CurrTree = &Octrees[ExtractOuterIndex(TreeID)];
	DepthReached = 0;

	for (uint32 CurrDepth = 1; CurrDepth <= Depth; CurrDepth++)
	{
		// Child not found, returning the deepest found parent
		if (!CurrTree->Children)
		{
			break;
		}

		CurrTree = &CurrTree->Children[ExtractChildIndex(TreeID, CurrDepth)];
		DepthReached = CurrDepth;
	}
	return CurrTree;
}

CPathOctree* ACPathVolume::FindTreeByWorldLocation(FVector WorldLocation, uint32& TreeID)
{
	FVector LocalCoords = WorldLocationToLocalCoordsInt3(WorldLocation);
	if (!IsInBounds(LocalCoords))
		return nullptr;

	TreeID = LocalCoordsInt3ToIndex(LocalCoords);
	return &Octrees[TreeID];
}

inline CPathOctree* ACPathVolume::FindLeafByWorldLocation(FVector WorldLocation, uint32& TreeID, bool MustBeFree)
{
	CPathOctree* CurrentTree = FindTreeByWorldLocation(WorldLocation, TreeID);
	CPathOctree* FoundLeaf = nullptr;
	if (CurrentTree)
	{
		FVector RelativeLocation = WorldLocation - GetOuterTreeWorldLocation(TreeID);

		if (CurrentTree->Children)
			FoundLeaf = FindLeafRecursive(RelativeLocation, TreeID, 0, CurrentTree);
		else
			FoundLeaf = CurrentTree;
	}

	// Checking if the found leaf is free, and if not returning its free neighbour
	if (MustBeFree && FoundLeaf && !FoundLeaf->GetIsFree())
	{
		/*CurrentTree = GetParentTree(TreeID);
		if (CurrentTree)
		{
			for (int i = 0; i < 8; i++)
			{
				if (CurrentTree->Children[i].GetIsFree())
					FoundLeaf = &CurrentTree->Children[i];
			}
		}

		if(!FoundLeaf->GetIsFree())*/
		FoundLeaf = nullptr;
	}

	return FoundLeaf;
}

CPathOctree* ACPathVolume::FindClosestFreeLeaf(FVector WorldLocation, uint32& TreeID, float SearchRange)
{
	uint32 OriginTreeID = 0xFFFFFFFF;
	CPathOctree* OriginTree = FindLeafByWorldLocation(WorldLocation, OriginTreeID, false);
	if (!OriginTree)
		return nullptr;

	if (OriginTree->GetIsFree())
	{
		TreeID = OriginTreeID;
		return OriginTree;
	}

	if (SearchRange <= 0)
	{
		uint32 Depth = FMath::Max((uint32)1, ExtractDepth(OriginTreeID) - 1);
		Depth = FMath::Min(Depth, (uint32)OctreeDepth);
		SearchRange = GetVoxelSizeByDepth(Depth);
	}

	// Nodes visited OR added to priority queue
	std::unordered_set<CPathAStarNode, CPathAStarNode::Hash> VisitedNodes;

	std::priority_queue<CPathAStarNode, std::deque<CPathAStarNode>, std::greater<CPathAStarNode>> Pq;
	std::priority_queue<CPathAStarNode, std::deque<CPathAStarNode>, std::greater<CPathAStarNode>> PqNeighbours;


	CPathAStarNode StartNode(OriginTreeID);
	StartNode.FitnessResult = 0;
	VisitedNodes.insert(StartNode);

	// STEP 1 - considering StartNode neighbours only, we dont check range cause neighbours take priority 
	// (its faster and solves almost all cases without needing to go to the other queue)

	std::vector<CPathAStarNode> StartNeighbours = FindFreeNeighbourLeafs(StartNode);
	for (CPathAStarNode NewNode : StartNeighbours)
	{		
		NewNode.WorldLocation = WorldLocationFromTreeID(NewNode.TreeID);

		// Fitness function here is distance from WorldLocation - The voxel extent, cause we want distance to the border of the voxel, not to it's center
		NewNode.FitnessResult = FVector::Distance(NewNode.WorldLocation, WorldLocation) - GetVoxelSizeByDepth(ExtractDepth(NewNode.TreeID)) / 2.f;

		VisitedNodes.insert(NewNode);
		PqNeighbours.push(NewNode);
	}

	while (PqNeighbours.size() > 0)
	{
		CPathAStarNode CurrentNode = PqNeighbours.top();
		PqNeighbours.pop();
		CPathOctree* Tree = FindTreeByID(CurrentNode.TreeID);
		if (Tree->GetIsFree())
		{
			if (!GetWorld()->LineTraceTestByChannel(WorldLocation, CurrentNode.WorldLocation, TraceChannel))
			{
				TreeID = CurrentNode.TreeID;
				//DrawDebugLine(GetWorld(), WorldLocation, CurrentNode.WorldLocation, FColor::Green, false, 1);
				return Tree;
			}
			//DrawDebugLine(GetWorld(), WorldLocation, CurrentNode.WorldLocation, FColor::Red, false, 1);
		}

		std::vector<CPathAStarNode> Neighbours = FindFreeNeighbourLeafs(CurrentNode);
		for (CPathAStarNode NewNode : Neighbours)
		{			
			// We dont want to revisit nodes
			if (!VisitedNodes.count(NewNode))
			{
				NewNode.WorldLocation = WorldLocationFromTreeID(NewNode.TreeID);
				// Fitness function here is distance from WorldLocation - The voxel extent, cause we want distance to the border of the voxel, not to it's center
				NewNode.FitnessResult = FVector::Distance(NewNode.WorldLocation, WorldLocation) - GetVoxelSizeByDepth(ExtractDepth(NewNode.TreeID)) / 2.f;

				VisitedNodes.insert(NewNode);
				// Search range condition
				if (NewNode.FitnessResult <= SearchRange)
				{
					Pq.push(NewNode);
				}
			}
		}
	}

	// STEP 2 - If no neighbour was free, we're looking for the closest node in range
	// We're putting neighbouring nodes as long as they are in SearchRange, until we find one that is free.
	// Priority queue ensures that we find the closest node to the given WorldLocation.
	while (Pq.size() > 0)
	{
		CPathAStarNode CurrentNode = Pq.top();
		Pq.pop();
		CPathOctree* Tree = FindTreeByID(CurrentNode.TreeID);
		if (Tree->GetIsFree())
		{
			if (!GetWorld()->LineTraceTestByChannel(WorldLocation, CurrentNode.WorldLocation, TraceChannel))
			{
				TreeID = CurrentNode.TreeID;
				//DrawDebugLine(GetWorld(), WorldLocation, CurrentNode.WorldLocation, FColor::Green, false, 1);
				return Tree;
			}
			//DrawDebugLine(GetWorld(), WorldLocation, CurrentNode.WorldLocation, FColor::Red, false, 1);
		}

		std::vector<CPathAStarNode> Neighbours = FindFreeNeighbourLeafs(CurrentNode);

		for (CPathAStarNode NewNode : Neighbours)
		{			
			// We dont want to revisit nodes
			if (!VisitedNodes.count(NewNode))
			{
				NewNode.WorldLocation = WorldLocationFromTreeID(NewNode.TreeID);
				// Fitness function here is distance from WorldLocation - The voxel extent, cause we want distance to the border of the voxel, not to it's center
				NewNode.FitnessResult = FVector::Distance(NewNode.WorldLocation, WorldLocation) - GetVoxelSizeByDepth(ExtractDepth(NewNode.TreeID)) / 2.f;

				VisitedNodes.insert(NewNode);
				// Search range condition
				if (NewNode.FitnessResult <= SearchRange)
				{
					Pq.push(NewNode);
				}
			}
		}
	}
	return nullptr;
}

CPathOctree* ACPathVolume::FindLeafRecursive(FVector RelativeLocation, uint32& TreeID, uint32 CurrentDepth, CPathOctree* CurrentTree)
{
	CurrentDepth += 1;

	// Determining which child the RelativeLocation is in
	uint32 ChildIndex = 0;
	if (RelativeLocation.X > 0.f)
		ChildIndex += 4;
	if (RelativeLocation.Z > 0.f)
		ChildIndex += 2;
	if (RelativeLocation.Y > 0.f)
		ChildIndex += 1;


	ReplaceChildIndex(TreeID, CurrentDepth, ChildIndex);

	CPathOctree* ChildTree = &CurrentTree->Children[ChildIndex];
	if (ChildTree->Children)
	{
		RelativeLocation = RelativeLocation - (LookupTable_ChildPositionOffsetMaskByIndex[ChildIndex] * (GetVoxelSizeByDepth(CurrentDepth) / 2.f));
		return FindLeafRecursive(RelativeLocation, TreeID, CurrentDepth, ChildTree);
	}
	else
	{
		ReplaceDepth(TreeID, CurrentDepth);
		return ChildTree;
	}

	return nullptr;
}

FVector ACPathVolume::GetOuterTreeWorldLocation(uint32 TreeID) const
{
	FVector LocalCoords = LocalCoordsInt3FromOuterIndex(ExtractOuterIndex(TreeID));
	LocalCoords *= GetVoxelSizeByDepth(0);
	return StartPosition + LocalCoords;
}

inline CPathOctree* ACPathVolume::GetParentTree(uint32 TreeId)
{
	uint32 Depth = ExtractDepth(TreeId);
	if (Depth)
	{
		ReplaceDepth(TreeId, Depth - 1);
		return FindTreeByID(TreeId, Depth);
	}
	return nullptr;
}



CPathOctree* ACPathVolume::FindNeighbourByID(uint32 TreeID, ENeighbourDirection Direction, uint32& NeighbourID)
{

	// Depth 0, getting neighbour from Octrees
	uint32 Depth = ExtractDepth(TreeID);
	if (Depth == 0)
	{
		int OuterIndex = ExtractOuterIndex(TreeID);
		FVector NeighbourLocalCoords = LocalCoordsInt3FromOuterIndex(OuterIndex) + LookupTable_NeighbourOffsetByDirection[Direction];

		if (!IsInBounds(NeighbourLocalCoords))
			return nullptr;

		NeighbourID = LocalCoordsInt3ToIndex(NeighbourLocalCoords);
		return &Octrees[NeighbourID];
	}

	uint8 ChildIndex = ExtractChildIndex(TreeID, Depth);
	int8 NeighbourChildIndex = LookupTable_NeighbourChildIndex[ChildIndex][Direction];

	// The neighbour a is child of the same octree
	if (NeighbourChildIndex >= 0)
	{
		NeighbourID = TreeID;
		ReplaceChildIndex(NeighbourID, Depth, NeighbourChildIndex);
		CPathOctree* Neighbour = FindTreeByID(NeighbourID, Depth);
		ReplaceDepth(NeighbourID, Depth);
		return Neighbour;
	}
	else
	{	// Getting the neighbour of parent Octree and then its correct child
		ReplaceDepth(TreeID, Depth - 1);
		CPathOctree* NeighbourOfParent = FindNeighbourByID(TreeID, Direction, NeighbourID);
		if (NeighbourOfParent)
		{
			if (NeighbourOfParent->Children)
			{
				// Look at the description of LookupTable_NeighbourChildIndex
				NeighbourChildIndex = -1 * NeighbourChildIndex - 1;
				ReplaceDepth(NeighbourID, Depth);
				ReplaceChildIndex(NeighbourID, Depth, NeighbourChildIndex);
				return &NeighbourOfParent->Children[NeighbourChildIndex];
			}
			else
			{
				// NeighbourID is already correct from calling FindNeighbourByID
				return NeighbourOfParent;
			}
		}
	}

	return nullptr;
}

std::vector<uint32> ACPathVolume::FindNeighbourLeafs(uint32 TreeID, bool MustBeFree)
{
	std::vector<uint32> FreeNeighbours;

	for (int Direction = 0; Direction < 6; Direction++)
	{
		uint32 NeighbourID = 0;
		CPathOctree* Neighbour = FindNeighbourByID(TreeID, (ENeighbourDirection)Direction, NeighbourID);
		if (Neighbour)
		{
			if (Neighbour->GetIsFree())
				FreeNeighbours.push_back(NeighbourID);
			else if (Neighbour->Children)
			{
				FindLeafsOnSide(Neighbour, NeighbourID, (ENeighbourDirection)LookupTable_OppositeSide[Direction], &FreeNeighbours, MustBeFree);
			}
			else if(!MustBeFree)
				FreeNeighbours.push_back(NeighbourID);
		}
	}

	return FreeNeighbours;
}

std::vector<CPathAStarNode> ACPathVolume::FindFreeNeighbourLeafs(CPathAStarNode& Node)
{
	std::vector<CPathAStarNode> FreeNeighbours;

	for (int Direction = 0; Direction < 6; Direction++)
	{
		uint32 NeighbourID = 0;
		CPathOctree* Neighbour = FindNeighbourByID(Node.TreeID, (ENeighbourDirection)Direction, NeighbourID);
		if (Neighbour)
		{
			if (Neighbour->GetIsFree())
				FreeNeighbours.push_back(CPathAStarNode(NeighbourID, Neighbour->Data));
			else if (Neighbour->Children)
			{
				FindLeafsOnSide(Neighbour, NeighbourID, (ENeighbourDirection)LookupTable_OppositeSide[Direction], &FreeNeighbours);
			}
		}
	}


	return FreeNeighbours;
}


void ACPathVolume::FindLeafsOnSide(uint32 TreeID, ENeighbourDirection Side, std::vector<uint32>* Vector, bool MustBeFree)
{
	uint32 TempDepthReached;
	FindLeafsOnSide(FindTreeByID(TreeID, TempDepthReached), TreeID, Side, Vector, MustBeFree);
}

void ACPathVolume::FindLeafsOnSide(CPathOctree* Tree, uint32 TreeID, ENeighbourDirection Side, std::vector<uint32>* Vector, bool MustBeFree)
{
#if WITH_EDITOR
	checkf(Tree->Children, TEXT("CPATH - FindAllLeafsOnSide, requested tree has no children"));
#endif
	uint8 NewDepth = ExtractDepth(TreeID) + 1;
	for (uint8 i = 0; i < 4; i++)
	{
		uint8 ChildIndex = LookupTable_ChildrenOnSide[Side][i];
		CPathOctree* Child = &Tree->Children[ChildIndex];
		uint32 ChildTreeID = TreeID;
		ReplaceChildIndexAndDepth(ChildTreeID, NewDepth, ChildIndex);
		if (Child->Children)
			FindLeafsOnSide(Child, ChildTreeID, Side, Vector);
		else
		{
			if (Child->GetIsFree() || !MustBeFree)
				Vector->push_back(ChildTreeID);
		}
	}
}

void ACPathVolume::FindLeafsOnSide(CPathOctree* Tree, uint32 TreeID, ENeighbourDirection Side, std::vector<CPathAStarNode>* Vector, bool MustBeFree)
{
#if WITH_EDITOR
	checkf(Tree->Children, TEXT("CPATH - FindAllLeafsOnSide, requested tree has no children"));
#endif
	uint8 NewDepth = ExtractDepth(TreeID) + 1;
	for (uint8 i = 0; i < 4; i++)
	{
		uint8 ChildIndex = LookupTable_ChildrenOnSide[Side][i];
		CPathOctree* Child = &Tree->Children[ChildIndex];
		uint32 ChildTreeID = TreeID;
		ReplaceChildIndexAndDepth(ChildTreeID, NewDepth, ChildIndex);
		if (Child->Children)
			FindLeafsOnSide(Child, ChildTreeID, Side, Vector);
		else
		{
			if (Child->GetIsFree() || !MustBeFree)
				Vector->push_back(CPathAStarNode(ChildTreeID, Child->Data));
		}
	}
}



inline uint32 ACPathVolume::GetFreeThreadID() const
{
	for (int ID = 0; ID < 64; ID++)
	{
		if (!ThreadIDs[ID])
			return ID;
	}

	// This will never return as max number of generation threads is always less than 64
	return 0;
}

void ACPathVolume::CleanFinishedGenerators()
{
	for (auto Generator = GeneratorThreads.begin(); Generator != GeneratorThreads.end(); Generator++)
	{
		ThreadIDs[(*Generator)->GenThreadID] = false;
		if (!(*Generator)->ThreadRef)
		{
			Generator = GeneratorThreads.erase(Generator);
		}
	}

}

void ACPathVolume::InitialGenerationUpdate()
{
	if (GeneratorsRunning.load() <= 0)
	{
		InitialGenerationCompleteAtom.store(true);
		InitialGenerationFinished = true;

		for (auto Generator = GeneratorThreads.begin(); Generator != GeneratorThreads.end(); Generator++)
		{
			for (int Depth = 0; Depth <= OctreeDepth; Depth++)
			{
				OctreeCountAtDepth[Depth] += Generator->get()->OctreeCountAtDepth[Depth];

			}
		}
		for (int Depth = 0; Depth <= OctreeDepth; Depth++)
		{
			TotalNodeCount += OctreeCountAtDepth[Depth];
		}


		CleanFinishedGenerators();
		GetWorld()->GetTimerManager().ClearTimer(GenerationTimerHandle);
		if (DynamicObstaclesUpdateRate > 0)
			GetWorld()->GetTimerManager().SetTimer(GenerationTimerHandle, this, &ACPathVolume::GenerationUpdate, 1.f / DynamicObstaclesUpdateRate, true);
	}

}

void ACPathVolume::GenerationUpdate()
{
#if WITH_EDITOR
	checkf(GeneratorsRunning.load() >= 0, TEXT("CPATH - Graph Generation:::GenerationUpdate - GeneratorsRunning was negative!!!!!"));
#endif
	// Garbage collecting generators that finished their job
	CleanFinishedGenerators();



	// We skip this update if generation from previous update is still running
	// This can be the cause if we set DynamicObstaclesUpdateRate too high, or when it's initial generation, 
	// or if there were a lot of pathfinding requests and generators are waiting for them to finish.
	if (GeneratorsRunning.load() == 0 && TrackedDynamicObstacles.size())
	{

		//Drawing previously updated trees
		/*for (auto TreeID : TreesToRegenerate)
		{
			std::vector<uint32> Subtrees;
			Subtrees.push_back(TreeID);
			GetAllSubtrees(TreeID, Subtrees);
			for (auto SubID : Subtrees)
			{
				DrawDebugVoxel(SubID, false, 1.f / DynamicObstaclesUpdateRate);
			}
		}*/

		// Adding indexes from previous update
		TreesToRegenerate = TreesToRegeneratePreviousUpdate;
		TreesToRegeneratePreviousUpdate.clear();

		// Adding new indexes
		for (auto Obstacle : TrackedDynamicObstacles)
		{
			if (IsValid(Obstacle))
			{
				Obstacle->AddIndexesToUpdate(this);
			}
		}

		// Creating threads
		// In case there is a lot of trees to update, we split the work into multiple threads to make it faster
		if (TreesToRegenerate.size())
		{

			uint32 ThreadCount = FMath::Min(FMath::Min(FPlatformMisc::NumberOfCores(), (int)TreesToRegenerate.size() / OuterIndexesPerThread), MaxGenerationThreads);
			ThreadCount = FMath::Max(ThreadCount, (uint32)1);
			uint32 NodesPerThread = (uint32)TreesToRegenerate.size() / ThreadCount;

			// Starting generation
			for (uint32 CurrentThread = 0; CurrentThread < ThreadCount; CurrentThread++)
			{
				uint32 LastIndex = NodesPerThread * (CurrentThread + 1);
				if (CurrentThread == ThreadCount - 1)
					LastIndex += TreesToRegenerate.size() % ThreadCount;

				int ThreadID = GetFreeThreadID();
				FString ThreadName = "CPathGenerator Dynamic, ID: ";
				ThreadName.AppendInt(ThreadID);
				GeneratorThreads.push_back(std::make_unique<FCPathAsyncVolumeGenerator>(this, NodesPerThread * CurrentThread, LastIndex, ThreadID, ThreadName, true));
				GeneratorThreads.back()->ThreadRef = FRunnableThread::Create(GeneratorThreads.back().get(), *ThreadName);
				if (GeneratorThreads.back()->ThreadRef)
				{
					ThreadIDs[ThreadID] = true;
				}
				else
				{
					GeneratorThreads.pop_back();
				}
			}
			//UE_LOG(LogTemp, Warning, TEXT("GENERATION UPDATE Tracked - %d, Indexes - %d, Threads - %d"), TrackedDynamicObstacles.size(), TreesToRegenerate.size(), ThreadCount);
		}
	}
}

void ACPathVolume::CalcFitness(CPathAStarNode& Node, FVector TargetLocation, int32 UserData)
{
	// Standard weithted A* Heuristic, f(n) = g(n) + e*h(n).   (e = 3.5f)
	if (Node.PreviousNode)
	{
		Node.DistanceSoFar = Node.PreviousNode->DistanceSoFar + FVector::Distance(Node.PreviousNode->WorldLocation, Node.WorldLocation);
	}
	Node.FitnessResult = Node.DistanceSoFar + 3.5f * FVector::Distance(Node.WorldLocation, TargetLocation);
}

bool ACPathVolume::RecheckOctreeAtDepth(CPathOctree* OctreeRef, FVector TreeLocation, uint32 Depth)
{
	bool IsFree = true;
	for (auto Shape : TraceShapesByDepth[Depth])
	{
		if (GetWorld()->OverlapAnyTestByChannel(TreeLocation, FQuat(FRotator(0)), TraceChannel, Shape))
		{
			IsFree = false;
			break;
		}
	}

	// This is mandatory, as AStar only considers nodes that are free. 
	OctreeRef->SetIsFree(IsFree);
	return IsFree;
}

const FVector ACPathVolume::LookupTable_ChildPositionOffsetMaskByIndex[8] = {
	{-1, -1, -1},
	{-1, 1, -1},
	{-1, -1, 1},
	{-1, 1, 1},

	{1, -1, -1},
	{1, 1, -1},
	{1, -1, 1},
	{1, 1, 1}
};


const FVector ACPathVolume::LookupTable_NeighbourOffsetByDirection[6] = {
	{0, -1, 0},
	{-1, 0, 0},
	{0, 1, 0},
	{1, 0, 0},
	{0, 0, -1},
	{0, 0, 1} };

const int8 ACPathVolume::LookupTable_NeighbourChildIndex[8][6] = {
	{-2, -5, 1, 4, -3, 2},
	{0, -6, -1, 5, -4, 3},
	{-4, -7, 3, 6, 0, -1},
	{2, -8, -3, 7, 1, -2},
	{-6, 0, 5, -1, -7, 6},
	{4, 1, -5, -2, -8, 7},
	{-8, 2, 7, -3, 4, -5},
	{6, 3, -7, -4, 5, -6},
};

const int8 ACPathVolume::LookupTable_ChildrenOnSide[6][4] = {
	{0, 2, 4, 6},
	{0, 1, 2, 3},
	{1, 3, 5, 7},
	{4, 5, 6, 7},
	{0, 1, 4, 5},
	{2, 3, 6, 7}
};

const int8 ACPathVolume::LookupTable_OppositeSide[6] = {
	2, 3, 0, 1, 5, 4 };
