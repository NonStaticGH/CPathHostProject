// Copyright Dominik Trautman. All Rights Reserved.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "WorldCollision.h"
#include <memory>
#include <chrono>
#include <vector>
#include <atomic>
#include <set>
#include <list>
#include "PhysicsInterfaceTypesCore.h"
#include "CPathOctree.h"
#include "CPathAsyncVolumeGeneration.h"
#include "CPathVolume.generated.h"

// TreeID settings
// If you change these, you will also need to change some masks in functions like ReplaceDepth, ExtractDepth, etc
#define DEPTH_0_BITS 21
#define DEPTH_0_LIMIT (uint32)1<<DEPTH_0_BITS
#define DEPTH_0_MASK 0x001FFFFF
#define DEPTH_MASK 0x00600000
#define MAX_DEPTH 3

// Time measurement macros
#define TIMENOW std::chrono::steady_clock::now()
// this is in ms
#define TIMEDIFF(BEGIN, END) ((double)std::chrono::duration_cast<std::chrono::nanoseconds>(END - BEGIN).count())/1000000.0 


UCLASS()
class ACPathVolume : public AActor
{
	GENERATED_BODY()

	friend class FCPathAsyncVolumeGenerator;
	friend class UCPathDynamicObstacle;
public:	
	ACPathVolume();

	virtual void Tick(float DeltaTime) override;

	virtual void BeginDestroy() override;


	// -------- BP EXPOSED ----------

	//Box to mark the area to generate graph in. It should not be rotated, the rotation will be ignored.
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = CPathSettings)
		class UBoxComponent* VolumeBox;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = CPathSettings)
		TEnumAsByte<ECollisionChannel>  TraceChannel;
	
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = CPathSettings)
		float AgentRadius = 20;
		
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = CPathSettings)
		float AgentHalfHeight = 75;

	// Spports Capsule, sphere and box.
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = CPathSettings)
		TEnumAsByte<EAgentShape> AgentShape = EAgentShape::Capsule;

	// How many timer per second do parts of the volume get regenerated based on dynamic obstacles, in seconds
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = CPathSettings)
		float DynamicObstaclesUpdateRate = 3;


	// Size of the smallest voxel size, default: AgentRadius*2
	// If you have a lot of tight spaces and want precise pathfinding, set this lower than your lowest Agent dimension.
	// This obviously comes with increased memory cost and speed.
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = CPathSettings)
		float VoxelSize = AgentRadius*2;

	// The smaller it is, the faster pathfinding, but smaller values lead to long generation time and higher memory comsumption
	// Smaller values also limit the size of the volume.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = CPathSettings, meta = (ClampMin = "0", ClampMax = "3", UIMin = "0", UIMax = "3"))
		int OctreeDepth = 2;

	// Drawing depths for debugging, if size of this array is different than OctreeDepth, this will potentially crash
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = CPathSettings)
		TArray<bool> DepthsToDraw;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = CPathSettings)
		bool DrawFree = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = CPathSettings)
		bool DrawOccupied = false;

	// If start or beginning of a path is unreachable, the pathfinding will try to search the WHOLE volume, hence the limit.
	// Default setting should be more than enough, unless you have a huge labirynth with millions of subtrees.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = CPathSettings)
		float PathfindingTimeLimit = 0.5f;

	// This is a read only info about generated graph
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = CPathGeneratedInfo)
		TArray<int> VoxelCountAtDepth;

	// Finds and draws a path from first call to 2nd call. Calls outside of volume dont count.
	UFUNCTION(BlueprintCallable, Category = CPathDebug)
		void DebugDrawNeighbours(FVector WorldLocation);

	UFUNCTION(BlueprintCallable, Category = CPathDebug)
		void SetDebugPathStart(FVector WorldLocation);

	UFUNCTION(BlueprintCallable, Category = CPathDebug)
		void SetDebugPathEnd(FVector WorldLocation);

	// Draws the octree structure around WorldLocation, up tp VoxelLlimit
	UFUNCTION(BlueprintCallable, Category = CPathDebug)
		void DrawAroundLocation(FVector WorldLocation, int VoxelLimit, float Duration);

	// Shape to use when generating graph/paths. If left empty, uses voxels for every trace. 
	//UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = CPathGeneratedInfo)
		//FCollisionShape TraceShape = FCollisionShape::MakeBox(FVector(GetVoxelSizeByDepth(OctreeDepth) / 2.f));

		// Shapes to use when checking if voxel is free or not
	std::vector<std::vector<FCollisionShape>> TraceShapesByDepth;

protected:
	
	virtual void BeginPlay() override;

	void GenerateGraph();

	CPathOctree* Octrees = nullptr;

protected:
	//position of the first voxel
	FVector StartPosition;

	//Dimension sizes of the Nodes array, XYZ 
	uint32 NodeCount[3];

	// ----- Lookup tables-------
	static const FVector LookupTable_ChildPositionOffsetMaskByIndex[8];
	static const FVector LookupTable_NeighbourOffsetByDirection[6];

	// Positive values = ChildIndex of the same parent, negative values = (-ChildIndex - 1) of neighbour at Direction [6]
	static const int8 LookupTable_NeighbourChildIndex[8][6];

	// First index is side as in ENeighbourDirection, and then you get indices of children on that side (in ascending order)
	static const int8 LookupTable_ChildrenOnSide[6][4];

	// Left returns right, up returns down, Front returns behind, etc
	static const int8 LookupTable_OppositeSide[6];


	
public:
	//----------- TreeID ------------------------------------------------------------------------

	// Returns the child with this tree id, or his parent at DepthReached in case the child doesnt exist
	CPathOctree* FindTreeByID(uint32 TreeID, uint32& DepthReached);

	inline CPathOctree* FindTreeByID(uint32 TreeID);

	// Returns a tree and its TreeID by world location, returns null if location outside of volume. Only for Outer index
	CPathOctree* FindTreeByWorldLocation(FVector WorldLocation, uint32& TreeID);

	// Returns a leaf and its TreeID by world location, returns null if location outside of volume. 
	inline CPathOctree* FindLeafByWorldLocation(FVector WorldLocation, uint32& TreeID, bool MustBeFree = 1);

	// Returns a free leaf and its TreeID by world location, as long as it exists in provided search range and WorldLocation is in this Volume
	// If SearchRange <= 0, it uses a default dynamic search range
	// If SearchRange is too large, you might get a free node that is inaccessible from provided WorldLocation
	CPathOctree* FindClosestFreeLeaf(FVector WorldLocation, uint32& TreeID, float SearchRange = -1);

	// Returns a neighbour of the tree with TreeID in given direction, also returns  TreeID if the neighbour if found
	CPathOctree* FindNeighbourByID(uint32 TreeID, ENeighbourDirection Direction, uint32& NeighbourID);

	// Returns a list of free adjecent leafs as TreeIDs
	std::vector<uint32> FindFreeNeighbourLeafs(uint32 TreeID);

	// Returns a parent of tree with given TreeID or null if TreeID has depth of 0
	inline CPathOctree* GetParentTree(uint32 TreeId);

	// Returns world location of a voxel at this TreeID. This returns CENTER of the voxel
	inline FVector WorldLocationFromTreeID(uint32 TreeID) const;

	inline FVector LocalCoordsInt3FromOuterIndex(uint32 OuterIndex) const;

	// Creates TreeID for AsyncOverlapByChannel
	inline uint32 CreateTreeID(uint32 Index, uint32 Depth) const;

	// Extracts Octrees array index from TreeID
	inline uint32 ExtractOuterIndex(uint32 TreeID) const;

	// Replaces Depth in the TreeID with NewDepth
	inline void ReplaceDepth(uint32& TreeID, uint32 NewDepth);

	// Extracts depth from TreeID
	inline uint32 ExtractDepth(uint32 TreeID) const;

	// Returns a number from  0 to 7 - a child index at requested Depth
	inline uint32 ExtractChildIndex(uint32 TreeID, uint32 Depth) const;

	// This assumes that child index at Depth is 000, if its not use ReplaceChildIndex
	inline void AddChildIndex(uint32& TreeID, uint32 Depth, uint32 ChildIndex);

	// Replaces child index at given depth
	inline void ReplaceChildIndex(uint32& TreeID, uint32 Depth, uint32 ChildIndex);

	// Replaces child index at given depth and also replaces depth to the same one
	inline void ReplaceChildIndexAndDepth(uint32& TreeID, uint32 Depth, uint32 ChildIndex);

	// Traverses the tree downwards and adds every tree to the container
	void GetAllSubtrees(uint32 TreeID, std::vector<uint32>& Container);

	// Volume is not safe to access as long as this is not 0, pathfinders should wait till this is 0
	std::atomic_int GeneratorsRunning = 0;

	// Volume wont start generating as long as this is not 0
	std::atomic_int PathfindersRunning = 0;

	
	std::set<class UCPathDynamicObstacle*> TrackedDynamicObstacles;

	// ----------- Other helper functions ---------------------

	inline float GetVoxelSizeByDepth(int Depth) const;

	// Draws the voxel, this takes all the drawing options into condition. If Duraiton is below 0, it never disappears. 
	// If Color = green, free trees are green and occupied are red.
	// Returns true if drawn, false otherwise
	bool DrawDebugVoxel(uint32 TreeID, bool DrawIfNotLeaf = true, float Duration = 0, FColor Color = FColor::Green, CPathVoxelDrawData* OutDrawData = nullptr);
	void DrawDebugVoxel(const CPathVoxelDrawData& DrawData, float Duration) const;

private:

	void AfterTracePreprocess();

	void UpdateNeighbours(CPathOctree* Tree, uint32 TreeID);

	// Returns an index in the Octree array from world position. NO BOUNDS CHECK
	inline int WorldLocationToIndex(FVector WorldLocation) const;

	// Multiplies local integer coordinates into index
	inline float LocalCoordsInt3ToIndex(FVector V) const;

	// Returns the X Y and Z relative to StartPosition and divided by VoxelSize. Multiply them to get the index. NO BOUNDS CHECK
	inline FVector WorldLocationToLocalCoordsInt3(FVector WorldLocation) const;

	// Returns world location of a tree at depth 0. Extracts only outer index from TreeID
	inline FVector GetOuterTreeWorldLocation(uint32 TreeID) const;

	// takes in what `WorldLocationToLocalCoordsInt3` returns and performs a bounds check
	inline bool IsInBounds(FVector LocalCoordsInt3) const;

	// Helper function for 'FindLeafByWorldLocation'. Relative location is location relative to the middle of CurrentTree
	CPathOctree* FindLeafRecursive(FVector RelativeLocation, uint32& TreeID, uint32 CurrentDepth, CPathOctree* CurrentTree);

	// Returns IDs of all free leafs on chosen side of a tree. Sides are indexed in the same way as neighbours, and adds them to passed Vector.
	// ASSUMES THAT PASSED TREE HAS CHILDREN
	void FindFreeLeafsOnSide(uint32 TreeID, ENeighbourDirection Side, std::vector<uint32>* Vector);
	
	// Same as the other version, but skips the part of getting a tree by TreeID so its faster
	void FindFreeLeafsOnSide(CPathOctree* Tree, uint32 TreeID, ENeighbourDirection Side, std::vector<uint32>* Vector);

	void GetAllSubtreesRec(uint32 TreeID, CPathOctree* Tree, std::vector<uint32>& Container, uint32 Depth);


// -------- GENERATION -----
	FTimerHandle GenerationTimerHandle;

	std::list<std::unique_ptr<FCPathAsyncVolumeGenerator>> GeneratorThreads;

	void CleanFinishedGenerators();
	void GenerationUpdate();

	std::set<int32> TreesToRegenerate;
	// This is so that when an actor moves, the previous space it was in needs to be regenerated as well
	std::set<int32> TreesToRegeneratePreviousUpdate;

	
	




// -------- DEBUGGING -----
	FVector DebugPathStart;
	bool HasDebugPathStarted = false;

	std::vector<CPathVoxelDrawData> PreviousDrawAroundLocationData;

	std::chrono::steady_clock::time_point GenerationStart;
	bool PrintGenerationTime = false;
};
