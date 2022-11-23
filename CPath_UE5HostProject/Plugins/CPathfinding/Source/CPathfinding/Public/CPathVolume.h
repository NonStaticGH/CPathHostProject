// Copyright Dominik Trautman. Published in 2022. All Rights Reserved.
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
#include "CPathDefines.h"
#include "CPathOctree.h"
#include "CPathNode.h"
#include "CPathAsyncVolumeGeneration.h"
#include "CPathVolume.generated.h"


UCLASS()
class CPATHFINDING_API ACPathVolume : public AActor
{
	GENERATED_BODY()

		friend class FCPathAsyncVolumeGenerator;
	friend class UCPathDynamicObstacle;
public:
	ACPathVolume();

	virtual void Tick(float DeltaTime) override;

	virtual void BeginDestroy() override;


	// ------- EXTENDABLE ------

	// Overwrite this function to change the priority of nodes as they are selected for the path.
	// Note that this is potentially called thousands of times per FindPath call, so it shouldnt be too complex (unless your graph not very dense)
	virtual void CalcFitness(CPathAStarNode& Node, FVector TargetLocation, int32 UserData);

	// Overwrite this function to change the default conditions of a tree being free/ocupied.
	// You may also save other information in the Data field of an Octree, as only the least significant bit is used.
	// This is called during graph generation, for every subtree including leafs, so potentially millions of times. 
	virtual bool RecheckOctreeAtDepth(CPathOctree* OctreeRef, FVector TreeLocation, uint32 Depth);


	// -------- BP EXPOSED ----------

	//Box to mark the area to generate graph in. It should not be rotated, the rotation will be ignored.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Components", meta = (EditCondition = "GenerationStarted==false"))
		class UBoxComponent* VolumeBox;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false"))
		TEnumAsByte<ECollisionChannel>  TraceChannel = ECollisionChannel::ECC_Visibility;

	// Spports Capsule, sphere and box.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false"))
		TEnumAsByte<EAgentShape> AgentShape = EAgentShape::Capsule;

	// In case of a box, this is X and Y extent. Z and Y should be the same, since the actual agent will most likely rotate.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false", ClampMin = "0", UIMin = "0"))
		float AgentRadius = 0;

	// In case of a box, this is Z extent.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false && AgentShape!=EAgentShape::Sphere", ClampMin = "0", UIMin = "0"))
		float AgentHalfHeight = 0;




	// Size of the smallest voxel edge.
	// In most cases, setting this to min(AgentRadius, AgentHalfHeight)*2 is enough.
	// For precise (dense) graph - set this to min(AgentRadius, AgentHalfHeight).
	// Small values increase memory cost, and potentially CPU load.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false", ClampMin = "0.1", UIMin = "0.1"))
		float VoxelSize = 60;

	// How many times per second do parts of the volume get regenerated based on dynamic obstacles, in seconds.
	// Values higher than 5 are an overkill, but for the purpose of user freedom, I leave it unlocked.
	// If no dynamic obstacles were added, this doesnt have any performance impact
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false", ClampMin = "0.01", UIMin = "0.01", ClampMax = "30", UIMax = "30"))
		float DynamicObstaclesUpdateRate = 3;

	// 2 Is optimal in most cases. If you have very large open speces with small amount of obstacles, then 3 will be better.
	// For dense labirynths with little to no open space, 1 or even 0 will be faster.
	// Check documentation for detailed performance guidance.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false", ClampMin = "0", ClampMax = "3", UIMin = "0", UIMax = "3"))
		int OctreeDepth = 2;


	// If want to call Generate() later or with some condition.
	// Note that volume wont be usable before it is generated
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath")
		bool GenerateOnBeginPlay = true;

	// Set a custom generation thread limit. By default, it's system's Physical Core count - 1.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath")
		bool OverwriteMaxGenerationThreads = false;

	// How many threads can graph generation split into. 
	// If left <=0 (RECOMMENDED), it uses system's Physical Core count - 1. 
	// Generation threads are allocated dynamically, so it only uses more than 1 thread when necessary.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath", meta = (EditCondition = "GenerationStarted==false && OverwriteMaxGenerationThreads==true", ClampMin = "0", ClampMax = "31", UIMin = "0", UIMax = "31"))
		int MaxGenerationThreads = 0;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath|Render")
		bool DrawFree = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CPath|Render")
		bool DrawOccupied = false;

	// You can hide selected depths from rendering
	UPROPERTY(EditAnywhere, BlueprintReadOnly, EditFixedSize, Category = "CPath|Render")
		TArray<bool> DepthsToDraw;

	// How thick should the green and red debug boxes be
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CPath|Render")
		float DebugBoxesThickness = 1.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CPath|Render")
		float DebugPathThickness = 1.5f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CPath|Info")
		bool GenerationStarted = false;

	// This is a read only info about initially generated graph
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CPath|Info")
		TArray<int> OctreeCountAtDepth = { 0, 0, 0, 0 };

	// This is a read only info about initially generated graph
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CPath|Info")
		int TotalNodeCount = 0;

	// Draws FREE neighbouring leafs
	UFUNCTION(BlueprintCallable, Category = "CPath|Render")
		void DebugDrawNeighbours(FVector WorldLocation);

	// Draws the octree structure around WorldLocation, up tp VoxelLlimit
	UFUNCTION(BlueprintCallable, Category = "CPath|Render")
		void DrawDebugNodesAroundLocation(FVector WorldLocation, int VoxelLimit, float Duration);

	// Draws path with points, for visualization only
	// Duration == 0 - draw for one frame
	// Duration < 0 - persistent
	UFUNCTION(BlueprintCallable, Category = "CPath|Render")
		void DrawDebugPath(const TArray<FCPathNode>& Path, float Duration, bool DrawPoints = true, FColor Color = FColor::Magenta);

	// Before this is true, the graph is inoperable
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CPath|Info")
		bool InitialGenerationFinished = false;

	// Shapes to use when checking if voxel is free or not
	std::vector<std::vector<FCollisionShape>> TraceShapesByDepth;

	// Returns false if graph couldnt start generating
	bool GenerateGraph();

protected:

	virtual void BeginPlay() override;

	CPathOctree* Octrees = nullptr;


public:

	// Location of the first voxel, set during graph generation
	FVector StartPosition;

	// Dimension sizes of the Nodes array, XYZ 
	uint32 NodeCount[3];

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

	// Returns a list of adjecent leafs as TreeIDs
	std::vector<uint32> FindNeighbourLeafs(uint32 TreeID, bool MustBeFree = true);

	// Returns a list of adjecent free leafs as CPathAStarNode
	std::vector<CPathAStarNode> FindFreeNeighbourLeafs(CPathAStarNode& Node);

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

	// This is for other threads to check if graph is accessible
	std::atomic_bool InitialGenerationCompleteAtom = false;

	// This is filled by DynamicObstacle component
	std::set<class UCPathDynamicObstacle*> TrackedDynamicObstacles;

	// ----------- Other helper functions ---------------------

	inline float GetVoxelSizeByDepth(int Depth) const;

	// Draws the voxel, this takes all the drawing options into condition. If Duraiton is below 0, it never disappears. 
	// If Color = green, free trees are green and occupied are red.
	// Returns true if drawn, false otherwise
	bool DrawDebugVoxel(uint32 TreeID, bool DrawIfNotLeaf = true, float Duration = 0, FColor Color = FColor::Green, CPathVoxelDrawData* OutDrawData = nullptr);
	void DrawDebugVoxel(const CPathVoxelDrawData& DrawData, float Duration) const;


protected:

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
	void FindLeafsOnSide(uint32 TreeID, ENeighbourDirection Side, std::vector<uint32>* Vector, bool MustBeFree = true);

	// Same as above, but skips the part of getting a tree by TreeID so its faster
	void FindLeafsOnSide(CPathOctree* Tree, uint32 TreeID, ENeighbourDirection Side, std::vector<uint32>* Vector, bool MustBeFree = true);

	// Same as above, but wrapped in CPathAStarNode
	void FindLeafsOnSide(CPathOctree* Tree, uint32 TreeID, ENeighbourDirection Side, std::vector<CPathAStarNode>* Vector, bool MustBeFree = true);

	// Internal function used in GetAllSubtrees
	void GetAllSubtreesRec(uint32 TreeID, CPathOctree* Tree, std::vector<uint32>& Container, uint32 Depth);


	// -------- GENERATION -----
	FTimerHandle GenerationTimerHandle;

	std::list<std::unique_ptr<FCPathAsyncVolumeGenerator>> GeneratorThreads;

	// Garbage collection
	void CleanFinishedGenerators();

	// Checking if initial generation has finished
	void InitialGenerationUpdate();

	// Checking if there are any trees to regenerate from dynamic obstacles
	void GenerationUpdate();

	std::set<int32> TreesToRegenerate;

	// This is so that when an actor moves, the previous space it was in needs to be regenerated as well
	std::set<int32> TreesToRegeneratePreviousUpdate;

	// This is set in GenerateGraph() using a formula that estimates total voxel count
	int OuterIndexesPerThread;

	bool ThreadIDs[64];

	inline uint32 GetFreeThreadID() const;


	// ----- Lookup tables-------
	static const FVector LookupTable_ChildPositionOffsetMaskByIndex[8];
	static const FVector LookupTable_NeighbourOffsetByDirection[6];

	// Positive values = ChildIndex of the same parent, negative values = (-ChildIndex - 1) of neighbour at Direction [6]
	static const int8 LookupTable_NeighbourChildIndex[8][6];

	// First index is side as in ENeighbourDirection, and then you get indices of children on that side (in ascending order)
	static const int8 LookupTable_ChildrenOnSide[6][4];

	// Left returns right, up returns down, Front returns behind, etc
	static const int8 LookupTable_OppositeSide[6];

	// Set in begin play
	float LookupTable_VoxelSizeByDepth[MAX_DEPTH + 1];


	// -------- DEBUGGING -----
	std::vector<CPathVoxelDrawData> PreviousDrawAroundLocationData;

	std::chrono::steady_clock::time_point GenerationStart;
	bool PrintGenerationTime = false;


};
