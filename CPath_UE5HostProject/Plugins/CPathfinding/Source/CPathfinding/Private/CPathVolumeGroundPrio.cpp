// // Copyright Dominik Trautman. All Rights Reserved.


#include "CPathVolumeGroundPrio.h"

void ACPathVolumeGroundPrio::CalcFitness(CPathAStarNode& Node, FVector TargetLocation, int32 UserData)
{
	// Standard weithted A* Heuristic, f(n) = g(n) + e*h(n).   (e = 3.5f)
	if (Node.PreviousNode)
	{
		Node.DistanceSoFar = Node.PreviousNode->DistanceSoFar + FVector::Distance(Node.PreviousNode->WorldLocation, Node.WorldLocation);
	}
	float CurrDistance = FVector::Distance(Node.WorldLocation, TargetLocation);


	if (CurrDistance > VoxelSize && !ExtractIsGroundFromData(Node.TreeUserData))
	{
		Node.DistanceSoFar += UserData;
	}

	Node.FitnessResult = Node.DistanceSoFar + 3.5f * CurrDistance;


	
}

bool ACPathVolumeGroundPrio::RecheckOctreeAtDepth(CPathOctree* OctreeRef, FVector TreeLocation, uint32 Depth)
{
	// We still want the normal trace to check if the node is free
	bool IsFree = Super::RecheckOctreeAtDepth(OctreeRef, TreeLocation, Depth);
	
	// We dont need to calculate anything if its not free since it won't be searched
	if (IsFree)
	{
		// Checking if this is a ground node
		uint32 IsGround = GetWorld()->LineTraceTestByChannel(TreeLocation, FVector(TreeLocation.X, TreeLocation.Y, TreeLocation.Z - VoxelSize*1.49), TraceChannel);
		
		// Setting IsGround to 2nd bit in tree's data
		OctreeRef->Data &= 0xFFFFFFFD;
		OctreeRef->Data |= (IsGround << 1);
	}


	return IsFree;
	
}
