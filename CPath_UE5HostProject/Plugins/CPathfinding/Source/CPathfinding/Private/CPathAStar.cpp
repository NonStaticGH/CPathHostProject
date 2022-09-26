// Copyright Dominik Trautman. All Rights Reserved.

#include "CPathAStar.h"
#include "CPathVolume.h"
#include "DrawDebugHelpers.h"
#include <queue>
#include <deque>
#include <vector>
#include <unordered_set>
#include <memory>
#include "Algo/Reverse.h"
#include "Engine/World.h"


CPathAStar::CPathAStar()
{
}

CPathAStar::~CPathAStar()
{
	
}

CPathAStarNode* CPathAStar::FindPath(ACPathVolume* VolumeRef, FVector Start, FVector End, uint32 SmoothingPasses, float TimeLimit, TArray<CPathAStarNode>* RawNodes)
{	
	Volume = VolumeRef;
	SearchTimeLimit = TimeLimit;	
	uint32 TempID;
	auto TimeStart = TIMENOW;

	// time limit in miliseconds
	double TimeLimitMS = SearchTimeLimit * 1000;	
	
	// The A* priority queue
	std::priority_queue<CPathAStarNode, std::deque<CPathAStarNode>, std::greater<CPathAStarNode>> Pq;

	// Nodes visited OR added to priority queue
	std::unordered_set<CPathAStarNode, CPathAStarNode::Hash> VisitedNodes;

	ProcessedNodes.clear();

	// Finding start and end node
	if (!Volume->FindClosestFreeLeaf(Start, TempID))
		return nullptr;
	CPathAStarNode StartNode(TempID);
	StartNode.WorldLocation = Start;

	if(!Volume->FindClosestFreeLeaf(End, TempID))
		return nullptr;

	CPathAStarNode TargetNode(TempID);
	TargetLocation = Volume->WorldLocationFromTreeID(TargetNode.TreeID);
	TargetNode.WorldLocation = TargetLocation;
	CalcFitness(TargetNode);
	CalcFitness(StartNode);
	Pq.push(StartNode);
	VisitedNodes.insert(StartNode);	
	CPathAStarNode* FoundPathEnd = nullptr;

	// A* loop
	while (Pq.size() > 0 && !bStop)
	{
		CPathAStarNode CurrentNode = Pq.top();
		Pq.pop();
		ProcessedNodes.push_back(std::make_unique<CPathAStarNode>(CurrentNode));

		if (CurrentNode == TargetNode)
		{
			FoundPathEnd = ProcessedNodes.back().get();			
			break;
		}
		
		std::vector<uint32> Neighbours = VolumeRef->FindFreeNeighbourLeafs(CurrentNode.TreeID);
		for (uint32 NewTreeID : Neighbours)
		{
			if (bStop)
				break;

			CPathAStarNode NewNode(NewTreeID);
			if (!VisitedNodes.count(NewNode))
			{
				NewNode.PreviousNode = ProcessedNodes.back().get();
				NewNode.WorldLocation = Volume->WorldLocationFromTreeID(NewNode.TreeID);
				CalcFitness(NewNode);

				VisitedNodes.insert(NewNode);
				Pq.push(NewNode);
			}
		}

		auto CurrDuration = TIMEDIFF(TimeStart, TIMENOW);
		if (CurrDuration >= TimeLimitMS)
		{
			bStop = true;			
			UE_LOG(LogTemp, Warning, TEXT("Pathfinding failed - OVERTIME= %lfms    NodesVisited= %d   NodesProcessed= %d"), CurrDuration, VisitedNodes.size(), ProcessedNodes.size());
		}
	}

	// Pathfinidng has been interrupted due to premature thread kill, so we dont want to return an incomplete path
	if (bStop)
	{
		UE_LOG(LogTemp, Warning, TEXT("PATHFINDING INTERRUPTED!!"));
		return nullptr;
	}

	
	if (FoundPathEnd)
	{
		// Adding last node that exactly reflects user's requested location
		uint32 LastTreeID;
		if (Volume->FindLeafByWorldLocation(End, LastTreeID, false))
		{
			ProcessedNodes.push_back(std::make_unique<CPathAStarNode>(CPathAStarNode(LastTreeID)));
			ProcessedNodes.back()->WorldLocation = End;
			ProcessedNodes.back()->PreviousNode = FoundPathEnd;
			FoundPathEnd = ProcessedNodes.back().get();
		}
		
		// For debugging
		if (RawNodes)
		{
			auto CurrNode = FoundPathEnd;
			while (CurrNode)
			{
				RawNodes->Add(*CurrNode);
				CurrNode = CurrNode->PreviousNode;
			}			
		}

		// Post processing to remove unnecessary nodes
		for (uint32 i = 0; i < SmoothingPasses; i++)
		{
			SmoothenPath(FoundPathEnd);
		}			
	}
	
	auto CurrDuration = TIMEDIFF(TimeStart, TIMENOW);
	UE_LOG(LogTemp, Warning, TEXT("PATHFINDING COMPLETE:  time= %lfms    NodesVisited= %d   NodesProcessed= %d"), CurrDuration, VisitedNodes.size(), ProcessedNodes.size());

	return FoundPathEnd;
}

void CPathAStar::DrawPath(const TArray<FCPathNode>& Path) const
{
	float Duration = 10;
	for (int i = 0; i < Path.Num()-1; i++)
	{
		DrawDebugLine(Volume->GetWorld(), Path[i].WorldLocation, Path[i + 1].WorldLocation, FColor::Magenta, false, Duration, 3, 1.5);
		DrawDebugPoint(Volume->GetWorld(), Path[i].WorldLocation, 60, FColor::Cyan, false, Duration);
	}
}

void CPathAStar::TransformToUserPath(CPathAStarNode* PathEnd, TArray<FCPathNode>& UserPath, bool bReverse)
{
	float Tolerance = FMath::Cos(FMath::DegreesToRadians(LineAngleToleranceDegrees));
	if (!PathEnd)
		return;
		
	CPathAStarNode* CurrNode = PathEnd;
	
	// Initializing variables for loop
	FVector Normal = CurrNode->WorldLocation - CurrNode->PreviousNode->WorldLocation;
	Normal.Normalize();
	UserPath.Add(FCPathNode(CurrNode->WorldLocation));

	while (CurrNode->PreviousNode && CurrNode->PreviousNode->PreviousNode)
	{		
		FVector NextNormal = CurrNode->PreviousNode->WorldLocation - CurrNode->PreviousNode->PreviousNode->WorldLocation;
		NextNormal.Normalize();
		
		
		if (FVector::DotProduct(Normal, NextNormal) >= Tolerance)
		{
			CurrNode->PreviousNode = CurrNode->PreviousNode->PreviousNode;
			Normal = CurrNode->WorldLocation - CurrNode->PreviousNode->WorldLocation;
			Normal.Normalize();
		}
		else
		{					
			UserPath.Add(FCPathNode(CurrNode->PreviousNode->WorldLocation));
			UserPath.Last().Normal = Normal;
					
			CurrNode = CurrNode->PreviousNode;
			Normal = NextNormal;
		}
	}
	if (CurrNode->PreviousNode)
	{
		UserPath.Add(FCPathNode(CurrNode->PreviousNode->WorldLocation));
		UserPath.Last().Normal = Normal;					
	}
	if (bReverse)
		Algo::Reverse(UserPath);

	
}

float CPathAStar::EucDistance(CPathAStarNode& Node, FVector Target) const
{
	return FVector::Distance(Node.WorldLocation, Target);
}

void CPathAStar::CalcFitness(CPathAStarNode& Node)
{	
	if (Node.PreviousNode)
	{		
		Node.DistanceSoFar = Node.PreviousNode->DistanceSoFar + EucDistance(*Node.PreviousNode, Node.WorldLocation);
	}
	Node.FitnessResult = Node.DistanceSoFar + 3.5f*EucDistance(Node, TargetLocation);
}

inline bool CPathAStar::CanSkip(FVector Start, FVector End)
{
	FHitResult HitResult;
	Volume->GetWorld()->SweepSingleByChannel(HitResult, Start, End, FQuat(FRotator(0, 0, 0)), Volume->TraceChannel, Volume->TraceShapesByDepth.back().back());

	return !HitResult.bBlockingHit;
}

void CPathAStar::SmoothenPath(CPathAStarNode* PathEnd)
{
	if (!PathEnd)
		return;
	CPathAStarNode* CurrNode = PathEnd;
	while (CurrNode->PreviousNode && CurrNode->PreviousNode->PreviousNode && !bStop)
	{
		if (CanSkip(CurrNode->WorldLocation, CurrNode->PreviousNode->PreviousNode->WorldLocation))
		{
			CurrNode->PreviousNode = CurrNode->PreviousNode->PreviousNode;
		}
		
		CurrNode = CurrNode->PreviousNode;
	}
}
