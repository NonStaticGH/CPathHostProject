// Copyright Dominik Trautman. Published in 2022. All Rights Reserved.

#include "CPathFindPath.h"
#include "CPathVolume.h"
#include <thread>
#include <queue>
#include <deque>
#include <vector>
#include <unordered_set>
#include <memory>
#include "Algo/Reverse.h"
#include "TimerManager.h"
#include "Engine/World.h"

// --------------------------------------------------------
// --------------------------------------------------------
// ---------------- A Star mehtods ------------------------

CPathAStar::CPathAStar()
{
}

CPathAStar::~CPathAStar()
{

}

CPathAStar* CPathAStar::GlobalInstance = nullptr;

CPathAStar* CPathAStar::GetInstance(UWorld* World)
{
	if (GlobalInstance)
	{
		return GlobalInstance;	
	}
	GlobalInstance = new CPathAStar();

	return GlobalInstance;
}

void CPathAStar::DeleteInstance(UWorld* World)
{
	if (GlobalInstance)
	{
		delete GlobalInstance;
		GlobalInstance = nullptr;
	}
}

ECPathfindingFailReason CPathAStar::FindPath(ACPathVolume* VolumeRef, FCPathResult* Result, FVector Start, FVector End, uint32 SmoothingPasses, int32 UserData, float TimeLimit, bool RequestRawPath, bool RequestUserPath)
{
	bStop = false;

#if WITH_EDITOR
	checkf(Result != nullptr, TEXT("CPATH - FindPath:::The result struct was nullptr"));
#endif

	if (!IsValid(VolumeRef))
	{
		Result->FailReason = VolumeNotValid;
		return VolumeNotValid;
		
	}
	if (!VolumeRef->InitialGenerationCompleteAtom.load())
	{
		Result->FailReason = VolumeNotGenerated;
		return VolumeNotGenerated;
	}

	auto TimeStart = TIMENOW;

	// time limit in miliseconds
	double TimeLimitMS = TimeLimit * 1000;

	CurrentVolumeRef = VolumeRef;

	// The A* priority queue
	std::priority_queue<CPathAStarNode, std::deque<CPathAStarNode>, std::greater<CPathAStarNode>> Pq;

	// Nodes visited OR added to priority queue
	std::unordered_set<CPathAStarNode, CPathAStarNode::Hash> VisitedNodes;

	// Nodes that were consumed from priority queue
	std::vector<std::unique_ptr<CPathAStarNode>> ProcessedNodes;

	// Finding start and end node
	uint32 TempID;
	if (!VolumeRef->FindClosestFreeLeaf(Start, TempID))
	{
		Result->FailReason = WrongStartLocation;
		return WrongStartLocation;
	}

	CPathAStarNode StartNode(TempID);
	StartNode.WorldLocation = Start;

	if (!VolumeRef->FindClosestFreeLeaf(End, TempID))
	{
		Result->FailReason = WrongEndLocation;
		return WrongEndLocation;
	}

	// Initializing priority queue
	CPathAStarNode TargetNode(TempID);
	TargetLocation = VolumeRef->WorldLocationFromTreeID(TargetNode.TreeID);
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

		std::vector<CPathAStarNode> Neighbours = VolumeRef->FindFreeNeighbourLeafs(CurrentNode);
		for (CPathAStarNode NewTreeNode : Neighbours)
		{

			if (!VisitedNodes.count(NewTreeNode))
			{
				NewTreeNode.PreviousNode = ProcessedNodes.back().get();
				NewTreeNode.WorldLocation = VolumeRef->WorldLocationFromTreeID(NewTreeNode.TreeID);

				// CalcFitness(NewNode); - this is inline and not virtual so in theory faster, but not extendable.
				// Also from my testing, the speed difference between the two was unnoticeable at 150000 nodes processed.

				VolumeRef->CalcFitness(NewTreeNode, TargetLocation, UserData);
				VisitedNodes.insert(NewTreeNode);
				Pq.push(NewTreeNode);
			}
		}

		if (TIMEDIFF(TimeStart, TIMENOW) >= TimeLimitMS)
		{
			Result->FailReason = Timeout;
			break;
		}
	}

	Result->SearchDuration = TIMEDIFF(TimeStart, TIMENOW);

	// Pathfinidng has been interrupted due to premature thread kill or timeout
	if (bStop)
	{
		if (Result->FailReason != Timeout)
		{
			Result->FailReason = Unknown;
			return Unknown;
		}
	}
	else if (Result->FailReason == Timeout)
	{
		return Timeout;
	}

	if (FoundPathEnd)
	{
		// Adding last node that exactly reflects user's requested location
		uint32 LastTreeID;
		if (VolumeRef->FindLeafByWorldLocation(End, LastTreeID, false))
		{
			ProcessedNodes.push_back(std::make_unique<CPathAStarNode>(CPathAStarNode(LastTreeID)));
			ProcessedNodes.back()->WorldLocation = End;
			ProcessedNodes.back()->PreviousNode = FoundPathEnd;
			FoundPathEnd = ProcessedNodes.back().get();
			VolumeRef->CalcFitness(*FoundPathEnd, TargetLocation, UserData);
		}

		// For debugging
		if (RequestRawPath)
		{
			auto CurrNode = FoundPathEnd;
			while (CurrNode)
			{
				Result->RawPathNodes.Add(*CurrNode);
				CurrNode = CurrNode->PreviousNode;
			}
		}
		Result->RawPathLength = FoundPathEnd->DistanceSoFar;
		// Post processing to remove unnecessary nodes
		for (uint32 i = 0; i < SmoothingPasses; i++)
		{
			SmoothenPath(FoundPathEnd);
		}
		Result->SearchDuration = TIMEDIFF(TimeStart, TIMENOW);
		Result->FailReason = None;
	}
	else
	{
		Result->FailReason = EndLocationUnreachable;
		return EndLocationUnreachable;
	}

#ifdef LOG_PATHFINDERS
	auto CurrDuration = TIMEDIFF(TimeStart, TIMENOW);
	UE_LOG(LogTemp, Warning, TEXT("FindPath:  time= %lfms  NodesVisited= %d  NodesProcessed= %d"), CurrDuration, VisitedNodes.size(), ProcessedNodes.size());
#endif

	if (RequestUserPath)
	{
		TransformToUserPath(FoundPathEnd, Result->UserPath);
	}
	Result->FailReason = None;
	return None;
}

void CPathAStar::TransformToUserPath(CPathAStarNode* PathEndNode, TArray<FCPathNode>& InUserPath, bool bReverse)
{
	float Tolerance = FMath::Cos(FMath::DegreesToRadians(LineAngleToleranceDegrees));
	if (!PathEndNode)
		return;

	CPathAStarNode* CurrNode = PathEndNode;

	// Initializing variables for loop
	FVector Normal = CurrNode->WorldLocation - CurrNode->PreviousNode->WorldLocation;
	Normal.Normalize();
	InUserPath.Add(FCPathNode(CurrNode->WorldLocation));

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
			InUserPath.Add(FCPathNode(CurrNode->PreviousNode->WorldLocation));
			InUserPath.Last().Normal = Normal;

			CurrNode = CurrNode->PreviousNode;
			Normal = NextNormal;
		}
	}
	if (CurrNode->PreviousNode)
	{
		InUserPath.Add(FCPathNode(CurrNode->PreviousNode->WorldLocation));
		InUserPath.Last().Normal = Normal;
	}
	if (bReverse)
		Algo::Reverse(InUserPath);


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
	Node.FitnessResult = Node.DistanceSoFar + 3.5f * EucDistance(Node, TargetLocation);
}

inline bool CPathAStar::CanSkip(FVector Start, FVector End)
{
	FHitResult HitResult;
	CurrentVolumeRef->GetWorld()->SweepSingleByChannel(HitResult, Start, End, FQuat(FRotator(0, 0, 0)), CurrentVolumeRef->TraceChannel, CurrentVolumeRef->TraceShapesByDepth.back().back());

	return !HitResult.bBlockingHit;
}

void CPathAStar::SmoothenPath(CPathAStarNode* PathEndNode)
{
	if (!PathEndNode)
		return;
	CPathAStarNode* CurrNode = PathEndNode;
	while (CurrNode->PreviousNode && CurrNode->PreviousNode->PreviousNode && !bStop)
	{
		if (CanSkip(CurrNode->WorldLocation, CurrNode->PreviousNode->PreviousNode->WorldLocation))
		{
			CurrNode->PreviousNode = CurrNode->PreviousNode->PreviousNode;
		}

		CurrNode = CurrNode->PreviousNode;
	}
}


// --------------------------------------------------------
// --------------------------------------------------------
// ---------------- UCPathAsyncFindPath methods ------------------------


UCPathAsyncFindPath* UCPathAsyncFindPath::FindPathAsync(ACPathVolume* Volume, FVector StartLocation, FVector EndLocation, int SmoothingPasses, int32 UserData, float TimeLimit)
{
#if WITH_EDITOR
	checkf(IsValid(Volume), TEXT("CPATH - FindPathAsync:::Volume was invalid"));
#endif

	UCPathAsyncFindPath* Instance = NewObject<UCPathAsyncFindPath>();
	Instance->RegisterWithGameInstance(Volume->GetGameInstance());
	Instance->Request.VolumeRef = Volume;
	Instance->Request.Start = StartLocation;
	Instance->Request.End = EndLocation;
	Instance->Request.SmoothingPasses = SmoothingPasses;
	Instance->Request.UserData = UserData;
	Instance->Request.TimeLimit = TimeLimit;
	return Instance;
}


void UCPathAsyncFindPath::Activate()
{
	if (!IsValid(Request.VolumeRef))
	{
		TArray<FCPathNode> EmptyPath;
		Failure.Broadcast(EmptyPath, TEnumAsByte(ECPathfindingFailReason::VolumeNotValid));
		SetReadyToDestroy();
		RemoveFromRoot();
	}
	else
	{
		auto FunctionName = GET_FUNCTION_NAME_CHECKED(UCPathAsyncFindPath, OnPathFound);
		Request.OnPathFound.BindUFunction(this, FunctionName);
		Request.RequestRawPath = false;
		Request.RequestUserPath = true;
		Request.VolumeRef->FindPathAsync(Request);
		//CurrentThread = FRunnableThread::Create(RunnableFindPath, TEXT("CPath Pathfinding Thread"));
		//AStar->Volume->GetWorld()->GetTimerManager().SetTimer(CheckThreadTimerHandle, this, &UCPathAsyncFindPath::CheckThreadStatus, 1.f / 30.f, true);
	}
}

void UCPathAsyncFindPath::BeginDestroy()
{
	Super::BeginDestroy();

}


void UCPathAsyncFindPath::OnPathFound(FCPathResult& PathResult)
{
	if (PathResult.FailReason == None)
	{
		Success.Broadcast(PathResult.UserPath, TEnumAsByte(PathResult.FailReason));
	}
	else
	{
		Failure.Broadcast(PathResult.UserPath, TEnumAsByte(PathResult.FailReason));
	}

	SetReadyToDestroy();
	RemoveFromRoot();
}
