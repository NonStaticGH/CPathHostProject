// Copyright Dominik Trautman. Published in 2022. All Rights Reserved.

#include "CPathDynamicObstacle.h"
#include "CPathVolume.h"

// Sets default values for this component's properties
UCPathDynamicObstacle::UCPathDynamicObstacle()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = false;
	PrimaryComponentTick.bStartWithTickEnabled = false;
	bAutoActivate = false;

	if (GetOwner())
	{
		if (!GetOwner()->IsRootComponentMovable())
		{
			UE_LOG(LogTemp, Warning, TEXT("CPath - Dynamic obstacle '%s' is not Movable."), *GetOwner()->GetName());
		}
		if (ActivateOnBeginPlay)
			GetOwner()->bGenerateOverlapEventsDuringLevelStreaming = true;
	}

	// ...
}


void UCPathDynamicObstacle::Activate(bool bReset)
{

	Super::Activate();

	TSubclassOf<ACPathVolume> Filter = ACPathVolume::StaticClass();
	GetOwner()->GetOverlappingActors(OverlappigVolumes, ACPathVolume::StaticClass());

	for (AActor* Volume : OverlappigVolumes)
	{
		Cast<ACPathVolume>(Volume)->TrackedDynamicObstacles.insert(this);
	}

}

void UCPathDynamicObstacle::Deactivate()
{
	Super::Deactivate();
	for (AActor* Volume : OverlappigVolumes)
	{
		auto CastedVolume = Cast<ACPathVolume>(Volume);
		if (IsValid(CastedVolume))
		{
			CastedVolume->TrackedDynamicObstacles.erase(this);
		}
	}
	OverlappigVolumes.Empty();
}

void UCPathDynamicObstacle::AddIndexesToUpdate(ACPathVolume* Volume)
{
	FVector Origin, Extent;
	GetOwner()->GetActorBounds(true, Origin, Extent);

	FVector XYZ = Volume->WorldLocationToLocalCoordsInt3(Origin);
	if (!Volume->IsInBounds(XYZ))
	{
		return;
	}
	uint32 Index = Volume->LocalCoordsInt3ToIndex(XYZ);
	FVector DistanceFromCenter = Origin - Volume->WorldLocationFromTreeID(Index);

	float VoxelSize = Volume->GetVoxelSizeByDepth(0);
	float VoxelExtent = VoxelSize / 2.f;

	// How many outer trees should we include in given direction
	int MaxOffsetInDirection[6];
	MaxOffsetInDirection[Left] = FMath::CeilToInt((Extent.Y - (VoxelExtent + DistanceFromCenter.Y)) / VoxelSize);
	MaxOffsetInDirection[Front] = FMath::CeilToInt((Extent.X - (VoxelExtent + DistanceFromCenter.X)) / VoxelSize);
	MaxOffsetInDirection[Right] = FMath::CeilToInt((Extent.Y - (VoxelExtent - DistanceFromCenter.Y)) / VoxelSize);
	MaxOffsetInDirection[Behind] = FMath::CeilToInt((Extent.X - (VoxelExtent - DistanceFromCenter.X)) / VoxelSize);
	MaxOffsetInDirection[Below] = FMath::CeilToInt((Extent.Z - (VoxelExtent + DistanceFromCenter.Z)) / VoxelSize);
	MaxOffsetInDirection[Above] = FMath::CeilToInt((Extent.Z - (VoxelExtent - DistanceFromCenter.Z)) / VoxelSize);

	FVector Offset = FVector::ZeroVector;
	for (int X = -MaxOffsetInDirection[Front]; X <= MaxOffsetInDirection[Behind]; X++)
	{
		Offset.X = X;
		for (int Y = -MaxOffsetInDirection[Left]; Y <= MaxOffsetInDirection[Right]; Y++)
		{
			Offset.Y = Y;
			for (int Z = -MaxOffsetInDirection[Below]; Z <= MaxOffsetInDirection[Above]; Z++)
			{
				Offset.Z = Z;

				FVector CurrXYZ = XYZ + Offset;
				if (Volume->IsInBounds(CurrXYZ))
				{
					Index = Volume->LocalCoordsInt3ToIndex(CurrXYZ);
					Volume->TreesToRegenerate.insert(Index);
					Volume->TreesToRegeneratePreviousUpdate.insert(Index);
				}
			}
		}
	}
}

void UCPathDynamicObstacle::EndPlay(EEndPlayReason::Type Reason)
{
	Deactivate();
}


void UCPathDynamicObstacle::BeginPlay()
{
	Super::BeginPlay();
	GetOwner()->OnActorBeginOverlap.AddDynamic(this, &UCPathDynamicObstacle::OnBeginOverlap);
	GetOwner()->OnActorEndOverlap.AddDynamic(this, &UCPathDynamicObstacle::OnBeginOverlap);
	if (ActivateOnBeginPlay)
	{
		Activate();
	}
}



void UCPathDynamicObstacle::OnBeginOverlap(AActor* Owner, AActor* OtherActor)
{
	if (IsActive())
	{
		ACPathVolume* Volume = Cast<ACPathVolume>(OtherActor);
		if (Volume)
		{
			Volume->TrackedDynamicObstacles.insert(this);
			OverlappigVolumes.Add(Volume);
		}
	}
}

void UCPathDynamicObstacle::OnEndOverlap(AActor* Owner, AActor* OtherActor)
{
	if (IsActive())
	{
		ACPathVolume* Volume = Cast<ACPathVolume>(OtherActor);
		if (Volume)
		{
			Volume->TrackedDynamicObstacles.erase(this);
			OverlappigVolumes.Remove(Volume);
		}
	}
}

