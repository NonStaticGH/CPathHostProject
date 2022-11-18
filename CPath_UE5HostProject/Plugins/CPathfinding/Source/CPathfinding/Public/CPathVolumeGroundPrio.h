// // Copyright Dominik Trautman. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "CPathVolume.h"
#include "CPathVolumeGroundPrio.generated.h"

/**
 * 
 */
UCLASS()
class CPATHFINDING_API ACPathVolumeGroundPrio : public ACPathVolume
{
	GENERATED_BODY()
public:
	virtual void CalcFitness(CPathAStarNode& Node, FVector TargetLocation, int32 UserData) override;

	virtual bool RecheckOctreeAtDepth(CPathOctree* OctreeRef, FVector TreeLocation, uint32 Depth);

	inline bool ExtractIsGroundFromData(uint32 TreeUserData)
	{
		return TreeUserData & 0x00000002;
	}

};
