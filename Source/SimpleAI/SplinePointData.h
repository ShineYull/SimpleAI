#pragma once

#include "CoreMinimal.h"
#include "SplinePointData.generated.h"

USTRUCT(BlueprintType)
struct FSplinePointData
{
	GENERATED_BODY()
	FSplinePointData() : bMoveForward(true) {}
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spline Point Data")
	bool bMoveForward;
};