#include "MySplineComponent.h"

UMySplineComponent::UMySplineComponent()
{
	int32 NumPoints = GetNumberOfSplinePoints();
	SplinePointDataArray.SetNum(NumPoints);
}

void UMySplineComponent::OnRegister()
{
	Super::OnRegister();
	
	int32 NumPoints = GetNumberOfSplinePoints();
	if(SplinePointDataArray.Num() != NumPoints)
	{
		SplinePointDataArray.SetNum(NumPoints);
	}
}

void UMySplineComponent::InitializeSplinePointData()
{
	int32 NumPoints = GetNumberOfSplinePoints();
	SplinePointDataArray.SetNum(NumPoints);
	for (int32 PointIndex = 0; PointIndex < NumPoints; ++PointIndex)
	{
		SplinePointDataArray[PointIndex].bMoveForward = true;
	}
}

bool UMySplineComponent::GetMoveForwardAtSplinePoint(int32 PointIndex) const
{
	if (SplinePointDataArray.IsValidIndex(PointIndex))
	{
		return SplinePointDataArray[PointIndex].bMoveForward;
	}
	return true;
}

void UMySplineComponent::SetMoveForwardAtSplinePoint(int32 PointIndex, bool bMoveForward)
{
	if (SplinePointDataArray.IsValidIndex(PointIndex))
	{
		SplinePointDataArray[PointIndex].bMoveForward = bMoveForward;
	}
}
