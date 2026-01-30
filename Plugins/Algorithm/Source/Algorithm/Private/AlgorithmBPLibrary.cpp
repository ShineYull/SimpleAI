// Copyright Epic Games, Inc. All Rights Reserved.

#include "AlgorithmBPLibrary.h"
#include "Algorithm.h"

#include "Containers/Queue.h"
#include "Containers/UnrealString.h"
#include "Algo/Reverse.h"
#include <Eigen/Dense>
#include "Kismet/KismetSystemLibrary.h"


UAlgorithmBPLibrary::UAlgorithmBPLibrary(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{

}

FVector2D UAlgorithmBPLibrary::RotateVector(const FVector2D& Vec, float AngleDegrees)
{
	float AngleRadians = FMath::DegreesToRadians(AngleDegrees);
	float CosAngle = FMath::Cos(AngleRadians);
	float SinAngle = FMath::Sin(AngleRadians);
	int32 RotatedX = FMath::RoundToInt(Vec.X * CosAngle - Vec.Y * SinAngle);
	int32 RotatedY = FMath::RoundToInt(Vec.X * SinAngle + Vec.Y * CosAngle);
	return FVector2D(RotatedX, RotatedY);
}

double UAlgorithmBPLibrary::Heuristic(const FPoint& A, const FPoint& B)
{
	// Octile heuristic (diagonal movement allowed)
	double dx = FMath::Abs(A.X - B.X);
	double dy = FMath::Abs(A.Y - B.Y);
	double D = 1.0;
	double D2 = 1.41421356237;
	return D * (dx + dy) + (D2 - 2 * D) * FMath::Min(dx, dy);
}

bool UAlgorithmBPLibrary::IsValid(const FPoint& P, const TArray<int32>& Grid, int32 GridWidth, int32 GridHeight)
{
	return P.Y >= 0 && P.Y < GridWidth && P.X >= 0 && P.X < GridHeight && Grid[P.X * GridWidth + P.Y] == 0;
}

TArray<FPoint> UAlgorithmBPLibrary::GetNeighbors(const FPoint& P)
{
	TArray<FPoint> Neighbors = {
	FPoint(P.X + 1, P.Y), FPoint(P.X - 1, P.Y),
	FPoint(P.X, P.Y + 1), FPoint(P.X, P.Y - 1),
	FPoint(P.X + 1, P.Y + 1), FPoint(P.X + 1, P.Y - 1),
	FPoint(P.X - 1, P.Y + 1), FPoint(P.X - 1, P.Y - 1)
	};

	return Neighbors;
}

TArray<FPoint> UAlgorithmBPLibrary::GetVehicleNeighbors(const FPoint& P, const FVector2D& ForwardVector)
{
	TArray<FPoint> Neighbors;

	// Generate neighbors for forward direction
	FVector2D Forward = ForwardVector;
	FVector2D ForwardLeft = RotateVector(ForwardVector, -45);
	FVector2D ForwardRight = RotateVector(ForwardVector, 45);

	// Generate neighbors for backward direction
	FVector2D Backward = -ForwardVector;
	FVector2D BackwardLeft = RotateVector(Backward, -45);
	FVector2D BackwardRight = RotateVector(Backward, 45);

	// Calculate neighboring points and add them to the array
	Neighbors.Add(FPoint(P.X + Forward.X, P.Y + Forward.Y));
	Neighbors.Add(FPoint(P.X + ForwardLeft.X, P.Y + ForwardLeft.Y));
	Neighbors.Add(FPoint(P.X + ForwardRight.X, P.Y + ForwardRight.Y));

	Neighbors.Add(FPoint(P.X + Backward.X, P.Y + Backward.Y));
	Neighbors.Add(FPoint(P.X + BackwardLeft.X, P.Y + BackwardLeft.Y));
	Neighbors.Add(FPoint(P.X + BackwardRight.X, P.Y + BackwardRight.Y));

	return Neighbors;
}


TArray<FPoint> UAlgorithmBPLibrary::ReconstructPath(TSharedPtr<FNode> Node)
{
	TArray<FPoint> Path;
	while (Node.IsValid())
	{
		Path.Add(Node->Point);
		Node = Node->Parent;
	}
	Algo::Reverse(Path);
	return Path;
}

void UAlgorithmBPLibrary::AStarPathfinding(const FPoint& Start, const FPoint& Goal, const TArray<int32>& Grid, int32 GridWidth, int32 GridHeight, TArray<FPoint>& PathPoints)
{
	TQueue<TSharedPtr<FNode>> OpenList;
	TMap<FPoint, double> GScore;
	TMap<FPoint, TSharedPtr<FNode>> AllNodes;
	FPoint ParentPoint;

	auto StartNode = MakeShared<FNode>(Start, 0.0, Heuristic(Start, Goal));
	OpenList.Enqueue(StartNode);
	GScore.Add(Start, 0.0);
	AllNodes.Add(Start, StartNode);

	while (!OpenList.IsEmpty())
	{
		TSharedPtr<FNode> Current;
		OpenList.Dequeue(Current);

		if (Current->Point == Goal)
		{
			PathPoints = ReconstructPath(Current);

			return;
		}

		if (Current->Parent != nullptr)
		{
			ParentPoint = Current->Parent->Point;
		}
		else
		{
			ParentPoint.X = Current->Point.X + 1;
			ParentPoint.Y = Current->Point.Y;
		}

		FVector2D ForwardVector(ParentPoint.X - Current->Point.X, ParentPoint.Y - Current->Point.Y);
		ForwardVector.Normalize();

		for (const FPoint& Neighbor : GetVehicleNeighbors(Current->Point, ForwardVector))
		{
			if (!IsValid(Neighbor, Grid, GridWidth, GridHeight)) continue;
			double TentativeGScore = GScore[Current->Point] + 1.0;
			if (!GScore.Contains(Neighbor) || TentativeGScore < GScore[Neighbor])
			{
				GScore.Add(Neighbor, TentativeGScore);
				auto NeighborNode = MakeShared<FNode>(Neighbor, TentativeGScore, Heuristic(Neighbor, Goal), Current);
				OpenList.Enqueue(NeighborNode);
				AllNodes.Add(Neighbor, NeighborNode);
			}
		}
	}

	PathPoints.Empty(); // No path found
}

void UAlgorithmBPLibrary::MergeClosePoints(const TArray<FVector>& PathPoints, TArray<FVector>& MergedPath, float DistanceThreshold)
{
	if (PathPoints.Num() < 2) return;

	MergedPath.Empty();
	MergedPath.Add(PathPoints[0]);

	for (int32 i = 1; i < PathPoints.Num(); ++i)
	{
		FVector PreviousPoint = MergedPath.Last();
		FVector CurrentPoint = PathPoints[i];

		float Distance = FVector::Dist(PreviousPoint, CurrentPoint);

		// If the points are close enough, merge them
		if (Distance < DistanceThreshold)
		{
			// Calculate the new merged point as the average of the two points
			FVector MergedPoint = (PreviousPoint + CurrentPoint) / 2.0f;
			MergedPath.Last() = MergedPoint;  // Update the last point in the merged path
		}
		else
		{
			// If the points are not close, add the current point to the merged path
			MergedPath.Add(CurrentPoint);
		}
	}
}

void UAlgorithmBPLibrary::AddMidpointIfFarApart(const TArray<FVector>& PathPoints, TArray<FVector>& InterpolatedPath, float MaxDistanceThreshold)
{
	if (PathPoints.Num() < 2)
	{
		InterpolatedPath = PathPoints;
		return;
	}

	InterpolatedPath.Empty();
	InterpolatedPath.Add(PathPoints[0]);

	for (int32 i = 1; i < PathPoints.Num(); ++i)
	{
		FVector PreviousPoint = InterpolatedPath.Last();
		FVector CurrentPoint = PathPoints[i];

		float Distance = FVector::Dist(PreviousPoint, CurrentPoint);

		if (Distance > MaxDistanceThreshold)
		{
			// Add a point halfway between PreviousPoint and CurrentPoint
			FVector MidPoint = (PreviousPoint + CurrentPoint) * 0.5f;
			InterpolatedPath.Add(MidPoint);
		}

		InterpolatedPath.Add(CurrentPoint);
	}
}

void UAlgorithmBPLibrary::RelaxPath(const TArray<FVector>& PathPoints, TArray<FVector>& RelaxedPath, int32 Iterations, float RelaxationFactor)
{
	if (PathPoints.Num() < 3 || Iterations < 1 || RelaxationFactor <= 0.0f || RelaxationFactor >= 1.0f)
	{
		RelaxedPath = PathPoints; // No relaxation needed or invalid parameters
		return;
	}

	RelaxedPath = PathPoints;

	for (int32 Iter = 0; Iter < Iterations; ++Iter)
	{
		TArray<FVector> TempPath = RelaxedPath;

		for (int32 i = 1; i < RelaxedPath.Num() - 1; ++i)
		{
			FVector PreviousPoint = TempPath[i - 1];
			FVector CurrentPoint = TempPath[i];
			FVector NextPoint = TempPath[i + 1];

			FVector NewPoint = (1 - RelaxationFactor) * CurrentPoint + RelaxationFactor * (PreviousPoint + NextPoint) / 2;
			RelaxedPath[i] = NewPoint;
		}
	}
}

TArray<double> UAlgorithmBPLibrary::CalculatePolynomialCoefficients(const TArray<double>& Values, int32 Degree)
{
	int32 N = Values.Num();

	if (N <= Degree) {
		UE_LOG(LogTemp, Error, TEXT("Not enough data points to fit the polynomial of the desired degree."));
		return TArray<double>();
	}

	Eigen::MatrixXd A(N, Degree + 1);
	Eigen::VectorXd B(N);

	for (int32 i = 0; i < N; ++i)
	{
		double t = i;
		B(i) = Values[i];
		for (int32 j = 0; j <= Degree; ++j)
		{
			A(i, j) = FMath::Pow(t, j);
		}
	}

	Eigen::VectorXd Coefficients = A.householderQr().solve(B);

	TArray<double> CoefficientsArray;
	for (int32 i = 0; i < Coefficients.size(); ++i)
	{
		CoefficientsArray.Add(Coefficients(i));
	}

	return CoefficientsArray;
}

double UAlgorithmBPLibrary::EvaluatePolynomial(const TArray<double>& Coefficients, double t)
{
	double Result = 0.0;
	for (int32 i = 0; i < Coefficients.Num(); ++i)
	{
		Result += Coefficients[i] * FMath::Pow(t, i);
	}
	return Result;
}

TArray<FVector> UAlgorithmBPLibrary::PredictFuturePositions(const TArray<FVector>& HistoryPositions, int32 Degree, int32 NumFuturePoints, double PredictionInterval)
{
	TArray<double> XValues;
	TArray<double> YValues;

	for (const FVector& Position : HistoryPositions) {
		XValues.Add(Position.X);
		YValues.Add(Position.Y);
	}

	TArray<double> XCoefficients = CalculatePolynomialCoefficients(XValues, Degree);
	TArray<double> YCoefficients = CalculatePolynomialCoefficients(YValues, Degree);

	TArray<FVector> FuturePositions;
	for (int32 i = 1; i <= NumFuturePoints; ++i)
	{
		double FutureIndex = (HistoryPositions.Num() - 1) + i * PredictionInterval;
		double FutureX = EvaluatePolynomial(XCoefficients, FutureIndex);
		double FutureY = EvaluatePolynomial(YCoefficients, FutureIndex);
		double FutureZ = 0.0;

		FVector FuturePosition = FVector(FutureX, FutureY, FutureZ);
		FuturePositions.Add(FuturePosition);
	}

	return FuturePositions;
}

TArray<FPoint> UAlgorithmBPLibrary::FloodFill(const FPoint& Start, const TArray<int32>& Grid, int32 GridWidth, int32 GridHeight)
{
	TArray<FPoint> FilledPoints;
	TQueue<FPoint> OpenList;
	TSet<FPoint> ClosedList;

	OpenList.Enqueue(Start);

	while (!OpenList.IsEmpty())
	{
		FPoint Current;
		OpenList.Dequeue(Current);

		if (!ClosedList.Contains(Current))
		{

			ClosedList.Add(Current);
			FilledPoints.Add(Current);

			TArray<FPoint> Neighbors = GetNeighbors(Current);
			for (const FPoint& Neighbor : Neighbors)
			{
				if (!ClosedList.Contains(Neighbor) && IsValid(Neighbor, Grid, GridWidth, GridHeight))
				{
					OpenList.Enqueue(Neighbor);
				}
			}
		}
	}

	return FilledPoints;
}