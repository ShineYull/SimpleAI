// Copyright Epic Games, Inc. All Rights Reserved.

#include "AlgorithmBPLibrary.h"
#include "Algorithm.h"

#include "Containers/Queue.h"
#include "Containers/UnrealString.h"
#include "Algo/Reverse.h"
#include <Eigen/Dense>

UAlgorithmBPLibrary::UAlgorithmBPLibrary(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{

}

double UAlgorithmBPLibrary::Heuristic(const FPoint& A, const FPoint& B, bool bUseOctileHeuristic)
{
	if (bUseOctileHeuristic)
	{
		// Octile heuristic (diagonal movement allowed)
		double dx = FMath::Abs(A.X - B.X);
		double dy = FMath::Abs(A.Y - B.Y);
		double D = 1.0;
		double D2 = 1.41421356237;
		return D * (dx + dy) + (D2 - 2 * D) * FMath::Min(dx, dy);
	}
	else
	{
		return FMath::Abs(A.X - B.X) + FMath::Abs(A.Y - B.Y); // Manhattan distance
	}
}

bool UAlgorithmBPLibrary::IsValid(const FPoint& P, const TArray<int32>& Grid, int32 GridWidth, int32 GridHeight)
{
	return P.Y >= 0 && P.Y < GridWidth && P.X >= 0 && P.X < GridHeight && Grid[P.X * GridWidth + P.Y] == 0;
}

TArray<FPoint> UAlgorithmBPLibrary::GetNeighbors(const FPoint& P, bool bUseOctileHeuristic)
{
	TArray<FPoint> Neighbors = {
	FPoint(P.X + 1, P.Y), FPoint(P.X - 1, P.Y),
	FPoint(P.X, P.Y + 1), FPoint(P.X, P.Y - 1)
	};

	if (bUseOctileHeuristic)
	{
		Neighbors.Append({
			FPoint(P.X + 1, P.Y + 1), FPoint(P.X + 1, P.Y - 1),
			FPoint(P.X - 1, P.Y + 1), FPoint(P.X - 1, P.Y - 1)
			});
	}

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

TArray<FPoint> UAlgorithmBPLibrary::AStarPathfinding(const FPoint& Start, const FPoint& Goal, const TArray<int32>& Grid, int32 GridWidth, int32 GridHeight, bool bUseOctileHeuristic)
{
	TQueue<TSharedPtr<FNode>> OpenList;
	TMap<FPoint, double> GScore;
	TMap<FPoint, TSharedPtr<FNode>> AllNodes;

	auto StartNode = MakeShared<FNode>(Start, 0.0, Heuristic(Start, Goal, bUseOctileHeuristic));
	OpenList.Enqueue(StartNode);
	GScore.Add(Start, 0.0);
	AllNodes.Add(Start, StartNode);

	while (!OpenList.IsEmpty())
	{
		TSharedPtr<FNode> Current;
		OpenList.Dequeue(Current);

		if (Current->Point == Goal)
		{
			return ReconstructPath(Current);
		}

		for (const FPoint& Neighbor : GetNeighbors(Current->Point, bUseOctileHeuristic))
		{
			if (!IsValid(Neighbor, Grid, GridWidth, GridHeight)) continue;
			double TentativeGScore = GScore[Current->Point] + 1.0;
			if (!GScore.Contains(Neighbor) || TentativeGScore < GScore[Neighbor])
			{
				GScore.Add(Neighbor, TentativeGScore);
				auto NeighborNode = MakeShared<FNode>(Neighbor, TentativeGScore, Heuristic(Neighbor, Goal, bUseOctileHeuristic), Current);
				OpenList.Enqueue(NeighborNode);
				AllNodes.Add(Neighbor, NeighborNode);
			}
		}
	}

	return TArray<FPoint>(); // No path found
}

TArray<double> UAlgorithmBPLibrary::CalculatePolynomialCoefficients(const TArray<double>& Values, int32 Degree)
{
	int32 N = Values.Num();
	if (N <= Degree)
	{
		UE_LOG(LogTemp, Error, TEXT("Not enough data points to fit the polynomial of the desired degree."));
		return TArray<double>();
	}
	
	Eigen::MatrixXd A(N, Degree + 1);
	Eigen::VectorXd B(N);
	
	for (int32 i = 0; i<N; ++i)
	{
		double t = i;
		B(i) = Values[i];
		for (int32 j =0; j <= Degree; ++j)
		{
			A(i,j) = FMath::Pow(t,j);
		}
	}
	
	Eigen::VectorXd Coefficients = A.householderQr().solve(B);
	
	TArray<double> CoefficientsArray;
	for (int32 i=0; i< Coefficients.size(); ++i)
	{
		CoefficientsArray.Add(Coefficients(i));
	}
	
	return CoefficientsArray;
}

double UAlgorithmBPLibrary::EvaluatePolynomial(const TArray<double>& Coefficients, double t)
{
	double Result = 0.0;
	
	for (int32 i=0; i<Coefficients.Num(); ++i)
	{
		Result += Coefficients[i] * FMath::Pow(t,i);
	}

	return Result;
}

TArray<FVector> UAlgorithmBPLibrary::PredictFuturePositions(const TArray<FVector>& HistoryPositions, int32 Degree,
	int32 NumFuturePoints, double PredictionInterval)
{
	TArray<double> XValues;
	TArray<double> YValues;
	
	for (const FVector& Position:HistoryPositions)
	{
		XValues.Add(Position.X);
		YValues.Add(Position.Y);
	}
	
	TArray<double> XCoefficients = CalculatePolynomialCoefficients(XValues,Degree);
	TArray<double> YCoefficients = CalculatePolynomialCoefficients(YValues,Degree);
	
	TArray<FVector> FuturePositions;
	for (int32 i=1; i<=NumFuturePoints; ++i)
	{
		double FutureIndex =(HistoryPositions.Num()- 1)+i*
		PredictionInterval;
		double FutureX = EvaluatePolynomial(XCoefficients,FutureIndex);
		double FutureY =EvaluatePolynomial(YCoefficients,FutureIndex);
		double FutureZ =0.0;
		FVector FuturePosition = FVector(FutureX, FutureY,FutureZ);
		FuturePositions.Add(FuturePosition);
	}
	
	return FuturePositions;
}

