// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Kismet/BlueprintFunctionLibrary.h"
#include "AlgorithmBPLibrary.generated.h"

USTRUCT(BlueprintType)
struct FPoint
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite)
	int32 X;

	UPROPERTY(BlueprintReadWrite)
	int32 Y;

	FPoint(): X(0), Y(0) {}
	FPoint(int32 InX, int32 InY) : X(InX), Y(InY) {}

	bool operator==(const FPoint& Other) const
	{
		return X == Other.X && Y == Other.Y;
	}

	bool operator!=(const FPoint& Other) const
	{
		return !(*this == Other);
	}
};

FORCEINLINE uint32 GetTypeHash(const FPoint& Point)
{
	return HashCombine(::GetTypeHash(Point.X), ::GetTypeHash(Point.Y));
}

UCLASS()
class UAlgorithmBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_UCLASS_BODY()

public:

	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	static void AStarPathfinding(const FPoint& Start, const FPoint& Goal, const TArray<int32>& Grid, int32 GridWidth, int32 GridHeight, TArray<FPoint>& PathPoints);

	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	static TArray<FPoint> FloodFill(const FPoint& Start, const TArray<int32>& Grid, int32 GridWidth, int32 GridHeight);

	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	static void MergeClosePoints(const TArray<FVector>& PathPoints, TArray<FVector>& MergedPath, float DistanceThreshold);
	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	static void AddMidpointIfFarApart(const TArray<FVector>& PathPoints, TArray<FVector>& InterpolatedPath, float MaxDistanceThreshold);
	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	static void RelaxPath(const TArray<FVector>& PathPoints, TArray<FVector>& RelaxedPath, int32 Iterations, float RelaxationFactor);

	static TArray<double> CalculatePolynomialCoefficients(const TArray<double>& Values, int32 Degree);
	static double EvaluatePolynomial(const TArray<double>& Coefficients, double t);
	UFUNCTION(BlueprintCallable, Category = "Math|Regression")
	static TArray<FVector> PredictFuturePositions(const TArray<FVector>& HistoryPositions, int32 Degree, int32 NumFuturePoints, double PredictionInterval = 1);

private:
	struct FNode
	{
		FPoint Point;
		double G;
		double H;
		TSharedPtr<FNode> Parent;

		FNode(const FPoint& InPoint, double InG, double InH, TSharedPtr<FNode> InParent = nullptr)
			: Point(InPoint), G(InG), H(InH), Parent(InParent) {}

		double F() const { return G + H; }
	};

	struct FCompareNode
	{
		bool operator()(const TSharedPtr<FNode>& Lhs, const TSharedPtr<FNode>& Rhs) const
		{
			return Lhs->F() > Rhs->F();
		}
	};
	
	static double Heuristic(const FPoint& A, const FPoint& B);
	static bool IsValid(const FPoint& P, const TArray<int32>& Grid, int32 GridWidth, int32 GridHeight);
	static TArray<FPoint> GetNeighbors(const FPoint& P);
	static TArray<FPoint> ReconstructPath(TSharedPtr<FNode> Node);

};
