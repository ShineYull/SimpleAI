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
	static TArray<FPoint> AStarPathfinding(const FPoint& Start, const FPoint& Goal, const TArray<int32>& Grid, int32 GridWidth, int32 GridHeight, bool bUseOctileHeuristic = true);

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

	static double Heuristic(const FPoint& A, const FPoint& B, bool bUseOctileHeuristic);
	static bool IsValid(const FPoint& P, const TArray<int32>& Grid, int32 GridWidth, int32 GridHeight);
	static TArray<FPoint> GetNeighbors(const FPoint& P, bool bUseOctileHeuristic);
	static TArray<FPoint> ReconstructPath(TSharedPtr<FNode> Node);
};
