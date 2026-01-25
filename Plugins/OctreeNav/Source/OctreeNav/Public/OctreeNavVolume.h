// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "OctreeNavVolume.generated.h"

class UProceduralMeshComponent;
class NavNode;

// ====== 八叉树节点 ======
struct FOctreeNode
{
	FBox Bounds;                 // 该节点的包围盒（世界空间，轴对齐）
	bool bIsLeaf = true;         // 是否叶子
	bool bBlocked = false;       // 该叶子里是否有阻挡（粗判）
	FOctreeNode* Children[8]{};  // 非叶子时有效（8个子节点）

	FOctreeNode(const FBox& InBounds)
		: Bounds(InBounds)
	{
		for (int i = 0; i < 8; ++i) Children[i] = nullptr;
	}

	~FOctreeNode()
	{
		for (int i = 0; i < 8; ++i)
		{
			if (Children[i]) { delete Children[i]; Children[i] = nullptr; }
		}
	}
};

UENUM(BlueprintType)
enum class EPathPreference : uint8
{
	Ground		UMETA(DisplayName = "Ground"),
	Fly			UMETA(DisplayName = "Fly"),
	Near		UMETA(DisplayName = "Near")
};

UCLASS()
class OCTREENAV_API AOctreeNavVolume : public AActor
{
	GENERATED_BODY()

public:
	AOctreeNavVolume();

private:
	// Components
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SimpleOctaNavVolume3D", meta = (AllowPrivateAccess = "true"))
	USceneComponent* DefaultSceneComponent = nullptr;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SimpleOctaNavVolume3D", meta = (AllowPrivateAccess = "true"))
	UProceduralMeshComponent* ProceduralMesh = nullptr;

	// Grid settings
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SimpleOctaNavVolume3D|Pathfinding", meta = (AllowPrivateAccess = "true", ClampMin = 1))
	int32 DivisionsX = 10;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SimpleOctaNavVolume3D|Pathfinding", meta = (AllowPrivateAccess = "true", ClampMin = 1))
	int32 DivisionsY = 10;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SimpleOctaNavVolume3D|Pathfinding", meta = (AllowPrivateAccess = "true", ClampMin = 1))
	int32 DivisionsZ = 10;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SimpleOctaNavVolume3D|Pathfinding", meta = (AllowPrivateAccess = "true", ClampMin = 1))
	float DivisionSize = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SimpleOctaNavVolume3D|Pathfinding", meta = (AllowPrivateAccess = "true", ClampMin = 0, ClampMax = 2))
	int32 MinSharedNeighborAxes = 0;

	// Look settings
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SimpleOctaNavVolume3D|Aesthetics", meta = (AllowPrivateAccess = "true", ClampMin = 0))
	float LineThickness = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SimpleOctaNavVolume3D|Aesthetics", meta = (AllowPrivateAccess = "true"))
	FLinearColor Color = FLinearColor(0.0f, 1.0f, 0.0f, 0.5f);

	// ====== Octree settings ======
	// 最大深度（4~6 一般够用）
	UPROPERTY(EditAnywhere, Category = "SimpleOctaNavVolume3D|Octree", meta = (ClampMin = 1, ClampMax = 10))
	int32 OctreeMaxDepth = 5;

	// 与网格分辨率对齐：叶子格子尺寸不小于这个
	UPROPERTY(EditAnywhere, Category = "SimpleOctaNavVolume3D|Octree", meta = (ClampMin = 1.0))
	float OctreeMinCellSize = 100.0f; // 默认与 DivisionSize 相同

public:
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void Tick(float DeltaTime) override;

	NavNode* GetNode(FIntVector coordinates) const;

	UFUNCTION(BlueprintCallable, Category = "SimpleOctaNavVolume3D")
	bool FindPath(
		const FVector& start,
		const FVector& destination,
		const TArray<TEnumAsByte<EObjectTypeQuery> >& object_types,
		UClass* actor_class_filter,
		TArray<FVector>& out_path,
		AActor* InActor,
		float DetectionRadius = 34.f,
		float DetectionHalfHeight = 44.f
	);

	UFUNCTION(BlueprintCallable, Category = "SimpleOctaNavVolume3D")
	UPARAM(DisplayName = "Coordinates") FIntVector ConvertLocationToCoordinates(const FVector& location);

	UFUNCTION(BlueprintCallable, Category = "SimpleOctaNavVolume3D")
	UPARAM(DisplayName = "World Location") FVector ConvertCoordinatesToLocation(const FIntVector& coordinates);

	UFUNCTION(BlueprintPure, Category = "SimpleOctaNavVolume3D")
	FORCEINLINE int32 GetTotalDivisions() const { return DivisionsX * DivisionsY * DivisionsZ; }

	UFUNCTION(BlueprintPure, Category = "SimpleOctaNavVolume3D")
	FORCEINLINE int32 GetDivisionsX() const { return DivisionsX; }

	UFUNCTION(BlueprintPure, Category = "SimpleOctaNavVolume3D")
	FORCEINLINE int32 GetDivisionsY() const { return DivisionsY; }

	UFUNCTION(BlueprintPure, Category = "SimpleOctaNavVolume3D")
	FORCEINLINE int32 GetDivisionsZ() const { return DivisionsZ; }

	UFUNCTION(BlueprintPure, Category = "SimpleOctaNavVolume3D")
	FORCEINLINE float GetDivisionSize() const { return DivisionSize; }

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	inline float GetGridSizeX() const { return DivisionsX * DivisionSize; }
	inline float GetGridSizeY() const { return DivisionsY * DivisionSize; }
	inline float GetGridSizeZ() const { return DivisionsZ * DivisionSize; }

private:
	// mesh helpers
	void CreateLine(const FVector& start, const FVector& end, const FVector& normal, TArray<FVector>& vertices, TArray<int32>& triangles);
	bool AreCoordinatesValid(const FIntVector& coordinates) const;
	void ClampCoordinates(FIntVector& coordinates) const;

	// precise overlap (capsule)
	bool IsActorOverlap(
		float AgentRadius,
		float AgentHalfHeight,
		AActor* InActor,
		FVector InLocation,
		const TArray<TEnumAsByte<EObjectTypeQuery> >& object_types,
		UClass* actor_class_filter) const;

	NavNode* FindNearestFreeNode(
		NavNode* FromNode,
		AActor* InActor,
		const TArray<TEnumAsByte<EObjectTypeQuery> >& object_types,
		UClass* actor_class_filter,
		float DetectionRadius,
		float DetectionHalfHeight) const;

private:
	// ====== Octree 构建与查询 ======
	FOctreeNode* OctreeRoot = nullptr;

	// 整个体积（世界空间，轴对齐）
	FBox GetWorldAlignedVolumeBox() const;

	// 递归构建
	FOctreeNode* BuildOctree(const FBox& Bounds, int32 Depth,
		const TArray<TEnumAsByte<EObjectTypeQuery> >& object_types,
		UClass* actor_class_filter);

	// 盒体是否被阻挡（粗判）
	bool BoxHasBlocking(const FBox& Bounds,
		const TArray<TEnumAsByte<EObjectTypeQuery> >& object_types,
		UClass* actor_class_filter) const;

	// 查询某一点所在叶子是否阻挡
	bool QueryPointBlocked(const FVector& WorldPoint) const;

	// 用于释放
	void DestroyOctree();

private:
	// nodes for A*
	NavNode* Nodes = nullptr;
};
