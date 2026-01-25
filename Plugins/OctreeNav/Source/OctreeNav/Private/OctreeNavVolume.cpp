#include "OctreeNavVolume.h"
#include "ProceduralMeshComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "UObject/ConstructorHelpers.h"
#include "Kismet/KismetMathLibrary.h"
#include "Kismet/KismetSystemLibrary.h"
#include "NavNode.h"
#include <set>
#include <unordered_map>

#include "Components/CapsuleComponent.h"
#include "GameFramework/Character.h"
#include "GameFramework/Actor.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"
#include "CollisionQueryParams.h"
#include "DrawDebugHelpers.h"

static UMaterial* GridMaterial = nullptr;

AOctreeNavVolume::AOctreeNavVolume()
{
	PrimaryActorTick.bCanEverTick = true;

	DefaultSceneComponent = CreateDefaultSubobject<USceneComponent>("DefaultSceneComponent");
	SetRootComponent(DefaultSceneComponent);

	ProceduralMesh = CreateDefaultSubobject<UProceduralMeshComponent>("ProceduralMesh");
	ProceduralMesh->SetupAttachment(GetRootComponent());
	ProceduralMesh->CastShadow = false;
	ProceduralMesh->SetEnableGravity(false);
	ProceduralMesh->bApplyImpulseOnDamage = false;
	ProceduralMesh->SetGenerateOverlapEvents(false);
	ProceduralMesh->CanCharacterStepUpOn = ECanBeCharacterBase::ECB_No;
	ProceduralMesh->SetCollisionProfileName("NoCollision");
	ProceduralMesh->bHiddenInGame = false;

	// 默认隐藏整个Actor（只显示线框网格）
	SetActorHiddenInGame(true);

	static ConstructorHelpers::FObjectFinder<UMaterial> materialFinder(TEXT("Material'/OctreeNav/M_Nav.M_Nav'"));
	GridMaterial = materialFinder.Object;
}

void AOctreeNavVolume::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	TArray<FVector> vertices;
	TArray<int32> triangles;

	FVector start = FVector::ZeroVector;
	FVector end = FVector::ZeroVector;

	// X方向线（沿Y拉）
	for (int32 z = 0; z <= DivisionsZ; ++z)
	{
		start.Z = end.Z = z * DivisionSize;
		for (int32 x = 0; x <= DivisionsX; ++x)
		{
			start.X = end.X = (x * DivisionSize);
			end.Y = GetGridSizeY();
			CreateLine(start, end, FVector::UpVector, vertices, triangles);
		}
	}

	start = FVector::ZeroVector;
	end = FVector::ZeroVector;

	// Y方向线（沿X拉）
	for (int32 z = 0; z <= DivisionsZ; ++z)
	{
		start.Z = end.Z = z * DivisionSize;
		for (int32 y = 0; y <= DivisionsY; ++y)
		{
			start.Y = end.Y = (y * DivisionSize);
			end.X = GetGridSizeX();
			CreateLine(start, end, FVector::UpVector, vertices, triangles);
		}
	}

	start = FVector::ZeroVector;
	end = FVector::ZeroVector;

	// Z方向线（沿Z拉）
	for (int32 x = 0; x <= DivisionsX; ++x)
	{
		start.X = end.X = x * DivisionSize;
		for (int32 y = 0; y <= DivisionsY; ++y)
		{
			start.Y = end.Y = (y * DivisionSize);
			end.Z = GetGridSizeZ();
			CreateLine(start, end, FVector::ForwardVector, vertices, triangles);
		}
	}

	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<FColor> Colors;
	TArray<FProcMeshTangent> Tangents;
	ProceduralMesh->CreateMeshSection(0, vertices, triangles, Normals, UVs, Colors, Tangents, false);

	UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(GridMaterial, this);
	MID->SetVectorParameterValue("Color", Color);
	MID->SetScalarParameterValue("Opacity", Color.A);
	ProceduralMesh->SetMaterial(0, MID);
}

void AOctreeNavVolume::BeginPlay()
{
	Super::BeginPlay();

	// 1) 分配A*节点（原逻辑）
	Nodes = new NavNode[GetTotalDivisions()];

	auto addNeighborIfValid = [&](NavNode* node, const FIntVector& neighbor_coordinates)
		{
			if (AreCoordinatesValid(neighbor_coordinates))
			{
				int32 sharedAxes = 0;
				if (node->Coordinates.X == neighbor_coordinates.X) ++sharedAxes;
				if (node->Coordinates.Y == neighbor_coordinates.Y) ++sharedAxes;
				if (node->Coordinates.Z == neighbor_coordinates.Z) ++sharedAxes;

				if (sharedAxes >= MinSharedNeighborAxes && sharedAxes < 3)
				{
					node->Neighbors.push_back(GetNode(neighbor_coordinates));
				}
			}
		};

	for (int32 z = 0; z < DivisionsZ; ++z)
		for (int32 y = 0; y < DivisionsY; ++y)
			for (int32 x = 0; x < DivisionsX; ++x)
			{
				NavNode* node = GetNode(FIntVector(x, y, z));
				node->Coordinates = FIntVector(x, y, z);

				// Above
				addNeighborIfValid(node, FIntVector(x + 1, y - 1, z + 1));
				addNeighborIfValid(node, FIntVector(x + 1, y + 0, z + 1));
				addNeighborIfValid(node, FIntVector(x + 1, y + 1, z + 1));

				addNeighborIfValid(node, FIntVector(x + 0, y - 1, z + 1));
				addNeighborIfValid(node, FIntVector(x + 0, y + 0, z + 1));
				addNeighborIfValid(node, FIntVector(x + 0, y + 1, z + 1));

				addNeighborIfValid(node, FIntVector(x - 1, y - 1, z + 1));
				addNeighborIfValid(node, FIntVector(x - 1, y + 0, z + 1));
				addNeighborIfValid(node, FIntVector(x - 1, y + 1, z + 1));

				// Middle
				addNeighborIfValid(node, FIntVector(x + 1, y - 1, z + 0));
				addNeighborIfValid(node, FIntVector(x + 1, y + 0, z + 0));
				addNeighborIfValid(node, FIntVector(x + 1, y + 1, z + 0));

				addNeighborIfValid(node, FIntVector(x + 0, y - 1, z + 0));
				addNeighborIfValid(node, FIntVector(x + 0, y + 1, z + 0));

				addNeighborIfValid(node, FIntVector(x - 1, y - 1, z + 0));
				addNeighborIfValid(node, FIntVector(x - 1, y + 0, z + 0));
				addNeighborIfValid(node, FIntVector(x - 1, y + 1, z + 0));

				// Below
				addNeighborIfValid(node, FIntVector(x + 1, y - 1, z - 1));
				addNeighborIfValid(node, FIntVector(x + 1, y + 0, z - 1));
				addNeighborIfValid(node, FIntVector(x + 1, y + 1, z - 1));

				addNeighborIfValid(node, FIntVector(x + 0, y - 1, z - 1));
				addNeighborIfValid(node, FIntVector(x + 0, y + 0, z - 1));
				addNeighborIfValid(node, FIntVector(x + 0, y + 1, z - 1));

				addNeighborIfValid(node, FIntVector(x - 1, y - 1, z - 1));
				addNeighborIfValid(node, FIntVector(x - 1, y + 0, z - 1));
				addNeighborIfValid(node, FIntVector(x - 1, y + 1, z - 1));
			}

	// 2) 构建八叉树（粗判）
	DestroyOctree();
	OctreeMinCellSize = FMath::Max(OctreeMinCellSize, DivisionSize); // 与网格分辨率对齐（下限）
	const FBox WorldBox = GetWorldAlignedVolumeBox();
	OctreeRoot = BuildOctree(WorldBox, 0, /*object types later*/ TArray<TEnumAsByte<EObjectTypeQuery>>(), nullptr);
}

void AOctreeNavVolume::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	DestroyOctree();

	delete[] Nodes;
	Nodes = nullptr;

	Super::EndPlay(EndPlayReason);
}

void AOctreeNavVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

// ====== A* ======
bool AOctreeNavVolume::FindPath(
	const FVector& start,
	const FVector& destination,
	const TArray<TEnumAsByte<EObjectTypeQuery> >& object_types,
	UClass* actor_class_filter,
	TArray<FVector>& out_path,
	AActor* InActor,
	float DetectionRadius,
	float DetectionHalfHeight)
{
	out_path.Empty();

	std::multiset<NavNode*, NodeCompare> openSet;
	std::unordered_map<NavNode*, NavNode*> cameFrom;
	std::unordered_map<NavNode*, float> gScores;

	NavNode* startNode = GetNode(ConvertLocationToCoordinates(start));
	NavNode* endNode = GetNode(ConvertLocationToCoordinates(destination));

	// 终点粗判（八叉树）
	if (OctreeRoot && QueryPointBlocked(ConvertCoordinatesToLocation(endNode->Coordinates)))
	{
		// 终点盒子被粗判为阻挡 → 回退最近可用格
		NavNode* NewGoal = FindNearestFreeNode(endNode, InActor, object_types, actor_class_filter, DetectionRadius, DetectionHalfHeight);
		if (NewGoal) endNode = NewGoal;
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("No free goal node by octree coarse test."));
			return false;
		}
	}

	// 终点精检（胶囊）
	if (IsActorOverlap(DetectionRadius, DetectionHalfHeight, nullptr,
		ConvertCoordinatesToLocation(endNode->Coordinates), object_types, actor_class_filter))
	{
		NavNode* NewGoal = FindNearestFreeNode(endNode, InActor, object_types, actor_class_filter, DetectionRadius, DetectionHalfHeight);
		if (NewGoal) endNode = NewGoal;
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("No free goal node by precise overlap."));
			return false;
		}
	}

	// 启发 & 邻居代价（维持你原始的简化版本）
	auto h = [endNode](NavNode* node)
		{
			return FVector::Distance(FVector(endNode->Coordinates), FVector(node->Coordinates));
		};
	auto distance = [](NavNode* a, NavNode* b)
		{
			return FVector::Distance(FVector(a->Coordinates), FVector(b->Coordinates));
		};
	auto gScore = [&gScores](NavNode* node)
		{
			auto it = gScores.find(node);
			return it != gScores.end() ? it->second : FLT_MAX;
		};

	startNode->FScore = h(startNode);
	openSet.insert(startNode);
	gScores[startNode] = 0.f;

	while (!openSet.empty())
	{
		NavNode* current = *openSet.begin();

		if (current == endNode)
		{
			// 重建路径
			out_path.Add(ConvertCoordinatesToLocation(current->Coordinates));
			while (true)
			{
				auto it = cameFrom.find(current);
				if (it != cameFrom.end())
				{
					current = it->second;
					out_path.Insert(ConvertCoordinatesToLocation(current->Coordinates), 0);
				}
				else
				{
					return true;
				}
			}
		}

		openSet.erase(openSet.begin());

		for (NavNode* neighbor : current->Neighbors)
		{
			// 先用八叉树粗判剪枝（可大幅减少精检次数）
			const FVector worldLocation = ConvertCoordinatesToLocation(neighbor->Coordinates);
			if (OctreeRoot && QueryPointBlocked(worldLocation))
			{
				continue; // 粗判已被阻挡，跳过
			}

			const float tentative = gScore(current) + distance(current, neighbor);

			if (tentative < gScore(neighbor))
			{
				// 再做一次精检（胶囊）
				bool traversable = !IsActorOverlap(DetectionRadius, DetectionHalfHeight, InActor, worldLocation, object_types, actor_class_filter);
				if (!traversable) continue;

				cameFrom[neighbor] = current;
				gScores[neighbor] = tentative;
				const float fScore = tentative + h(neighbor);
				neighbor->FScore = fScore;
				openSet.insert(neighbor);
			}
		}
	}

	return false;
}

// ====== 坐标转换 & 网格工具 ======
FIntVector AOctreeNavVolume::ConvertLocationToCoordinates(const FVector& location)
{
	FIntVector coordinates;
	const FVector gridSpaceLocation = UKismetMathLibrary::InverseTransformLocation(GetActorTransform(), location);

	coordinates.X = DivisionsX * (gridSpaceLocation.X / GetGridSizeX());
	coordinates.Y = DivisionsY * (gridSpaceLocation.Y / GetGridSizeY());
	coordinates.Z = DivisionsZ * (gridSpaceLocation.Z / GetGridSizeZ());

	ClampCoordinates(coordinates);
	return coordinates;
}

FVector AOctreeNavVolume::ConvertCoordinatesToLocation(const FIntVector& coordinates)
{
	FIntVector clamped = coordinates;
	ClampCoordinates(clamped);

	FVector gridSpaceLocation(0, 0, 0);
	gridSpaceLocation.X = (clamped.X * DivisionSize) + (DivisionSize * 0.5f);
	gridSpaceLocation.Y = (clamped.Y * DivisionSize) + (DivisionSize * 0.5f);
	gridSpaceLocation.Z = (clamped.Z * DivisionSize) + (DivisionSize * 0.5f);

	return UKismetMathLibrary::TransformLocation(GetActorTransform(), gridSpaceLocation);
}

void AOctreeNavVolume::CreateLine(const FVector& start, const FVector& end, const FVector& normal, TArray<FVector>& vertices, TArray<int32>& triangles)
{
	const float half = LineThickness * 0.5f;
	FVector line = end - start;
	line.Normalize();

	auto pushQuad = [&](const FVector& thicknessDir)
		{
			triangles.Add(vertices.Num() + 2);
			triangles.Add(vertices.Num() + 1);
			triangles.Add(vertices.Num() + 0);

			triangles.Add(vertices.Num() + 2);
			triangles.Add(vertices.Num() + 3);
			triangles.Add(vertices.Num() + 1);

			vertices.Add(start + (thicknessDir * half));
			vertices.Add(end + (thicknessDir * half));
			vertices.Add(start - (thicknessDir * half));
			vertices.Add(end - (thicknessDir * half));
		};

	FVector d1 = UKismetMathLibrary::Cross_VectorVector(line, normal);
	FVector d2 = UKismetMathLibrary::Cross_VectorVector(line, d1);
	pushQuad(d1);
	pushQuad(d2);
}


bool AOctreeNavVolume::AreCoordinatesValid(const FIntVector& c) const
{
	return c.X >= 0 && c.X < DivisionsX &&
		c.Y >= 0 && c.Y < DivisionsY &&
		c.Z >= 0 && c.Z < DivisionsZ;
}

//限制
void AOctreeNavVolume::ClampCoordinates(FIntVector& c) const
{
	c.X = FMath::Clamp(c.X, 0, DivisionsX - 1);
	c.Y = FMath::Clamp(c.Y, 0, DivisionsY - 1);
	c.Z = FMath::Clamp(c.Z, 0, DivisionsZ - 1);
}

// 物体碰撞检测（胶囊体检测）
bool AOctreeNavVolume::IsActorOverlap(
	float AgentRadius,
	float AgentHalfHeight,
	AActor* InActor,
	FVector InLocation,
	const TArray<TEnumAsByte<EObjectTypeQuery> >& object_types,
	UClass* actor_class_filter) const
{
	FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(IsActorOverlap), false, InActor);
	if (InActor) { QueryParams.AddIgnoredActor(InActor); }

	FCollisionObjectQueryParams ObjectParams;
	for (auto ObjType : object_types)
	{
		ObjectParams.AddObjectTypesToQuery(UEngineTypes::ConvertToCollisionChannel(ObjType));
	}

	const FCollisionShape Capsule = FCollisionShape::MakeCapsule(AgentRadius, AgentHalfHeight);

	TArray<FOverlapResult> Overlaps;
	bool bHasOverlap = GetWorld()->OverlapMultiByObjectType(
		Overlaps, InLocation, FQuat::Identity, ObjectParams, Capsule, QueryParams);

	if (!bHasOverlap) return false;

	for (const FOverlapResult& R : Overlaps)
	{
		if (AActor* Hit = R.GetActor())
		{
			if (!actor_class_filter || Hit->IsA(actor_class_filter))
			{
				return true;
			}
		}
	}
	return false;
}

// ====== 邻近可用节点（原逻辑保留） ======
NavNode* AOctreeNavVolume::FindNearestFreeNode(
	NavNode* FromNode,
	AActor* InActor,
	const TArray<TEnumAsByte<EObjectTypeQuery> >& object_types,
	UClass* actor_class_filter,
	float DetectionRadius,
	float DetectionHalfHeight) const
{
	TQueue<NavNode*> Q;
	TSet<NavNode*>   Visited;
	Q.Enqueue(FromNode);
	Visited.Add(FromNode);

	while (!Q.IsEmpty())
	{
		NavNode* Cur = nullptr;
		Q.Dequeue(Cur);

		const FVector WL = const_cast<AOctreeNavVolume*>(this)->ConvertCoordinatesToLocation(Cur->Coordinates);

		// 先八叉树粗判
		if (OctreeRoot && QueryPointBlocked(WL))
		{
			// 被粗判为阻挡，跳过
		}
		else
		{
			// 再精检
			const bool bBlocked = IsActorOverlap(DetectionRadius, DetectionHalfHeight, InActor, WL, object_types, actor_class_filter);
			if (!bBlocked) return Cur;
		}

		for (NavNode* N : Cur->Neighbors)
		{
			if (!Visited.Contains(N))
			{
				Visited.Add(N);
				Q.Enqueue(N);
			}
		}
	}
	return nullptr;
}

NavNode* AOctreeNavVolume::GetNode(FIntVector coordinates) const
{
	ClampCoordinates(coordinates);

	const int32 divisionPerLevel = DivisionsX * DivisionsY;
	const int32 index = (coordinates.Z * divisionPerLevel) + (coordinates.Y * DivisionsX) + coordinates.X;
	return &Nodes[index];
}

// ====== Octree 构建 / 查询 ======
FBox AOctreeNavVolume::GetWorldAlignedVolumeBox() const
{
	// 假设体积轴对齐（未考虑旋转），将局部(0,0,0)~(SizeX,SizeY,SizeZ) 映射到世界
	const FVector WorldMin = UKismetMathLibrary::TransformLocation(GetActorTransform(), FVector(0, 0, 0));
	const FVector WorldMax = UKismetMathLibrary::TransformLocation(GetActorTransform(), FVector(GetGridSizeX(), GetGridSizeY(), GetGridSizeZ()));
	FVector Min(FMath::Min(WorldMin.X, WorldMax.X), FMath::Min(WorldMin.Y, WorldMax.Y), FMath::Min(WorldMin.Z, WorldMax.Z));
	FVector Max(FMath::Max(WorldMin.X, WorldMax.X), FMath::Max(WorldMin.Y, WorldMax.Y), FMath::Max(WorldMin.Z, WorldMax.Z));
	return FBox(Min, Max);
}

bool AOctreeNavVolume::BoxHasBlocking(const FBox& Bounds,
	const TArray<TEnumAsByte<EObjectTypeQuery> >& object_types,
	UClass* actor_class_filter) const
{
	// 只做“是否有重叠”的快速判定（不区分类型以外细节）
	FCollisionObjectQueryParams ObjParams;
	for (auto ObjType : object_types)
	{
		ObjParams.AddObjectTypesToQuery(UEngineTypes::ConvertToCollisionChannel(ObjType));
	}

	const FVector Center = Bounds.GetCenter();
	const FVector Extent = Bounds.GetExtent();

	FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(OctreeBoxTest), false);
	bool bHit = GetWorld()->OverlapAnyTestByObjectType(
		Center, FQuat::Identity, ObjParams, FCollisionShape::MakeBox(Extent), QueryParams);

	// 如果你只想统计“阻挡类”，可以在这里进一步用 Trace 等做过滤；当前我们把“有指定ObjType重叠”视为阻挡。
	return bHit;
}

FOctreeNode* AOctreeNavVolume::BuildOctree(
	const FBox& Bounds, int32 Depth,
	const TArray<TEnumAsByte<EObjectTypeQuery> >& object_types,
	UClass* actor_class_filter)
{
	FOctreeNode* Node = new FOctreeNode(Bounds);

	// 叶子条件：到达深度 或 尺寸足够小（与网格体素相当）
	const FVector Size = Bounds.GetSize();
	const float   CellMax = FMath::Max3(Size.X, Size.Y, Size.Z);
	const bool    bLeafBySize = (CellMax <= OctreeMinCellSize + KINDA_SMALL_NUMBER);

	if (Depth >= OctreeMaxDepth || bLeafBySize)
	{
		Node->bIsLeaf = true;
		Node->bBlocked = BoxHasBlocking(Bounds, object_types, actor_class_filter);
		return Node;
	}

	// 非叶子：划分8个子盒
	Node->bIsLeaf = false;
	const FVector C = Bounds.GetCenter();
	const FVector Min = Bounds.Min;
	const FVector Max = Bounds.Max;

	// 八个子盒（按XYZ低/高组合）
	FBox ChildBoxes[8] = {
		FBox(FVector(Min.X, Min.Y, Min.Z), FVector(C.X,   C.Y,   C.Z)),   // 0
		FBox(FVector(C.X,  Min.Y, Min.Z), FVector(Max.X,  C.Y,   C.Z)),   // 1
		FBox(FVector(Min.X, C.Y,  Min.Z), FVector(C.X,    Max.Y,  C.Z)),  // 2
		FBox(FVector(C.X,  C.Y,  Min.Z), FVector(Max.X,   Max.Y,  C.Z)),  // 3
		FBox(FVector(Min.X, Min.Y, C.Z), FVector(C.X,     C.Y,    Max.Z)),// 4
		FBox(FVector(C.X,  Min.Y, C.Z), FVector(Max.X,    C.Y,    Max.Z)),// 5
		FBox(FVector(Min.X, C.Y,  C.Z), FVector(C.X,      Max.Y,  Max.Z)),// 6
		FBox(FVector(C.X,  C.Y,  C.Z), FVector(Max.X,     Max.Y,  Max.Z)) // 7
	};

	bool bAllBlocked = true;
	for (int i = 0; i < 8; ++i)
	{
		Node->Children[i] = BuildOctree(ChildBoxes[i], Depth + 1, object_types, actor_class_filter);
		bAllBlocked &= (Node->Children[i]->bIsLeaf && Node->Children[i]->bBlocked);
	}

	// 父节点不直接设置阻挡，但可以根据需要做汇总；这里父节点不使用 bBlocked。
	return Node;
}

bool AOctreeNavVolume::QueryPointBlocked(const FVector& WorldPoint) const
{
	if (!OctreeRoot) return false;

	const FOctreeNode* Node = OctreeRoot;
	while (Node && !Node->bIsLeaf)
	{
		const FVector C = Node->Bounds.GetCenter();
		const bool hiX = WorldPoint.X >= C.X;
		const bool hiY = WorldPoint.Y >= C.Y;
		const bool hiZ = WorldPoint.Z >= C.Z;
		int idx = (hiX ? 1 : 0) | ((hiY ? 1 : 0) << 1) | ((hiZ ? 1 : 0) << 2);
		Node = Node->Children[idx];
	}
	return (Node ? Node->bBlocked : false);
}

void AOctreeNavVolume::DestroyOctree()
{
	if (OctreeRoot)
	{
		delete OctreeRoot;
		OctreeRoot = nullptr;
	}
}
