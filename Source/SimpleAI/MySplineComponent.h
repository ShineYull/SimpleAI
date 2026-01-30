// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SplineComponent.h"
#include "SplinePointData.h"
#include "MySplineComponent.generated.h"


UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class SIMPLEAI_API UMySplineComponent : public USplineComponent
{
	GENERATED_BODY()
	
public:
	UMySplineComponent();

protected:
	virtual void OnRegister() override;
	
public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category ="Spline Point Data")
	TArray<FSplinePointData> SplinePointDataArray;
	
	UFUNCTION(BlueprintCallable, Category ="Spline Point Data")
	void InitializeSplinePointData();
	
	UFUNCTION(BlueprintCallable,Category = "Spline Point Data")
	bool GetMoveForwardAtSplinePoint(int32 PointIndex) const;
	
	UFUNCTION(BlueprintCallable, Category = "Spline Point Data")
	void SetMoveForwardAtSplinePoint(int32 PointIndex, bool bMoveForward);
};
