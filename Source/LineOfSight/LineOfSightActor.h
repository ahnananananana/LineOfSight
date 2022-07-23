// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "LineOfSightActor.generated.h"

UCLASS()
class LINEOFSIGHT_API ALineOfSightActor : public AActor
{
	GENERATED_BODY()

	TArray<UStaticMeshComponent*> m_arrAdjMeshes;
	TMap<UStaticMeshComponent*, TArray<FVector>> m_mapDetectedPoints;

protected:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Vectex Trace Optimization", Category = "Line Of Sight")
	bool m_bIsOptimize;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Show Trace Debug", Category = "Line Of Sight")
	bool m_bIsShowTraceDebug;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Trace Count", Category = "Line Of Sight", meta = (ClampMin = "1"))
	int m_iTraceCount = 100;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Trace Length", Category = "Line Of Sight", meta = (ClampMin = "0"))
	float m_fTraceLength = 1000.f;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "View Angle", Category = "Line Of Sight", meta = (ClampMin = "0", ClampMax = "360", UIMin = "0", UIMax = "360"))
	float m_fViewAngle = 60.f;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Trace Offset", Category = "Line Of Sight", meta = (ClampMin = "0"))
	float m_fTraceOffset = 10.f;

public:
	UFUNCTION(BlueprintCallable)
	TArray<FVector> GetDetectedPoints();

public:
	ALineOfSightActor();

public:

	UFUNCTION(BlueprintCallable)
	void AddAdjMesh(UStaticMeshComponent* _pMesh);
	UFUNCTION(BlueprintCallable)
	void RemoveAdjMesh(UStaticMeshComponent* _pMesh);

};
