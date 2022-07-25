// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "LineOfSightActor.generated.h"

USTRUCT(Blueprintable)
struct FMeshPoint
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(BlueprintReadOnly, DisplayName = "Point")
	FVector point;
	UPROPERTY(BlueprintReadOnly, DisplayName = "Mesh")
	UStaticMeshComponent* mesh;

	FMeshPoint() = default;
	FMeshPoint(UStaticMeshComponent* _mesh, double _x, double _y, double _z) : mesh(_mesh), point(_x, _y, _z) {}
	FMeshPoint(UStaticMeshComponent* _mesh, const FVector& _point) : mesh(_mesh), point(_point) {}
};

UCLASS()
class LINEOFSIGHT_API ALineOfSightActor : public AActor
{
	GENERATED_BODY()

	TArray<UStaticMeshComponent*> m_arrAdjMeshes;
	TMap<UStaticMeshComponent*, TArray<FMeshPoint>> m_mapDetectedObstacles;
	double m_dBaseAngle;
	FVector m_vActorLoc;
	bool m_bIsPointCalculated;
	TArray<FMeshPoint> m_arrDetectedPoints;

protected:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Vectex Trace Optimization", Category = "Line Of Sight")
	bool m_bIsOptimize;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Show Trace Debug", Category = "Line Of Sight")
	bool m_bIsShowTraceDebug;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Trace Count", Category = "Line Of Sight", meta = (ClampMin = "1"))
	int m_iTraceCount = 100;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Trace Length", Category = "Line Of Sight", meta = (ClampMin = "0"))
	double m_dTraceLength = 1000;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "View Angle", Category = "Line Of Sight", meta = (ClampMin = "0", ClampMax = "360", UIMin = "0", UIMax = "360"))
	double m_dViewAngle = 60;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Trace Offset", Category = "Line Of Sight", meta = (ClampMin = "0"))
	double m_dTraceOffset = 10;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Trace Angle Offset", Category = "Line Of Sight", meta = (ClampMin = "0"))
	double m_dTraceAngleOffset = .001;

public:
	UFUNCTION(BlueprintCallable)
	TArray<FMeshPoint> GetDetectedPoints();

	void Tick(float _fDelta) override;

public:
	ALineOfSightActor();

public:
	UFUNCTION(BlueprintCallable)
	void AddAdjMesh(UStaticMeshComponent* _pMesh);
	UFUNCTION(BlueprintCallable)
	void RemoveAdjMesh(UStaticMeshComponent* _pMesh);
	UFUNCTION(BlueprintCallable)
	bool SortByAngle(const FMeshPoint& _lhs, const FMeshPoint& _rhs);

private:
	FMeshPoint GetTracedPoint(const FVector& _vBasePoint, double _dAngle);
	void CalculateValidPoints();
	bool TryFindTwoLineCrossPoint(const FVector& _p1, const FVector& _p2, const FVector& vP1P2, const FVector& _p3, const FVector& _p4, double _dDistSquared, FVector& _vCrossPoint);
	FVector FindLeftPointBetweenPointAndLine(const FVector& _vBasePoint, const FVector& _vLintPoint1, const FVector& _vLintPoint2, double _dLengthSquared);
	FVector FindRightPointBetweenPointAndLine(const FVector& _vBasePoint, const FVector& _vLintPoint1, const FVector& _vLintPoint2, double _dLengthSquared);
};
