// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "LineOfSightActor.generated.h"

class USphereComponent;
class UPrimitiveComponent;
class ULineOfSightVisibility;

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

	USphereComponent* m_pSphere;
	TArray<UStaticMeshComponent*> m_arrAdjMeshes;
	TMap<UStaticMeshComponent*, TArray<FMeshPoint>> m_mapDetectedObstacles;
	double m_dBaseAngle;
	FVector m_vActorLoc;
	bool m_bIsPointCalculated;
	TArray<FMeshPoint> m_arrDetectedPoints;
	TArray<FMeshPoint> m_arrValidPoints;
	TSet<ULineOfSightVisibility*> m_setVisibles;

	FVector m_vForward;
	FVector m_vBack;
	FVector m_vLeftTrace;
	FVector m_vRightTrace;
	double m_dTraceLengthSquared;
	FVector m_vLTP;
	FVector m_vRTP;
	double m_dFLDot;
	FVector m_vMeshLoc;
	FVector m_vMLV, m_vMRV;
	bool m_bLTVInclude, m_bRTVInclude;

protected:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Following Actor", Category = "Line Of Sight")
	AActor* m_pFollowingActor;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Follow Offset", Category = "Line Of Sight")
	FVector m_vFollowOffset;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Vectex Trace Optimization", Category = "Line Of Sight")
	bool m_bIsOptimize;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Show Trace", Category = "Line Of Sight")
	bool m_bIsShowTrace;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Render", Category = "Line Of Sight")
	bool m_bIsRender = true;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Trace Count", Category = "Line Of Sight", meta = (ClampMin = "1"))
	int m_iTraceCount = 100;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Trace Length", Category = "Line Of Sight", meta = (ClampMin = "0"))
	double m_dTraceLength = 1000;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "View Angle", Category = "Line Of Sight", meta = (ClampMin = "0", ClampMax = "180", UIMin = "0", UIMax = "180"))
	double m_dViewAngle = 60;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Trace Offset", Category = "Line Of Sight", meta = (ClampMin = "0"))
	double m_dTraceOffset = 10;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Angle Per Trace", Category = "Line Of Sight", meta = (ClampMin = "0.1"))
	double m_dAnglePerTrace = 3;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, DisplayName = "Trace Angle Offset", Category = "Line Of Sight", meta = (ClampMin = "0"))
	double m_dTraceAngleOffset = .001;
	/*UPROPERTY(EditAnywhere, DisplayName = "Hide Channel", Category = "Line Of Sight", meta = (Bitmask, BitmaskEnum = "ECollisionChannel"))
	int32 m_iHideChannel;*/

public:
	UFUNCTION(BlueprintCallable)
	TArray<FMeshPoint> GetDetectedPoints();

	void BeginPlay() override;
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
	void ProcessCube(UStaticMeshComponent* pMeshCom, TArray<FMeshPoint>& arrPoints);
	void ProcessCylinder(UStaticMeshComponent* pMeshCom, TArray<FMeshPoint>& arrPoints);
	void DrawPoint(const FVector& _vLoc, const FColor& _color, double _dDepthPriority = 1);
	void DrawLine(const FVector& _vStart, const FVector& _vEnd, const FColor& _color);
	bool IsInViewAngle(const FVector& vLeftTrace, const FVector& vRightTrace, const FVector& vMLV, const FVector& vMRV, bool& bLTVInclude, bool& bRTVInclude);
	void AddPoint(const FVector& _vPoint, UStaticMeshComponent* _pMeshCom);
	void AddPointIfTraceHit(const FVector& _vPoint, UStaticMeshComponent* _pMeshCom);
	void AddEdgePoint(const FVector& _vPoint, UStaticMeshComponent* _pMeshCom, double _dAngleOffset);
	FMeshPoint GetTracedPoint(const FVector& _vBasePoint, double _dAngle);
	void CalculateValidPoints();
	bool TryFindTwoLineCrossPoint(const FVector& _p1, const FVector& _p2, const FVector& vP1P2, const FVector& _p3, const FVector& _p4, double _dDistSquared, FVector& _vCrossPoint);
	FVector FindLeftPointBetweenPointAndLine(const FVector& _vBasePoint, const FVector& _vLintPoint1, const FVector& _vLintPoint2, double _dLengthSquared);
	FVector FindRightPointBetweenPointAndLine(const FVector& _vBasePoint, const FVector& _vLintPoint1, const FVector& _vLintPoint2, double _dLengthSquared);
	void FindPointsBetweenPointAndCircle(const FVector& _vPoint, const FVector& _vCircleCenter, double _dRadiusSquared, double _dDistSquared, FVector& _vOutLeftPoint, FVector& _vOutRightPoint);

public:
	UFUNCTION()
	void OnOverlapBegin(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);
	UFUNCTION()
	void OnOverlapEnd(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex);
};
