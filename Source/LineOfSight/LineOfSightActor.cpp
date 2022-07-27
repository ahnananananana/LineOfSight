#include "LineOfSightActor.h"
#include "AI/Navigation/NavCollisionBase.h"
#include "DrawDebugHelpers.h"
#include "GenericPlatform/GenericPlatformMath.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Components/SphereComponent.h"
#include "LineOfSightVisibility.h"

#define LOG(text) UE_LOG(LogTemp, Log, TEXT(#text))

ALineOfSightActor::ALineOfSightActor()
{
	PrimaryActorTick.bCanEverTick = true;
}

void ALineOfSightActor::BeginPlay()
{
	Super::BeginPlay();
	//FCollisionObjectQueryParams coqp(m_iHideChannel);

	m_pSphere = FindComponentByClass<USphereComponent>();
	if (m_pSphere)
	{
		m_pSphere->SetSphereRadius(m_dTraceLength);

		m_pSphere->OnComponentBeginOverlap.AddDynamic(this, &ALineOfSightActor::OnOverlapBegin);
		m_pSphere->OnComponentEndOverlap.AddDynamic(this, &ALineOfSightActor::OnOverlapEnd);

		TArray<AActor*> arrActors;
		m_pSphere->GetOverlappingActors(arrActors);
		for (AActor* actor : arrActors)
		{
			if (UStaticMeshComponent* pMeshCom = actor->FindComponentByClass<UStaticMeshComponent>())
			{
				AddAdjMesh(pMeshCom);
			}
		}
	}
}

void ALineOfSightActor::Tick(float _fDelta)
{
	Super::Tick(_fDelta);

	m_bIsPointCalculated = false;
	if (m_pSphere->GetUnscaledSphereRadius() != m_dTraceLength)
	{
		m_pSphere->SetSphereRadius(m_dTraceLength, false);
	}
}

void ALineOfSightActor::OnOverlapBegin(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult)
{
	if (UStaticMeshComponent* pMeshCom = OtherActor->FindComponentByClass<UStaticMeshComponent>())
	{
		AddAdjMesh(pMeshCom);
	}
}

void ALineOfSightActor::OnOverlapEnd(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex)
{
	if (UStaticMeshComponent* pMeshCom = OtherActor->FindComponentByClass<UStaticMeshComponent>())
	{
		RemoveAdjMesh(pMeshCom);
	}

	if (ULineOfSightVisibility* pVisibility = OtherActor->FindComponentByClass<ULineOfSightVisibility>())
	{
		pVisibility->SetVisible(false);
	}
}

//TODO: 두 큐브가 서로 겹치는 부분에서 어두운 부분 생김, 추가 트레이스도 잘 안됨
//TODO: 비스듬한 면 Offset 처리해야
TArray<FMeshPoint> ALineOfSightActor::GetDetectedPoints()
{
	if (!m_bIsPointCalculated)
		CalculateValidPoints();

	return m_arrDetectedPoints;
}

void ALineOfSightActor::AddAdjMesh(UStaticMeshComponent* _pMeshCom)
{
	if (_pMeshCom && !m_mapDetectedObstacles.Contains(_pMeshCom))
	{
		if (UStaticMesh* pMesh = _pMeshCom->GetStaticMesh())
		{
			if (UNavCollisionBase* pNavCol = pMesh->GetNavCollision())
			{
				//Actor의 Z값이상인 것만 필터링
				TArray<FMeshPoint>& arrPoints = m_mapDetectedObstacles.Emplace(_pMeshCom);
				float fActorPosZ = GetActorLocation().Z;
				for (const FVector& p : pNavCol->GetConvexCollision().VertexBuffer)
				{
					const FVector& tp = _pMeshCom->GetOwner()->GetActorTransform().TransformPosition(p);
					if (tp.Z <= fActorPosZ)
					{
						//해당 Z 위치로 변경 후 추가
						arrPoints.Emplace(_pMeshCom, tp.X, tp.Y, fActorPosZ);
					}
				}
			}
		}

		if (ULineOfSightVisibility* v = _pMeshCom->GetOwner()->FindComponentByClass<ULineOfSightVisibility>())
		{
			m_setVisibles.Emplace(v);
		}
	}
}

void ALineOfSightActor::RemoveAdjMesh(UStaticMeshComponent* _pMeshCom)
{
	if (_pMeshCom)
	{
		m_mapDetectedObstacles.Remove(_pMeshCom);

		if (ULineOfSightVisibility* v = _pMeshCom->GetOwner()->FindComponentByClass<ULineOfSightVisibility>())
		{
			m_setVisibles.Remove(v);
		}
	}
}

bool ALineOfSightActor::SortByAngle(const FMeshPoint& _lhs, const FMeshPoint& _rhs)
{
	FVector v1 = _lhs.point - m_vActorLoc;
	FVector v2 = _rhs.point - m_vActorLoc;

	double rot1 = v1.Rotation().Yaw >= 0 ? v1.Rotation().Yaw : 360 + v1.Rotation().Yaw;
	double rot2 = v2.Rotation().Yaw >= 0 ? v2.Rotation().Yaw : 360 + v2.Rotation().Yaw;

	rot1 -= m_dBaseAngle;
	if (rot1 < 0)
	{
		rot1 = 360 + rot1;
	}

	rot2 -= m_dBaseAngle;
	if (rot2 < 0)
	{
		rot2 = 360 + rot2;
	}

	return rot1 < rot2;
}

bool ALineOfSightActor::IsInViewAngle(const FVector& _vLeftTrace, const FVector& _vRightTrace, const FVector& _vMLV, const FVector& _vMRV, bool& _bLTVInclude, bool& _bRTVInclude)
{
	/*double dLTA = _vLeftTrace.Rotation().Yaw >= 0 ? _vLeftTrace.Rotation().Yaw : 360 + _vLeftTrace.Rotation().Yaw;
	double dRTA = _vRightTrace.Rotation().Yaw >= 0 ? _vRightTrace.Rotation().Yaw : 360 + _vRightTrace.Rotation().Yaw;
	double dMLA = _vMLV.Rotation().Yaw >= 0 ? _vMLV.Rotation().Yaw : 360 + _vMLV.Rotation().Yaw;
	double dMRA = _vMRV.Rotation().Yaw >= 0 ? _vMRV.Rotation().Yaw : 360 + _vMRV.Rotation().Yaw;

	double dBaseAngle = dMLA;

	UE_LOG(LogTemp, Log, TEXT("dLTA %f"), dLTA);
	UE_LOG(LogTemp, Log, TEXT("dRTA %f"), dRTA);
	UE_LOG(LogTemp, Log, TEXT("dMLA %f"), dMLA);
	UE_LOG(LogTemp, Log, TEXT("dMRA %f"), dMRA);
	LOG(----------------)
	dLTA -= dBaseAngle;
	if (dLTA < 0)
	{
		dLTA = 360 + dLTA;
	}

	dRTA -= dBaseAngle;
	if (dRTA < 0)
	{
		dRTA = 360 + dRTA;
	}

	dMLA -= dBaseAngle;
	if (dMLA < 0)
	{
		dMLA = 360 + dMLA;
	}

	dMRA -= dBaseAngle;
	if (dMRA < 0)
	{
		dMRA = 360 + dMRA;
	}

	UE_LOG(LogTemp, Log, TEXT("dLTA %f"), dLTA);
	UE_LOG(LogTemp, Log, TEXT("dRTA %f"), dRTA);
	UE_LOG(LogTemp, Log, TEXT("dMLA %f"), dMLA);
	UE_LOG(LogTemp, Log, TEXT("dMRA %f"), dMRA);

	_bLTVInclude = dMRA < dRTA;
	_bRTVInclude = dMLA < dRTA;

	return dMRA <= dMLA || _bLTVInclude || _bRTVInclude;*/
	double dObsDot = _vMRV.Dot(_vMLV);

	_bLTVInclude = _vMLV.Dot(_vLeftTrace) >= dObsDot && _vMRV.Dot(_vLeftTrace) >= dObsDot;
	_bRTVInclude = _vMLV.Dot(_vRightTrace) >= dObsDot && _vMRV.Dot(_vRightTrace) >= dObsDot;

	bool bValid = _bLTVInclude || _bRTVInclude;

	//물체 전체가 시야각 안에 들어온 경우
	dObsDot = _vLeftTrace.Dot(_vRightTrace);
	bValid |= _vMLV.Dot(_vLeftTrace) >= dObsDot && _vMLV.Dot(_vRightTrace) >= dObsDot &&
		_vMRV.Dot(_vLeftTrace) >= dObsDot && _vMRV.Dot(_vRightTrace) >= dObsDot;

	return bValid;
}

void ALineOfSightActor::AddPoint(const FVector& _vPoint, UStaticMeshComponent* _pMeshCom)
{
	TArray<AActor*> ignore{ _pMeshCom->GetOwner() };
	FHitResult result;
	if (!UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, _vPoint, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
	{
		m_arrValidPoints.Emplace(_pMeshCom, _vPoint);
	}
}

void ALineOfSightActor::AddPointIfTraceHit(const FVector& _vPoint, UStaticMeshComponent* _pMeshCom)
{
	TArray<AActor*> ignore;
	FHitResult result;
	if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, _vPoint, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
	{
		if (result.GetActor() == _pMeshCom->GetOwner())
			m_arrValidPoints.Emplace(_pMeshCom, result.Location);
	}
}

void ALineOfSightActor::AddEdgePoint(const FVector& _vPoint, UStaticMeshComponent* _pMeshCom, double _dAngleOffset)
{
	TArray<AActor*> ignore{ _pMeshCom->GetOwner() };
	FHitResult result;
	if (!UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, _vPoint, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
	{
		m_arrValidPoints.Emplace(_pMeshCom, _vPoint);
	}

	FVector vEdgePassed = (_vPoint - m_vActorLoc).GetUnsafeNormal() * m_dTraceLength;
	vEdgePassed = vEdgePassed.RotateAngleAxis(_dAngleOffset, FVector::UpVector);
	vEdgePassed = m_vActorLoc + vEdgePassed;
	if (!UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, vEdgePassed, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
	{
		m_arrValidPoints.Emplace(_pMeshCom, vEdgePassed);
	}
	else
	{
		if(FVector::DistSquared(m_vActorLoc, _vPoint) < FVector::DistSquared(m_vActorLoc, result.ImpactPoint))
			m_arrValidPoints.Emplace(result.GetActor()->FindComponentByClass<UStaticMeshComponent>(), result.ImpactPoint);
	}
}

FMeshPoint ALineOfSightActor::GetTracedPoint(const FVector& _vBasePoint, double _dAngle)
{
	FVector vTraceDir = (_vBasePoint - m_vActorLoc).GetUnsafeNormal();
	vTraceDir = vTraceDir.RotateAngleAxis(_dAngle, FVector3d::UpVector);

	FVector rTraceEnd = m_vActorLoc + vTraceDir * m_dTraceLength;
	TArray<AActor*> temp;
	FHitResult result;
	if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, rTraceEnd, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, temp, EDrawDebugTrace::Type::None, result, true))
	{
		return { result.GetActor()->FindComponentByClass<UStaticMeshComponent>(), result.Location };
	}
	else
	{
		return { nullptr, rTraceEnd };
	}
}

bool ALineOfSightActor::TryFindTwoLineCrossPoint(const FVector& _p1, const FVector& _p2, const FVector& vP1P2, const FVector& _p3, const FVector& _p4, double _dDistSquared, FVector& _vCrossPoint)
{
	double denominator = (_p1.X - _p2.X) * (_p3.Y - _p4.Y) - (_p1.Y - _p2.Y) * (_p3.X - _p4.X);
	//평행하거나 일치하는 경우
	if (denominator == 0)
		return false;

	_vCrossPoint.X = ((_p1.X * _p2.Y - _p1.Y * _p2.X) * (_p3.X - _p4.X) - (_p1.X - _p2.X) * (_p3.X * _p4.Y - _p3.Y * _p4.X)) / denominator;
	_vCrossPoint.Y = ((_p1.X * _p2.Y - _p1.Y * _p2.X) * (_p3.Y - _p4.Y) - (_p1.Y - _p2.Y) * (_p3.X * _p4.Y - _p3.Y * _p4.X)) / denominator;
	_vCrossPoint.Z = m_vActorLoc.Z;

	//각 점이 시점과 종점 사이에 있는지 체크
	return vP1P2.Equals((_vCrossPoint - _p1).GetUnsafeNormal()) && FVector::DistSquared(_p1, _vCrossPoint) <= _dDistSquared;
}

FVector ALineOfSightActor::FindLeftPointBetweenPointAndLine(const FVector& _vBasePoint, const FVector& _vLintPoint1, const FVector& _vLintPoint2, double _dLengthSquared)
{
	FVector v = (_vLintPoint2 - _vLintPoint1).GetUnsafeNormal();

	double A = _vLintPoint1.X - m_vActorLoc.X;
	double B = _vLintPoint1.Y - m_vActorLoc.Y;
	double C = v.X;
	double D = v.Y;

	double a = C * C + D * D;
	double b = 2 * A * C + 2 * B * D;
	double c = A * A + B * B - _dLengthSquared;

	double squared = b * b - 4 * a * c;

	//출발점과 더 가까워야하므로 -
	double t = (-b - FMath::Sqrt(squared)) / (2 * a);

	FVector vResult = _vLintPoint1 + v * t;
	vResult.Z = m_vActorLoc.Z;
	return vResult;
}

FVector ALineOfSightActor::FindRightPointBetweenPointAndLine(const FVector& _vBasePoint, const FVector& _vLintPoint1, const FVector& _vLintPoint2, double _dLengthSquared)
{
	FVector v = (_vLintPoint2 - _vLintPoint1).GetUnsafeNormal();

	double A = _vLintPoint1.X - m_vActorLoc.X;
	double B = _vLintPoint1.Y - m_vActorLoc.Y;
	double C = v.X;
	double D = v.Y;

	double a = C * C + D * D;
	double b = 2 * A * C + 2 * B * D;
	double c = A * A + B * B - _dLengthSquared;

	double squared = b * b - 4 * a * c;

	//출발점과 더 가까워야하므로 -
	double t = (-b + FMath::Sqrt(squared)) / (2 * a);

	FVector vResult = _vLintPoint1 + v * t;
	vResult.Z = m_vActorLoc.Z;
	return vResult;
}

void ALineOfSightActor::FindPointsBetweenPointAndCircle(const FVector& _vPoint, const FVector& _vCircleCenter, double _dRadiusSquared, double _dDistSquared, FVector& _vOutLeftPoint, FVector& _vOutRightPoint)
{
	double rr = _dRadiusSquared;
	double d = _vCircleCenter.X;
	double e = _vCircleCenter.Y;
	double f = _vPoint.X;
	double g = _vPoint.Y;
	double tt = _dDistSquared;

	double A = (f * f - d * d + g * g - e * e + 2 * rr - tt) / (2 * (g - e));
	double B = (d - f) / (g - e);
	double C = A - e;
	double a = B * B + 1;
	double b = -2 * d + 2 * B * C;
	double c = d * d + C * C - rr;

	double sqrt = FMath::Sqrt(b * b - 4 * a * c);

	double x = (-b - sqrt) / (2 * a);
	double y = A + B * x;
	_vOutLeftPoint = FVector(x, y, m_vActorLoc.Z);

	x = (-b + sqrt) / (2 * a);
	y = A + B * x;
	_vOutRightPoint = FVector(x, y, m_vActorLoc.Z);

	//시점에서 원의 중심으로 가는 벡터의 왼쪽 벡터와 내적하여 -면 오른쪽점, +면 왼쪽점
	const FVector& vAtoM = (_vCircleCenter - _vPoint).GetUnsafeNormal();
	const FVector& vBaseRight = vAtoM.Cross(FVector::UpVector);

	if (vBaseRight.Dot((_vOutLeftPoint - _vPoint).GetUnsafeNormal()) < 0)
	{
		FVector temp = _vOutLeftPoint;
		_vOutLeftPoint = _vOutRightPoint;
		_vOutRightPoint = temp;
	}
}

void ALineOfSightActor::CalculateValidPoints()
{
	if (m_bIsPointCalculated)
		return;

	m_bIsPointCalculated = true;

	m_arrDetectedPoints.Empty();

	//View Angle 안에 있는지 체크
	//왼쪽 Trace 벡터와 Forward 벡터의 내적보다 큰 점들은 내부에 있는 점
	m_vActorLoc = GetActorLocation();
	vForward = GetActorForwardVector();
	vBack = -vForward;
	vLeftTrace = vForward.RotateAngleAxis(-m_dViewAngle / 2, FVector3d::UpVector);
	vRightTrace = vForward.RotateAngleAxis(m_dViewAngle / 2, FVector3d::UpVector);
	dTraceLengthSquared = m_dTraceLength * m_dTraceLength;
	vLTP = m_vActorLoc + vLeftTrace * m_dTraceLength;
	vRTP = m_vActorLoc + vRightTrace * m_dTraceLength;
	dFLDot = vForward.Dot(vLeftTrace);

	if (m_bIsShowTraceDebug)
	{
		DrawDebugLine(GetWorld(), m_vActorLoc, m_vActorLoc + vLeftTrace * m_dTraceLength, FColor::Blue, false, -1, 1, 1);
		DrawDebugLine(GetWorld(), m_vActorLoc, m_vActorLoc + vRightTrace * m_dTraceLength, FColor::Blue, false, -1, 1, 1);
	}

	for (const auto& pair : m_mapDetectedObstacles)
	{
		UStaticMeshComponent* pMeshCom = pair.Key;
		//UE_LOG(LogTemp, Log, TEXT("%s"), *UKismetSystemLibrary::GetDisplayName(pMeshCom->GetOwner()));
		TArray<FMeshPoint> arrPoints = pair.Value;
		if (arrPoints.IsEmpty())
		{
			UE_LOG(LogTemp, Error, TEXT("No Points"));
			continue;
		}

		vMeshLoc = pMeshCom->GetOwner()->GetActorLocation();
		vMeshLoc.Z = m_vActorLoc.Z;

		//점 갯수로 큐브인지 원기둥인지 구분
		//큐브인 경우
		if (arrPoints.Num() == 4)
		{
			ProcessCube(pMeshCom, arrPoints);
		}
		else
		{
			ProcessCylinder(pMeshCom, arrPoints);
		}

		//Point와 Mesh를 맵핑한 다음 서로 다른 Mesh 위에 있는 Point롤 삼각형을 만들시 그 사이를 Trace해야
		//Forward 벡터와의 내적을 왼쪽 Trace 벡터와 비교하여 크다면 시야범위 내에 있음
		for (const FMeshPoint& p : m_arrValidPoints)
		{
			FVector v = p.point - m_vActorLoc;
			v.Normalize();

			if (vForward.Dot(v) + .001 >= dFLDot)
			{
				m_arrDetectedPoints.Emplace(p);
			}
		}
	}

	//모든 점을 다시 각도로 정렬
	m_dBaseAngle = vBack.Rotation().Yaw >= 0 ? vBack.Rotation().Yaw : 360 + vBack.Rotation().Yaw;
	m_arrDetectedPoints.Sort([&](const FMeshPoint& lhs, const FMeshPoint& rhs) { return SortByAngle(lhs, rhs); });

	TArray<TPair<int, int>> arrSections;
	if (!m_arrDetectedPoints.IsEmpty())
	{
		if (!vLeftTrace.Equals((m_arrDetectedPoints[0].point - m_vActorLoc).GetUnsafeNormal()))
		{
			m_arrDetectedPoints.Insert({ nullptr, m_vActorLoc + vLeftTrace * m_dTraceLength }, 0);
		}

		if (!vRightTrace.Equals((m_arrDetectedPoints.Last().point - m_vActorLoc).GetUnsafeNormal()))
		{
			m_arrDetectedPoints.Emplace(nullptr, m_vActorLoc + vRightTrace * m_dTraceLength);
		}

		for (int i = 0; i < m_arrDetectedPoints.Num() - 1; ++i)
		{
			if (m_arrDetectedPoints[i].mesh != m_arrDetectedPoints[i + 1].mesh)
			{
				arrSections.Emplace(i, i + 1);
			}
		}
	}
	else
	{
		m_arrDetectedPoints.Emplace(nullptr, m_vActorLoc + vLeftTrace * m_dTraceLength);
		m_arrDetectedPoints.Emplace(nullptr, m_vActorLoc + vRightTrace * m_dTraceLength);

		arrSections.Emplace(0, 1);
	}

	for (ULineOfSightVisibility* v : m_setVisibles)
	{
		v->SetVisible(false);
	}

	for (FMeshPoint& p : m_arrDetectedPoints)
	{
		if (p.mesh)
		{
			if (ULineOfSightVisibility* visibility = p.mesh->GetOwner()->FindComponentByClass<ULineOfSightVisibility>())
			{
				if (m_setVisibles.Contains(visibility))
				{
					visibility->SetVisible(true);
				}
			}
		}
	}


	int iIdxOffset = 0;
	for (const TPair<int, int> s : arrSections)
	{
		const FVector& vLeft = (m_arrDetectedPoints[s.Key + iIdxOffset].point - m_vActorLoc).GetUnsafeNormal();
		const FVector& vRight = (m_arrDetectedPoints[s.Value + iIdxOffset].point - m_vActorLoc).GetUnsafeNormal();

		double dAngle = acos(vLeft.Dot(vRight)) * 180 / PI;
		int iTraceToAdd = dAngle / m_dAnglePerTrace;

		for (int i = 1; i <= iTraceToAdd; ++i)
		{
			FVector vDir = vLeft.RotateAngleAxis(i * m_dAnglePerTrace, FVector::UpVector);
			m_arrDetectedPoints.Insert({ nullptr, m_vActorLoc + vDir * m_dTraceLength }, s.Value + iIdxOffset);
			++iIdxOffset;
		}
	}

	if (m_bIsShowTraceDebug)
	{
		for (const FMeshPoint& p : m_arrDetectedPoints)
		{
			DrawPoint(p.point, FColor::Red);
			DrawLine(m_vActorLoc, p.point, FColor::Red);
		}
	}
}

void ALineOfSightActor::ProcessCube(UStaticMeshComponent* pMeshCom, TArray<FMeshPoint>& arrPoints)
{
	//장애물과 시점으로의 벡터를 기준으로 점들의 각도를 계산
	const FVector& vAngleBase = (m_vActorLoc - vMeshLoc).GetUnsafeNormal();
	m_dBaseAngle = vAngleBase.Rotation().Yaw >= 0 ? vAngleBase.Rotation().Yaw : 360 + vAngleBase.Rotation().Yaw;

	m_arrValidPoints.Empty();
	//각도에 따라 정렬
	arrPoints.Sort([&](const FMeshPoint& lhs, const FMeshPoint& rhs) { return SortByAngle(lhs, rhs); });

	//시야범위 안에 있는 점 체크
	TArray<int> arrInRangePointIdx;
	double dMaxDis = -1;
	double dMinDis = MAX_dbl;
	int iMaxIdx = 0;
	int iMinIdx = 0;
	for (int i = 0; i < arrPoints.Num(); ++i)
	{
		double dis = FVector::DistSquared(m_vActorLoc, arrPoints[i].point);
		if (dis < dMinDis)
		{
			dMinDis = dis;
			iMinIdx = i;
		}

		if (dis > dMaxDis)
		{
			dMaxDis = dis;
			iMaxIdx = i;
		}

		//시야범위 안에 있는지 체크
		FVector v = (arrPoints[i].point - m_vActorLoc).GetUnsafeNormal();
		if (dis <= dTraceLengthSquared && vForward.Dot(v) + .001 >= dFLDot)
		{
			arrInRangePointIdx.Emplace(i);
		}
	}

	vMLV = (arrPoints[0].point - m_vActorLoc).GetUnsafeNormal();
	vMRV = (arrPoints.Last().point - m_vActorLoc).GetUnsafeNormal();

	if (!IsInViewAngle(vLeftTrace, vRightTrace, vMLV, vMRV, bLTVInclude, bRTVInclude))
	{
		return;
	}

	m_arrValidPoints.Reserve(4);
	//1면만 보고 있는지 2면을 보고 있는지 구분
	//가운데 각도 2개 중 가장 가까운 점이 있다면 2면, 아니라면 1면

	bool bIsTwoSide = iMinIdx == 1 || iMinIdx == 2;

	int vMostLeftIdx = 0;
	int vMostRightIdx = 3;
	FMeshPoint vMostLeftPoint = arrPoints[0];
	FMeshPoint vMostRightPoint = arrPoints[3];

	if (bIsTwoSide)
	{
		FVector v = (arrPoints[iMinIdx].point - m_vActorLoc).GetUnsafeNormal();
		//2면을 볼 수 있는 위치에서 1면만 보는 경우
		if (vForward.Dot(v) + .001 < dFLDot)
		{
			bIsTwoSide = false;
			if (GetActorRightVector().Dot((arrPoints[iMinIdx].point - m_vActorLoc).GetUnsafeNormal()) > 0)
			{
				//TODO: vMostRightPoint 네이밍을 적절하게 바꿔야
				vMostRightPoint = arrPoints[iMinIdx];
				vMostRightIdx = iMinIdx;
			}
			else
			{
				vMostLeftPoint = arrPoints[iMinIdx];
				vMostLeftIdx = iMinIdx;
			}
		}
	}

	//2면인 경우
	if (bIsTwoSide)
	{
		//가장 먼 점은 어차피 필요없으므로 제거
		arrInRangePointIdx.Remove(iMaxIdx);

		const FMeshPoint& vMidPoint = arrPoints[iMinIdx];
		//Trace 거리 안에 포함되는 점의 갯수에 따라 처리
		//0개인 경우
		if (arrInRangePointIdx.Num() == 0)
		{
			return;
		}
		//1개인 경우
		else if (arrInRangePointIdx.Num() == 1)
		{
			{
				FVector vLeftPoint;
				//일단 왼쪽 Trace 벡터와 오른쪽 Trace 벡터 방향의 직선과의 교점 계산
				FVector vLeftTraceEnd = m_vActorLoc + vLeftTrace * m_dTraceLength;

				//거리 밖이라면 유효한 점이 아니므로 해당 점을 다시 계산
				//시점과 직선 간의 거리가 시야거리만큼인 점을 계산
				if (!TryFindTwoLineCrossPoint(m_vActorLoc, vLeftTraceEnd, vLeftTrace, vMostLeftPoint.point, vMidPoint.point, dTraceLengthSquared, vLeftPoint))
				{
					vLeftPoint = FindLeftPointBetweenPointAndLine(m_vActorLoc, vMostLeftPoint.point, vMidPoint.point, dTraceLengthSquared);
				}

				AddEdgePoint(vLeftPoint, pMeshCom, -m_dTraceAngleOffset);
			}

			m_arrValidPoints.Emplace(GetTracedPoint(vMidPoint.point, 0));

			{
				FVector vRightPoint;
				//일단 왼쪽 Trace 벡터와 오른쪽 Trace 벡터 방향의 직선과의 교점 계산
				FVector vRightTraceEnd = m_vActorLoc + vRightTrace * m_dTraceLength;

				//거리 밖이라면 유효한 점이 아니므로 해당 점을 다시 계산
				//시점과 직선 간의 거리가 시야거리만큼인 점을 계산
				if (!TryFindTwoLineCrossPoint(m_vActorLoc, vRightTraceEnd, vRightTrace, vMidPoint.point, vMostRightPoint.point, dTraceLengthSquared, vRightPoint))
				{
					vRightPoint = FindRightPointBetweenPointAndLine(m_vActorLoc, vMidPoint.point, vMostRightPoint.point, dTraceLengthSquared);
				}

				AddEdgePoint(vRightPoint, pMeshCom, m_dTraceAngleOffset);
			}
		}
		//2개인 경우
		else if (arrInRangePointIdx.Num() == 2)
		{
			//왼쪽 점과 가까운 점만 포함하는 경우
			if (arrInRangePointIdx[0] == 0)
			{
				//오른쪽 점과 가운데 점 사이의 점을 계산하여 추가
				{
					FVector vRightPoint;
					//일단 왼쪽 Trace 벡터와 오른쪽 Trace 벡터 방향의 직선과의 교점 계산
					FVector vRightTraceEnd = m_vActorLoc + vRightTrace * m_dTraceLength;

					if (!TryFindTwoLineCrossPoint(m_vActorLoc, vRightTraceEnd, vRightTrace, vMidPoint.point, vMostRightPoint.point, dTraceLengthSquared, vRightPoint))
					{
						vRightPoint = FindRightPointBetweenPointAndLine(m_vActorLoc, vMidPoint.point, vMostRightPoint.point, dTraceLengthSquared);
					}

					AddPoint(vRightPoint, pMeshCom);
				}

				AddEdgePoint(vMostLeftPoint.point, pMeshCom, -m_dTraceAngleOffset);
				AddPoint(vMidPoint.point, pMeshCom);
			}
			//오른쪽 점과 가까운 점만 포함하는 경우
			else if (arrInRangePointIdx[1] == 3)
			{
				//왼쪽 점과 가운데 점 사이의 점을 계산하여 추가
				{
					FVector vLeftPoint;
					//일단 왼쪽 Trace 벡터와 오른쪽 Trace 벡터 방향의 직선과의 교점 계산
					FVector vLeftTraceEnd = m_vActorLoc + vLeftTrace * m_dTraceLength;

					if (!TryFindTwoLineCrossPoint(m_vActorLoc, vLeftTraceEnd, vLeftTrace, vMostLeftPoint.point, vMidPoint.point, dTraceLengthSquared, vLeftPoint))
					{
						vLeftPoint = FindLeftPointBetweenPointAndLine(m_vActorLoc, vMostLeftPoint.point, vMidPoint.point, dTraceLengthSquared);
					}

					AddPoint(vLeftPoint, pMeshCom);
				}

				AddEdgePoint(vMostRightPoint.point, pMeshCom, m_dTraceAngleOffset);
				AddPoint(vMidPoint.point, pMeshCom);
			}
		}
		//3, 4개인 경우
		else
		{
			AddEdgePoint(vMostLeftPoint.point, pMeshCom, -m_dTraceAngleOffset);
			AddEdgePoint(vMostRightPoint.point, pMeshCom, m_dTraceAngleOffset);
			AddPoint(vMidPoint.point, pMeshCom);
		}
	}
	//1면인 경우
	else
	{
		//뒤쪽 점들은 고려 안함
		arrInRangePointIdx.RemoveAll([&](int e) {return e != vMostLeftIdx && e != vMostRightIdx; });

		//Trace 거리 안에 포함되는 점의 갯수에 따라 처리
		//0개인 경우
		if (arrInRangePointIdx.Num() == 0)
		{
			FVector vLeftPoint, vRightPoint;
			//일단 왼쪽 Trace 벡터와 오른쪽 Trace 벡터 방향의 직선과의 교점 계산
			FVector vLeftTraceEnd = m_vActorLoc + vLeftTrace * m_dTraceLength;
			FVector vRightTraceEnd = m_vActorLoc + vRightTrace * m_dTraceLength;

			if (!TryFindTwoLineCrossPoint(m_vActorLoc, vLeftTraceEnd, vLeftTrace, vMostLeftPoint.point, vMostRightPoint.point, dTraceLengthSquared, vLeftPoint))
			{
				vLeftPoint = FindLeftPointBetweenPointAndLine(m_vActorLoc, vMostLeftPoint.point, vMostRightPoint.point, dTraceLengthSquared);
			}

			if (!TryFindTwoLineCrossPoint(m_vActorLoc, vRightTraceEnd, vRightTrace, vMostLeftPoint.point, vMostRightPoint.point, dTraceLengthSquared, vRightPoint))
			{
				vRightPoint = FindRightPointBetweenPointAndLine(m_vActorLoc, vMostLeftPoint.point, vMostRightPoint.point, dTraceLengthSquared);
			}

			AddPoint(vLeftPoint, pMeshCom);
			AddPoint(vRightPoint, pMeshCom);
		}
		//1개인 경우
		else if (arrInRangePointIdx.Num() == 1)
		{
			//왼쪽 점이 포함된 경우
			if (arrInRangePointIdx[0] == 0)
			{
				//오른쪽 Trace 벡터와의 교점 계산
				FVector vRightPoint;
				FVector vRightTraceEnd = m_vActorLoc + vRightTrace * m_dTraceLength;
				TryFindTwoLineCrossPoint(m_vActorLoc, vRightTraceEnd, vRightTrace, vMostLeftPoint.point, vMostRightPoint.point, dTraceLengthSquared, vRightPoint);

				AddEdgePoint(vMostLeftPoint.point, pMeshCom, -m_dTraceAngleOffset);
				AddPoint(vRightPoint, pMeshCom);
			}
			//오른쪽 점이 포함된 경우
			else if (arrInRangePointIdx[0] == 3)
			{
				//계산된 점 중 왼쪽 점에 가까운 점을 추가
				FVector vLeftPoint;
				FVector vLeftTraceEnd = m_vActorLoc + vLeftTrace * m_dTraceLength;
				TryFindTwoLineCrossPoint(m_vActorLoc, vLeftTraceEnd, vLeftTrace, vMostLeftPoint.point, vMostRightPoint.point, dTraceLengthSquared, vLeftPoint);

				AddEdgePoint(vMostRightPoint.point, pMeshCom, m_dTraceAngleOffset);
				AddPoint(vLeftPoint, pMeshCom);
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("Algorithm Broken!"));
			}
		}
		//2개인 경우
		else
		{
			AddEdgePoint(vMostLeftPoint.point, pMeshCom, -m_dTraceAngleOffset);
			AddEdgePoint(vMostRightPoint.point, pMeshCom, m_dTraceAngleOffset);
		}
	}
}

void ALineOfSightActor::ProcessCylinder(UStaticMeshComponent* pMeshCom, TArray<FMeshPoint>& arrPoints)
{
	double rr = FVector::DistSquared(vMeshLoc, arrPoints[0].point);

	FVector vMLP, vMRP, vLP, vRP;
	FindPointsBetweenPointAndCircle(m_vActorLoc, vMeshLoc, FVector::DistSquared(vMeshLoc, arrPoints[0].point), FVector::DistSquared(m_vActorLoc, vMeshLoc), vMLP, vMRP);

	vMLV = (vMLP - m_vActorLoc).GetUnsafeNormal();
	vMRV = (vMRP - m_vActorLoc).GetUnsafeNormal();

	//장애물이 시야각과 겹치는지 체크
	if (!IsInViewAngle(vLeftTrace, vRightTrace, vMLV, vMRV, bLTVInclude, bRTVInclude))
	{
		return;
	}

	//시야각이 원 안으로 들어가는 경우
	if (bLTVInclude && bRTVInclude)
	{
		double dDisLTP = FVector::DistSquared(vLTP, vMeshLoc);
		double dDisRTP = FVector::DistSquared(vRTP, vMeshLoc);
		double dDotLTP = vForward.Dot((vLTP - vMeshLoc).GetUnsafeNormal());
		double dDotRTP = vForward.Dot((vRTP - vMeshLoc).GetUnsafeNormal());

		//시야각점 모두 원 내부에 있는경우
		if (dDisLTP <= rr && dDisRTP <= rr)
		{
			AddPointIfTraceHit(vLTP, pMeshCom);
			AddPointIfTraceHit(vRTP, pMeshCom);
		}
		//시야각점 모두 원 외부에 있는경우
		else if (dDisLTP > rr && dDisRTP > rr)
		{
			if (dDotLTP < 0 && dDotRTP < 0)
			{
				FindPointsBetweenPointAndCircle(m_vActorLoc, vMeshLoc, FVector::DistSquared(vMeshLoc, arrPoints[0].point), dTraceLengthSquared, vLP, vRP);

				AddPoint(vLP, pMeshCom);
				AddPoint(vRP, pMeshCom);
			}
			else if (dDotLTP >= 0 && dDotRTP >= 0)
			{
				AddPointIfTraceHit(vLTP, pMeshCom);
				AddPointIfTraceHit(vRTP, pMeshCom);
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("Not expected this case!"));
			}
		}
		//왼쪽점은 내부, 오른쪽점은 외부
		else if (dDisLTP <= rr && dDisRTP > rr)
		{
			//두 점 모두 원의 앞쪽에 있는 경우
			if (dDotLTP < 0 && dDotRTP < 0)
			{
				FindPointsBetweenPointAndCircle(m_vActorLoc, vMeshLoc, FVector::DistSquared(vMeshLoc, arrPoints[0].point), dTraceLengthSquared, vLP, vRP);

				AddPoint(vRP, pMeshCom);

				AddPointIfTraceHit(vLTP, pMeshCom);
			}
			//두 점 모두 원의 뒤쪽에 있는 경우
			else if (dDotLTP >= 0 && dDotRTP >= 0)
			{
				AddPointIfTraceHit(vLTP, pMeshCom);
				AddPointIfTraceHit(vRTP, pMeshCom);
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("Not expected this case!"));
			}
		}
		//왼쪽점은 외부, 오른쪽점은 내부
		else if (dDisLTP > rr && dDisRTP <= rr)
		{
			//두 점 모두 원의 앞쪽에 있는 경우
			if (dDotLTP < 0 && dDotRTP < 0)
			{
				FindPointsBetweenPointAndCircle(m_vActorLoc, vMeshLoc, FVector::DistSquared(vMeshLoc, arrPoints[0].point), dTraceLengthSquared, vLP, vRP);

				AddPoint(vLP, pMeshCom);

				AddPointIfTraceHit(vRTP, pMeshCom);
			}
			//두 점 모두 원의 뒤쪽에 있는 경우
			else if (dDotLTP >= 0 && dDotRTP >= 0)
			{
				AddPointIfTraceHit(vLTP, pMeshCom);
				AddPointIfTraceHit(vRTP, pMeshCom);
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("Not expected this case!"));
			}
		}
	}
	else if (bLTVInclude)
	{
		//View가 원의 오른쪽 부분을 포함할경우
		if (FVector::DistSquared(m_vActorLoc, vMRP) <= dTraceLengthSquared)
		{
			AddEdgePoint(vMRP, pMeshCom, m_dTraceAngleOffset);
			AddPointIfTraceHit(vLTP, pMeshCom);
		}
		else
		{
			//LTP가 원 내부에 있는 경우
			if (FVector::DistSquared(vMeshLoc, vLTP) <= rr)
			{
				FindPointsBetweenPointAndCircle(m_vActorLoc, vMeshLoc, FVector::DistSquared(vMeshLoc, arrPoints[0].point), dTraceLengthSquared, vLP, vRP);

				AddPoint(vRP, pMeshCom);
				AddPointIfTraceHit(vLTP, pMeshCom);
			}
			//외부에 있는 경우
			else
			{
				FindPointsBetweenPointAndCircle(m_vActorLoc, vMeshLoc, FVector::DistSquared(vMeshLoc, arrPoints[0].point), dTraceLengthSquared, vLP, vRP);

				AddPoint(vLP, pMeshCom);
				AddPoint(vRP, pMeshCom);
			}
		}
	}
	else if (bRTVInclude)
	{
		//View가 원의 왼쪽 부분을 포함할경우
		if (FVector::DistSquared(m_vActorLoc, vMLP) <= dTraceLengthSquared)
		{
			AddEdgePoint(vMLP, pMeshCom, -m_dTraceAngleOffset);
			AddPointIfTraceHit(vRTP, pMeshCom);
		}
		else
		{
			//RTP가 원 내부에 있는 경우
			if (FVector::DistSquared(vMeshLoc, vRTP) <= rr)
			{
				FindPointsBetweenPointAndCircle(m_vActorLoc, vMeshLoc, FVector::DistSquared(vMeshLoc, arrPoints[0].point), dTraceLengthSquared, vLP, vRP);

				AddPoint(vLP, pMeshCom);
				AddPointIfTraceHit(vRTP, pMeshCom);
			}
			//외부에 있는 경우
			else
			{
				FindPointsBetweenPointAndCircle(m_vActorLoc, vMeshLoc, FVector::DistSquared(vMeshLoc, arrPoints[0].point), dTraceLengthSquared, vLP, vRP);
				
				AddPoint(vLP, pMeshCom);
				AddPoint(vRP, pMeshCom);
			}
		}
	}
	else
	{
		//View가 완전히 원기둥을 포함할 경우
		if (FVector::DistSquared(m_vActorLoc, vMLP) <= dTraceLengthSquared)
		{
			AddEdgePoint(vMLP, pMeshCom, -m_dTraceAngleOffset);
			AddEdgePoint(vMRP, pMeshCom, m_dTraceAngleOffset);
		}
		//부채꼴의 앞쪽에 살짝 걸치는 경우
		else
		{
			FindPointsBetweenPointAndCircle(m_vActorLoc, vMeshLoc, FVector::DistSquared(vMeshLoc, arrPoints[0].point), dTraceLengthSquared, vLP, vRP);

			AddPoint(vLP, pMeshCom);
			AddPoint(vRP, pMeshCom);
		}
	}
}

void ALineOfSightActor::DrawPoint(const FVector& _vLoc, const FColor& _color)
{
	if(m_bIsShowTraceDebug)
		DrawDebugPoint(GetWorld(), _vLoc, 5, _color, false, -1, .99);
}

void ALineOfSightActor::DrawLine(const FVector& _vStart, const FVector& _vEnd, const FColor& _color)
{
	if (m_bIsShowTraceDebug)
		DrawDebugLine(GetWorld(), _vStart, _vEnd, _color, false, -1, 1, .99);
}
