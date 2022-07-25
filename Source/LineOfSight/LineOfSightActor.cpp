#include "LineOfSightActor.h"
#include "AI/Navigation/NavCollisionBase.h"
#include "DrawDebugHelpers.h"
#include "GenericPlatform/GenericPlatformMath.h"
#include "Kismet/KismetSystemLibrary.h"

void ALineOfSightActor::Tick(float _fDelta)
{
	Super::Tick(_fDelta);
	m_bIsPointCalculated = false;
}

ALineOfSightActor::ALineOfSightActor()
{
	PrimaryActorTick.bCanEverTick = true;
}

//TODO: 두 큐브가 서로 겹치는 부분에서 어두운 부분 생김
TArray<FMeshPoint> ALineOfSightActor::GetDetectedPoints()
{
	if (!m_bIsPointCalculated)
		CalculateValidPoints();

	return m_arrDetectedPoints;
}

void ALineOfSightActor::AddAdjMesh(UStaticMeshComponent* _pMeshCom)
{
	UE_LOG(LogTemp, Log, TEXT("AddAdjMesh"));
	if (_pMeshCom && !m_mapDetectedObstacles.Contains(_pMeshCom))
	{
		UE_LOG(LogTemp, Log, TEXT("AddAdjMesh2"));
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
	}
}

void ALineOfSightActor::RemoveAdjMesh(UStaticMeshComponent* _pMeshCom)
{
	UE_LOG(LogTemp, Log, TEXT("RemoveAdjMesh1"));
	if (_pMeshCom)
	{
		UE_LOG(LogTemp, Log, TEXT("RemoveAdjMesh2"));
		//m_mapDetectedObstacles.Remove(_pMeshCom);
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

FMeshPoint ALineOfSightActor::GetTracedPoint(const FVector& _vBasePoint, double _dAngle)
{
	FVector vTraceDir = (_vBasePoint - m_vActorLoc).GetUnsafeNormal();
	vTraceDir = vTraceDir.RotateAngleAxis(_dAngle, FVector3d::UpVector);

	FVector rTraceEnd = m_vActorLoc + vTraceDir * m_dTraceLength;
	TArray<AActor*> temp;
	FHitResult result;
	if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, rTraceEnd, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, temp, EDrawDebugTrace::Type::ForOneFrame, result, true))
	{
		return { result.GetActor()->FindComponentByClass<UStaticMeshComponent>(), result.Location };
	}
	else
	{
		return { nullptr, rTraceEnd };
	}
}

void ALineOfSightActor::CalculateValidPoints()
{
	if (m_bIsPointCalculated)
		return;

	m_bIsPointCalculated = true;

	m_arrDetectedPoints.Empty();

	double dTraceAngleOffset = .001;

	//View Angle 안에 있는지 체크
	//왼쪽 Trace 벡터와 Forward 벡터의 내적보다 큰 점들은 내부에 있는 점
	m_vActorLoc = GetActorLocation();
	const FVector& vForward = GetActorForwardVector();
	const FVector& vBack = -vForward;
	const FVector& vLeftTrace = vForward.RotateAngleAxis(-m_dViewAngle / 2, FVector3d::UpVector);
	const FVector& vRightTrace = vForward.RotateAngleAxis(m_dViewAngle / 2, FVector3d::UpVector);
	double dTraceLengthSquared = m_dTraceLength * m_dTraceLength;

	double dFLDot = vForward.Dot(vLeftTrace);

	if (m_bIsShowTraceDebug)
	{
		DrawDebugLine(GetWorld(), m_vActorLoc, m_vActorLoc + vLeftTrace * m_dTraceLength, FColor::Blue, false, -1, 1);
		DrawDebugLine(GetWorld(), m_vActorLoc, m_vActorLoc + vRightTrace * m_dTraceLength, FColor::Blue, false, -1, 1);
	}

	for (const auto& pair : m_mapDetectedObstacles)
	{
		UStaticMeshComponent* pMeshCom = pair.Key;
		UKismetSystemLibrary::GetDisplayName(pMeshCom->GetOwner());
		//장애물과 시점으로의 벡터를 기준으로 점들의 각도를 계산
		const FVector& vAngleBase = (m_vActorLoc - pMeshCom->GetOwner()->GetActorLocation()).GetUnsafeNormal();
		m_dBaseAngle = vAngleBase.Rotation().Yaw >= 0 ? vAngleBase.Rotation().Yaw : 360 + vAngleBase.Rotation().Yaw;

		TArray<FMeshPoint> arrPoints = pair.Value;
		TArray<FMeshPoint> arrValidPoints;
		//각도에 따라 정렬
		arrPoints.Sort([&](const FMeshPoint& lhs, const FMeshPoint& rhs) { return SortByAngle(lhs, rhs); });

		//장애물이 시야각과 겹치는지 체크
		{
			bool bValid = false;

			const FVector& vObsLeft = (arrPoints[0].point - m_vActorLoc).GetUnsafeNormal();
			const FVector& vObsRight = (arrPoints.Last().point - m_vActorLoc).GetUnsafeNormal();

			double dObsDot = vObsRight.Dot(vObsLeft);
			bValid |= vObsLeft.Dot(vLeftTrace) >= dObsDot && vObsRight.Dot(vLeftTrace) >= dObsDot ||
				vObsLeft.Dot(vRightTrace) >= dObsDot && vObsRight.Dot(vRightTrace) >= dObsDot;

			//물체 전체가 시야각 안에 들어온 경우
			dObsDot = vLeftTrace.Dot(vRightTrace);
			bValid |= vObsLeft.Dot(vLeftTrace) >= dObsDot && vObsLeft.Dot(vRightTrace) >= dObsDot &&
				vObsRight.Dot(vLeftTrace) >= dObsDot && vObsRight.Dot(vRightTrace) >= dObsDot;

			if (!bValid)
				continue;
		}

		//점 갯수로 큐브인지 원기둥인지 구분
		//큐브인 경우
		if (arrPoints.Num() == 4)
		{
			arrValidPoints.Reserve(4);
			//1면만 보고 있는지 2면을 보고 있는지 구분
			//가운데 각도 2개 중 가장 가까운 점이 있다면 2면, 아니라면 1면
			TArray<int> arrInRangePointIdx;
			double dMaxDis = -1;
			double dMinDis = MAX_dbl;
			int iMaxIdx = 0;
			int iMinIdx = 0;
			for (int i = 0; i < 4; ++i)
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

			DrawDebugPoint(GetWorld(), arrPoints[0].point, 10, FColor::Blue, false, -1, 1);
			DrawDebugPoint(GetWorld(), arrPoints[1].point, 10, FColor::Red, false, -1, 1);
			DrawDebugPoint(GetWorld(), arrPoints[2].point, 10, FColor::Cyan, false, -1, 1);
			DrawDebugPoint(GetWorld(), arrPoints[3].point, 10, FColor::Black, false, -1, 1);

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
					UE_LOG(LogTemp, Error, TEXT("Cube 2 side but zero point!"));
					continue;
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

						arrValidPoints.Emplace(GetTracedPoint(vLeftPoint, dTraceAngleOffset));
						arrValidPoints.Emplace(GetTracedPoint(vLeftPoint, -dTraceAngleOffset));

						DrawDebugPoint(GetWorld(), vLeftPoint, 10, FColor::Blue, false, -1, 1);
					}

					arrValidPoints.Emplace(GetTracedPoint(vMidPoint.point, 0));

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

						arrValidPoints.Emplace(GetTracedPoint(vRightPoint, dTraceAngleOffset));
						arrValidPoints.Emplace(GetTracedPoint(vRightPoint, -dTraceAngleOffset));

						DrawDebugPoint(GetWorld(), vRightPoint, 10, FColor::Red, false, -1, 1);
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

							arrValidPoints.Emplace(GetTracedPoint(vRightPoint, -dTraceAngleOffset));
							arrValidPoints.Emplace(GetTracedPoint(vRightPoint, dTraceAngleOffset));

							DrawDebugPoint(GetWorld(), vRightPoint, 10, FColor::Red, false, -1, 1);
						}

						arrValidPoints.Emplace(GetTracedPoint(vMostLeftPoint.point, -dTraceAngleOffset));
						arrValidPoints.Emplace(GetTracedPoint(vMostLeftPoint.point, dTraceAngleOffset));
						arrValidPoints.Emplace(GetTracedPoint(vMidPoint.point, 0));
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

							arrValidPoints.Emplace(GetTracedPoint(vLeftPoint, -dTraceAngleOffset));
							arrValidPoints.Emplace(GetTracedPoint(vLeftPoint, dTraceAngleOffset));

							DrawDebugPoint(GetWorld(), vLeftPoint, 10, FColor::Blue, false, -1, 1);
						}

						arrValidPoints.Emplace(GetTracedPoint(vMidPoint.point, 0));
						arrValidPoints.Emplace(GetTracedPoint(vMostRightPoint.point, -dTraceAngleOffset));
						arrValidPoints.Emplace(GetTracedPoint(vMostRightPoint.point, dTraceAngleOffset));
					}
				}
				//3, 4개인 경우
				else
				{
					arrValidPoints.Emplace(GetTracedPoint(vMostLeftPoint.point, -dTraceAngleOffset));
					arrValidPoints.Emplace(GetTracedPoint(vMostLeftPoint.point, dTraceAngleOffset));
					arrValidPoints.Emplace(GetTracedPoint(vMidPoint.point, 0));
					arrValidPoints.Emplace(GetTracedPoint(vMostRightPoint.point, -dTraceAngleOffset));
					arrValidPoints.Emplace(GetTracedPoint(vMostRightPoint.point, dTraceAngleOffset));
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

					arrValidPoints.Emplace(GetTracedPoint(vLeftPoint, 0));
					arrValidPoints.Emplace(GetTracedPoint(vRightPoint, 0));

					DrawDebugPoint(GetWorld(), vLeftPoint, 10, FColor::Blue, false, -1, 1);
					DrawDebugPoint(GetWorld(), vRightPoint, 10, FColor::Red, false, -1, 1);
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

						arrValidPoints.Emplace(GetTracedPoint(vMostLeftPoint.point, dTraceAngleOffset));
						arrValidPoints.Emplace(GetTracedPoint(vMostLeftPoint.point, -dTraceAngleOffset));
						arrValidPoints.Emplace(GetTracedPoint(vRightPoint, 0));
					}
					//오른쪽 점이 포함된 경우
					else if (arrInRangePointIdx[0] == 3)
					{
						//계산된 점 중 왼쪽 점에 가까운 점을 추가
						FVector vLeftPoint;
						FVector vLeftTraceEnd = m_vActorLoc + vLeftTrace * m_dTraceLength;
						TryFindTwoLineCrossPoint(m_vActorLoc, vLeftTraceEnd, vLeftTrace, vMostLeftPoint.point, vMostRightPoint.point, dTraceLengthSquared, vLeftPoint);

						arrValidPoints.Emplace(GetTracedPoint(vMostRightPoint.point, dTraceAngleOffset));
						arrValidPoints.Emplace(GetTracedPoint(vMostRightPoint.point, -dTraceAngleOffset));
						arrValidPoints.Emplace(GetTracedPoint(vLeftPoint, 0));
					}
					else
					{
						UE_LOG(LogTemp, Error, TEXT("Algorithm Broken!"));
					}
				}
				//2개인 경우
				else
				{
					arrValidPoints.Emplace(GetTracedPoint(vMostRightPoint.point, dTraceAngleOffset));
					arrValidPoints.Emplace(GetTracedPoint(vMostRightPoint.point, -dTraceAngleOffset));
					arrValidPoints.Emplace(GetTracedPoint(vMostLeftPoint.point, dTraceAngleOffset));
					arrValidPoints.Emplace(GetTracedPoint(vMostLeftPoint.point, -dTraceAngleOffset));
				}
			}
		}
		else
		{

		}

		//Point와 Mesh를 맵핑한 다음 서로 다른 Mesh 위에 있는 Point롤 삼각형을 만들시 그 사이를 Trace해야
		//Forward 벡터와의 내적을 왼쪽 Trace 벡터와 비교하여 크다면 시야범위 내에 있음
		for (const FMeshPoint& p : arrValidPoints)
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
