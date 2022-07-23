#include "LineOfSightActor.h"
#include "AI/Navigation/NavCollisionBase.h"
#include "DrawDebugHelpers.h"
#include "GenericPlatform/GenericPlatformMath.h"

ALineOfSightActor::ALineOfSightActor()
{
	PrimaryActorTick.bCanEverTick = true;
}


TArray<FVector> ALineOfSightActor::GetDetectedPoints()
{
	TArray<FVector> result;

	//View Angle 안에 있는지 체크
	//왼쪽 Trace 벡터와 Forward 벡터의 내적보다 큰 점들은 내부에 있는 점
	m_vActorLoc = GetActorLocation();
	const FVector& vForward = GetActorForwardVector();
	const FVector& vBack = -vForward;
	const FVector& vLeft = vForward.RotateAngleAxis(-m_fViewAngle / 2, FVector3d::UpVector);
	const FVector& vRight = vForward.RotateAngleAxis(m_fViewAngle / 2, FVector3d::UpVector);

	m_dBaseAngle = vBack.Rotation().Yaw >= 0 ? vBack.Rotation().Yaw : 360 + vBack.Rotation().Yaw;

	double dFLDot = vForward.Dot(vLeft);

	if (m_bIsShowTraceDebug)
	{
		DrawDebugLine(GetWorld(), m_vActorLoc, m_vActorLoc + vLeft * m_fTraceLength, FColor::Blue);
		DrawDebugLine(GetWorld(), m_vActorLoc, m_vActorLoc + vRight * m_fTraceLength, FColor::Blue);
	}

	for (const auto& pair : m_mapDetectedObstacles)
	{
		TArray<FVector> arrPoints = pair.Value;
		TArray<FVector> arrValidPoints;
		//각도에 따라 정렬
		arrPoints.Sort([&](const FVector& lhs, const FVector& rhs) { return SortByAngle(lhs, rhs); });

		//점 갯수로 큐브인지 원기둥인지 구분
		//큐브인 경우
		if(arrPoints.Num() == 4)
		{
			arrValidPoints.Reserve(4);
			//1면만 보고 있는지 2면을 보고 있는지 구분
			//가운데 각도 2개 중 가장 가까운 점이 있다면 2면, 아니라면 1면
			TArray<int> arrInRangePointIdx;
			double dMinDis = MAX_dbl;
			int iMinIdx = 0;
			for (int i = 0; i < 4; ++i)
			{
				double dis = FVector::Dist(m_vActorLoc, arrPoints[i]);
				if (dis < dMinDis)
				{
					dMinDis = dis;
					iMinIdx = i;
				}

				//시야범위 안에 있는지 체크
				if (dis <= m_fTraceLength)
				{
					arrInRangePointIdx.Add(i);
				}
			}

			const FVector& vLeftPoint = arrPoints[0];
			const FVector& vRightPoint = arrPoints[3];
			//2면인 경우
			if (iMinIdx == 1 || iMinIdx == 2)
			{
				const FVector& vMidPoint = arrPoints[iMinIdx];
				//Trace 거리 안에 포함되는 점의 갯수에 따라 처리
				//0개인 경우
				if (arrInRangePointIdx.Num() == 0)
				{
					UE_LOG(LogTemp, Error, TEXT("큐브 2면인데 점이 0개!"));
				}
				//1개인 경우
				else if (arrInRangePointIdx.Num() == 1)
				{
					//가운데 점과 양쪽 점 사이의 점을 각각 계산하여 추가
					arrValidPoints.Add(vMidPoint);
				}
				//2개인 경우
				else if (arrInRangePointIdx.Num() == 2)
				{
					//왼쪽 점과 가까운 점만 포함하는 경우
					if (arrInRangePointIdx[0] == 0)
					{
						//오른쪽 점과 가운데 점 사이의 점을 계산하여 추가

						arrValidPoints.Add(vLeftPoint);
						arrValidPoints.Add(vMidPoint);
					}
					//오른쪽 점과 가까운 점만 포함하는 경우
					else if (arrInRangePointIdx[1] == 3)
					{
						//왼쪽 점과 가운데 점 사이의 점을 계산하여 추가

						arrValidPoints.Add(vMidPoint);
						arrValidPoints.Add(vRightPoint);
					}
				}
				//3, 4개인 경우
				else
				{
					arrValidPoints.Add(vLeftPoint);
					if (arrInRangePointIdx.Num() == 3)
					{
						arrValidPoints.Add(vMidPoint);
					}
					else
					{
						arrValidPoints.Add(arrPoints[1]);
						arrValidPoints.Add(arrPoints[2]);
					}
					arrValidPoints.Add(vRightPoint);
				}
			}
			//1면인 경우
			else
			{
				//Trace 거리 안에 포함되는 점의 갯수에 따라 처리
				//0개인 경우
				if (arrInRangePointIdx.Num() == 0)
				{
					//오른쪽 점과 왼쪽 점으로 이루어진 직선 위의 점 중
					//시점과의 거리가 시야거리만큼 있는 2개의 점을 계산하여 추가
				}
				//1개인 경우
				else if (arrInRangePointIdx.Num() == 1)
				{
					//오른쪽 점과 왼쪽 점으로 이루어진 직선 위의 점 중
					//시점과의 거리가 시야거리만큼 있는 2개의 점을 계산

					//왼쪽 점이 포함된 경우
					if (arrInRangePointIdx[0] == 0)
					{
						//계산된 점 중 오른쪽 점에 가까운 점을 추가
						arrValidPoints.Add(vLeftPoint);
					}
					//오른쪽 점이 포함된 경우
					else if (arrInRangePointIdx[0] == 3)
					{
						//계산된 점 중 왼쪽 점에 가까운 점을 추가
						arrValidPoints.Add(vRightPoint);
					}
					else
					{
						UE_LOG(LogTemp, Error, TEXT("Algorithm Broken!"));
					}
				}
				//2개인 경우
				else if (arrInRangePointIdx.Num() == 2)
				{
					//뒤쪽 점이 있는 경우
					if (arrInRangePointIdx[0] != 0 || arrInRangePointIdx[1] != 3)
					{
						//오른쪽 점과 왼쪽 점으로 이루어진 직선 위의 점 중
						//시점과의 거리가 시야거리만큼 있는 2개의 점을 계산

						//왼쪽 점이 포함된 경우
						if (arrInRangePointIdx[0] == 0)
						{
							//계산된 점 중 오른쪽 점에 가까운 점을 추가
							arrValidPoints.Add(vLeftPoint);
						}
						//오른쪽 점이 포함된 경우
						else
						{
							//계산된 점 중 왼쪽 점에 가까운 점을 추가
							arrValidPoints.Add(vRightPoint);
						}
					}
					//앞쪽 점들만 있는 경우
					else
					{
						arrValidPoints.Add(vLeftPoint);
						arrValidPoints.Add(vRightPoint);
					}
				}
				//3, 4개인 경우
				else
				{
					//양 끝쪽에 있는 점만 추가
					arrValidPoints.Add(vLeftPoint);
					arrValidPoints.Add(vRightPoint);
				}
			}
		}
		else
		{

		}

		//Forward 벡터와의 내적을 왼쪽 Trace 벡터와 비교하여 크다면 시야범위 내에 있음
		for (const FVector& p : arrValidPoints)
		{
			FVector v = p - m_vActorLoc;
			v.Normalize();
			if (vForward.Dot(v) >= dFLDot && FVector::Dist(m_vActorLoc, p) <= m_fTraceLength)
			{
				result.Add(p);
				break;
			}
		}
	}

	//모든 점을 다시 각도로 정렬
	result.Sort([&](const FVector& lhs, const FVector& rhs) { return SortByAngle(lhs, rhs); });

	return result;
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
				TArray<FVector>& arrPoints = m_mapDetectedObstacles.Emplace(_pMeshCom);
				float fActorPosZ = GetActorLocation().Z;
				for (const FVector& p : pNavCol->GetConvexCollision().VertexBuffer)
				{
					const FVector& tp = _pMeshCom->GetOwner()->GetActorTransform().TransformPosition(p);
					if (tp.Z <= fActorPosZ)
					{
						//해당 Z 위치로 변경 후 추가
						arrPoints.Emplace(tp.X, tp.Y, fActorPosZ);
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
		m_mapDetectedObstacles.Remove(_pMeshCom);
	}
}

bool ALineOfSightActor::SortByAngle(const FVector& lhs, const FVector& rhs)
{
	FVector v1 = lhs - m_vActorLoc;
	FVector v2 = rhs - m_vActorLoc;

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