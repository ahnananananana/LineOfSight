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

//TODO: �� ť�갡 ���� ��ġ�� �κп��� ��ο� �κ� ����
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
				//Actor�� Z���̻��� �͸� ���͸�
				TArray<FMeshPoint>& arrPoints = m_mapDetectedObstacles.Emplace(_pMeshCom);
				float fActorPosZ = GetActorLocation().Z;
				for (const FVector& p : pNavCol->GetConvexCollision().VertexBuffer)
				{
					const FVector& tp = _pMeshCom->GetOwner()->GetActorTransform().TransformPosition(p);
					if (tp.Z <= fActorPosZ)
					{
						//�ش� Z ��ġ�� ���� �� �߰�
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

	//View Angle �ȿ� �ִ��� üũ
	//���� Trace ���Ϳ� Forward ������ �������� ū ������ ���ο� �ִ� ��
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
		//��ֹ��� ���������� ���͸� �������� ������ ������ ���
		const FVector& vAngleBase = (m_vActorLoc - pMeshCom->GetOwner()->GetActorLocation()).GetUnsafeNormal();
		m_dBaseAngle = vAngleBase.Rotation().Yaw >= 0 ? vAngleBase.Rotation().Yaw : 360 + vAngleBase.Rotation().Yaw;

		TArray<FMeshPoint> arrPoints = pair.Value;
		TArray<FMeshPoint> arrValidPoints;
		//������ ���� ����
		arrPoints.Sort([&](const FMeshPoint& lhs, const FMeshPoint& rhs) { return SortByAngle(lhs, rhs); });

		//��ֹ��� �þ߰��� ��ġ���� üũ
		{
			bool bValid = false;

			const FVector& vObsLeft = (arrPoints[0].point - m_vActorLoc).GetUnsafeNormal();
			const FVector& vObsRight = (arrPoints.Last().point - m_vActorLoc).GetUnsafeNormal();

			double dObsDot = vObsRight.Dot(vObsLeft);
			bValid |= vObsLeft.Dot(vLeftTrace) >= dObsDot && vObsRight.Dot(vLeftTrace) >= dObsDot ||
				vObsLeft.Dot(vRightTrace) >= dObsDot && vObsRight.Dot(vRightTrace) >= dObsDot;

			//��ü ��ü�� �þ߰� �ȿ� ���� ���
			dObsDot = vLeftTrace.Dot(vRightTrace);
			bValid |= vObsLeft.Dot(vLeftTrace) >= dObsDot && vObsLeft.Dot(vRightTrace) >= dObsDot &&
				vObsRight.Dot(vLeftTrace) >= dObsDot && vObsRight.Dot(vRightTrace) >= dObsDot;

			if (!bValid)
				continue;
		}

		//�� ������ ť������ ��������� ����
		//ť���� ���
		if (arrPoints.Num() == 4)
		{
			arrValidPoints.Reserve(4);
			//1�鸸 ���� �ִ��� 2���� ���� �ִ��� ����
			//��� ���� 2�� �� ���� ����� ���� �ִٸ� 2��, �ƴ϶�� 1��
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

				//�þ߹��� �ȿ� �ִ��� üũ
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
				//2���� �� �� �ִ� ��ġ���� 1�鸸 ���� ���
				if (vForward.Dot(v) + .001 < dFLDot)
				{
					bIsTwoSide = false;
					if (GetActorRightVector().Dot((arrPoints[iMinIdx].point - m_vActorLoc).GetUnsafeNormal()) > 0)
					{
						//TODO: vMostRightPoint ���̹��� �����ϰ� �ٲ��
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

			//2���� ���
			if (bIsTwoSide)
			{
				//���� �� ���� ������ �ʿ�����Ƿ� ����
				arrInRangePointIdx.Remove(iMaxIdx);

				const FMeshPoint& vMidPoint = arrPoints[iMinIdx];
				//Trace �Ÿ� �ȿ� ���ԵǴ� ���� ������ ���� ó��
				//0���� ���
				if (arrInRangePointIdx.Num() == 0)
				{
					UE_LOG(LogTemp, Error, TEXT("Cube 2 side but zero point!"));
					continue;
				}
				//1���� ���
				else if (arrInRangePointIdx.Num() == 1)
				{
					{
						FVector vLeftPoint;
						//�ϴ� ���� Trace ���Ϳ� ������ Trace ���� ������ �������� ���� ���
						FVector vLeftTraceEnd = m_vActorLoc + vLeftTrace * m_dTraceLength;

						//�Ÿ� ���̶�� ��ȿ�� ���� �ƴϹǷ� �ش� ���� �ٽ� ���
						//������ ���� ���� �Ÿ��� �þ߰Ÿ���ŭ�� ���� ���
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
						//�ϴ� ���� Trace ���Ϳ� ������ Trace ���� ������ �������� ���� ���
						FVector vRightTraceEnd = m_vActorLoc + vRightTrace * m_dTraceLength;

						//�Ÿ� ���̶�� ��ȿ�� ���� �ƴϹǷ� �ش� ���� �ٽ� ���
						//������ ���� ���� �Ÿ��� �þ߰Ÿ���ŭ�� ���� ���
						if (!TryFindTwoLineCrossPoint(m_vActorLoc, vRightTraceEnd, vRightTrace, vMidPoint.point, vMostRightPoint.point, dTraceLengthSquared, vRightPoint))
						{
							vRightPoint = FindRightPointBetweenPointAndLine(m_vActorLoc, vMidPoint.point, vMostRightPoint.point, dTraceLengthSquared);
						}

						arrValidPoints.Emplace(GetTracedPoint(vRightPoint, dTraceAngleOffset));
						arrValidPoints.Emplace(GetTracedPoint(vRightPoint, -dTraceAngleOffset));

						DrawDebugPoint(GetWorld(), vRightPoint, 10, FColor::Red, false, -1, 1);
					}
				}
				//2���� ���
				else if (arrInRangePointIdx.Num() == 2)
				{
					//���� ���� ����� ���� �����ϴ� ���
					if (arrInRangePointIdx[0] == 0)
					{
						//������ ���� ��� �� ������ ���� ����Ͽ� �߰�
						{
							FVector vRightPoint;
							//�ϴ� ���� Trace ���Ϳ� ������ Trace ���� ������ �������� ���� ���
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
					//������ ���� ����� ���� �����ϴ� ���
					else if (arrInRangePointIdx[1] == 3)
					{
						//���� ���� ��� �� ������ ���� ����Ͽ� �߰�
						{
							FVector vLeftPoint;
							//�ϴ� ���� Trace ���Ϳ� ������ Trace ���� ������ �������� ���� ���
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
				//3, 4���� ���
				else
				{
					arrValidPoints.Emplace(GetTracedPoint(vMostLeftPoint.point, -dTraceAngleOffset));
					arrValidPoints.Emplace(GetTracedPoint(vMostLeftPoint.point, dTraceAngleOffset));
					arrValidPoints.Emplace(GetTracedPoint(vMidPoint.point, 0));
					arrValidPoints.Emplace(GetTracedPoint(vMostRightPoint.point, -dTraceAngleOffset));
					arrValidPoints.Emplace(GetTracedPoint(vMostRightPoint.point, dTraceAngleOffset));
				}
			}
			//1���� ���
			else
			{
				//���� ������ ��� ����
				arrInRangePointIdx.RemoveAll([&](int e) {return e != vMostLeftIdx && e != vMostRightIdx; });

				//Trace �Ÿ� �ȿ� ���ԵǴ� ���� ������ ���� ó��
				//0���� ���
				if (arrInRangePointIdx.Num() == 0)
				{
					FVector vLeftPoint, vRightPoint;
					//�ϴ� ���� Trace ���Ϳ� ������ Trace ���� ������ �������� ���� ���
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
				//1���� ���
				else if (arrInRangePointIdx.Num() == 1)
				{
					//���� ���� ���Ե� ���
					if (arrInRangePointIdx[0] == 0)
					{
						//������ Trace ���Ϳ��� ���� ���
						FVector vRightPoint;
						FVector vRightTraceEnd = m_vActorLoc + vRightTrace * m_dTraceLength;
						TryFindTwoLineCrossPoint(m_vActorLoc, vRightTraceEnd, vRightTrace, vMostLeftPoint.point, vMostRightPoint.point, dTraceLengthSquared, vRightPoint);

						arrValidPoints.Emplace(GetTracedPoint(vMostLeftPoint.point, dTraceAngleOffset));
						arrValidPoints.Emplace(GetTracedPoint(vMostLeftPoint.point, -dTraceAngleOffset));
						arrValidPoints.Emplace(GetTracedPoint(vRightPoint, 0));
					}
					//������ ���� ���Ե� ���
					else if (arrInRangePointIdx[0] == 3)
					{
						//���� �� �� ���� ���� ����� ���� �߰�
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
				//2���� ���
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

		//Point�� Mesh�� ������ ���� ���� �ٸ� Mesh ���� �ִ� Point�� �ﰢ���� ����� �� ���̸� Trace�ؾ�
		//Forward ���Ϳ��� ������ ���� Trace ���Ϳ� ���Ͽ� ũ�ٸ� �þ߹��� ���� ����
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

	//��� ���� �ٽ� ������ ����
	m_dBaseAngle = vBack.Rotation().Yaw >= 0 ? vBack.Rotation().Yaw : 360 + vBack.Rotation().Yaw;
	m_arrDetectedPoints.Sort([&](const FMeshPoint& lhs, const FMeshPoint& rhs) { return SortByAngle(lhs, rhs); });
}

bool ALineOfSightActor::TryFindTwoLineCrossPoint(const FVector& _p1, const FVector& _p2, const FVector& vP1P2, const FVector& _p3, const FVector& _p4, double _dDistSquared, FVector& _vCrossPoint)
{
	double denominator = (_p1.X - _p2.X) * (_p3.Y - _p4.Y) - (_p1.Y - _p2.Y) * (_p3.X - _p4.X);
	//�����ϰų� ��ġ�ϴ� ���
	if (denominator == 0)
		return false;

	_vCrossPoint.X = ((_p1.X * _p2.Y - _p1.Y * _p2.X) * (_p3.X - _p4.X) - (_p1.X - _p2.X) * (_p3.X * _p4.Y - _p3.Y * _p4.X)) / denominator;
	_vCrossPoint.Y = ((_p1.X * _p2.Y - _p1.Y * _p2.X) * (_p3.Y - _p4.Y) - (_p1.Y - _p2.Y) * (_p3.X * _p4.Y - _p3.Y * _p4.X)) / denominator;
	_vCrossPoint.Z = m_vActorLoc.Z;

	//�� ���� ������ ���� ���̿� �ִ��� üũ
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

	//������� �� ��������ϹǷ� -
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

	//������� �� ��������ϹǷ� -
	double t = (-b + FMath::Sqrt(squared)) / (2 * a);

	FVector vResult = _vLintPoint1 + v * t;
	vResult.Z = m_vActorLoc.Z;
	return vResult;
}
