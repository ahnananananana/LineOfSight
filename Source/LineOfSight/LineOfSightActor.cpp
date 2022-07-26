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

//TODO: �� ť�갡 ���� ��ġ�� �κп��� ��ο� �κ� ����, �߰� Ʈ���̽��� �� �ȵ�
//TODO: �񽺵��� �� Offset ó���ؾ�
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

bool ALineOfSightActor::IsInViewAngle(const FVector& _vLeftTrace, const FVector& _vRightTrace, const FVector& _vMLV, const FVector& _vMRV, bool& _bLTVInclude, bool& _bRTVInclude)
{
	double dObsDot = _vMRV.Dot(_vMLV);

	_bLTVInclude = _vMLV.Dot(_vLeftTrace) >= dObsDot && _vMRV.Dot(_vLeftTrace) >= dObsDot;
	_bRTVInclude = _vMLV.Dot(_vRightTrace) >= dObsDot && _vMRV.Dot(_vRightTrace) >= dObsDot;

	bool bValid = _bLTVInclude || _bRTVInclude;

	//��ü ��ü�� �þ߰� �ȿ� ���� ���
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
	const FVector& vLTP = m_vActorLoc + vLeftTrace * m_dTraceLength;
	const FVector& vRTP = m_vActorLoc + vRightTrace * m_dTraceLength;

	double dFLDot = vForward.Dot(vLeftTrace);

	if (m_bIsShowTraceDebug)
	{
		DrawDebugLine(GetWorld(), m_vActorLoc, m_vActorLoc + vLeftTrace * m_dTraceLength, FColor::Blue, false, -1, 1,1);
		DrawDebugLine(GetWorld(), m_vActorLoc, m_vActorLoc + vRightTrace * m_dTraceLength, FColor::Orange, false, -1, 1,1);
	}

	for (const auto& pair : m_mapDetectedObstacles)
	{
		UStaticMeshComponent* pMeshCom = pair.Key;
		UE_LOG(LogTemp, Log, TEXT("%s"), *UKismetSystemLibrary::GetDisplayName(pMeshCom->GetOwner()));
		TArray<FMeshPoint> arrPoints = pair.Value;
		if (arrPoints.IsEmpty())
		{
			UE_LOG(LogTemp, Error, TEXT("No Points"));
			continue;
		}

		FVector vMeshLoc = pMeshCom->GetOwner()->GetActorLocation();
		vMeshLoc.Z = m_vActorLoc.Z;

		FVector vMLV, vMRV;
		bool bLTVInclude, bRTVInclude;
		//�� ������ ť������ ��������� ����
		//ť���� ���
		if (arrPoints.Num() == 4)
		{
			//��ֹ��� ���������� ���͸� �������� ������ ������ ���
			const FVector& vAngleBase = (m_vActorLoc - vMeshLoc).GetUnsafeNormal();
			m_dBaseAngle = vAngleBase.Rotation().Yaw >= 0 ? vAngleBase.Rotation().Yaw : 360 + vAngleBase.Rotation().Yaw;

			m_arrValidPoints.Empty();
			//������ ���� ����
			arrPoints.Sort([&](const FMeshPoint& lhs, const FMeshPoint& rhs) { return SortByAngle(lhs, rhs); });

			//�þ߹��� �ȿ� �ִ� �� üũ
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

				//�þ߹��� �ȿ� �ִ��� üũ
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
				continue;
			}

			m_arrValidPoints.Reserve(4);
			//1�鸸 ���� �ִ��� 2���� ���� �ִ��� ����
			//��� ���� 2�� �� ���� ����� ���� �ִٸ� 2��, �ƴ϶�� 1��

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

			if (m_bIsShowTraceDebug)
			{
				DrawDebugPoint(GetWorld(), arrPoints[0].point, 10, FColor::Blue, false, -1, 1);
				DrawDebugPoint(GetWorld(), arrPoints[1].point, 10, FColor::Red, false, -1, 1);
				DrawDebugPoint(GetWorld(), arrPoints[2].point, 10, FColor::Cyan, false, -1, 1);
				DrawDebugPoint(GetWorld(), arrPoints[3].point, 10, FColor::Black, false, -1, 1);
			}

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

						AddEdgePoint(vLeftPoint, pMeshCom, -m_dTraceAngleOffset);

						//DrawDebugPoint(GetWorld(), vLeftPoint, 10, FColor::Blue, false, -1, 1);
					}

					m_arrValidPoints.Emplace(GetTracedPoint(vMidPoint.point, 0));

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

						AddEdgePoint(vRightPoint, pMeshCom, m_dTraceAngleOffset);

						//DrawDebugPoint(GetWorld(), vRightPoint, 10, FColor::Red, false, -1, 1);
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

							AddPoint(vRightPoint, pMeshCom);

							//DrawDebugPoint(GetWorld(), vRightPoint, 10, FColor::Red, false, -1, 1);
						}

						AddEdgePoint(vMostLeftPoint.point, pMeshCom, -m_dTraceAngleOffset);
						AddPoint(vMidPoint.point, pMeshCom);
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

							AddPoint(vLeftPoint, pMeshCom);

							//DrawDebugPoint(GetWorld(), vLeftPoint, 10, FColor::Blue, false, -1, 1);
						}

						AddEdgePoint(vMostRightPoint.point, pMeshCom, m_dTraceAngleOffset);
						AddPoint(vMidPoint.point, pMeshCom);
					}
				}
				//3, 4���� ���
				else
				{
					AddEdgePoint(vMostLeftPoint.point, pMeshCom, -m_dTraceAngleOffset);
					AddEdgePoint(vMostRightPoint.point, pMeshCom, m_dTraceAngleOffset);
					AddPoint(vMidPoint.point, pMeshCom);
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

					AddPoint(vLeftPoint, pMeshCom);
					AddPoint(vRightPoint, pMeshCom);

					//DrawDebugPoint(GetWorld(), vLeftPoint, 10, FColor::Blue, false, -1, 1);
					//DrawDebugPoint(GetWorld(), vRightPoint, 10, FColor::Red, false, -1, 1);
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

						AddEdgePoint(vMostLeftPoint.point, pMeshCom, -m_dTraceAngleOffset);
						AddPoint(vRightPoint, pMeshCom);
					}
					//������ ���� ���Ե� ���
					else if (arrInRangePointIdx[0] == 3)
					{
						//���� �� �� ���� ���� ����� ���� �߰�
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
				//2���� ���
				else
				{
					AddEdgePoint(vMostLeftPoint.point, pMeshCom, -m_dTraceAngleOffset);
					AddEdgePoint(vMostRightPoint.point, pMeshCom, m_dTraceAngleOffset);
				}
			}
		}
		else
		{
			double rr = FVector::DistSquared(vMeshLoc, arrPoints[0].point);
			double dRadius = FVector::Dist(vMeshLoc, arrPoints[0].point);
			double r = dRadius;
			double d = vMeshLoc.X;
			double e = vMeshLoc.Y;
			double f = m_vActorLoc.X;
			double g = m_vActorLoc.Y;
			double t = FVector::Dist(m_vActorLoc, vMeshLoc);

			double A = (f * f - d * d + g * g - e * e + 2 * r * r - t * t) / (2 * (g - e));
			double B = (d - f) / (g - e);
			double C = A - e;
			double a = B * B + 1;
			double b = -2 * d + 2 * B * C;
			double c = d * d + C * C - r * r;

			double sqrt = FMath::Sqrt(b * b - 4 * a * c);

			double x = (-b - sqrt) / (2 * a);
			double y = A + B * x;
			FVector vMLP(x, y, m_vActorLoc.Z);

			x = (-b + sqrt) / (2 * a);
			y = A + B * x;
			FVector vMRP(x, y, m_vActorLoc.Z);

			//�������� ���� �߽����� ���� ������ ���� ���Ϳ� �����Ͽ� -�� ��������, +�� ������
			const FVector& vAtoM = (vMeshLoc - m_vActorLoc).GetUnsafeNormal();
			const FVector& vBaseRight = vAtoM.Cross(FVector::UpVector);

			DrawDebugLine(GetWorld(), m_vActorLoc, m_vActorLoc + vAtoM * 100, FColor::Black);
			DrawDebugLine(GetWorld(), m_vActorLoc, m_vActorLoc + vBaseRight * 100, FColor::Cyan);

			if (vBaseRight.Dot((vMLP - m_vActorLoc).GetUnsafeNormal()) < 0)
			{
				FVector temp = vMLP;
				vMLP = vMRP;
				vMRP = temp;
			}

			vMLV = (vMLP - m_vActorLoc).GetUnsafeNormal();
			vMRV = (vMRP - m_vActorLoc).GetUnsafeNormal();

			DrawDebugPoint(GetWorld(), vMLP, 10, FColor::Blue, false, -1, 1);
			DrawDebugPoint(GetWorld(), vMRP, 10, FColor::Green, false, -1, 1);

			//��ֹ��� �þ߰��� ��ġ���� üũ
			if (!IsInViewAngle(vLeftTrace, vRightTrace, vMLV, vMRV, bLTVInclude, bRTVInclude))
			{
				continue;
			}

			//�þ߰��� �� ������ ���� ���
			if (bLTVInclude && bRTVInclude)
			{
				UE_LOG(LogTemp, Log, TEXT("bLTVInclude && bRTVInclude"));
				double dDisLTP = FVector::DistSquared(vLTP, vMeshLoc);
				double dDisRTP = FVector::DistSquared(vRTP, vMeshLoc);
				double dDotLTP = vForward.Dot((vLTP - vMeshLoc).GetUnsafeNormal());
				double dDotRTP = vForward.Dot((vRTP - vMeshLoc).GetUnsafeNormal());

				UE_LOG(LogTemp, Log, TEXT("%f %f %f"), dDisLTP, dDisRTP, rr);
				//�þ߰��� ��� �� ���ο� �ִ°��
				if (dDisLTP <= rr && dDisRTP <= rr)
				{
					UE_LOG(LogTemp, Log, TEXT("dDisLTP <= rr && dDisRTP <= rr"));
					TArray<AActor*> ignore;
					FHitResult result;
					if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, vLTP, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
					{
						if (result.GetActor() == pMeshCom->GetOwner())
							m_arrValidPoints.Emplace(pMeshCom, result.Location);
					}

					DrawDebugPoint(GetWorld(), result.Location, 10, FColor::Blue, false, -1, 1);
					if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, vRTP, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
					{
						if (result.GetActor() == pMeshCom->GetOwner())
							m_arrValidPoints.Emplace(pMeshCom, result.Location);
					}

					DrawDebugPoint(GetWorld(), result.Location, 10, FColor::Green, false, -1, 1);

				}
				//�þ߰��� ��� �� �ܺο� �ִ°��
				else if (dDisLTP > rr && dDisRTP > rr)
				{
					UE_LOG(LogTemp, Log, TEXT("dDisLTP > rr && dDisRTP > rr"));
					if (dDotLTP < 0 && dDotRTP < 0)
					{
						UE_LOG(LogTemp, Log, TEXT("dDotLTP < 0 && dDotRTP < 0"));
						t = m_dTraceLength;

						A = (f * f - d * d + g * g - e * e + r * r - t * t) / (2 * (g - e));
						B = (d - f) / (g - e);
						C = A - e;
						a = B * B + 1;
						b = -2 * d + 2 * B * C;
						c = d * d + C * C - r * r;

						sqrt = FMath::Sqrt(b * b - 4 * a * c);

						x = (-b - sqrt) / (2 * a);
						y = A + B * x;
						FVector vLP(x, y, m_vActorLoc.Z);

						x = (-b + sqrt) / (2 * a);
						y = A + B * x;
						FVector vRP(x, y, m_vActorLoc.Z);

						AddPoint(vLP, pMeshCom);
						AddPoint(vRP, pMeshCom);

						DrawDebugPoint(GetWorld(), vLP, 10, FColor::Blue, false, -1, 1);
						DrawDebugPoint(GetWorld(), vRP, 10, FColor::Green, false, -1, 1);
					}
					else if (dDotLTP >= 0 && dDotRTP >= 0)
					{
						UE_LOG(LogTemp, Log, TEXT("dDotLTP >= 0 && dDotRTP >= 0"));
						TArray<AActor*> ignore;
						FHitResult result;
						if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, vLTP, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
						{
							if (result.GetActor() == pMeshCom->GetOwner())
								m_arrValidPoints.Emplace(pMeshCom, result.Location);
						}

						DrawDebugPoint(GetWorld(), result.Location, 10, FColor::Blue, false, -1, 1);
						if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, vRTP, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
						{
							if (result.GetActor() == pMeshCom->GetOwner())
								m_arrValidPoints.Emplace(pMeshCom, result.Location);
						}
						DrawDebugPoint(GetWorld(), result.Location, 10, FColor::Green, false, -1, 1);
					}
					else
					{
						UE_LOG(LogTemp, Error, TEXT("Not expected this case!"));
					}
				}
				//�������� ����, ���������� �ܺ�
				else if (dDisLTP <= rr && dDisRTP > rr)
				{
					UE_LOG(LogTemp, Log, TEXT("dDisLTP <= rr && dDisLTP > rr"));
					//�� �� ��� ���� ���ʿ� �ִ� ���
					if (dDotLTP < 0 && dDotRTP < 0)
					{
						t = m_dTraceLength;

						A = (f * f - d * d + g * g - e * e + r * r - t * t) / (2 * (g - e));
						B = (d - f) / (g - e);
						C = A - e;
						a = B * B + 1;
						b = -2 * d + 2 * B * C;
						c = d * d + C * C - r * r;

						sqrt = FMath::Sqrt(b * b - 4 * a * c);

						x = (-b - sqrt) / (2 * a);
						y = A + B * x;
						FVector vLP(x, y, m_vActorLoc.Z);

						x = (-b + sqrt) / (2 * a);
						y = A + B * x;
						FVector vRP(x, y, m_vActorLoc.Z);

						if (vLeftTrace.Dot((vLP - m_vActorLoc).GetUnsafeNormal()) < vLeftTrace.Dot((vRP - m_vActorLoc).GetUnsafeNormal()))
						{
							FVector temp = vLP;
							vLP = vRP;
							vRP = temp;
						}

						AddPoint(vRP, pMeshCom);
						DrawDebugPoint(GetWorld(), vRP, 10, FColor::Blue, false, -1, 1);

						TArray<AActor*> ignore;
						FHitResult result;
						if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, vLTP, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
						{
							if (result.GetActor() == pMeshCom->GetOwner())
								m_arrValidPoints.Emplace(pMeshCom, result.Location);
						}
						DrawDebugPoint(GetWorld(), result.Location, 10, FColor::Green, false, -1, 1);
					}
					//�� �� ��� ���� ���ʿ� �ִ� ���
					else if (dDotLTP >= 0 && dDotRTP >= 0)
					{
						TArray<AActor*> ignore;
						FHitResult result;
						if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, vLTP, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
						{
							if (result.GetActor() == pMeshCom->GetOwner())
								m_arrValidPoints.Emplace(pMeshCom, result.Location);
						}
						DrawDebugPoint(GetWorld(), result.Location, 10, FColor::Blue, false, -1, 1);

						if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, vRTP, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
						{
							if (result.GetActor() == pMeshCom->GetOwner())
								m_arrValidPoints.Emplace(pMeshCom, result.Location);
						}
						DrawDebugPoint(GetWorld(), result.Location, 10, FColor::Green, false, -1, 1);
					}
					else
					{
						UE_LOG(LogTemp, Error, TEXT("Not expected this case!"));
					}
				}
				//�������� �ܺ�, ���������� ����
				else if (dDisLTP > rr && dDisRTP <= rr)
				{
					UE_LOG(LogTemp, Log, TEXT("dDisRTP > rr && dDisRTP <= rr"));
					//�� �� ��� ���� ���ʿ� �ִ� ���
					if (dDotLTP < 0 && dDotRTP < 0)
					{
						t = m_dTraceLength;

						A = (f * f - d * d + g * g - e * e + r * r - t * t) / (2 * (g - e));
						B = (d - f) / (g - e);
						C = A - e;
						a = B * B + 1;
						b = -2 * d + 2 * B * C;
						c = d * d + C * C - r * r;

						sqrt = FMath::Sqrt(b * b - 4 * a * c);

						x = (-b - sqrt) / (2 * a);
						y = A + B * x;
						FVector vLP(x, y, m_vActorLoc.Z);

						x = (-b + sqrt) / (2 * a);
						y = A + B * x;
						FVector vRP(x, y, m_vActorLoc.Z);

						if (vLeftTrace.Dot((vLP - m_vActorLoc).GetUnsafeNormal()) < vLeftTrace.Dot((vRP - m_vActorLoc).GetUnsafeNormal()))
						{
							FVector temp = vLP;
							vLP = vRP;
							vRP = temp;
						}

						AddPoint(vLP, pMeshCom);
						DrawDebugPoint(GetWorld(), vLP, 10, FColor::Blue, false, -1, 1);

						TArray<AActor*> ignore;
						FHitResult result;
						if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, vRTP, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
						{
							if (result.GetActor() == pMeshCom->GetOwner())
								m_arrValidPoints.Emplace(pMeshCom, result.Location);
						}
						DrawDebugPoint(GetWorld(), result.Location, 10, FColor::Green, false, -1, 1);
					}
					//�� �� ��� ���� ���ʿ� �ִ� ���
					else if (dDotLTP >= 0 && dDotRTP >= 0)
					{
						TArray<AActor*> ignore;
						FHitResult result;
						if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, vLTP, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
						{
							if (result.GetActor() == pMeshCom->GetOwner())
								m_arrValidPoints.Emplace(pMeshCom, result.Location);
						}
						DrawDebugPoint(GetWorld(), result.Location, 10, FColor::Blue, false, -1, 1);

						if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, vRTP, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
						{
							if (result.GetActor() == pMeshCom->GetOwner())
								m_arrValidPoints.Emplace(pMeshCom, result.Location);
						}
						DrawDebugPoint(GetWorld(), result.Location, 10, FColor::Green, false, -1, 1);
					}
					else
					{
						UE_LOG(LogTemp, Error, TEXT("Not expected this case!"));
					}
				}
			}
			else if (bLTVInclude)
			{
				//View�� ���� ������ �κ��� �����Ұ��
				if (FVector::DistSquared(m_vActorLoc, vMRP) <= dTraceLengthSquared)
				{
					AddEdgePoint(vMRP, pMeshCom, m_dTraceAngleOffset);

					TArray<AActor*> ignore;
					FHitResult result;
					if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, vLTP, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
					{
						if (result.GetActor() == pMeshCom->GetOwner())
							m_arrValidPoints.Emplace(pMeshCom, result.Location);
					}
				}
				else
				{
					//LTP�� �� ���ο� �ִ� ���
					if (FVector::DistSquared(vMeshLoc, vLTP) <= r * r)
					{
						t = m_dTraceLength;

						A = (f * f - d * d + g * g - e * e + r * r - t * t) / (2 * (g - e));
						B = (d - f) / (g - e);
						C = A - e;
						a = B * B + 1;
						b = -2 * d + 2 * B * C;
						c = d * d + C * C - r * r;

						sqrt = FMath::Sqrt(b * b - 4 * a * c);

						x = (-b - sqrt) / (2 * a);
						y = A + B * x;
						FVector vLP(x, y, m_vActorLoc.Z);

						x = (-b + sqrt) / (2 * a);
						y = A + B * x;
						FVector vRP(x, y, m_vActorLoc.Z);

						if (vRightTrace.Dot((vLP - m_vActorLoc).GetUnsafeNormal()) > vRightTrace.Dot((vRP - m_vActorLoc).GetUnsafeNormal()))
						{
							FVector temp = vLP;
							vLP = vRP;
							vRP = temp;
						}

						AddPoint(vRP, pMeshCom);

						TArray<AActor*> ignore;
						FHitResult result;
						if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, vLTP, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
						{
							if (result.GetActor() == pMeshCom->GetOwner())
								m_arrValidPoints.Emplace(pMeshCom, result.Location);
						}

						DrawDebugPoint(GetWorld(), vRP, 10, FColor::Blue, false, -1, 1);
						DrawDebugPoint(GetWorld(), result.Location, 10, FColor::Green, false, -1, 1);
					}
					//�ܺο� �ִ� ���
					else
					{
						t = m_dTraceLength;

						A = (f * f - d * d + g * g - e * e + r * r - t * t) / (2 * (g - e));
						B = (d - f) / (g - e);
						C = A - e;
						a = B * B + 1;
						b = -2 * d + 2 * B * C;
						c = d * d + C * C - r * r;

						sqrt = FMath::Sqrt(b * b - 4 * a * c);

						x = (-b - sqrt) / (2 * a);
						y = A + B * x;
						FVector vLP(x, y, m_vActorLoc.Z);

						x = (-b + sqrt) / (2 * a);
						y = A + B * x;
						FVector vRP(x, y, m_vActorLoc.Z);

						AddPoint(vLP, pMeshCom);
						AddPoint(vRP, pMeshCom);

						DrawDebugPoint(GetWorld(), vLP, 10, FColor::Blue, false, -1, 1);
						DrawDebugPoint(GetWorld(), vRP, 10, FColor::Green, false, -1, 1);
					}
				}
			}
			else if (bRTVInclude)
			{
				//View�� ���� ���� �κ��� �����Ұ��
				if (FVector::DistSquared(m_vActorLoc, vMLP) <= dTraceLengthSquared)
				{
					AddEdgePoint(vMLP, pMeshCom, -m_dTraceAngleOffset);

					TArray<AActor*> ignore;
					FHitResult result;
					if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, vRTP, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
					{
						if (result.GetActor() == pMeshCom->GetOwner())
							m_arrValidPoints.Emplace(pMeshCom, result.Location);
					}
				}
				else
				{
					//RTP�� �� ���ο� �ִ� ���
					if (FVector::DistSquared(vMeshLoc, vRTP) <= r * r)
					{
						t = m_dTraceLength;

						A = (f * f - d * d + g * g - e * e + r * r - t * t) / (2 * (g - e));
						B = (d - f) / (g - e);
						C = A - e;
						a = B * B + 1;
						b = -2 * d + 2 * B * C;
						c = d * d + C * C - r * r;

						sqrt = FMath::Sqrt(b * b - 4 * a * c);

						x = (-b - sqrt) / (2 * a);
						y = A + B * x;
						FVector vLP(x, y, m_vActorLoc.Z);

						x = (-b + sqrt) / (2 * a);
						y = A + B * x;
						FVector vRP(x, y, m_vActorLoc.Z);

						if (vLeftTrace.Dot((vLP - m_vActorLoc).GetUnsafeNormal()) < vLeftTrace.Dot((vRP - m_vActorLoc).GetUnsafeNormal()))
						{
							FVector temp = vLP;
							vLP = vRP;
							vRP = temp;
						}

						AddPoint(vLP, pMeshCom);

						TArray<AActor*> ignore;
						FHitResult result;
						if (UKismetSystemLibrary::LineTraceSingle(GetWorld(), m_vActorLoc, vRTP, UEngineTypes::ConvertToTraceType(ECC_Visibility), false, ignore, EDrawDebugTrace::Type::None, result, true))
						{
							if(result.GetActor() == pMeshCom->GetOwner())
								m_arrValidPoints.Emplace(pMeshCom, result.Location);
						}

						DrawDebugPoint(GetWorld(), vLP, 10, FColor::Blue, false, -1, 1);
						DrawDebugPoint(GetWorld(), result.Location, 10, FColor::Green, false, -1, 1);
					}
					//�ܺο� �ִ� ���
					else
					{
						t = m_dTraceLength;

						A = (f * f - d * d + g * g - e * e + r * r - t * t) / (2 * (g - e));
						B = (d - f) / (g - e);
						C = A - e;
						a = B * B + 1;
						b = -2 * d + 2 * B * C;
						c = d * d + C * C - r * r;

						sqrt = FMath::Sqrt(b * b - 4 * a * c);

						x = (-b - sqrt) / (2 * a);
						y = A + B * x;
						FVector vLP(x, y, m_vActorLoc.Z);

						x = (-b + sqrt) / (2 * a);
						y = A + B * x;
						FVector vRP(x, y, m_vActorLoc.Z);

						AddPoint(vLP, pMeshCom);
						AddPoint(vRP, pMeshCom);

						DrawDebugPoint(GetWorld(), vLP, 10, FColor::Blue, false, -1, 1);
						DrawDebugPoint(GetWorld(), vRP, 10, FColor::Green, false, -1, 1);
					}
				}
			}
			else
			{
				//View�� ������ ������� ������ ���
				if (FVector::DistSquared(m_vActorLoc, vMLP) <= dTraceLengthSquared)
				{
					AddEdgePoint(vMLP, pMeshCom, -m_dTraceAngleOffset);
					AddEdgePoint(vMRP, pMeshCom, m_dTraceAngleOffset);
				}
				//��ä���� ���ʿ� ��¦ ��ġ�� ���
				else
				{
					t = m_dTraceLength;

					A = (f * f - d * d + g * g - e * e + r * r - t * t) / (2 * (g - e));
					B = (d - f) / (g - e);
					C = A - e;
					a = B * B + 1;
					b = -2 * d + 2 * B * C;
					c = d * d + C * C - r * r;

					sqrt = FMath::Sqrt(b * b - 4 * a * c);

					x = (-b - sqrt) / (2 * a);
					y = A + B * x;
					FVector vLP(x, y, m_vActorLoc.Z);

					x = (-b + sqrt) / (2 * a);
					y = A + B * x;
					FVector vRP(x, y, m_vActorLoc.Z);

					AddPoint(vLP, pMeshCom);
					AddPoint(vRP, pMeshCom);

					DrawDebugPoint(GetWorld(), vLP, 10, FColor::Blue, false, -1, 1);
					DrawDebugPoint(GetWorld(), vRP, 10, FColor::Green, false, -1, 1);
				}
			}
		}

		//Point�� Mesh�� ������ ���� ���� �ٸ� Mesh ���� �ִ� Point�� �ﰢ���� ����� �� ���̸� Trace�ؾ�
		//Forward ���Ϳ��� ������ ���� Trace ���Ϳ� ���Ͽ� ũ�ٸ� �þ߹��� ���� ����
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

	//��� ���� �ٽ� ������ ����
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
			DrawDebugLine(GetWorld(), m_vActorLoc, p.point, FColor::Red, false, -1, 1);
			DrawDebugPoint(GetWorld(), p.point, 5, FColor::Red, false, -1, 1);
		}
	}
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
