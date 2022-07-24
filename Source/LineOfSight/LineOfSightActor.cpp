#include "LineOfSightActor.h"
#include "AI/Navigation/NavCollisionBase.h"
#include "DrawDebugHelpers.h"
#include "GenericPlatform/GenericPlatformMath.h"
#include "Kismet/KismetSystemLibrary.h"

ALineOfSightActor::ALineOfSightActor()
{
	PrimaryActorTick.bCanEverTick = true;
}

//TODO: �� ť�갡 ���� ��ġ�� �κп��� ��ο� �κ� ����
TArray<FMeshPoint> ALineOfSightActor::GetDetectedPoints()
{
	TArray<FMeshPoint> result;

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
		//��ֹ��� ���������� ���͸� �������� ������ ������ ���
		const FVector& vAngleBase = (m_vActorLoc - pMeshCom->GetOwner()->GetActorLocation()).GetUnsafeNormal();
		m_dBaseAngle = vAngleBase.Rotation().Yaw >= 0 ? vAngleBase.Rotation().Yaw : 360 + vAngleBase.Rotation().Yaw;
		UE_LOG(LogTemp, Log, TEXT("%s"), *UKismetSystemLibrary::GetDisplayName(pMeshCom->GetOwner()));
		DrawDebugLine(GetWorld(), m_vActorLoc, m_vActorLoc + vAngleBase * 500, FColor::Red, false, -1, 1);

		TArray<FMeshPoint> arrPoints = pair.Value;

		////4�� ��� ���� �ȿ� ������ ��ŵ
		//bool bVaild = false;
		//for (const FMeshPoint& p : arrPoints)
		//{
		//	FVector v = (p.point - m_vActorLoc).GetUnsafeNormal();
		//	if (vForward.Dot(v) + .001 >= dFLDot)
		//	{
		//		bVaild = true;
		//		break;
		//	}
		//}

		//if(!bVaild)
		//	continue;

		TArray<FMeshPoint> arrValidPoints;
		//������ ���� ����
		arrPoints.Sort([&](const FMeshPoint& lhs, const FMeshPoint& rhs) { return SortByAngle(lhs, rhs); });

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
					arrInRangePointIdx.Add(i);
				}
			}

			bool bIsTwoSide = iMinIdx == 1 || iMinIdx == 2;

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
						vMostRightPoint = arrPoints[iMinIdx];
					}
					else
					{
						vMostLeftPoint = arrPoints[iMinIdx];
					}
				}
			}

			DrawDebugPoint(GetWorld(), arrPoints[0].point, 10, FColor::Blue, false, -1, 1);
			DrawDebugPoint(GetWorld(), arrPoints[1].point, 10, FColor::Red, false, -1, 1);
			DrawDebugPoint(GetWorld(), arrPoints[2].point, 10, FColor::Cyan, false, -1, 1);
			DrawDebugPoint(GetWorld(), arrPoints[3].point, 10, FColor::Black, false, -1, 1);


			UE_LOG(LogTemp, Log, TEXT("arrInRangePointIdx.Num %d"), arrInRangePointIdx.Num());

			//2���� ���
			if (bIsTwoSide)
			{
				UE_LOG(LogTemp, Log, TEXT("2"));
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
					//��� ���� ���� �� ������ ���� ���� ����Ͽ� �߰�
					arrValidPoints.Add(vMidPoint);
				}
				//2���� ���
				else if (arrInRangePointIdx.Num() == 2)
				{
					//���� ���� ����� ���� �����ϴ� ���
					if (arrInRangePointIdx[0] == 0)
					{
						//������ ���� ��� �� ������ ���� ����Ͽ� �߰�

						arrValidPoints.Add(vMostLeftPoint);
						arrValidPoints.Add(vMidPoint);
					}
					//������ ���� ����� ���� �����ϴ� ���
					else if (arrInRangePointIdx[1] == 3)
					{
						//���� ���� ��� �� ������ ���� ����Ͽ� �߰�

						arrValidPoints.Add(vMidPoint);
						arrValidPoints.Add(vMostRightPoint);
					}
				}
				//3, 4���� ���
				else
				{
					arrValidPoints.Add(vMostLeftPoint);
					if (arrInRangePointIdx.Num() == 3)
					{
						arrValidPoints.Add(vMidPoint);
					}
					else
					{
						arrValidPoints.Add(arrPoints[1]);
						arrValidPoints.Add(arrPoints[2]);
					}
					arrValidPoints.Add(vMostRightPoint);
				}
			}
			//1���� ���
			else
			{
				//���� ������ ��� ����
				arrInRangePointIdx.RemoveAll([](int e) {return e == 1 || e == 2; });

				UE_LOG(LogTemp, Log, TEXT("1"));
				//Trace �Ÿ� �ȿ� ���ԵǴ� ���� ������ ���� ó��
				//0���� ���
				if (arrInRangePointIdx.Num() == 0)
				{
					FVector vLeftPoint, vRightPoint;
					//�ϴ� ���� Trace ���Ϳ� ������ Trace ���� ������ �������� ���� ���
					FVector vLeftTraceEnd = m_vActorLoc + vLeftTrace * m_dTraceLength;
					bool isLeftCrossPointValid = TryFindTwoLineCrossPoint(m_vActorLoc, vLeftTraceEnd, vMostLeftPoint.point, vMostRightPoint.point, vLeftPoint);

					FVector vRightTraceEnd = m_vActorLoc + vRightTrace * m_dTraceLength;
					bool isRightCrossPointValid = TryFindTwoLineCrossPoint(m_vActorLoc, vRightTraceEnd, vMostLeftPoint.point, vMostRightPoint.point, vRightPoint);

					vLeftPoint.Z = m_vActorLoc.Z;
					vRightPoint.Z = m_vActorLoc.Z;

					//�� ���� ������ ���� ���̿� �ִ��� üũ
					if (isLeftCrossPointValid)
					{
						isLeftCrossPointValid = vLeftTrace.Equals((vLeftPoint - m_vActorLoc).GetUnsafeNormal()) &&
							FVector::DistSquared(m_vActorLoc, vLeftPoint) <= dTraceLengthSquared;
					}

					if (isRightCrossPointValid)
					{
						isRightCrossPointValid = vRightTrace.Equals((vRightPoint - m_vActorLoc).GetUnsafeNormal()) &&
							FVector::DistSquared(m_vActorLoc, vRightPoint) <= dTraceLengthSquared;
					}

					//TODO: ���� Trace ���Ϳ� ������ ���͵��� ��ġ������ Ȯ���ؾ�
					////ť�� �� �� ���̿� �ִ��� üũ
					FVector vTemp = (vMostRightPoint.point - vMostLeftPoint.point).GetUnsafeNormal();
					double dTemp = FVector::DistSquared(vMostLeftPoint.point, vMostRightPoint.point);
					if (isLeftCrossPointValid)
					{
						isLeftCrossPointValid = FVector::DistSquared(vMostLeftPoint.point, vLeftPoint) <= dTemp &&
							vTemp.Equals((vLeftPoint - vMostLeftPoint.point).GetUnsafeNormal());
					}

					if (isRightCrossPointValid)
					{
						isRightCrossPointValid = FVector::DistSquared(vMostLeftPoint.point, vRightPoint) <= dTemp &&
							vTemp.Equals((vRightPoint - vMostLeftPoint.point).GetUnsafeNormal());
					}

					//�Ÿ� ���̶�� ��ȿ�� ���� �ƴϹǷ� �ش� ���� �ٽ� ���
					//������ ���� ���� �Ÿ��� �þ߰Ÿ���ŭ�� ���� ���
					if (!isLeftCrossPointValid || !isRightCrossPointValid)
					{
						FVector v = (vMostRightPoint.point - vMostLeftPoint.point).GetUnsafeNormal();

						double A = vMostLeftPoint.point.X - m_vActorLoc.X;
						double B = vMostLeftPoint.point.Y - m_vActorLoc.Y;
						double C = v.X;
						double D = v.Y;

						double a = C * C + D * D;
						double b = 2 * A * C + 2 * B * D;
						double c = A * A + B * B - dTraceLengthSquared;

						double squared = b * b - 4 * a * c;

						if (!isLeftCrossPointValid)
						{
							//Left Point���� ����ϱ⿡ �� ��������ϹǷ� -
							double t = (-b - FMath::Sqrt(squared)) / (2 * a);
							vLeftPoint = vMostLeftPoint.point + v * t;
							vLeftPoint.Z = m_vActorLoc.Z;
							isLeftCrossPointValid = true;
						}

						if (!isRightCrossPointValid)
						{
							//Left Point���� ����ϱ⿡ �� �־���ϹǷ� +
							double t = (-b + FMath::Sqrt(squared)) / (2 * a);
							vRightPoint = vMostLeftPoint.point + v * t;
							vRightPoint.Z = m_vActorLoc.Z;
							isRightCrossPointValid = true;
						}
					}

					if (isLeftCrossPointValid)
						arrValidPoints.Emplace(pMeshCom, vLeftPoint, false);

					if (isRightCrossPointValid)
						arrValidPoints.Emplace(pMeshCom, vRightPoint, false);

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
						TryFindTwoLineCrossPoint(m_vActorLoc, vRightTraceEnd, vMostLeftPoint.point, vMostRightPoint.point, vRightPoint);

						vRightPoint.Z = m_vActorLoc.Z;

						arrValidPoints.Emplace(vMostLeftPoint);
						arrValidPoints.Emplace(pMeshCom, vRightPoint, false);
					}
					//������ ���� ���Ե� ���
					else if (arrInRangePointIdx[0] == 3)
					{
						//���� �� �� ���� ���� ����� ���� �߰�
						FVector vLeftPoint;
						FVector vLeftTraceEnd = m_vActorLoc + vLeftTrace * m_dTraceLength;
						TryFindTwoLineCrossPoint(m_vActorLoc, vLeftTraceEnd, vMostLeftPoint.point, vMostRightPoint.point, vLeftPoint);

						vLeftPoint.Z = m_vActorLoc.Z;

						arrValidPoints.Emplace(pMeshCom, vLeftPoint, false);
						arrValidPoints.Emplace(vMostRightPoint);
					}
					else
					{
						UE_LOG(LogTemp, Error, TEXT("Algorithm Broken!"));
					}
				}
				//2���� ���
				else if (arrInRangePointIdx.Num() == 2)
				{
					//���� ���� �ִ� ���
					if (arrInRangePointIdx[0] != 0 || arrInRangePointIdx[1] != 3)
					{
						//������ ���� ���� ������ �̷���� ���� ���� �� ��
						//�������� �Ÿ��� �þ߰Ÿ���ŭ �ִ� 2���� ���� ���

						//���� ���� ���Ե� ���
						if (arrInRangePointIdx[0] == 0)
						{
							//���� �� �� ������ ���� ����� ���� �߰�
							arrValidPoints.Add(vMostLeftPoint);
						}
						//������ ���� ���Ե� ���
						else
						{
							//���� �� �� ���� ���� ����� ���� �߰�
							arrValidPoints.Add(vMostRightPoint);
						}
					}
					//���� ���鸸 �ִ� ���
					else
					{
						arrValidPoints.Add(vMostLeftPoint);
						arrValidPoints.Add(vMostRightPoint);
					}
				}
				//3, 4���� ���
				else
				{
					//�� ���ʿ� �ִ� ���� �߰�
					arrValidPoints.Add(vMostLeftPoint);
					arrValidPoints.Add(vMostRightPoint);
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

			//DrawDebugPoint(GetWorld(), p.point, 10, FColor::Red, false, -1, 1);

			if (vForward.Dot(v) + .001 >= dFLDot)
			{
				UE_LOG(LogTemp, Log, TEXT("p %s"), *p.point.ToString());
				result.Add(p);
			}
		}
	}

	//��� ���� �ٽ� ������ ����
	result.Sort([&](const FMeshPoint& lhs, const FMeshPoint& rhs) { return SortByAngle(lhs, rhs); });

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

bool ALineOfSightActor::TryFindTwoLineCrossPoint(const FVector& _p1, const FVector& _p2, const FVector& _p3, const FVector& _p4, FVector& _vCrossPoint)
{
	double denominator = (_p1.X - _p2.X) * (_p3.Y - _p4.Y) - (_p1.Y - _p2.Y) * (_p3.X - _p4.X);
	//�����ϰų� ��ġ�ϴ� ���
	if (denominator == 0)
		return false;

	_vCrossPoint.X = ((_p1.X * _p2.Y - _p1.Y * _p2.X) * (_p3.X - _p4.X) - (_p1.X - _p2.X) * (_p3.X * _p4.Y - _p3.Y * _p4.X)) / denominator;
	_vCrossPoint.Y = ((_p1.X * _p2.Y - _p1.Y * _p2.X) * (_p3.Y - _p4.Y) - (_p1.Y - _p2.Y) * (_p3.X * _p4.Y - _p3.Y * _p4.X)) / denominator;

	return true;
}
