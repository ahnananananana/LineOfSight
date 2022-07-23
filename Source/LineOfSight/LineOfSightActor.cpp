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

	//View Angle �ȿ� �ִ��� üũ
	//���� Trace ���Ϳ� Forward ������ �������� ū ������ ���ο� �ִ� ��
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
		//������ ���� ����
		arrPoints.Sort([&](const FVector& lhs, const FVector& rhs) { return SortByAngle(lhs, rhs); });

		//�� ������ ť������ ��������� ����
		//ť���� ���
		if(arrPoints.Num() == 4)
		{
			arrValidPoints.Reserve(4);
			//1�鸸 ���� �ִ��� 2���� ���� �ִ��� ����
			//��� ���� 2�� �� ���� ����� ���� �ִٸ� 2��, �ƴ϶�� 1��
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

				//�þ߹��� �ȿ� �ִ��� üũ
				if (dis <= m_fTraceLength)
				{
					arrInRangePointIdx.Add(i);
				}
			}

			const FVector& vLeftPoint = arrPoints[0];
			const FVector& vRightPoint = arrPoints[3];
			//2���� ���
			if (iMinIdx == 1 || iMinIdx == 2)
			{
				const FVector& vMidPoint = arrPoints[iMinIdx];
				//Trace �Ÿ� �ȿ� ���ԵǴ� ���� ������ ���� ó��
				//0���� ���
				if (arrInRangePointIdx.Num() == 0)
				{
					UE_LOG(LogTemp, Error, TEXT("ť�� 2���ε� ���� 0��!"));
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

						arrValidPoints.Add(vLeftPoint);
						arrValidPoints.Add(vMidPoint);
					}
					//������ ���� ����� ���� �����ϴ� ���
					else if (arrInRangePointIdx[1] == 3)
					{
						//���� ���� ��� �� ������ ���� ����Ͽ� �߰�

						arrValidPoints.Add(vMidPoint);
						arrValidPoints.Add(vRightPoint);
					}
				}
				//3, 4���� ���
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
			//1���� ���
			else
			{
				//Trace �Ÿ� �ȿ� ���ԵǴ� ���� ������ ���� ó��
				//0���� ���
				if (arrInRangePointIdx.Num() == 0)
				{
					//������ ���� ���� ������ �̷���� ���� ���� �� ��
					//�������� �Ÿ��� �þ߰Ÿ���ŭ �ִ� 2���� ���� ����Ͽ� �߰�
				}
				//1���� ���
				else if (arrInRangePointIdx.Num() == 1)
				{
					//������ ���� ���� ������ �̷���� ���� ���� �� ��
					//�������� �Ÿ��� �þ߰Ÿ���ŭ �ִ� 2���� ���� ���

					//���� ���� ���Ե� ���
					if (arrInRangePointIdx[0] == 0)
					{
						//���� �� �� ������ ���� ����� ���� �߰�
						arrValidPoints.Add(vLeftPoint);
					}
					//������ ���� ���Ե� ���
					else if (arrInRangePointIdx[0] == 3)
					{
						//���� �� �� ���� ���� ����� ���� �߰�
						arrValidPoints.Add(vRightPoint);
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
							arrValidPoints.Add(vLeftPoint);
						}
						//������ ���� ���Ե� ���
						else
						{
							//���� �� �� ���� ���� ����� ���� �߰�
							arrValidPoints.Add(vRightPoint);
						}
					}
					//���� ���鸸 �ִ� ���
					else
					{
						arrValidPoints.Add(vLeftPoint);
						arrValidPoints.Add(vRightPoint);
					}
				}
				//3, 4���� ���
				else
				{
					//�� ���ʿ� �ִ� ���� �߰�
					arrValidPoints.Add(vLeftPoint);
					arrValidPoints.Add(vRightPoint);
				}
			}
		}
		else
		{

		}

		//Forward ���Ϳ��� ������ ���� Trace ���Ϳ� ���Ͽ� ũ�ٸ� �þ߹��� ���� ����
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

	//��� ���� �ٽ� ������ ����
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
				//Actor�� Z���̻��� �͸� ���͸�
				TArray<FVector>& arrPoints = m_mapDetectedObstacles.Emplace(_pMeshCom);
				float fActorPosZ = GetActorLocation().Z;
				for (const FVector& p : pNavCol->GetConvexCollision().VertexBuffer)
				{
					const FVector& tp = _pMeshCom->GetOwner()->GetActorTransform().TransformPosition(p);
					if (tp.Z <= fActorPosZ)
					{
						//�ش� Z ��ġ�� ���� �� �߰�
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