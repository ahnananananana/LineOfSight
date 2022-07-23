#include "LineOfSightActor.h"
#include "AI/Navigation/NavCollisionBase.h"
#include "DrawDebugHelpers.h"

ALineOfSightActor::ALineOfSightActor()
{
	PrimaryActorTick.bCanEverTick = true;
}


TArray<FVector> ALineOfSightActor::GetDetectedPoints()
{
	TArray<FVector> result;

	//View Angle �ȿ� �ִ��� üũ
	//���� Trace ���Ϳ� Forward ������ �������� ū ������ ���ο� �ִ� ��
	const FVector& vActorLoc = GetActorLocation();
	const FVector& vForward = GetActorForwardVector();
	const FVector& vBack = -vForward;
	const FVector& vLeft = vForward.RotateAngleAxis(-m_fViewAngle / 2, FVector3d::UpVector);
	const FVector& vRight = vForward.RotateAngleAxis(m_fViewAngle / 2, FVector3d::UpVector);

	double dBaseAngle = vBack.Rotation().Yaw >= 0 ? vBack.Rotation().Yaw : 360 + vBack.Rotation().Yaw;

	double dFLDot = vForward.Dot(vLeft);

	if (m_bIsShowTraceDebug)
	{
		DrawDebugLine(GetWorld(), vActorLoc, vActorLoc + vLeft * m_fTraceLength, FColor::Blue);
		DrawDebugLine(GetWorld(), vActorLoc, vActorLoc + vRight * m_fTraceLength, FColor::Blue);
	}

	for (const auto& pair : m_mapDetectedPoints)
	{
		const TArray<FVector>& arrPoints = pair.Value;
		for (const FVector& p : arrPoints)
		{
			FVector v = p - vActorLoc;
			v.Normalize();
			if (vForward.Dot(v) >= dFLDot && FVector::Dist(vActorLoc, p) <= m_fTraceLength)
			{
				for (const FVector& a : arrPoints)
				{
					if(FVector::Dist(vActorLoc, a) <= m_fTraceLength)
						result.Add(a);
				}
				break;
			}
		}
	}

	result.Sort([&](const FVector& lhs, const FVector& rhs)
		{
			FVector v1 = lhs - vActorLoc;
			FVector v2 = rhs - vActorLoc;

			double rot1 = v1.Rotation().Yaw >= 0 ? v1.Rotation().Yaw : 360 + v1.Rotation().Yaw;
			double rot2 = v2.Rotation().Yaw >= 0 ? v2.Rotation().Yaw : 360 + v2.Rotation().Yaw;

			rot1 -= dBaseAngle;
			if (rot1 < 0)
			{
				rot1 = 360 + rot1;
			}

			rot2 -= dBaseAngle;
			if (rot2 < 0)
			{
				rot2 = 360 + rot2;
			}

			return rot1 < rot2;
		});

	return result;
}

void ALineOfSightActor::AddAdjMesh(UStaticMeshComponent* _pMeshCom)
{
	UE_LOG(LogTemp, Log, TEXT("AddAdjMesh"));
	if (_pMeshCom && !m_mapDetectedPoints.Contains(_pMeshCom))
	{
		UE_LOG(LogTemp, Log, TEXT("AddAdjMesh2"));
		if (UStaticMesh* pMesh = _pMeshCom->GetStaticMesh())
		{
			if (UNavCollisionBase* pNavCol = pMesh->GetNavCollision())
			{
				//Actor�� Z���̻��� �͸� ���͸�
				TArray<FVector>& arrPoints = m_mapDetectedPoints.Emplace(_pMeshCom);
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
		m_mapDetectedPoints.Remove(_pMeshCom);
	}
}