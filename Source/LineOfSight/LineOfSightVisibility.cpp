#include "LineOfSightVisibility.h"

ULineOfSightVisibility::ULineOfSightVisibility()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void ULineOfSightVisibility::BeginPlay()
{
	Super::BeginPlay();

	m_pMeshCom = GetMeshComponent();
}

void ULineOfSightVisibility::SetVisible(bool _bVisible)
{
	if (UStaticMeshComponent* pMeshCom = GetMeshComponent())
	{
		pMeshCom->SetVisibility(_bVisible);
	}
}

UStaticMeshComponent* ULineOfSightVisibility::GetMeshComponent()
{
	if (!m_pMeshCom)
	{
		m_pMeshCom = GetOwner()->FindComponentByClass<UStaticMeshComponent>();
		if (m_pMeshCom)
		{
			m_pMeshCom->SetVisibility(false);
		}
		UE_LOG(LogTemp, Error, TEXT("No Static Mesh Component!"));
	}

	return m_pMeshCom;
}

