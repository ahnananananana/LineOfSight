// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "LineOfSightVisibility.generated.h"

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class LINEOFSIGHT_API ULineOfSightVisibility : public UActorComponent
{
	GENERATED_BODY()

	bool m_bIsVisible;
	UStaticMeshComponent* m_pMeshCom;

public:
	ULineOfSightVisibility();

	void BeginPlay() override;

public:	
	void SetVisible(bool _bVisible);

private:
	UStaticMeshComponent* GetMeshComponent();
};
