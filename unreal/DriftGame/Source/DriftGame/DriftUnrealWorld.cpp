#include "DriftUnrealWorld.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/OverlapResult.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "DriftUE.h"

namespace
{
	FCollisionObjectQueryParams TrackObjects()
	{
		FCollisionObjectQueryParams Objects(ECC_WorldStatic);
		Objects.AddObjectTypesToQuery(ECC_WorldDynamic);
		return Objects;
	}

	struct FSurfaceWord
	{
		const TCHAR* Word;
		drift::Surface Surface;
	};

	const FSurfaceWord GSurfaceWords[] = {
		{TEXT("ROAD"), drift::Surface::Asphalt},
		{TEXT("ASPHALT"), drift::Surface::Asphalt},
		{TEXT("KERB"), drift::Surface::Kerb},
		{TEXT("GRASS"), drift::Surface::Grass},
		{TEXT("DIRT"), drift::Surface::Dirt},
		{TEXT("GRAVEL"), drift::Surface::Gravel},
		{TEXT("SAND"), drift::Surface::Sand},
		{TEXT("WALL"), drift::Surface::Concrete},
		{TEXT("CONCRETE"), drift::Surface::Concrete},
	};

	// The surface word that appears last in the name wins, so a track called
	// "grass_valley" still reads "grass_valley_ROAD_3" as road.
	bool SurfaceFromName(const FString& Name, drift::Surface& Out)
	{
		const FString Upper = Name.ToUpper();
		int32 Best = INDEX_NONE;
		for (const FSurfaceWord& W : GSurfaceWords)
		{
			const int32 At = Upper.Find(W.Word, ESearchCase::CaseSensitive, ESearchDir::FromEnd);
			if (At != INDEX_NONE && At > Best)
			{
				Best = At;
				Out = W.Surface;
			}
		}
		return Best != INDEX_NONE;
	}
}

drift::Surface FDriftUnrealWorld::SurfaceOf(const UPrimitiveComponent* Comp)
{
	drift::Surface S = drift::Surface::Asphalt;
	if (!Comp)
	{
		return S;
	}
	for (const FName& Tag : Comp->ComponentTags)
	{
		if (SurfaceFromName(Tag.ToString(), S))
		{
			return S;
		}
	}
	if (const UStaticMeshComponent* SM = Cast<UStaticMeshComponent>(Comp))
	{
		if (const UStaticMesh* Mesh = SM->GetStaticMesh())
		{
			if (SurfaceFromName(Mesh->GetName(), S))
			{
				return S;
			}
		}
	}
	SurfaceFromName(Comp->GetName(), S);
	return S;
}

drift::GroundHit FDriftUnrealWorld::Raycast(const drift::Vec3& From, const drift::Vec3& To)
{
	drift::GroundHit G;
	if (!World)
	{
		return G;
	}
	FCollisionQueryParams Params(SCENE_QUERY_STAT(DriftWheel), true, Ignore);
	FHitResult Hit;
	if (!World->LineTraceSingleByObjectType(Hit, DriftUE::Pos(From), DriftUE::Pos(To), TrackObjects(), Params))
	{
		return G;
	}
	G.hit = true;
	G.point = DriftUE::CorePos(Hit.ImpactPoint);
	G.normal = DriftUE::CoreDir(Hit.ImpactNormal);
	G.surface = SurfaceOf(Hit.GetComponent());
	return G;
}

void FDriftUnrealWorld::BodyContacts(const drift::Vec3& Center, const drift::Mat3& Rot, const drift::Vec3& HalfExtents, std::vector<drift::BodyContact>& Out)
{
	if (!World)
	{
		return;
	}
	const FVector UCenter = DriftUE::Pos(Center);
	const FQuat UQuat = DriftUE::Rot(Rot);
	// Body-local half extents: Unreal X/Y/Z = DriftCore -Z/X/Y.
	const FCollisionShape Box = FCollisionShape::MakeBox(FVector(HalfExtents.z, HalfExtents.x, HalfExtents.y) * 100.0);
	FCollisionQueryParams Params(SCENE_QUERY_STAT(DriftBody), false, Ignore);
	TArray<FOverlapResult> Overlaps;
	if (!World->OverlapMultiByObjectType(Overlaps, UCenter, UQuat, TrackObjects(), Box, Params))
	{
		return;
	}
	for (const FOverlapResult& Ov : Overlaps)
	{
		UPrimitiveComponent* Comp = Ov.GetComponent();
		if (!Comp)
		{
			continue;
		}
		FMTDResult Mtd;
		if (!Comp->ComputePenetration(Mtd, Box, UCenter, UQuat) || Mtd.Distance <= 0.f)
		{
			continue;
		}
		drift::BodyContact C;
		C.normal = DriftUE::CoreDir(Mtd.Direction).Normalized();
		C.depth = Mtd.Distance * 0.01;
		// Deepest point of the box against the obstacle.
		C.point = Center;
		const drift::Vec3 Axes[3] = {Rot.c0, Rot.c1, Rot.c2};
		const double Half[3] = {HalfExtents.x, HalfExtents.y, HalfExtents.z};
		for (int K = 0; K < 3; ++K)
		{
			C.point -= Axes[K] * (Half[K] * drift::Sign(Axes[K].Dot(C.normal)));
		}
		C.wall = C.normal.y < 0.7;
		Out.push_back(C);
	}
}
