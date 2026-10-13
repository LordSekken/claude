// DriftCore's view of the Unreal level: wheel ray casts and body overlaps.
// The surface under a wheel comes from the mesh name Drift Lab exports
// (ROAD_*, KERB_*, GRASS_*, DIRT_*, GRAVEL_*, SAND_*, WALL_*) or from a
// component tag with one of those words, so hand-built levels work too.
#pragma once

#include "CoreMinimal.h"
#include "drift/Vehicle.h"

class AActor;
class UPrimitiveComponent;
class UWorld;

class FDriftUnrealWorld final : public drift::World
{
public:
	FDriftUnrealWorld(UWorld* InWorld, const AActor* InIgnore) : World(InWorld), Ignore(InIgnore) {}

	virtual drift::GroundHit Raycast(const drift::Vec3& From, const drift::Vec3& To) override;
	virtual void BodyContacts(const drift::Vec3& Center, const drift::Mat3& Rot, const drift::Vec3& HalfExtents, std::vector<drift::BodyContact>& Out) override;

	static drift::Surface SurfaceOf(const UPrimitiveComponent* Comp);

private:
	UWorld* World;
	const AActor* Ignore;
};
