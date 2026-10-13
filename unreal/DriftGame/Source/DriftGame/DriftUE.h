// Conversions between DriftCore's frame (right-handed, metres: +X right,
// +Y up, -Z forward) and Unreal's (left-handed, centimetres: +X forward,
// +Y right, +Z up). The axis map is a reflection, which is exactly the
// handedness change, so rotations convert by mapping the forward/up axes.
#pragma once

#include "CoreMinimal.h"
#include "drift/Math.h"

namespace DriftUE
{
	inline FVector Dir(const drift::Vec3& G) { return FVector(-G.z, G.x, G.y); }
	inline FVector Pos(const drift::Vec3& G) { return Dir(G) * 100.0; }
	inline drift::Vec3 CoreDir(const FVector& U) { return drift::Vec3(U.Y, U.Z, -U.X); }
	inline drift::Vec3 CorePos(const FVector& U) { return CoreDir(U) * 0.01; }

	inline FQuat Rot(const drift::Mat3& M)
	{
		return FRotationMatrix::MakeFromXZ(Dir(-M.c2), Dir(M.c1)).ToQuat();
	}

	inline drift::Mat3 CoreRot(const FQuat& Q)
	{
		const drift::Vec3 Fwd = CoreDir(Q.GetForwardVector());
		const drift::Vec3 Up = CoreDir(Q.GetUpVector());
		drift::Mat3 M;
		M.c0 = Fwd.Cross(Up);   // right
		M.c1 = Up;
		M.c2 = -Fwd;
		M.Orthonormalize();
		return M;
	}
}
