using System.IO;
using UnrealBuildTool;

public class DriftGame : ModuleRules
{
	public DriftGame(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new string[] {
			"Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput", "PhysicsCore", "Sockets", "Networking"
		});
		// DriftCore (engine-independent physics) lives next to this project:
		// unreal/DriftCore. Its sources are compiled into this module by
		// DriftCoreBuild.cpp.
		PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(ModuleDirectory, "..", "..", "..", "DriftCore", "include")));
	}
}
