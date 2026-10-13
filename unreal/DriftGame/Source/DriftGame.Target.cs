using UnrealBuildTool;

public class DriftGameTarget : TargetRules
{
	public DriftGameTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("DriftGame");
	}
}
