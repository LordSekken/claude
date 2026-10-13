using UnrealBuildTool;

public class DriftGameEditorTarget : TargetRules
{
	public DriftGameEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("DriftGame");
	}
}
