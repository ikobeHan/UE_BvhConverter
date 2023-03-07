// Copyright 2020-2021 Kenneth Claassen. All Rights Reserved.

using UnrealBuildTool;

public class BvhConverterEditor : ModuleRules
{
	public BvhConverterEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PrivateIncludePaths.AddRange(
			new string[] {
                //"MotionSymphony/Public/Data",
                //"MotionSymphony/Public/MotionMatchingUtil",
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
                "Core",
                "AnimGraph",
                "AnimGraphRuntime",
                "BlueprintGraph",
                "BvhConverter",
				// ... add other public dependencies that you statically link with here ...
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
                "Projects",
                "InputCore",
                "UnrealEd",
                "ToolMenus",
                "CoreUObject",
                "Engine",
                "Slate",
                "SlateCore",

				// ... add private dependencies that you statically link with here ...	
				"AssetTools",
                "EditorStyle",
                "AppFramework",
                "Persona",
                "AnimationEditor",
                "Kismet",
                "ContentBrowser",
                "AnimationModifiers",
                "DesktopPlatform",
                "Json",
                "JsonUtilities",
                "AnimationBlueprintLibrary",
                "ContentBrowserData",
                "PropertyEditor"
            }
			);

        PrivateIncludePathModuleNames.AddRange(
           new string[] {
                "AssetTools",
           }
           );



        DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
            );
	}
}
