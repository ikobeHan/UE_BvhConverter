
#include "AnimGraph/AnimGraphNode_BvhConvert.h"
#include "UnrealWidgetFwd.h"
#include "AnimNodeEditModes.h"
#include "Kismet2/CompilerResultsLog.h"


#define LOCTEXT_NAMESPACE "UAnimGraphNode_BvhConvert"

UAnimGraphNode_BvhConvert::UAnimGraphNode_BvhConvert(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

void UAnimGraphNode_BvhConvert::ValidateAnimNodeDuringCompilation(USkeleton* ForSkeleton, FCompilerResultsLog& MessageLog)
{
	Super::ValidateAnimNodeDuringCompilation(ForSkeleton, MessageLog);
}

FText UAnimGraphNode_BvhConvert::GetControllerDescription() const
{
	return LOCTEXT("BvhConvert_NodeTitle", "Bvh Converter");
}

FText UAnimGraphNode_BvhConvert::GetTooltipText() const
{
	return LOCTEXT("AnimGraphNode_BvhConvert_Tooltip", "Bvh Convert to Posee");
}

FText UAnimGraphNode_BvhConvert::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return GetControllerDescription();
}

void UAnimGraphNode_BvhConvert::CopyNodeDataToPreviewNode(FAnimNode_Base* InPreviewNode)
{
}

FEditorModeID UAnimGraphNode_BvhConvert::GetEditorMode() const
{
	return AnimNodeEditModes::AnimNode;
}

void UAnimGraphNode_BvhConvert::CopyPinDefaultsToNodeData(UEdGraphPin* InPin)
{

}


#undef LOCTEXT_NAMESPACE
