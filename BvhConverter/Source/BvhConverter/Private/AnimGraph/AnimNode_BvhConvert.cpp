#include "AnimGraph/AnimNode_BvhConvert.h"
#include "AnimationRuntime.h"
#include "Animation/AnimInstanceProxy.h"


FAnimNode_BvhConvert::FAnimNode_BvhConvert()
{
}

FAnimNode_BvhConvert::~FAnimNode_BvhConvert()
{
}


void FAnimNode_BvhConvert::CacheBones_AnyThread(const FAnimationCacheBonesContext& Context)
{
	DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(CacheBones_AnyThread)
	FAnimNode_SkeletalControlBase::CacheBones_AnyThread(Context);
}

void FAnimNode_BvhConvert::GatherDebugData(FNodeDebugData& DebugData)
{
	DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(GatherDebugData)
	const FString DebugLine = DebugData.GetNodeName(this);
	DebugData.AddDebugItem(DebugLine);
	//Todo: Add more debug information here!
}


void FAnimNode_BvhConvert::OnInitializeAnimInstance(const FAnimInstanceProxy* InProxy, const UAnimInstance* InAnimInstance)
{
	DECLARE_SCOPE_HIERARCHICAL_COUNTER_FUNC()
	Super::OnInitializeAnimInstance(InProxy, InAnimInstance);

	InitRetargtor(InAnimInstance);
}

void FAnimNode_BvhConvert::InitRetargtor(const UAnimInstance* InAnimInstance)
{
	USkeletalMeshComponent* Component = InAnimInstance->GetOwningComponent();
	if (Component && Component->GetSkeletalMeshAsset())
	{
		// bone mapping
		// 1.set 'ml_u_poser_'
		ml_u_poser_.SetUPoseableMeshComponent(Component);

		// 2. set UE bone to preset-rig mapping;
		for (auto Com : BoneMaping)
		{
			ml::JointTag _Tag = ml::JointTag((uint8)Com.Key - 1);
			ml_u_poser_.SetJointTag(TCHAR_TO_UTF8(*Com.Value.UE_Bone), _Tag);
		}

		// 3 Build Virtual Skeleton for ue skeleon，must after step 1 and 2.
		ml_u_poser_.BuildSkeleton();

		// 4.Load Ref-pose Bvh and motion bvh
		FString T_pose_filename = FPaths::ProjectPluginsDir() + "BvhConverter/Content/" + T_Pose_Bvh_Filename;
		FString Motion_filename = FPaths::ProjectPluginsDir() + "BvhConverter/Content/" + Motion_Bvh_Filename;
		if (!FPaths::FileExists(T_pose_filename))
		{
			return;
		}

		if (!FPaths::FileExists(Motion_filename))
		{
			return;
		}

		ml::LoadBVH_UE4(T_pose_filename, Motion_filename, motion_);

		// 5.set Bvh bone to preset-rig mapping
		for (auto Com : BoneMaping)
		{
			ml::JointTag _Tag = ml::JointTag((uint8)Com.Key - 1);
			motion_.editable_body()->SetJointTag(TCHAR_TO_UTF8(*Com.Value.Bvh_Joint), _Tag);
		}
	}
}

void FAnimNode_BvhConvert::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
	DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(Initialize_AnyThread)
	FAnimNode_SkeletalControlBase::Initialize_AnyThread(Context);
	GetEvaluateGraphExposedInputs().Execute(Context);
}

//retrun seconds
float FAnimNode_BvhConvert::GetTotalLength()
{
	return motion_.size() * (1.0 / motion_.fps());
}

void FAnimNode_BvhConvert::UpdateInternal(const FAnimationUpdateContext& Context)
{
	DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(UpdateInternal)
	GetEvaluateGraphExposedInputs().Execute(Context);
	
	if (motion_.size() == 0)
		return;

	if (bLoopAnimation && InternalTimeAccumulator > GetTotalLength())
	{
		InternalTimeAccumulator = InternalTimeAccumulator - GetTotalLength();
	}
	else
	{
		InternalTimeAccumulator = FMath::Clamp(InternalTimeAccumulator, 0.f, GetTotalLength());
	}

	float t = (PlayRate < 0.0f) ? GetTotalLength() - InternalTimeAccumulator : InternalTimeAccumulator;
	ml::Posture pose = motion_.GetPostureAtTime_ms(t);
	ml_u_poser_.Retarget(pose, CurframePose);

	InternalTimeAccumulator += Context.GetDeltaTime() * FMath::Abs(PlayRate);
}

void FAnimNode_BvhConvert::FAnimNode_BvhConvert::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	DECLARE_SCOPE_HIERARCHICAL_COUNTER_ANIMNODE(EvaluateSkeletalControl_AnyThread)
	Output.ResetToRefPose();

	//遍历骨架的每根骨骼，只要CurframePose有数据就覆盖该骨骼的
	USkeletalMeshComponent* Component = Output.AnimInstanceProxy->GetSkelMeshComponent();
	if (Component && Component->GetSkeletalMeshAsset())
	{
		const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();
		
		for (TMap<FName, FTransform>::TIterator ItrMap(CurframePose); ItrMap; ++ItrMap)
		{
			int32 Index = Component->GetBoneIndex(ItrMap.Key());
			if (Index != INDEX_NONE)
			{
				FTransform SourceTM = ItrMap.Value();

				FCompactPoseBoneIndex CompactPoseBone = BoneContainer.GetCompactPoseIndexFromSkeletonPoseIndex(FSkeletonPoseBoneIndex(Index));
				//FTransform NewBoneTM = Output.Pose.GetComponentSpaceTransform(CompactPoseBone);

				OutBoneTransforms.Add(FBoneTransform(CompactPoseBone, SourceTM));
			}
		}

		OutBoneTransforms.Sort([](const FBoneTransform& A, const FBoneTransform& B) {
			return (A.BoneIndex <= B.BoneIndex);
			});
	}
}


bool FAnimNode_BvhConvert::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	return true;
}
