// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "Animation/AnimNodeBase.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "Motion/ml.h"			

#include "AnimNode_BvhConvert.generated.h"

class USkeletalMeshComponent;

//为了暴露到蓝图配置 并且对应ml库的枚举
UENUM(BlueprintType)
enum class EPresetJointTag : uint8
{
	UNKNOWN = 0,
	PELVIS = 1, SPINE, SPINE1, SPINE2, CHEST, NECK, HEAD, HEAD_END,
	L_HIP, L_KNEE, L_ANKLE, L_FOOT, L_TOE, L_TOE_END,
	R_HIP, R_KNEE, R_ANKLE, R_FOOT, R_TOE, R_TOE_END,
	L_CLAVICLE, L_SHOULDER, L_ELBOW, L_WRIST, L_PALM, L_PALM_END,
	R_CLAVICLE, R_SHOULDER, R_ELBOW, R_WRIST, R_PALM, R_PALM_END
};


//BVH 到 Skeleton 的骨架映射 EPresetJointTag = { BvhJoint: SkeletonBone }
USTRUCT(BlueprintType)
struct FBoneMapConfig
{
	GENERATED_BODY()
public:
	UPROPERTY(EditAnywhere)
	FString Bvh_Joint;
	UPROPERTY(EditAnywhere)
	FString UE_Bone;
};

class USkeletalMeshComponent;

/**
 *	Convert Bvh Data to Pose Data
 */
USTRUCT(BlueprintInternalUseOnly)
struct BVHCONVERTER_API FAnimNode_BvhConvert : public FAnimNode_SkeletalControlBase
{
	GENERATED_USTRUCT_BODY()

public:
	/*这里需要BVH数据的第一帧需要是T-pose或A-pose
	* 需要和骨架的默认姿势一致,否则会影响重定向的效果
	*/
	UPROPERTY(Category = "BVH", EditAnywhere)
	FString T_Pose_Bvh_Filename = "bvh/APose.bvh";

	UPROPERTY(Category = "BVH", EditAnywhere)
	FString Motion_Bvh_Filename = "bvh/Motion.bvh";

	//BVH 到 Skeleton 的骨架映射,类似Rig绑定得对应关系， JointTag = {BvhJoint : SkeletonBone}
	UPROPERTY(Category = "BVH", EditAnywhere)
	TMap<EPresetJointTag, FBoneMapConfig > BoneMaping;


	//TODO：通过正则表达式自动配置boneMapping
	//UPROPERTY(Category = "BVH", EditAnywhere)
	//bool AutoMap = false;

	/*是否循环*/
	UPROPERTY(EditAnywhere, Category = Settings, meta = (PinHiddenByDefault))
	bool bLoopAnimation = true;

	// The play rate multiplier. Can be negative, which will cause the animation to play in reverse.
	UPROPERTY(EditAnywhere, Category = Settings, meta = (PinHiddenByDefault))
	float PlayRate = 1.0f;

	/** Accumulated time used to reference the asset in this node */
	UPROPERTY(BlueprintReadWrite, Transient, Category = DoNotEdit)
	float InternalTimeAccumulator = 0.0f;
private:
	ml::Motion motion_;
	ml::UE4Poser ml_u_poser_;

	//BoneName，Transform
	UPROPERTY(Transient)
	TMap<FName, FTransform> CurframePose;

public:
	FAnimNode_BvhConvert();
	virtual ~FAnimNode_BvhConvert() override;

	virtual void OnInitializeAnimInstance(const FAnimInstanceProxy* InProxy, const UAnimInstance* InAnimInstance) override;
	virtual bool NeedsOnInitializeAnimInstance() const override { return true; }

	virtual void Initialize_AnyThread(const FAnimationInitializeContext& Context) override;
	virtual void UpdateInternal(const FAnimationUpdateContext& Context) override;
	//virtual void Evaluate_AnyThread(FPoseContext& Output) override;
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual void CacheBones_AnyThread(const FAnimationCacheBonesContext& Context)  override;
	virtual void GatherDebugData(FNodeDebugData& DebugData) override;

	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;

	void InitRetargtor(const UAnimInstance* InAnimInstance);

	//return bvh motion length in seconds
	float GetTotalLength();
};
