#pragma once
#include "registration.h"
#include "segmentation.h"

#include <module/classmacro.h>
#include <isimple.h>
const char* const g_clsidSimple = "94071767-ba6b-4769-9eb4-2ebf469289f9";

#ifdef __DLLEXPORT
#define __DLL_INTERFACE _declspec(dllexport)    // 导出函数 - 生成dll文件时使用
#else
#define __DLL_INTERFACE _declspec(dllimport)    // 导入函数 -使用dll是使用
#endif

class PoseEstimation :public ISimple
{
		X3BEGIN_CLASS_DECLARE(PoseEstimation, g_clsidSimple)
        X3DEFINE_INTERFACE_ENTRY(ISimple)
    X3END_CLASS_DECLARE()

	public:
	Registration *p_registration_;
	Segmentation *p_segmentation_;
	Eigen::Matrix4f final_transform;
	Eigen::Matrix4f sac_transform;
	Eigen::Matrix4f icp_transform;
	PointCloud::Ptr sac_output;
	PointCloud::Ptr icp_output;

	PoseEstimation();
	~PoseEstimation();

	void Init();
	Eigen::Matrix4f Compute(PointCloud::Ptr source, PointCloud::Ptr target);
	void test();
};

//__DLL_INTERFACE  PoseEstimation *GetInstance();

