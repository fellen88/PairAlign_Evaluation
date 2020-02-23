// pose_estimation.cpp : 定义 DLL 应用程序的导出函数。

#include "stdafx.h"

#define __DLLEXPORT
#include "pose_estimation.h"

PoseEstimation::PoseEstimation():sac_output(new pcl::PointCloud<pcl::PointXYZ>),
								 icp_output(new pcl::PointCloud<pcl::PointXYZ>)
{
	p_registration_ = new Registration(true);
}

PoseEstimation::~PoseEstimation()
{
}

void PoseEstimation::Init()
{

}

void PoseEstimation::Start(PointCloud::Ptr source, PointCloud::Ptr target)
{
	p_registration_->SACPareAlign(source, target, sac_output, sac_transform, true);
	p_registration_->PairAlign(target, sac_output, icp_output, icp_transform, true);
}

__DLL_INTERFACE PoseEstimation * GetInstance()
{
	PoseEstimation* p_pose_estimation_ = new PoseEstimation();
	p_pose_estimation_->Init();
	return p_pose_estimation_;
}

