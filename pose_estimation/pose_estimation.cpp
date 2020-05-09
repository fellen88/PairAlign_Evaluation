// pose_estimation.cpp : 定义 DLL 应用程序的导出函数。

#include "stdafx.h"

#define __DLLEXPORT

#include "pose_estimation.h"


PoseEstimation::PoseEstimation():sac_output(new pcl::PointCloud<pcl::PointXYZ>),
								 icp_output(new pcl::PointCloud<pcl::PointXYZ>),
	               p_registration_(new Registration(true))
{
	LOG(INFO) << "PoseEstimation()";
}

PoseEstimation::~PoseEstimation()
{
}

void PoseEstimation::Init()
{
	LOG(INFO) << "Class PoseEstimation Init";
}


Eigen::Matrix4f PoseEstimation::Compute(PointCloud::Ptr source, PointCloud::Ptr target)
{
  p_registration_->SACPareAlign(source, target, sac_output, sac_transform, true);
  p_registration_->PairAlign(target, sac_output, icp_output, icp_transform, true);
	final_transform = icp_transform * sac_transform;
	return final_transform;
}

void PoseEstimation::Compute()
{
	p_registration_->RecieveImage();
}

__DLL_INTERFACE PoseEstimation * GetInstance()
{
	LOG(INFO) << "GetInstance()";
  PoseEstimation* p_pose_estimation_ = new PoseEstimation(); 
  p_pose_estimation_->Init();
  return p_pose_estimation_;
}



