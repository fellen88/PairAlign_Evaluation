#ifndef REGISTRATION_H
#define REGISTRATION_H  

#include "stdafx.h"
#include "features.h"
#include "preprocessing.h"
//Åä×¼
#include <pcl/registration/icp.h> 
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>

class Registration:public Features, public PreProcessing
{
  public:

  bool DEBUG_VISUALIZER;

  Registration(bool DebugVisualizer);
  void SACPareAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, Eigen::Matrix4f &SAC_transform, bool downsample);
  void PairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);
};
#endif