#ifndef REGISTRATION_H
#define REGISTRATION_H  

#include "visualizer.h"
#include "features.h"
//Åä×¼
#include <pcl/registration/icp.h> 
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>

class Registration
{
  public:

  bool DEBUG_VISUALIZER;
  visualizer::PointCloudVisualizer pointcloud_visualizer_;

  Registration(bool DebugVisualizer);
	fpfhFeature::Ptr ComputeFpfh(const PointCloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree);
	void SACPareAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, Eigen::Matrix4f &SAC_transform, bool downsample);
  void PairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);
};
#endif