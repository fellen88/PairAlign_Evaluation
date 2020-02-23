#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkoutputwindow.h>
#include <vtkAutoInit.h>
#include <pcl/features/moment_of_inertia_estimation.h>

int
data_process(int argc, char **argv,  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud);

void
singleview_sample(int i, char **argv, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud);

void model_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);
