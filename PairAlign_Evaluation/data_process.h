#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>

#include <vtkVersion.h>

#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
 
#include <vtkoutputwindow.h>
#include <vtkAutoInit.h>

int
data_process(int argc, char **argv,  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud);

void
singleview_sample(int i, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud);
