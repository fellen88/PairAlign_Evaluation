// Pointcloud_Registration.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。

#include "3d_features.h"
#include "3d_registration.h"
#include "pcl_visualizer.h"
#include "data_process.h"

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		cout << "please input pointcloud" << endl;
		return -1;
	}

	PointCloud::Ptr source(new PointCloud);
	PointCloud::Ptr target(new PointCloud);

	data_process(argc, argv, source);
	singleview_sample(2, argv, target);

	//**************************** registration ****************************//
	Eigen::Matrix4f Final_Transform;
	PointCloud::Ptr output(new PointCloud);
	PointCloud::Ptr pre_output(new PointCloud);
	PointCloud::Ptr pre_output_sac(new PointCloud);

	registration reg(false);
	reg.SAC_IA_PareAlign(target, source,  pre_output_sac, true);
	//reg.prePairAlign(target, source, pre_output, true);
    reg.pairAlign(source, pre_output_sac, output, Final_Transform, true);
	//***********************************************************************//

	//************************** visualization ******************************//
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("CAD Registration"));
	int v1;
	int v2;
	view->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	view->setBackgroundColor(0, 0, 0, v1);
	view->setBackgroundColor(0, 0, 0, v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source, 250, 250, 250);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target, 0, 250, 0);
	view->addPointCloud(source, sources_cloud_color, "sources_cloud_v1", v1);
	view->addPointCloud(target, target_cloud_color, "target_cloud_v1", v1);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v1");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>aligend_cloud_color(output, 0, 255, 0);
	view->addPointCloud(output, aligend_cloud_color, "aligend_cloud_v2", v2);
	//view->addPointCloud(source, sources_cloud_color, "sources_cloud_v2", v2);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligend_cloud_v2");
	//view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sources_cloud_v2");
	view->spin();
	//*********************************************************************//

	//pcl::io::savePCDFile("crou_output.pcd", *align);
	//system("pause");
	return 0;
}

