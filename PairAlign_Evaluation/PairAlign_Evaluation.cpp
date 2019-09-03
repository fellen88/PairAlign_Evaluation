// Pointcloud_Registration.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。

#include "3d_features.h"
#include "3d_registration.h"
#include "pcl_visualizer.h"
#include "data_process.h"
#include <random>

constexpr auto Pi = 3.14159;

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

	Eigen::Matrix4f Final_Transform;
	PointCloud::Ptr output(new PointCloud);
	PointCloud::Ptr pre_output(new PointCloud);
	PointCloud::Ptr pre_output_sac(new PointCloud);
	PointCloud::Ptr target_transformed(new PointCloud);
	std::default_random_engine random_engine;
	std::uniform_real_distribution<double> random_distribution(-0.5, 0.5); //随机数分布对象, 范围0-1之间
	bool real_overturn = false;
	bool overturn = false;
	int n_count = 0;
	registration reg(false);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("CAD Registration"));
	int v1;
	int v2;
	view->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	view->setBackgroundColor(0, 0, 0, v1);
	view->setBackgroundColor(0, 0, 0, v2);
    view->addCoordinateSystem(0.1);  
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source, 250, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target, 0, 250, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_transformed_cloud_color(target, 255, 250, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligend_cloud_color(output, 255, 0, 0);

	while (n_count < 10)
	{
		//target random transformation 
		n_count++;
		Eigen::Vector3d real_euler_angle;
		Eigen::Vector3d euler_angle;
		real_euler_angle[2] = 2 * Pi * random_distribution(random_engine);
		std::cout << random_distribution(random_engine) << endl;
		real_euler_angle[1] = 2 * Pi * random_distribution(random_engine);
		std::cout << random_distribution(random_engine) << endl;
		real_euler_angle[0] = 2 * Pi * random_distribution(random_engine);
		cout << real_euler_angle[2] << endl;
		cout << real_euler_angle[1] << endl;
		cout << real_euler_angle[0] << endl;
		real_overturn = false;
		overturn = false;

		if (real_euler_angle[2] > Pi/2 && real_euler_angle[2] < 3*Pi/2)
		{
			real_overturn = !real_overturn;
		}
		if (real_euler_angle[1] > Pi/2 && real_euler_angle[1] < 3*Pi/2)
		{
			real_overturn = !real_overturn;
		}
		if (real_euler_angle[0] > Pi/2 && real_euler_angle[0] < 3*Pi/2)
		{
			//real_overturn = !real_overturn;//绕Z轴旋转不发生翻转！
		}
		cout << "real_overturn = " << real_overturn << endl;

		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(real_euler_angle(2),Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(real_euler_angle(1),Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(real_euler_angle(0),Eigen::Vector3d::UnitZ()));
		Eigen::Matrix3d rotation_matrix;
		rotation_matrix = yawAngle * pitchAngle * rollAngle;
		float translation_x = 0, translation_y = 0, translation_z = 0;
		translation_x = 0.06 * random_distribution(random_engine) + 0.02;
		translation_y = 0.06 * random_distribution(random_engine) + 0.02;
		translation_z = 0.06 * random_distribution(random_engine) + 0.02;
		Eigen::Matrix4f real_transformation_matrix = Eigen::Matrix4f::Identity();
		for(int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
			{
				real_transformation_matrix(i, j) = rotation_matrix(i, j);
			}
		real_transformation_matrix(0, 3) = translation_x;
		real_transformation_matrix(1, 3) = translation_y;
		real_transformation_matrix(2, 3) = translation_z;
		pcl::transformPointCloud(*target, *target_transformed, real_transformation_matrix);
		//end target transformation
	
		reg.SAC_IA_PareAlign(source, target_transformed, pre_output_sac, true);
		//reg.prePairAlign(target, source, pre_output, true);
		reg.pairAlign(target_transformed, pre_output_sac, output, Final_Transform, true);
		Eigen::Matrix3d final_rotation_matrix;
		for(int i = 0; i < 3; i++)		 for(int j = 0; j < 3; j++)		 {		  final_rotation_matrix(i, j) = Final_Transform(i, j);		 }
		euler_angle = final_rotation_matrix.eulerAngles(2, 1, 0);//顺序Z, Y, X		for (int t = 0; t < 3; t++)		{			if (euler_angle[t] < 0);		//	euler_angle[t] = euler_angle[t] + 2 * Pi;		}		if (euler_angle[2] > Pi/2 && euler_angle[2] < 3*Pi/2)
		{
			overturn = !overturn;
		}
		if (euler_angle[1] > Pi/2 && euler_angle[1] < 3*Pi/2)
		{
			overturn = !overturn;
		}
		if (euler_angle[0] > Pi/2 && euler_angle[0] < 3*Pi/2)
		{
			//overturn = !overturn;//绕Z轴旋转不发生翻转！
		}
		cout << "overturn = " << overturn << endl;
		if (real_overturn != overturn)
		{
			cout << "overturn!!!" << endl;
		}
			cout << "euler_angle[2] = " << euler_angle[2] << endl;		cout << "euler_angle[1] = " << euler_angle[1] << endl;		cout << "euler_angle[0] = " << euler_angle[0] << endl;

		//************************** visualization ******************************//
		view->removeAllPointClouds();
		view->addPointCloud(source, sources_cloud_color, "sources_cloud_v1", v1);
		view->addPointCloud(target, target_cloud_color, "target_cloud_v1", v1);
		view->addPointCloud(target_transformed, target_transformed_cloud_color, "target_transformed_cloud_v1", v1);
		view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");
		view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v1");
		view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_transformed_cloud_v1");

		view->addPointCloud(output, aligend_cloud_color, "aligend_cloud_v2", v2);
		//view->addPointCloud(source, sources_cloud_color, "sources_cloud_v2", v2);
		view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligend_cloud_v2");
		//view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sources_cloud_v2");
		view->spin();
		//*********************************************************************//
	}
	return 0;
}

