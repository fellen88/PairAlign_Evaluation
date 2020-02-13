// Pointcloud_Registration.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。

#include "3d_features.h"
#include "3d_registration.h"
#include "3d_visualizer.h"
#include "3d_data_process.h"
#include <random>

constexpr auto Pi = 3.14159;

void model_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, pcl::visualization::PCLVisualizer::Ptr pcl_viewer)
{
		//******************************OBB包围盒计算*************************************//
		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud(input_cloud);
		feature_extractor.compute();

		std::vector <float> moment_of_inertia;
		std::vector <float> eccentricity;
		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;
		pcl::PointXYZ min_point_OBB;
		pcl::PointXYZ max_point_OBB;
		pcl::PointXYZ position_OBB;
		Eigen::Matrix3f rotational_matrix_OBB;
		float major_value, middle_value, minor_value;
		Eigen::Vector3f major_vector, middle_vector, minor_vector;
		Eigen::Vector3f mass_center;

		feature_extractor.getMomentOfInertia(moment_of_inertia);
		feature_extractor.getEccentricity(eccentricity);
		feature_extractor.getAABB(min_point_AABB, max_point_AABB);
		feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
		feature_extractor.getEigenValues(major_value, middle_value, minor_value);
		feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
		feature_extractor.getMassCenter(mass_center);

		//***********************************可视化重心、包围盒和坐标系******************************************//
		//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		//pcl_viewer->setBackgroundColor(0, 0, 0);
		//pcl_viewer->addCoordinateSystem(0.5);
		//pcl_viewer->initCameraParameters();
		pcl_viewer->addPointCloud<pcl::PointXYZ>(input_cloud, "sample cloud");
		//pcl_viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

		Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
		Eigen::Quaternionf quat(rotational_matrix_OBB);
		//pcl_viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

		pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
		pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
		pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
		pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
		pcl_viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
		pcl_viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
		pcl_viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
		
		Eigen::Vector3f p1(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
		Eigen::Vector3f p2(min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
		Eigen::Vector3f p3(max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
		Eigen::Vector3f p4(max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
		Eigen::Vector3f p5(min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
		Eigen::Vector3f p6(min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
		Eigen::Vector3f p7(max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
		Eigen::Vector3f p8(max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

		p1 = rotational_matrix_OBB * p1 + position;
		p2 = rotational_matrix_OBB * p2 + position;
		p3 = rotational_matrix_OBB * p3 + position;
		p4 = rotational_matrix_OBB * p4 + position;
		p5 = rotational_matrix_OBB * p5 + position;
		p6 = rotational_matrix_OBB * p6 + position;
		p7 = rotational_matrix_OBB * p7 + position;
		p8 = rotational_matrix_OBB * p8 + position;

		pcl::PointXYZ pt1(p1(0), p1(1), p1(2));
		pcl::PointXYZ pt2(p2(0), p2(1), p2(2));
		pcl::PointXYZ pt3(p3(0), p3(1), p3(2));
		pcl::PointXYZ pt4(p4(0), p4(1), p4(2));
		pcl::PointXYZ pt5(p5(0), p5(1), p5(2));
		pcl::PointXYZ pt6(p6(0), p6(1), p6(2));
		pcl::PointXYZ pt7(p7(0), p7(1), p7(2));
		pcl::PointXYZ pt8(p8(0), p8(1), p8(2));
		
		pcl_viewer->addLine(pt1, pt2, 1.0, 0.0, 0.0, "1 edge");
		pcl_viewer->addLine(pt1, pt4, 1.0, 0.0, 0.0, "2 edge");
		pcl_viewer->addLine(pt1, pt5, 1.0, 0.0, 0.0, "3 edge");
		pcl_viewer->addLine(pt5, pt6, 1.0, 0.0, 0.0, "4 edge");
		pcl_viewer->addLine(pt5, pt8, 1.0, 0.0, 0.0, "5 edge");
		pcl_viewer->addLine(pt2, pt6, 1.0, 0.0, 0.0, "6 edge");
		pcl_viewer->addLine(pt6, pt7, 1.0, 0.0, 0.0, "7 edge");
		pcl_viewer->addLine(pt7, pt8, 1.0, 0.0, 0.0, "8 edge");
		pcl_viewer->addLine(pt2, pt3, 1.0, 0.0, 0.0, "9 edge");
		pcl_viewer->addLine(pt4, pt8, 1.0, 0.0, 0.0, "10 edge");
		pcl_viewer->addLine(pt3, pt4, 1.0, 0.0, 0.0, "11 edge");
		pcl_viewer->addLine(pt3, pt7, 1.0, 0.0, 0.0, "12 edge");
		
		//while (!pcl_viewer->wasStopped())
		//{
		//	pcl_viewer->spinOnce(100);
		//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		//}

		//************************************计算旋转矩阵***********************************//
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

		cloud_in->width = 4;
		cloud_in->height = 1;
		cloud_in->is_dense = false;
		cloud_in->resize(cloud_in->width * cloud_in->height);

		cloud_out->width = 4;
		cloud_out->height = 1;
		cloud_out->is_dense = false;
		cloud_out->resize(cloud_out->width * cloud_out->height);

		//输入四个点
		cloud_in->points[0].x = center.x;
		cloud_in->points[0].y = center.y;
		cloud_in->points[0].z = center.z;

		cloud_in->points[1].x = x_axis.x;
		cloud_in->points[1].y = x_axis.y;
		cloud_in->points[1].z = x_axis.z;

		cloud_in->points[2].x = y_axis.x;
		cloud_in->points[2].y = y_axis.y;
		cloud_in->points[2].z = y_axis.z;

		cloud_in->points[3].x = z_axis.x;
		cloud_in->points[3].y = z_axis.y;
		cloud_in->points[3].z = z_axis.z;

		//目标四个点
		cloud_out->points[0].x = 0;
		cloud_out->points[0].y = 0;
		cloud_out->points[0].z = 0;

		cloud_out->points[1].x = 1;
		cloud_out->points[1].y = 0;
		cloud_out->points[1].z = 0;

		cloud_out->points[2].x = 0;
		cloud_out->points[2].y = 1;
		cloud_out->points[2].z = 0;

		cloud_out->points[3].x = 0;
		cloud_out->points[3].y = 0;
		cloud_out->points[3].z = 1;

		//利用SVD方法求解变换矩阵  
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation2;
		TESVD.estimateRigidTransformation(*cloud_in, *cloud_out, transformation2);
		//输出变换矩阵信息  
		std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << std::endl;
		printf("\n");
		printf("    | %6.3f %6.3f %6.3f | \n", transformation2(0, 0), transformation2(0, 1), transformation2(0, 2));
		printf("R = | %6.3f %6.3f %6.3f | \n", transformation2(1, 0), transformation2(1, 1), transformation2(1, 2));
		printf("    | %6.3f %6.3f %6.3f | \n", transformation2(2, 0), transformation2(2, 1), transformation2(2, 2));
		printf("\n");
		printf("t = < %0.3f, %0.3f, %0.3f >\n", transformation2(0, 3), transformation2(1, 3), transformation2(2, 3));

		*output_cloud = *cloud_in;
	
		// Executing the transformation
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
		// You can either apply transform_1 or transform_2; they are the same
		pcl::transformPointCloud(*input_cloud, *transformed_cloud, transformation2);
		pcl::io::savePCDFileASCII("transformed_cloud.pcd", *transformed_cloud);
}

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
	Eigen::Matrix4f SAC_Transform;
	Eigen::Matrix4f ICP_Transform;
	PointCloud::Ptr output(new PointCloud);
	PointCloud::Ptr pre_output(new PointCloud);
	PointCloud::Ptr pre_output_sac(new PointCloud);
	PointCloud::Ptr source_transformed(new PointCloud);
	PointCloud::Ptr target_transformed(new PointCloud);
	std::default_random_engine random_engine;
	std::uniform_real_distribution<double> random_distribution(0, 1); //随机数分布对象, 范围0-1之间
	bool real_overturn = false;
	bool overturn = false;
	int n_count = 0;
	int n_count_correct = 0;
	registration reg(true);

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

	PointCloud::Ptr Point_OXYZ(new PointCloud);
	PointCloud::Ptr Point_Transformed(new PointCloud);

	model_transform(source, Point_OXYZ, view);
	float translation_error_sum = 0.0;
	float rotation_error_sum = 0.0;
	float translation_error = 0.0;
	float rotation_error = 0.0;
	float translation_error_temp = 0.0;
	int count_trans_error01 = 0;
	int count_trans_error03 = 0;
	int count_trans_error05 = 0;
	int count_trans_error10 = 0;
	int count_trans_error15 = 0;
	float translation_error_01 = 0;
	float translation_error_03 = 0;
	float translation_error_05 = 0;
	float translation_error_10 = 0;
	float translation_error_15 = 0;

	int count_rot_error01 = 0;
	int count_rot_error03 = 0;
	int count_rot_error05 = 0;
	int count_rot_error10 = 0;
	int count_rot_error15 = 0;
	float rotation_error_01 = 0;
	float rotation_error_03 = 0;
	float rotation_error_05 = 0;
	float rotation_error_10 = 0;
	float rotation_error_15 = 0;
	
	while (n_count < 5000)
	{
		//target random transformation 
		n_count++;
		cout << "##################################################################" << endl;
		cout << "Evaluation No." << n_count << endl;
		Eigen::Vector3d real_euler_angle;
		Eigen::Vector3d euler_angle;
		real_euler_angle[2] = 2 * Pi * random_distribution(random_engine);
		//std::cout << random_distribution(random_engine) << endl;
		real_euler_angle[1] = 2 * Pi * random_distribution(random_engine);
		//std::cout << random_distribution(random_engine) << endl;
		real_euler_angle[0] = 2 * Pi * random_distribution(random_engine);
		//cout << real_euler_angle[2] << endl;
		//cout << real_euler_angle[1] << endl;
		//cout << real_euler_angle[0] << endl;

		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(real_euler_angle(2),Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(real_euler_angle(1),Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(real_euler_angle(0),Eigen::Vector3d::UnitZ()));
		Eigen::Matrix3d rotation_matrix;
		rotation_matrix = yawAngle * pitchAngle * rollAngle;
		float translation_x = 0, translation_y = 0, translation_z = 0;
		translation_x = 1 * random_distribution(random_engine) + 0.02;
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
		std::cout << "The Global Rotation and translation matrices are : \n" << std::endl;
        printf("\n");
        printf("    | %6.3f %6.3f %6.3f %6.3f | \n", real_transformation_matrix(0, 0), real_transformation_matrix(0, 1), real_transformation_matrix(0, 2), real_transformation_matrix(0, 3));
        printf("R = | %6.3f %6.3f %6.3f %6.3f | \n", real_transformation_matrix(1, 0), real_transformation_matrix(1, 1), real_transformation_matrix(1, 2), real_transformation_matrix(1, 3));
        printf("    | %6.3f %6.3f %6.3f %6.3f | \n", real_transformation_matrix(2, 0), real_transformation_matrix(2, 1), real_transformation_matrix(2, 2), real_transformation_matrix(2, 3));
        printf("    | %6.3f %6.3f %6.3f %6.3f | \n", real_transformation_matrix(3, 0), real_transformation_matrix(3, 1), real_transformation_matrix(3, 2), real_transformation_matrix(3, 3));
        printf("\n");
        printf("****************************************************** \n");

		//end target transformation
	
		//reg.prePairAlign(target_transformed, source, pre_output,SAC_Transform, true);
		//reg.pairAlign(target_transformed, pre_output, output, ICP_Transform, true);
		reg.SAC_IA_PareAlign(source, target_transformed, pre_output_sac, SAC_Transform, true);
		reg.pairAlign(target_transformed, pre_output_sac, output, ICP_Transform, true);
		Final_Transform = ICP_Transform * SAC_Transform;
		pcl::transformPointCloud(*source, *source_transformed, Final_Transform);

	  std::cout << "The Global Rotation and translation matrices are : \n" << std::endl;
      printf("\n");
      printf("    | %6.3f %6.3f %6.3f %6.3f | \n", Final_Transform(0, 0), Final_Transform(0, 1), Final_Transform(0, 2), Final_Transform(0, 3));
      printf("R = | %6.3f %6.3f %6.3f %6.3f | \n", Final_Transform(1, 0), Final_Transform(1, 1), Final_Transform(1, 2), Final_Transform(1, 3));
      printf("    | %6.3f %6.3f %6.3f %6.3f | \n", Final_Transform(2, 0), Final_Transform(2, 1), Final_Transform(2, 2), Final_Transform(2, 3));
      printf("    | %6.3f %6.3f %6.3f %6.3f | \n", Final_Transform(3, 0), Final_Transform(3, 1), Final_Transform(3, 2), Final_Transform(3, 3));
      printf("\n");
      printf("****************************************************** \n");

	  Point_Transformed->points.clear();
	  pcl::transformPointCloud(*Point_OXYZ, *Point_Transformed, Final_Transform.inverse() * real_transformation_matrix);

	  //物体姿态z方向向上的情况，传出标志，对最终变换矩阵绕物体x轴旋转180度
	  Eigen::Vector3f ObjectZ(Point_Transformed->points[3].x - Point_Transformed->points[0].x, Point_Transformed->points[3].y - Point_Transformed->points[0].y, Point_Transformed->points[3].z - Point_Transformed->points[0].z);
      float NormOfObjectZ = sqrt(ObjectZ(0)*ObjectZ(0) + ObjectZ(1)*ObjectZ(1) + ObjectZ(2)*ObjectZ(2));
      Eigen::Vector3f CameraZ( Point_OXYZ->points[3].x -  Point_OXYZ->points[0].x,  Point_OXYZ->points[3].y -  Point_OXYZ->points[0].y,  Point_OXYZ->points[3].z -  Point_OXYZ->points[0].z);
      float NormOfCameraZ = sqrt(CameraZ(0)*CameraZ(0) +  CameraZ(1)* CameraZ(1) +  CameraZ(2)* CameraZ(2));
      float Theta = 180*acos((ObjectZ.dot(CameraZ))/(NormOfObjectZ*NormOfCameraZ))/Pi;
      cout << "rotation error : " << Theta << endl;
	  if (Theta < 90)
	  {
		  n_count_correct++;
	      rotation_error_sum += abs(Theta);
	      rotation_error = rotation_error_sum / n_count_correct;
	  }
	  if (abs(Theta) < 1)
	  {
		  count_rot_error01++;
	  }
	  else if (abs(Theta) < 3)
	  {
		  count_rot_error03++;
	  }
	  else if (abs(Theta) < 5)
	  {
		  count_rot_error05++;
	  }
	  else if (abs(Theta) < 10)
	  {
		  count_rot_error10++;
	  }
	  else if (abs(Theta) < 15)
	  {
		  count_rot_error15++;
	  }

	 
	
	  cout << "rotation error (average) :" << rotation_error << endl;
	  
	  rotation_error_01 = float(count_rot_error01*100) / n_count;
	  rotation_error_03 = float(count_rot_error03*100) / n_count;
	  rotation_error_05 = float(count_rot_error05*100) / n_count;
	  rotation_error_10 = float(count_rot_error10*100) / n_count;
	  rotation_error_15 = float(count_rot_error15*100) / n_count;

	  cout << "rotation error < 1 degree :" << rotation_error_01 << "%" << endl;
	  cout << "rotation error < 3 degree :" << rotation_error_03 << "%" << endl;
	  cout << "rotation error < 5 degree :" << rotation_error_05 << "%" << endl;
	  cout << "rotation error < 10 degree :" << rotation_error_10 << "%" << endl;
	  cout << "rotation error < 15 degree :" << rotation_error_15 << "%" << endl;
	  cout << "rotation error > 15 degree :" << (100-rotation_error_01-rotation_error_03-rotation_error_05-rotation_error_10-rotation_error_15)<< "%" << endl;


	  float x_error = 1000 * (Point_Transformed->points[0].x - Point_OXYZ->points[0].x);
	  float y_error = 1000 * (Point_Transformed->points[0].y - Point_OXYZ->points[0].y);
	  float z_error = 1000 * (Point_Transformed->points[0].z - Point_OXYZ->points[0].z);

	  cout << "x error : " << x_error << endl;
	  cout << "y error : " << y_error << endl;
	  cout << "z error : " << z_error << endl;
	  translation_error_temp = (abs(x_error) + abs(y_error) + abs(z_error))/3;

	  cout << "translation error : " << translation_error_temp << endl;

	  if (Theta < 90)
	  {
	      translation_error_sum += (abs(x_error) + abs(y_error) + abs(z_error)) / 3;
	      translation_error = translation_error_sum / n_count_correct;
	  }
	  cout << "translation error (average): " << translation_error << endl;
	  cout << "upside accuracy rate : " << (n_count_correct * 100/ n_count) << "%" << endl;

	   //精度小于1mm
	  if (translation_error_temp  < 1.0)
	  {
		  count_trans_error01++;
	  }
	  //精度小于3mm
	  else if (translation_error_temp < 3.0)
	  {
		  count_trans_error03++;
	  }

	  //精度小于5mm
	  else if (translation_error_temp < 5.0)
	  {
		  count_trans_error05++;
	  }
	  
	  //精度小于10mm
	  else if (translation_error_temp < 10.0)
	  {
		  count_trans_error10++;
	  }
	  //精度小于5mm
	  else if (translation_error_temp < 15.0)
	  {
		  count_trans_error15++;
	  }

		  translation_error_01 = (float)count_trans_error01 * 100 / n_count;
		  translation_error_03 = (float)count_trans_error03 * 100 / n_count;
		  translation_error_05 = (float)count_trans_error05 * 100 / n_count;
		  translation_error_10 = (float)count_trans_error10 * 100 / n_count;
		  translation_error_15 = (float)count_trans_error15 * 100 / n_count;
	  cout << "translation error < 1mm :" << translation_error_01 << "%" << endl;
	  cout << "translation error < 3mm :" << translation_error_03 << "%" << endl;
	  cout << "translation error < 5mm :" << translation_error_05 << "%" << endl;
	  cout << "translation error < 10mm :" << translation_error_10 << "%" << endl;
	  cout << "translation error < 15mm :" << translation_error_15 << "%" << endl;
	  cout << "translation error > 15mm :" << (100-translation_error_01-translation_error_03-translation_error_05-translation_error_10-translation_error_15)<< "%" << endl;

	  if(false)
	  //if (Theta > 90)
	  {
		  //************************** visualization ******************************//
		  view->removeAllPointClouds();
		  view->addPointCloud(source_transformed, sources_cloud_color, "sources_cloud_v1", v1);
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
	}
	system("pause");
	return 0;
}

