// Pointcloud_Registration.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。

#include "data_process.h"
#include <random>

#include "../pose_estimation/pose_estimation.h"
#pragma comment(lib, "../X64/release/pose_estimation.lib")

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
	PointCloud::Ptr source_transformed(new PointCloud);
	PointCloud::Ptr target_transformed(new PointCloud);
	std::default_random_engine random_engine;
	std::uniform_real_distribution<double> random_distribution(0, 1); //随机数分布对象, 范围0-1之间
	bool real_overturn = false;
	bool overturn = false;
	int n_count = 0;
	int n_count_correct = 0;

	PointCloud::Ptr Point_OXYZ(new PointCloud);
	PointCloud::Ptr Point_Transformed(new PointCloud);

	model_transform(source, Point_OXYZ);
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

	PoseEstimation *pose_estimation_ =GetInstance();
	
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

		//*************************************************************************************//
		LOG(INFO) << "start pose estimation ";
		pose_estimation_->Start(source, target_transformed);
		//*************************************************************************************//
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
	}
	system("pause");
	return 0;
}

