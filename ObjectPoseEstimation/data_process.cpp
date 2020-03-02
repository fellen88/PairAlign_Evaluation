
#include "data_process.h"

VTK_MODULE_INIT(vtkRenderingOpenGL);

inline double
uniform_deviate (int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}
 
inline void
randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3,
                     Eigen::Vector4f& p)
{
  float r1 = static_cast<float> (uniform_deviate (rand ()));
  float r2 = static_cast<float> (uniform_deviate (rand ()));
  float r1sqr = std::sqrt (r1);
  float OneMinR1Sqr = (1 - r1sqr);
  float OneMinR2 = (1 - r2);
  a1 *= OneMinR1Sqr;
  a2 *= OneMinR1Sqr;
  a3 *= OneMinR1Sqr;
  b1 *= OneMinR2;
  b2 *= OneMinR2;
  b3 *= OneMinR2;
  c1 = r1sqr * (r2 * c1 + b1) + a1;
  c2 = r1sqr * (r2 * c2 + b2) + a2;
  c3 = r1sqr * (r2 * c3 + b3) + a3;
  p[0] = c1;
  p[1] = c2;
  p[2] = c3;
  p[3] = 0;
}
 
inline void
randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p, bool calcNormal, Eigen::Vector3f& n)
{
  float r = static_cast<float> (uniform_deviate (rand ()) * totalArea);
 
  std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
  vtkIdType el = vtkIdType (low - cumulativeAreas->begin ());
 
  double A[3], B[3], C[3];
  vtkIdType npts = 0;
  vtkIdType *ptIds = NULL;
  polydata->GetCellPoints (el, npts, ptIds);
  polydata->GetPoint (ptIds[0], A);
  polydata->GetPoint (ptIds[1], B);
  polydata->GetPoint (ptIds[2], C);
  if (calcNormal)
  {
    // OBJ: Vertices are stored in a counter-clockwise order by default
    Eigen::Vector3f v1 = Eigen::Vector3f (A[0], A[1], A[2]) - Eigen::Vector3f (C[0], C[1], C[2]);
    Eigen::Vector3f v2 = Eigen::Vector3f (B[0], B[1], B[2]) - Eigen::Vector3f (C[0], C[1], C[2]);
    n = v1.cross (v2);
    n.normalize ();
  }
  randomPointTriangle (float (A[0]), float (A[1]), float (A[2]),
                       float (B[0]), float (B[1]), float (B[2]),
                       float (C[0]), float (C[1]), float (C[2]), p);
}
 
void
uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, bool calc_normal, pcl::PointCloud<pcl::PointXYZ> & cloud_out)
{
  polydata->BuildCells ();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();
 
  double p1[3], p2[3], p3[3], totalArea = 0;
  std::vector<double> cumulativeAreas (cells->GetNumberOfCells (), 0);
  size_t i = 0;
  vtkIdType npts = 0, *ptIds = NULL;
  for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds); i++)
  {
    polydata->GetPoint (ptIds[0], p1);
    polydata->GetPoint (ptIds[1], p2);
    polydata->GetPoint (ptIds[2], p3);
    totalArea += vtkTriangle::TriangleArea (p1, p2, p3);
    cumulativeAreas[i] = totalArea;
  }
 
  cloud_out.points.resize (n_samples);
  cloud_out.width = static_cast<pcl::uint32_t> (n_samples);
  cloud_out.height = 1;
 
  for (i = 0; i < n_samples; i++)
  {
    Eigen::Vector4f p;
    Eigen::Vector3f n;
    randPSurface (polydata, &cumulativeAreas, totalArea, p, calc_normal, n);
	cloud_out.points[i].x = p[0] / 1000;
    cloud_out.points[i].y = p[1] / 1000;
    cloud_out.points[i].z = p[2] / 1000;
   // if (calc_normal)
   // {
   //   cloud_out.points[i].normal_x = n[0];
   //   cloud_out.points[i].normal_y = n[1];
   //   cloud_out.points[i].normal_z = n[2];
   // }
  }
}
 
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
 
const int default_number_samples = 100000;
const float default_leaf_size = 0.01f;
 
int
data_process (int argc, char **argv, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud)
{
  // Parse command line arguments
  int SAMPLE_POINTS_ = default_number_samples;
  parse_argument (argc, argv, "-n_samples", SAMPLE_POINTS_);
  float leaf_size = default_leaf_size;
  parse_argument (argc, argv, "-leaf_size", leaf_size);
  bool vis_result = ! find_switch (argc, argv, "-no_vis_result");
  const bool write_normals = find_switch (argc, argv, "-write_normals");
 
  // Parse the command line arguments for .ply and PCD files
  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
 // if (pcd_file_indices.size () != 1)
 // {
 //   print_error ("Need a single output PCD file to continue.\n");
 //   return (-1);
 // }
  std::vector<int> ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");
  std::vector<int> obj_file_indices = parse_file_extension_argument (argc, argv, ".obj");
  std::vector<int> stl_file_indices = parse_file_extension_argument (argc, argv, ".stl");
  if (ply_file_indices.size () == 0 && obj_file_indices.size () == 0 && stl_file_indices.size() == 0)
  {
    print_error ("Need a single input PLY/OBJ/STL file to continue.\n");
    return (-1);
  }
 
  vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New ();
  if (ply_file_indices.size () != 0)
  {
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFilePLY (argv[ply_file_indices[0]], mesh);
    pcl::io::mesh2vtk (mesh, polydata1);
  }
  else if (stl_file_indices.size () != 0)
  {
	pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFile (argv[stl_file_indices[0]], mesh);
    pcl::io::mesh2vtk (mesh, polydata1);
  }
  else if (obj_file_indices.size () != 0)
  {
    vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New ();
    readerQuery->SetFileName (argv[obj_file_indices[0]]);
    readerQuery->Update ();
    polydata1 = readerQuery->GetOutput ();
  }


  vtkOutputWindow::SetGlobalWarningDisplay(0);
  //make sure that the polygons are triangles!
  vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New ();
#if VTK_MAJOR_VERSION < 6
  triangleFilter->SetInput (polydata1);
#else
  triangleFilter->SetInputData (polydata1);
#endif
  triangleFilter->Update ();
 
  vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  triangleMapper->SetInputConnection (triangleFilter->GetOutputPort ());
  triangleMapper->Update ();
  polydata1 = triangleMapper->GetInput ();
 
 // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointNormal>);
  uniform_sampling (polydata1, SAMPLE_POINTS_, write_normals, *transformed_cloud);
  //pcl::io::savePCDFileASCII("data_process.pcd", *transformed_cloud);

//示输入点坐标
//std::vector<pcl::PointXYZ>::iterator iter;
//for (iter = transformed_cloud->points.begin(); iter != transformed_cloud->points.end(); iter++)
//{
//	if (iter->x != 0)
//	{
//		cout << "point.x = " << iter->x << endl;
//		cout << "point.y = " << iter->y << endl;
//		cout << "point.z = " << iter->z << endl;
//	}
//}

  return 1;
}


void 
singleview_sample(int i, char **argv, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud)
{
	/*+++++++++++++++++++++++++单视角点云获取+++++++++++++++++++++++++++++++*/
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkSTLReader> readerQuery = vtkSmartPointer<vtkSTLReader>::New();
	//读取CAD模型
	readerQuery->SetFileName(argv[2]);
	readerQuery->Update();
	polydata = readerQuery->GetOutput();
	polydata->GetNumberOfPoints();

	//单视角点云获取
	float resx = 512;
	float resy = resx;
	std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > views_xyz;
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
	std::vector<float> entropies;
	pcl::visualization::PCLVisualizer vis;
	vis.addModelFromPolyData(polydata, "mesh", 0);
	vis.setRepresentationToSurfaceForAllActors();
	vis.renderViewTesselatedSphere(resx, resy, views_xyz, poses, entropies, 0, 90, 100 , TRUE);
	//for (int i = 0; i < views_xyz.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZ> views_cloud;
		pcl::transformPointCloud<pcl::PointXYZ>(views_xyz[i], *transformed_cloud, poses[i].inverse());

		//点云单位转换mm—>m， 矩阵方法
		Eigen::Matrix4f transformation_axis = Eigen::Matrix4f::Identity();
		transformation_axis(0, 0) = 0.001;
		transformation_axis(1, 1) = 0.001;
		transformation_axis(2, 2) = 0.001;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transformation_axis);

	//	std::stringstream ss;
	//	ss << "cloud_view_" << i << ".ply";
	//	pcl::io::savePLYFile(ss.str(), views_cloud);
	}
	
	//for (int i = 0; i < views_xyz.size(); i++)
	//{
	//	pcl::io::savePCDFileASCII("../single_view_pcd/"+std::to_string(i)+".pcd", views_xyz[i]);
	//}	
}


void model_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
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
	boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl_viewer->setBackgroundColor(0, 0, 0);
	pcl_viewer->addCoordinateSystem(0.5);
	pcl_viewer->initCameraParameters();
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

	pcl_viewer->spin();

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
	//pcl::io::savePCDFileASCII("transformed_cloud.pcd", *transformed_cloud);
}
