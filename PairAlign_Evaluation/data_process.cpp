
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
  if (ply_file_indices.size () != 1 && obj_file_indices.size () != 1 && stl_file_indices.size() != 1)
  {
    print_error ("Need a single input PLY/OBJ/STL file to continue.\n");
    return (-1);
  }
 
  vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New ();
  if (ply_file_indices.size () == 1)
  {
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFilePLY (argv[ply_file_indices[0]], mesh);
    pcl::io::mesh2vtk (mesh, polydata1);
  }
  else if (stl_file_indices.size () == 1)
  {
	pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFile (argv[stl_file_indices[0]], mesh);
    pcl::io::mesh2vtk (mesh, polydata1);
  }
  else if (obj_file_indices.size () == 1)
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

  return 1;
}

void 
singleview_sample(int i, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud)
{
	/*+++++++++++++++++++++++++单视角点云获取+++++++++++++++++++++++++++++++*/
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkSTLReader> readerQuery = vtkSmartPointer<vtkSTLReader>::New();
	//读取CAD模型
	readerQuery->SetFileName("guide.stl");
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
	vis.renderViewTesselatedSphere(resx, resy, views_xyz, poses, entropies, 0, 90, 1, TRUE);
	//for (int i = 0; i < views_xyz.size(); i++)
	{
	//	pcl::PointCloud<pcl::PointXYZ> views_cloud;
		pcl::transformPointCloud<pcl::PointXYZ>(views_xyz[i], *transformed_cloud, poses[i]);

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
		
}
