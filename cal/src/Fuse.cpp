#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
//#include <pcl/registration/ndt.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/sift_keypoint.h>

#include <cmath>
#include <string>

using namespace std;

string file1, file2;
int scan_val;
bool col_diff, keys;

int parseArgument(char*);
void viewOne (pcl::visualization::PCLVisualizer&);
void color_correct(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

int main(int argc, char** argv)
{
  col_diff = false;
  keys = false;
  scan_val = 0;
  int i = 1;
  while(i < argc)
  {
      scan_val = parseArgument(argv[i]);
      i++;
  }
  if(file1.empty())
      file1 = "table1_c.pcd";
  if(file2.empty())
      file2 = "table2_c.pcd";


  // load point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCDReader reader;
  reader.read<pcl::PointXYZRGB> ("../dat/filt/" + file1, *cloud1);
  reader.read<pcl::PointXYZRGB> ("../dat/filt/" + file2, *cloud2);


  //color_correct(cloud1);
  //color_correct(cloud2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr m3 (new pcl::PointCloud<pcl::PointXYZ>);

  
  // Iterative Closest Point
  pcl::PointCloud<pcl::PointXYZ>::Ptr m1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr m2 (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read<pcl::PointXYZ> ("../dat/match/" + file1, *m1);
  reader.read<pcl::PointXYZ> ("../dat/match/" + file2, *m2);

  // rescale clouds
  // desired scale
  float dscale = 1;
  float sc1 = 0;
  for(size_t i = 0;i < m1->points.size()-1;i++)
    for(size_t j = i+1;j < m1->points.size();j++)
      sc1 += std::sqrt(std::pow(m1->points[i].x - m1->points[j].x, 2) + std::pow(m1->points[i].y - m1->points[j].y, 2) + std::pow(m1->points[i].z - m1->points[j].z, 2));
  float sc2 = 0;
  for(size_t i = 0;i < m2->points.size()-1;i++)
    for(size_t j = i+1;j < m2->points.size();j++)
      sc2 += std::sqrt(std::pow(m2->points[i].x - m2->points[j].x, 2) + std::pow(m2->points[i].y - m2->points[j].y, 2) + std::pow(m2->points[i].z - m2->points[j].z, 2));
  Eigen::Affine3f rs1 = Eigen::Affine3f::Identity();
  rs1.scale(dscale / sc1);
  Eigen::Affine3f rs2 = Eigen::Affine3f::Identity();
  rs2.scale(dscale / sc2);

  pcl::transformPointCloud (*cloud1, *cloud1, rs1);
  pcl::transformPointCloud (*m1, *m1, rs1);
  pcl::transformPointCloud (*cloud2, *cloud2, rs2);
  pcl::transformPointCloud (*m2, *m2, rs2);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(m2);
  icp.setInputTarget(m1);
  icp.align(*m3);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  
  pcl::transformPointCloud (*cloud2, *cloud3, icp.getFinalTransformation());

  /*
  // Normal Dist Transform
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1.0);
  // Setting max number of registration iterations.
  ndt.setMaximumIterations (35);
  // Setting point cloud to be aligned.
  ndt.setInputSource (cloud2);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (cloud1);
  // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
  // Calculating required rigid transform to align the input cloud to the target cloud.
  ndt.align (*cloud3, init_guess);
  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
              << " score: " << ndt.getFitnessScore () << std::endl;
  */

  
  Eigen::Vector4f c1, c2;
  pcl::compute3DCentroid(*cloud1, c1);
  pcl::compute3DCentroid(*cloud2, c2);
  cout << "Cloud A Center: \n" << c1 << "\n\n";
  cout << "Cloud B Center: \n" << c2 << endl;
/*
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);

  const float min_scale = 00.1f;
  const int n_octaves = 6;
  const int n_scales_per_octave = 10;
  const float min_contrast = 0.00f;
  pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale> result;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
  sift.setSearchMethod(tree);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(cloud1);

  cout << "Run SIFT\n";

  sift.compute(result);

  cout << "SIFT calc complete\n";
  cout << result << endl;
  cout << "Copying to cloud\n";;

  copyPointCloud(result, *cloud3);
*/

  // display clouds
  pcl::visualization::CloudViewer viewer ("Fuse Example");
  viewer.showCloud(cloud1, "Cloud A");
  viewer.showCloud(cloud3, "Cloud B");
  if(keys)
  {
    viewer.showCloud(m1, "Keypoints A");
    viewer.showCloud(m3, "Keypoints B");
  }
	viewer.runOnVisualizationThreadOnce(viewOne);
	
	while (!viewer.wasStopped ())
	{ }
}





int parseArgument(char* arg)
{
	int option;
	float foption;
	char buf[1000];
	
	
  if(scan_val == 0)
  {
    if(1==sscanf(arg,"=%d", &option))
    {
        
      return 0;
    }
    if(1==sscanf(arg,"=%f", &foption))
    {
        
      return 0;
    }
    if(1==sscanf(arg,"-%s", buf))
    {
      if(strcmp(buf, "c") == 0 || strcmp(buf, "-colors") == 0)
      {
        col_diff = true;
        return 0;
      }
      if(strcmp(buf, "k") == 0 || strcmp(buf, "-keys") == 0)
      {
        keys = true;
        return 0;
      }
    }

    if(file1.empty())
    {
        file1 = arg;
        return 0;
    }
    if(file2.empty())
    {
        file2 = arg;
        return 0;
    }
  }/*
  else if(scan_val == 1)
  {
    sscanf(arg, "%s",buf);
    filename = buf;
    return 0;
  }*/


	printf("could not parse argument \"%s\"!!!!\n", arg);
  return 0;
}

void viewOne (pcl::visualization::PCLVisualizer& viewer)
{
  viewer.setCameraPosition(-0.118567, -0.0371279, -0.194652, -0.0633438, 0.00920603, 0.453505, 0.0395004, -0.860972, -0.507116);

  if(col_diff)
  {
	  viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1, 0.2, 0.2, "Cloud A");
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 0.2, 0.2, 1, "Cloud B");
  }
  if(keys)
  {
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1, 1, 0, "Keypoints A");
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 0, 1, 1, "Keypoints B");
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, "Keypoints A");
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, "Keypoints B");
  }
}

void color_correct(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    for(size_t i = 0;i < (*cloud).size();i++)
    {
        float temp = ((*cloud)[i].r + (*cloud)[i].g + (*cloud)[i].b) / 3.0;
        (*cloud)[i].r = temp;
        (*cloud)[i].g = temp;
        (*cloud)[i].b = temp;
    }
}