// general
#include <iostream>
#include <cmath>
#include <string>

// misc pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>

#include <pcl/filters/voxel_grid.h>

// fusion requirements
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/keypoints/sift_keypoint.h>
#include "pcl/features/normal_3d.h"
#include "pcl/features/fpfh.h"
#include <pcl/correspondence.h>
#include "pcl/kdtree/kdtree_flann.h"
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>



// constant values for testing
extern const int filt_type;
extern const int filt_N;
extern const float filt_R;
extern const float sift_ms;
extern const int sift_no;
extern const int sift_ns;
extern const int sift_mc;
extern float norm_r;
extern float feat_r;
extern float corr_v;
extern bool extr_down;
extern float extr_dres;


using namespace std;

// features being used
typedef pcl::PointCloud<pcl::FPFHSignature33> FeatureSpace;

string file1, file2;
int scan_val;
bool col_diff, keys, key_sphere, c_disp, show_filt;

int parseArgument(char*);
void viewOne (pcl::visualization::PCLVisualizer&);
void color_correct(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);


void key_detect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointWithScale>::Ptr &keys, FeatureSpace::Ptr &feats)
{
  string filt_name = "";
  // Preprocess filtering
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filt (new pcl::PointCloud<pcl::PointXYZRGB>);
  if(filt_type == 1)
  {
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(filt_R);
    outrem.setMinNeighborsInRadius (filt_N);
    // apply filter
    outrem.filter (*filt);

    filt_name = "Radius Filter";
  }
  else if(filt_type == 2)
  {
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (filt_N);
    sor.setStddevMulThresh (filt_R);
    sor.filter (*filt);

    filt_name = "Statistical Filter";
  }
  else
  {
    copyPointCloud(*cloud, *filt);
  }
  
  // SIFT detection
  pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale> result;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
  sift.setSearchMethod(tree);
  sift.setScales(sift_ms, sift_no, sift_ns);
  sift.setMinimumContrast(sift_mc);
  sift.setInputCloud(filt);
  sift.compute(*keys);

  // get normals
  pcl::PointCloud<pcl::Normal>::Ptr norms (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
  norm_est.setInputCloud (filt); // possibly use raw cloud
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  norm_est.setSearchMethod (tree2);
  norm_est.setRadiusSearch (norm_r);
  norm_est.compute(*norms);

  // get features
  pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree3 (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  fpfh_est.setSearchMethod (tree3);
  fpfh_est.setRadiusSearch (feat_r);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keys_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud (*keys, *keys_rgb);
  fpfh_est.setSearchSurface (filt); // possibly use raw cloud instead
  fpfh_est.setInputNormals (norms);  
  fpfh_est.setInputCloud (keys_rgb);
  fpfh_est.compute (*feats);


  if(c_disp)
  {
    cout << "Original Size: " << cloud->points.size() << "\n";
    if(!filt_name.empty())
      cout << filt_name << " Size: " << filt->points.size() << "\n";
    cout << "Keypoints: " << keys->points.size() << endl;
  }
  if(show_filt)
    copyPointCloud(*filt, *cloud);
}
void correlate_keypoints(pcl::PointCloud<pcl::PointWithScale>::Ptr &ms, pcl::PointCloud<pcl::PointWithScale>::Ptr &md, FeatureSpace::Ptr &fs, FeatureSpace::Ptr &fd)
{
  // correlation vector
  vector<int> corr;
  vector<float> cscore;
  // Resize
  corr.resize (fs->size ());
  cscore.resize (fs->size ());

  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
  pcl::KdTreeFLANN<pcl::FPFHSignature33> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (fd);

  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < fs->size (); ++i)
  {
    descriptor_kdtree.nearestKSearch (*fs, i, k, k_indices, k_squared_distances);
    corr[i] = k_indices[0];
    cscore[i] = k_squared_distances[0];
  }

  float sm = 0;
  for(size_t i = 0;i < cscore.size();i++)
    sm += cscore[i];
  sm /= corr_v * cscore.size();
  
  pcl::PointCloud<pcl::PointWithScale> mts;
  pcl::PointCloud<pcl::PointWithScale> mtd;
  size_t j = 0;
  for(size_t i = 0;i < cscore.size();i++)
    if(cscore[i] > sm)
    {
      mts.points[j] = ms->points[i];
      mtd.points[j] = md->points[corr[i]];
      j++;
    }
  
  copyPointCloud(mts, *ms);
  copyPointCloud(mtd, *md);
}
Eigen::Matrix<float, 4, 4> register_clouds(pcl::PointCloud<pcl::PointWithScale>::Ptr &ms, pcl::PointCloud<pcl::PointWithScale>::Ptr &md)
{
  // rescale clouds
  float scs = 0; // ms scale measurement
  for(size_t i = 0;i < ms->points.size()-1;i++)
    for(size_t j = i+1;j < ms->points.size();j++)
      scs += std::sqrt(std::pow(ms->points[i].x - ms->points[j].x, 2) + std::pow(ms->points[i].y - ms->points[j].y, 2) + std::pow(ms->points[i].z - ms->points[j].z, 2));
  float scd = 0; // md scale measurement
  for(size_t i = 0;i < md->points.size()-1;i++)
    for(size_t j = i+1;j < md->points.size();j++)
      scd += std::sqrt(std::pow(md->points[i].x - md->points[j].x, 2) + std::pow(md->points[i].y - md->points[j].y, 2) + std::pow(md->points[i].z - md->points[j].z, 2));
  // normalize
  Eigen::Affine3f rs = Eigen::Affine3f::Identity();
  rs.scale(scd / scs);

  // rescale source
  pcl::transformPointCloud (*ms, *ms, rs);

  pcl::PointCloud<pcl::PointWithScale>::Ptr mt (new pcl::PointCloud<pcl::PointWithScale>);
  pcl::IterativeClosestPoint<pcl::PointWithScale, pcl::PointWithScale> icp;
  icp.setInputSource(ms);
  icp.setInputTarget(md);
  icp.align(*mt);

  return scd * icp.getFinalTransformation() / scs;
}





int main(int argc, char** argv)
{
  show_filt = false;
  c_disp = false;
  col_diff = false;
  keys = true;
  scan_val = 0;
  int i = 1;
  while(i < argc)
  {
      scan_val = parseArgument(argv[i]);
      i++;
  }
  if(file1.empty())
      file1 = "table1.pcd";
  if(file2.empty())
      file2 = "table2.pcd";


  // load point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCDReader reader;
  reader.read<pcl::PointXYZRGB> ("../dat/filt/" + file1, *cloud1);
  reader.read<pcl::PointXYZRGB> ("../dat/filt/" + file2, *cloud2);


  // downsample for testing
  if(extr_down)
  {
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud2);
    sor.setLeafSize (extr_dres, extr_dres, extr_dres);
    sor.filter (*cloud2);
  }

  //color_correct(cloud1);
  //color_correct(cloud2);
/*
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
*/
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
  

  
  Eigen::Vector4f c1, c2;
  pcl::compute3DCentroid(*cloud1, c1);
  pcl::compute3DCentroid(*cloud2, c2);
  cout << "Cloud A Center: \n" << c1 << "\n\n";
  cout << "Cloud B Center: \n" << c2 << endl;
*/

  if(c_disp)
    cout << "Cloud 1\n";
  pcl::PointCloud<pcl::PointWithScale>::Ptr keys1 (new pcl::PointCloud<pcl::PointWithScale>);
  FeatureSpace::Ptr feats1 (new FeatureSpace);
  key_detect(cloud1, keys1, feats1);

  if(c_disp)
    cout << "\nCloud 2\n";
  pcl::PointCloud<pcl::PointWithScale>::Ptr keys2 (new pcl::PointCloud<pcl::PointWithScale>);
  FeatureSpace::Ptr feats2 (new FeatureSpace);
  key_detect(cloud2, keys2, feats2);

  if(c_disp)
    cout << "\nCorrelate Clouds\n";
  correlate_keypoints(keys2, keys1, feats2, feats1);

  if(c_disp)
    cout << "\nCompute Transform\n";
  Eigen::Matrix<float, 4, 4> s2d;
  s2d = register_clouds(keys2, keys1);

  pcl::transformPointCloud (*cloud2, *cloud2, s2d);
  pcl::transformPointCloud (*keys2, *keys2, s2d);

  // display clouds
  pcl::visualization::CloudViewer viewer ("Fuse Example");
  viewer.showCloud(cloud1, "Cloud A");
  viewer.showCloud(cloud2, "Cloud B");
  if(keys)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr dk1 (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(*keys1, *dk1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr dk2 (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(*keys2, *dk2);
    
    viewer.showCloud(dk1, "Keypoints A");
    viewer.showCloud(dk2, "Keypoints B");
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
        col_diff = !col_diff;
        return 0;
      }
      if(strcmp(buf, "k") == 0 || strcmp(buf, "-keys") == 0)
      {
        keys = !keys;
        return 0;
      }
      if(strcmp(buf, "s") == 0 || strcmp(buf, "-sphere") == 0)
      {
        key_sphere = !key_sphere;
        return 0;
      }
      if(strcmp(buf, "d") == 0 || strcmp(buf, "-console") == 0)
      {
        c_disp = !c_disp;
        return 0;
      }
      if(strcmp(buf, "f") == 0 || strcmp(buf, "-filt") == 0)
      {
        show_filt = !show_filt;
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
/*
  if(key_sphere)
  {
    // Draw each keypoint as a sphere
    for (size_t i = 0; i < keys1->size (); ++i)
    {
      // Get the point data
      const pcl::PointWithScale & p = keys1->points[i];

      // Pick the radius of the sphere *
      float r = 0.1 * p.scale;
      // * Note: the scale is given as the standard deviation of a Gaussian blur, so a
      //   radius of 2*p.scale is a good illustration of the extent of the keypoint

      // Generate a unique string for each sphere
      std::stringstream ss ("keypoint");
      ss << i;

      // Add a sphere at the keypoint
      viewer.addSphere (p, r, 0.0, 1.0, 1.0, ss.str ());
    }
  }*/

  if(col_diff)
  {
	  viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1, 0.2, 0.2, "Cloud A");
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 0.2, 0.2, 1, "Cloud B");
  }
  if(keys)
  {
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1, 1, 0, "Keypoints A");
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 0, 1, 1, "Keypoints B");
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 10, "Keypoints A");
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 10, "Keypoints B");
  }

  //viewer.saveScreenshot("~/data/results/img1.png");
  //viewer.saveScreenshot("../dat/results/img1.png");
}
