#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/ndt.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>

#include <pcl/common/centroid.h>
#include <pcl/keypoints/sift_keypoint.h>

#include <string>

using namespace std;

string file1, file2;
int scan_val;

int parseArgument(char*);
void viewOne (pcl::visualization::PCLVisualizer&);
void color_correct(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

int main(int argc, char** argv)
{
    scan_val = 0;
    int i = 1;
    while(i < argc)
    {
        scan_val = parseArgument(argv[i]);
        i++;
    }
    if(file1.empty())
        file1 = "phone.pcd";
    if(file2.empty())
        file2 = "phone_r.pcd";


    // load point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGB> ("../dat/filt/" + file1, *cloud1);
    reader.read<pcl::PointXYZRGB> ("../dat/filt/" + file2, *cloud2);


    //color_correct(cloud1);
    //color_correct(cloud2);


    /*
    // Iterative Closest Point
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud2);
    icp.setInputTarget(cloud1);
    icp.align(*cloud3);
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
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
    viewer.showCloud(cloud2, "Cloud B");
    //viewer.showCloud(cloud3, "Keypoints");
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
    if(1==sscanf(arg,"-f%s", buf))
    {
      return 1;
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
	//viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1, 0.2, 0.2, "Cloud A");
    //viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 0.2, 0.2, 1, "Cloud B");
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1, 0, 0, "Keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 5, "Keypoints");
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