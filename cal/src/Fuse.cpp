#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>

#include <string>

using namespace std;

string file1, file2;
int scan_val;

int parseArgument(char*);
void viewOne (pcl::visualization::PCLVisualizer&);

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ> ("../dat/pcl/" + file1, *cloud1);
    reader.read<pcl::PointXYZ> ("../dat/pcl/" + file2, *cloud2);



    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud2);
    icp.setInputTarget(cloud1);
    icp.align(*cloud3);
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;


    // display clouds
    pcl::visualization::CloudViewer viewer ("Fuse Example");
	viewer.showCloud(cloud1, "Cloud A");
    viewer.showCloud(cloud2, "Cloud B");
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
        file1 = arg;
    if(file2.empty())
        file2 = arg;
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
	viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1, 0.2, 0.2, "Cloud A");
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 0.2, 0.2, 1, "Cloud B");
}