#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <string>


std::string head;

void viewOne (pcl::visualization::PCLVisualizer& viewer)
{
	//viewer.setBackgroundColor(1.0, 0.5, 1.0);
}

void parseArgument(char* arg)
{
	int option;
	float foption;
	char buf[1000];
	
	
	if(1==sscanf(arg,"file=%s",buf))
	{
		head = buf;
		return;
	}


	printf("could not parse argument \"%s\"!!!!\n", arg);
}


int main( int argc, char** argv )
{
	head = "vicon_1_2_v2.pcd";
	for(int i=1; i<argc;i++)
		parseArgument(argv[i]);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile (head, *cloud);
	
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud(cloud);
	
	viewer.runOnVisualizationThreadOnce(viewOne);
	
	while (!viewer.wasStopped ())
	{ }
	
	
	
	return 0;
}
