#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int K;
float T;

void parseArgument(char* arg)
{
	int option;
	float foption;
	char buf[1000];
	
	
	if(1==sscanf(arg,"K=%d", &option))
	{
		K = option;
		return;
	}
	if(1==sscanf(arg,"T=%f", &foption))
	{
		T = foption;
		return;
	}


	printf("could not parse argument \"%s\"!!!!\n", arg);
}


int main (int argc, char** argv)
{
  K = 1000;
  T = 0.01;
  for(int i=1; i<argc;i++)
	parseArgument(argv[i]);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZRGB> ("table2.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (K);
  sor.setStddevMulThresh (T);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("table2_inliers.pcd", *cloud_filtered, false);


  sor.setNegative (true);
  sor.filter (*cloud_outliers);
  
  std::cerr << "Cloud outliers: " << std::endl;
  std::cerr << *cloud_outliers << std::endl;
  
  writer.write<pcl::PointXYZRGB> ("table2_outliers.pcd", *cloud_outliers, false);

exit(0);
}
