#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int K;
float T;
int scan_val;
std::string filename;

int parseArgument(char* arg)
{
	int option;
	float foption;
	char buf[1000];
	
	
  if(scan_val == 0)
  {
    if(1==sscanf(arg,"K=%d", &option))
    {
      K = option;
      return 0;
    }
    if(1==sscanf(arg,"T=%f", &foption))
    {
      T = foption;
      return 0;
    }
    if(1==sscanf(arg,"-f%s"), buf)
    {
      return 1;
    }
  }
  else if(scan_val == 1)
  {
    sscanf(arg, "%s",buf);
    filename = buf;
    return 0;
  }


	printf("could not parse argument \"%s\"!!!!\n", arg);
  return 0;
}


int main (int argc, char** argv)
{
  filename = "phone.pcd";
  K = 100;
  T = 0.01;
  scan_val = 0;
  int i = 1;
  while(i < argc)
  {
    scan_val = parseArgument(argv[i]);
    i++;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZRGB> ("../dat/pcl/" + filename, *cloud);

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
  writer.write<pcl::PointXYZRGB> ("../dat/filt/cloud_inliers.pcd", *cloud_filtered, false);


  sor.setNegative (true);
  sor.filter (*cloud_outliers);
  
  std::cerr << "Cloud outliers: " << std::endl;
  std::cerr << *cloud_outliers << std::endl;
  
  writer.write<pcl::PointXYZRGB> ("../dat/filt/cloud_outliers.pcd", *cloud_outliers, false);

exit(0);
}
