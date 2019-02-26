#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

int K;
float T;
int scan_val;
std::string filename;
bool rad_rem;

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
    if(1==sscanf(arg,"-%s", buf))
    {
      if(strcmp(buf, "f") == 0)
        return 1;
      else if(strcmp(buf, "r") == 0)
      {
        rad_rem = true;
        return 0;
      }
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
  // defaults
  filename = "phone.pcd";
  rad_rem = false;
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
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile ("../dat/pcl/" + filename, *cloud);

  std::cout << "Cloud before filtering: " << std::endl;
  std::cout << *cloud << std::endl;

  if(rad_rem)
  {
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(T);
    outrem.setMinNeighborsInRadius (K);
    // apply filter
    outrem.filter (*cloud_filtered);
  }
  else
  {
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (K);
    sor.setStddevMulThresh (T);
    sor.filter (*cloud_filtered);
  }

  std::cout << "Cloud after filtering: " << std::endl;
  std::cout << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("../dat/filt/" + filename, *cloud_filtered, false);

/*
  sor.setNegative (true);
  sor.filter (*cloud_outliers);
  
  std::cerr << "Cloud outliers: " << std::endl;
  std::cerr << *cloud_outliers << std::endl;
  */
  //writer.write<pcl::PointXYZRGB> ("../dat/filt/cloud_outliers.pcd", *cloud_outliers, false);

  return 0;
  //exit(0);
}
