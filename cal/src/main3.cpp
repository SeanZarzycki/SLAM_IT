/* \author Geoffrey Biggs */


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>


bool color_div;

// --------------
// -----Help-----
// --------------
void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}




pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "Cloud 1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud 1");
  if(color_div)
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Cloud 1");
  viewer->addCoordinateSystem (0.2);
  viewer->initCameraParameters ();
  viewer->setCameraPosition(0, 0, -2, 0, 0, 1, 0, 1, 0);
  return (viewer);
}


// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
  bool two_en = false;
  color_div = false;
  // get filename
  std::string filename, filename2;
  int temp = pcl::console::find_argument (argc, argv, "-f");
  if(temp >= 0 && temp < argc)
    filename = argv[temp+1];
  else
    filename = "cloud_inliers.pcd";

  temp = pcl::console::find_argument (argc, argv, "-t");
  if(temp >= 0 && temp < argc)
  {
    two_en = true;
    filename2 = argv[temp+1];
  }
  temp = pcl::console::find_argument (argc, argv, "-c");
  if(temp > 0)
    color_div = !color_div;
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  

    std::cout << "RGB colour visualisation example\n";


  // ------------------------------------
  // -----Create example point cloud-----
  // ------------------------------------
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile ("../dat/fuse_dataset/" + filename, *cloud1);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  if(two_en)
  {
    Eigen::Matrix4f s2d;
    s2d <<  -0.00508278,     1.27822 ,  0.0106919 ,  0.0634904,
            -1.23778, -0.00759102 ,   0.319097 ,  -0.314879,
            0.319147, -0.00908433 ,    1.23776 , -0.0596386,
            0,           0,           0     ,      1;
    /*s2d << 0, 1, 0, 0,
          -1, 0, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;*/


    pcl::io::loadPCDFile ("../dat/fuse_dataset/" + filename2, *cloud2);
    pcl::transformPointCloud(*cloud2, *cloud2, s2d);
  }


  pcl::visualization::PCLVisualizer::Ptr viewer;
  
  viewer = rgbVis(cloud1);
  if(two_en)
  {
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, "Cloud 2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud 2");
    if(color_div)
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "Cloud 2");
  }

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
