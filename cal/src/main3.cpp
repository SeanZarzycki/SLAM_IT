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
  bool three_en = false;
  color_div = false;
  // get filename
  std::string filename, filename2, filename3;
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
  temp = pcl::console::find_argument (argc, argv, "-a");
  if(temp >= 0 && temp < argc)
  {
    two_en = false;
    three_en = true;
    filename = "Run5.pcd";
    filename2 = "MarkRun.pcd";
    filename3 = "run3.pcd";
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGB>);
  if(two_en)
  {
    Eigen::Matrix4f s2d = Eigen::Matrix4f::Identity();
    /*
    s2d <<  -0.00508278,     1.27822 ,  0.0106919 ,  0.0634904,
            -1.23778, -0.00759102 ,   0.319097 ,  -0.314879,
            0.319147, -0.00908433 ,    1.23776 , -0.0596386,
            0,           0,           0     ,      1;
    */
    s2d <<   0.740968 ,  0.108541 ,  0.017397 ,-0.0610801,
 -0.106948  , 0.739215, -0.0569192 , 0.0375726,
-0.0254156 , 0.0538191 ,  0.746709 ,0.00188237,
         0   ,       0      ,    0   ,       1;
    /*
    s2d << 0, 1, 0, 0,
          -1, 0, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;*/


    pcl::io::loadPCDFile ("../dat/fuse_dataset/" + filename2, *cloud2);
    pcl::transformPointCloud(*cloud2, *cloud2, s2d);
  }
  else if(three_en)
  {
    Eigen::Matrix4f s2d1, s2d2;
    s2d1 <<  0.0351418 ,    1.18898  , 0.0384377 ,  0.0401717,
            -1.14412  , 0.0232494   , 0.326854  , -0.319899,
              0.32579 , -0.0466033  ,   1.14371 ,0.000299394,
                    0  ,         0   ,        0  ,         1;
    s2d2 <<   0.740968 ,  0.108541 ,  0.017397 ,-0.0610801,
 -0.106948  , 0.739215, -0.0569192 , 0.0375726,
-0.0254156 , 0.0538191 ,  0.746709 ,0.00188237,
         0   ,       0      ,    0   ,       1;


    pcl::io::loadPCDFile ("../dat/fuse_dataset/" + filename2, *cloud2);
    pcl::transformPointCloud(*cloud2, *cloud2, s2d1);
    pcl::io::loadPCDFile ("../dat/fuse_dataset/" + filename3, *cloud3);
    pcl::transformPointCloud(*cloud3, *cloud3, s2d2);
  }


  pcl::visualization::PCLVisualizer::Ptr viewer;
  
  viewer = rgbVis(cloud1);
  if(two_en || three_en)
  {
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, "Cloud 2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud 2");
    if(color_div)
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "Cloud 2");
  }
  if(three_en)
  {
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud3, "Cloud 3");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud 3");
    if(color_div)
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "Cloud 3");
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
