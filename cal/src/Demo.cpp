

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <cmath>
#include <string>



float sz = 2;
int anim_frames;
bool key_show, col_diff, anim;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr m1 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr m2 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1d (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2d (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr m1d (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr m2d (new pcl::PointCloud<pcl::PointXYZ>);
pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
// --------------
// -----Help-----
// --------------
void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-d           Show each point cloud as a different color\n"
            << "-k           Display keypoints\n"
            << "\n\n";
}


void toggle_color()
{
    col_diff = !col_diff;

    if(col_diff)
    {
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.2, 0.2, "Cloud A");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.2, 0.2, 1, "Cloud B");

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sz, "Cloud A");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sz, "Cloud B");
    }
    else
    {
        viewer->removePointCloud("Cloud A");
        viewer->removePointCloud("Cloud B");
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud1d, "Cloud A");
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud2d, "Cloud B");

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sz, "Cloud A");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sz, "Cloud B");
    }
}
void toggle_keypoints()
{
    key_show = !key_show;

    if(key_show)
    {
        viewer->addPointCloud<pcl::PointXYZ> (m1d, "Keypoints A");
        viewer->addPointCloud<pcl::PointXYZ> (m2d, "Keypoints B");

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "Keypoints A");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 1, "Keypoints B");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Keypoints A");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Keypoints B");
    }
    else
    {
        viewer->removePointCloud("Keypoints A");
        viewer->removePointCloud("Keypoints B");
    }
}


std::string num2str(int val, int length)
{
    std::string str = "";
    while(val > 0)
    {
        str = (char) ((val % 10) + 48) + str;
        val /= 10;
    }
    while(str.length() < length)
        str = "0" + str;

    return str;
}

unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    if (event.getKeySym () == "k" && event.keyDown ())
    {
        toggle_keypoints();
    }
    if (event.getKeySym () == "d" && event.keyDown ())
    {
        toggle_color();
    }
    if (event.getKeySym () == "a" && event.keyDown ())
    {
        //std::cout << "\"a\" was pressed\n";
        anim = true;
    }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}


void save_frame(int ct)
{
    viewer->saveScreenshot("/home/steve/repos/SLAM_IT/cal/dat/results/img" + num2str(ct, 5) + ".png");
}



// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
  key_show = false;
  col_diff = false;
  anim = false;
  anim_frames = 50;
  // get filename
  std::string file1, file2;
  file1 = argv[1];
  file2 = argv[2];

  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  int temp = -1;
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }

  if (pcl::console::find_argument (argc, argv, "-d") >= 0)
  {
    col_diff = true;
    std::cout << "Color Difference Enabled\n";
  }
  if (pcl::console::find_argument (argc, argv, "-k") >= 0)
  {
    key_show = true;
    std::cout << "Showing Keypoints\n";
  }
  if ((temp = pcl::console::find_argument (argc, argv, "-f")) >= 0)
  {
    sscanf(argv[temp + 1],"%d", &anim_frames);
    std::cout << "Frames set to " << anim_frames << "\n";
  }

  pcl::io::loadPCDFile ("../dat/filt/" + file1, *cloud1);
  pcl::io::loadPCDFile ("../dat/filt/" + file2, *cloud2);
  pcl::io::loadPCDFile ("../dat/match/" + file1, *m1);
  pcl::io::loadPCDFile ("../dat/match/" + file2, *m2);


  // transform to world coordinates
  Eigen::Matrix4f c2w = Eigen::Matrix4f::Identity();
  c2w(0,0) = 0.997089772981969;c2w(0,1) = -0.0751854130436809;c2w(0,2) = 0.0126150021885371;c2w(0,3) = 0.0612435649249535;
  c2w(1,0) = -0.0519053087165223;c2w(1,1) = -0.548310981519826;c2w(1,2) = 0.834662151095764;c2w(1,3) = -0.815416296113814;
  c2w(2,0) = -0.0558374743501907;c2w(2,1) = -0.832887880335773;c2w(2,2) = -0.550617794162135;c2w(2,3) = 0.802145285959664;
  pcl::transformPointCloud (*cloud1, *cloud1, c2w);
  pcl::transformPointCloud (*m1, *m1, c2w);
  pcl::transformPointCloud (*cloud2, *cloud2, c2w);
  pcl::transformPointCloud (*m2, *m2, c2w);



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
  icp.align(*m2d);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  
  pcl::transformPointCloud (*cloud2, *cloud2d, icp.getFinalTransformation());
  *cloud1d = *cloud1;
  *m1d = *m1;






  viewer->setBackgroundColor (1, 1, 1);
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());

  viewer->addPointCloud<pcl::PointXYZRGB> (cloud1d, "Cloud A");
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud2d, "Cloud B");
  if(key_show)
  {
    key_show = false;
    toggle_keypoints();
  }
  if(col_diff)
  {
    col_diff = false;
    toggle_color();
  }
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sz, "Cloud A");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sz, "Cloud B");

  viewer->initCameraParameters ();
  //viewer->setCameraPosition(0, -0.75, 0, 0, 0, 0.15, 0, 0, 1);
  viewer->setCameraPosition(-0.198897, -0.364415, 0.321396, 0, 0, 0.15, 0.197776, 0.326731, 0.924192);
  viewer->setSize(1024, 512);

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce(100);
    if(anim)
    {
        int ct = 0;
        std::cout << "Start animation\n";
        key_show = true;
        col_diff = true;
        toggle_color();
        toggle_keypoints();
        for(int i = 0;i < anim_frames;i++)
        {
            Eigen::Affine3f at = Eigen::Affine3f::Identity();
            at.rotate (Eigen::AngleAxisf (2*M_PI * i / (anim_frames - 1), Eigen::Vector3f::UnitZ()));
            pcl::transformPointCloud (*cloud1, *cloud1d, at);
            pcl::transformPointCloud (*cloud2, *cloud2d, at);
            viewer->updatePointCloud(cloud1d, "Cloud A");
            viewer->updatePointCloud(cloud2d, "Cloud B");
            if(key_show)
            {
                pcl::transformPointCloud (*m1, *m1d, at);
                pcl::transformPointCloud (*m2, *m2d, at);
                viewer->updatePointCloud(m1d, "Keypoints A");
                viewer->updatePointCloud(m2d, "Keypoints B");
            }
            viewer->spinOnce();

            save_frame(ct);
            ct++;

            boost::this_thread::sleep (boost::posix_time::microseconds (50000));
        }
        toggle_color();
        toggle_keypoints();
        for(int i = 0;i < anim_frames;i++)
        {
            Eigen::Affine3f at = Eigen::Affine3f::Identity();
            at.rotate (Eigen::AngleAxisf (2*M_PI * i / (anim_frames - 1), Eigen::Vector3f::UnitZ()));
            pcl::transformPointCloud (*cloud1, *cloud1d, at);
            pcl::transformPointCloud (*cloud2, *cloud2d, at);
            viewer->updatePointCloud(cloud1d, "Cloud A");
            viewer->updatePointCloud(cloud2d, "Cloud B");
            if(key_show)
            {
                pcl::transformPointCloud (*m1, *m1d, at);
                pcl::transformPointCloud (*m2, *m2d, at);
                viewer->updatePointCloud(m1d, "Keypoints A");
                viewer->updatePointCloud(m2d, "Keypoints B");
            }
            viewer->spinOnce();

            save_frame(ct);
            ct++;

            boost::this_thread::sleep (boost::posix_time::microseconds (50000));
        }
        for(int i = 0;i < anim_frames;i++)
        {
            float val = 1.0f * i / (anim_frames - 1);
            Eigen::Matrix4f at = Eigen::Matrix4f::Identity() * (1 - val) + icp.getFinalTransformation() * val;
            pcl::transformPointCloud (*cloud2, *cloud2d, at);
            viewer->updatePointCloud(cloud2d, "Cloud B");
            if(key_show)
            {
                pcl::transformPointCloud (*m2, *m2d, at);
                viewer->updatePointCloud(m2d, "Keypoints B");
            }
            viewer->spinOnce();

            save_frame(ct);
            ct++;

            boost::this_thread::sleep (boost::posix_time::microseconds (50000));
        }
        Eigen::Affine3f at = Eigen::Affine3f::Identity();
        at.rotate (Eigen::AngleAxisf (2*M_PI / (anim_frames - 1), Eigen::Vector3f::UnitZ()));
        for(int i = 0;i < anim_frames;i++)
        {
            pcl::transformPointCloud (*cloud1d, *cloud1d, at);
            pcl::transformPointCloud (*cloud2d, *cloud2d, at);
            viewer->updatePointCloud(cloud1d, "Cloud A");
            viewer->updatePointCloud(cloud2d, "Cloud B");
            if(key_show)
            {
                pcl::transformPointCloud (*m1d, *m1d, at);
                pcl::transformPointCloud (*m2d, *m2d, at);
                viewer->updatePointCloud(m1d, "Keypoints A");
                viewer->updatePointCloud(m2d, "Keypoints B");
            }
            viewer->spinOnce();

            save_frame(ct);
            ct++;

            boost::this_thread::sleep (boost::posix_time::microseconds (50000));
        }
        toggle_color();
        toggle_keypoints();
        at = Eigen::Affine3f::Identity();
        at.rotate (Eigen::AngleAxisf (2*M_PI / (anim_frames - 1), Eigen::Vector3f::UnitZ()));
        for(int i = 0;i < anim_frames;i++)
        {
            pcl::transformPointCloud (*cloud1d, *cloud1d, at);
            pcl::transformPointCloud (*cloud2d, *cloud2d, at);
            viewer->updatePointCloud(cloud1d, "Cloud A");
            viewer->updatePointCloud(cloud2d, "Cloud B");
            if(key_show)
            {
                pcl::transformPointCloud (*m1d, *m1d, at);
                pcl::transformPointCloud (*m2d, *m2d, at);
                viewer->updatePointCloud(m1d, "Keypoints A");
                viewer->updatePointCloud(m2d, "Keypoints B");
            }
            viewer->spinOnce();

            save_frame(ct);
            ct++;

            boost::this_thread::sleep (boost::posix_time::microseconds (50000));
        }

        anim = false;
    }
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
