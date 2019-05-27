

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



float sz = 1;
float lsz = 2;
int anim_frames;
bool key_show, mat_show, col_diff, anim, save_on;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr k1 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr k2 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr k3 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1d (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2d (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3d (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr k1d (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr k2d (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr k3d (new pcl::PointCloud<pcl::PointXYZ>);
pcl::CorrespondencesPtr corr2to1 (new pcl::Correspondences);
pcl::CorrespondencesPtr corr3to1 (new pcl::Correspondences);
pcl::CorrespondencesPtr inlie2to1 (new pcl::Correspondences);
pcl::CorrespondencesPtr inlie3to1 (new pcl::Correspondences);
pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));




void toggle_color()
{
    col_diff = !col_diff;

    if(col_diff)
    {
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.2, 0.2, "Cloud A");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.4, 0.4, 1, "Cloud B");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.2, 1, 0.2, "Cloud C");

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sz, "Cloud A");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sz, "Cloud B");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sz, "Cloud C");
    }
    else
    {
        viewer->removePointCloud("Cloud A");
        viewer->removePointCloud("Cloud B");
        viewer->removePointCloud("Cloud C");
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud1d, "Cloud A");
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud2d, "Cloud B");
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud3d, "Cloud C");

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sz, "Cloud A");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sz, "Cloud B");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sz, "Cloud C");
    }
}
void toggle_keypoints()
{
  key_show = !key_show;

  if(key_show)
  {
      viewer->addPointCloud<pcl::PointXYZ> (k1d, "Keypoints A");
      viewer->addPointCloud<pcl::PointXYZ> (k2d, "Keypoints B");
      viewer->addPointCloud<pcl::PointXYZ> (k3d, "Keypoints C");

      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "Keypoints A");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 1, "Keypoints B");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "Keypoints C");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Keypoints A");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Keypoints B");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Keypoints C");
  }
  else
  {
      viewer->removePointCloud("Keypoints A");
      viewer->removePointCloud("Keypoints B");
      viewer->removePointCloud("Keypoints C");
  }
}
void toggle_matches()
{
  mat_show = !mat_show;

  if(mat_show)
  {
    for (size_t i = 0; i < inlie2to1->size (); ++i)
    {
      // Generate a unique string for each line
      std::stringstream ss;
      ss << "line2to1_" << i;

      // Draw the line
      std::string tmp = ss.str();
      viewer->addLine (k1d->at(inlie2to1->at(i).index_query), k2d->at(inlie2to1->at(i).index_match), 0.7, 1, 0.7, tmp);
      viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lsz, ss.str());
    }
    for (size_t i = 0; i < inlie3to1->size (); ++i)
    {
      // Generate a unique string for each line
      std::stringstream ss;
      ss << "line3to1_" << i;

      // Draw the line
      std::string tmp = ss.str();
      viewer->addLine (k1d->at(inlie3to1->at(i).index_query), k3d->at(inlie3to1->at(i).index_match), 1, 0.7, 0.7, tmp);
      viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lsz, tmp);
    }
  }
  else
  {
    viewer->removeAllShapes();
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
    while(str.length() < static_cast<size_t>(length))
        str = "0" + str;

    return str;
}

unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    //pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    if (event.getKeySym () == "k" && event.keyDown ())
    {
        toggle_keypoints();
    }
    if (event.getKeySym () == "l" && event.keyDown ())
    {
        toggle_matches();
    }
    if (event.getKeySym () == "c" && event.keyDown ())
    {
        toggle_color();
    }
    if (event.getKeySym () == "a" && event.keyDown ())
    {
        //std::cout << "\"a\" was pressed\n";
        anim = true;
    }
}



void save_frame(int mode, int ct)
{
  if(save_on)
  {
    switch(mode)
    {
      case 1:
        viewer->saveScreenshot("/home/steve/repos/SLAM_IT/cal/dat/results/stage1/img" + num2str(ct, 5) + ".png");
        break;
      case 2:
        viewer->saveScreenshot("/home/steve/repos/SLAM_IT/cal/dat/results/stage2/img" + num2str(ct, 5) + ".png");
        break;
      case 3:
        viewer->saveScreenshot("/home/steve/repos/SLAM_IT/cal/dat/results/stage3/img" + num2str(ct, 5) + ".png");
        break;
      case 4:
        viewer->saveScreenshot("/home/steve/repos/SLAM_IT/cal/dat/results/stage4/img" + num2str(ct, 5) + ".png");
        break;
      case 5:
        viewer->saveScreenshot("/home/steve/repos/SLAM_IT/cal/dat/results/stage5/img" + num2str(ct, 5) + ".png");
        break;
      case 6:
        viewer->saveScreenshot("/home/steve/repos/SLAM_IT/cal/dat/results/stage6/img" + num2str(ct, 5) + ".png");
        break;
      case 7:
        viewer->saveScreenshot("/home/steve/repos/SLAM_IT/cal/dat/results/stage7/img" + num2str(ct, 5) + ".png");
        break;
    }
  }
}



// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
  save_on = false;
  key_show = false;
  col_diff = false;
  anim = false;
  anim_frames = 50;
  // get filename
  std::string file1, file2, file3;
  file1 = "Run5.pcd";
  file2 = "MarkRun.pcd";
  file3 = "run3.pcd";

  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  int temp = -1;
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    //printUsage (argv[0]);
    return 0;
  }

  if (pcl::console::find_argument (argc, argv, "-c") >= 0)
  {
    col_diff = true;
    std::cout << "Color Difference Enabled\n";
  }
  if (pcl::console::find_argument (argc, argv, "-s") >= 0)
  {
    save_on = true;
    std::cout << "Saving animation frames\n";
  }
  if (pcl::console::find_argument (argc, argv, "-k") >= 0)
  {
    key_show = true;
    std::cout << "Showing Keypoints\n";
  }
  if (pcl::console::find_argument (argc, argv, "-k") >= 0)
  {
    mat_show = true;
    std::cout << "Showing Matched Keypoints\n";
  }
  if ((temp = pcl::console::find_argument (argc, argv, "-f")) >= 0)
  {
    sscanf(argv[temp + 1],"%d", &anim_frames);
    std::cout << "Frames set to " << anim_frames << "\n";
  }

  pcl::io::loadPCDFile ("../dat/fuse_dataset/" + file1, *cloud1);
  pcl::io::loadPCDFile ("../dat/fuse_dataset/" + file2, *cloud2);
  pcl::io::loadPCDFile ("../dat/fuse_dataset/" + file3, *cloud3);


  // transform to world coordinates
  int count;
  std::string line;
  Eigen::Matrix4f b2a = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f c2a = Eigen::Matrix4f::Identity();
  std::ifstream fl;
  fl.open("../dat/match/" + file2.substr(0, file2.length() - 4) + "To" + file1.substr(0, file1.length() - 4) + ".txt");
  // transformation
  for(int i = 0;i < 4;i++)
  {
    getline(fl, line);
    float a, b, c, d;
    sscanf(line.c_str(), "%f %f %f %f", &a, &b, &c, &d);
    b2a(i, 0) = a;
    b2a(i, 1) = b;
    b2a(i, 2) = c;
    b2a(i, 3) = d;
  }
  // keypoints file1
  getline(fl, line);
  sscanf(line.c_str(), "%*s %d", &count);
  k1->resize(count);
  for(int i = 0;i < count;i++)
  {
    float x, y, z;
    getline(fl, line);
    sscanf(line.c_str(), "%f %f %f", &x, &y, &z);
    k1->points[i].x = x;
    k1->points[i].y = y;
    k1->points[i].z = z;
  }
  // keypoints file2
  getline(fl, line);
  sscanf(line.c_str(), "%*s %d", &count);
  k2->resize(count);
  for(int i = 0;i < count;i++)
  {
    float x, y, z;
    getline(fl, line);
    sscanf(line.c_str(), "%f %f %f", &x, &y, &z);
    k2->points[i].x = x;
    k2->points[i].y = y;
    k2->points[i].z = z;
  }
  // Initial Correspondences 2 to 1
  getline(fl, line);
  sscanf(line.c_str(), "%*s %d", &count);
  for(int i = 0;i < count;i++)
  {
    int a, b;
    getline(fl, line);
    sscanf(line.c_str(), "%d %d", &a, &b);
    pcl::Correspondence tmp;
    tmp.index_query = a;
    tmp.index_match = b;
    tmp.distance = 0;
    corr2to1->push_back(tmp);
  }
  // Inlier Correspondences 2 to 1
  getline(fl, line);
  sscanf(line.c_str(), "%*s %d", &count);
  for(int i = 0;i < count;i++)
  {
    int a, b;
    getline(fl, line);
    sscanf(line.c_str(), "%d %d", &a, &b);
    pcl::Correspondence tmp;
    tmp.index_query = a;
    tmp.index_match = b;
    tmp.distance = 0;
    inlie2to1->push_back(tmp);
  }
  fl.close();
  fl.open("../dat/match/" + file3.substr(0, file3.length() - 4) + "To" + file1.substr(0, file1.length() - 4) + ".txt");
  // transform
  for(int i = 0;i < 4;i++)
  {
    getline(fl, line);
    float a, b, c, d;
    sscanf(line.c_str(), "%f %f %f %f", &a, &b, &c, &d);
    c2a(i, 0) = a;
    c2a(i, 1) = b;
    c2a(i, 2) = c;
    c2a(i, 3) = d;
  }
  // keypoints file1
  getline(fl, line);
  sscanf(line.c_str(), "%*s %d", &count);
  for(int i = 0;i < count;i++)
    getline(fl, line);
  // keypoints file3
  getline(fl, line);
  sscanf(line.c_str(), "%*s %d", &count);
  k3->resize(count);
  for(int i = 0;i < count;i++)
  {
    float x, y, z;
    getline(fl, line);
    sscanf(line.c_str(), "%f %f %f", &x, &y, &z);
    k3->points[i].x = x;
    k3->points[i].y = y;
    k3->points[i].z = z;
  }
  // Initial Correspondences 3 to 1
  getline(fl, line);
  sscanf(line.c_str(), "%*s %d", &count);
  for(int i = 0;i < count;i++)
  {
    int a, b;
    getline(fl, line);
    sscanf(line.c_str(), "%d %d", &a, &b);
    pcl::Correspondence tmp;
    tmp.index_query = a;
    tmp.index_match = b;
    tmp.distance = 0;
    corr3to1->push_back(tmp);
  }
  // Inlier Correspondences 3 to 1
  getline(fl, line);
  sscanf(line.c_str(), "%*s %d", &count);
  for(int i = 0;i < count;i++)
  {
    int a, b;
    getline(fl, line);
    sscanf(line.c_str(), "%d %d", &a, &b);
    pcl::Correspondence tmp;
    tmp.index_query = a;
    tmp.index_match = b;
    tmp.distance = 0;
    inlie3to1->push_back(tmp);
  }
  fl.close();

  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp2 (new pcl::PointCloud<pcl::PointXYZ>);
  tmp1->resize(8);
  tmp1->points[0].x = 0; tmp1->points[0].y = 0; tmp1->points[0].z = 0;
  tmp1->points[1].x = 5; tmp1->points[1].y = 0; tmp1->points[1].z = 0;
  tmp1->points[2].x = 5; tmp1->points[2].y = 0; tmp1->points[2].z = 7;
  tmp1->points[3].x = 0; tmp1->points[3].y = 0; tmp1->points[3].z = 7;
  tmp1->points[4].x = 0; tmp1->points[4].y = 9; tmp1->points[4].z = 7;
  tmp1->points[5].x = 5; tmp1->points[5].y = 9; tmp1->points[5].z = 7;
  tmp1->points[6].x = 5; tmp1->points[6].y = 9; tmp1->points[6].z = 0;
  tmp1->points[7].x = 0; tmp1->points[7].y = 9; tmp1->points[7].z = 0;
  Eigen::Matrix4f tmpT; // convert from zyrtec coordinates to table coordinates
  tmpT << 1, 0, 0, (33.5/2-6.0),
          0, 1, 0, (21.0-33.5/2),
          0, 0, 1, (27.0+1.0/8),
          0, 0, 0, 1;
  pcl::transformPointCloud (*tmp1, *tmp1, tmpT);
  tmp2->resize(8);
  tmp2->points[0].x = -0.06611; tmp2->points[0].y = 0.9553; tmp2->points[0].z = 0.1019;
  tmp2->points[1].x = -0.15280; tmp2->points[1].y = 0.9400; tmp2->points[1].z = 0.1025;
  tmp2->points[2].x = -0.14040; tmp2->points[2].y = 0.8661; tmp2->points[2].z = 0.1932;
  tmp2->points[3].x = -0.05004; tmp2->points[3].y = 0.8946; tmp2->points[3].z = 0.2033;
  tmp2->points[4].x = -0.07075; tmp2->points[4].y = 1.0090; tmp2->points[4].z = 0.2885;
  tmp2->points[5].x = -0.15850; tmp2->points[5].y = 0.9863; tmp2->points[5].z = 0.2869;
  tmp2->points[6].x = -0.17000; tmp2->points[6].y = 1.0750; tmp2->points[6].z = 0.2035;
  tmp2->points[7].x = -0.08266; tmp2->points[7].y = 1.0670; tmp2->points[7].z = 0.1947;
  Eigen::Matrix4f tmpT2; // convert from zyrtec coordinates to table coordinates
  tmpT2 << 1, 0, 0, 0,
           0, 0, 1, 0,
           0, 1, 0, 0,
           0, 0, 0, 1;
  pcl::transformPointCloud (*tmp2, *tmp2, tmpT2);
  float sc1 = 0; // true scale measurement
  for(size_t i = 0;i < tmp1->size()-1;i++)
    for(size_t j = i+1;j < tmp1->size();j++)
      sc1 += std::sqrt(std::pow(tmp1->points[i].x - tmp1->points[j].x, 2) + std::pow(tmp1->points[i].y - tmp1->points[j].y, 2) + std::pow(tmp1->points[i].z - tmp1->points[j].z, 2));
  float sc2 = 0; // point cloud scale measurement
  for(size_t i = 0;i < tmp2->size()-1;i++)
    for(size_t j = i+1;j < tmp2->size();j++)
      sc2 += std::sqrt(std::pow(tmp2->points[i].x - tmp2->points[j].x, 2) + std::pow(tmp2->points[i].y - tmp2->points[j].y, 2) + std::pow(tmp2->points[i].z - tmp2->points[j].z, 2));
  // normalize
  Eigen::Matrix4f rs = Eigen::Matrix4f::Identity();
  float scale = sc1 / sc2;
  for(int i = 0;i < 3;i++)
    rs(i, i) = scale;
  pcl::transformPointCloud (*tmp2, *tmp2, rs);
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans;
  Eigen::Matrix4f tr;
  trans.estimateRigidTransformation(*tmp2, *tmp1, tr);
  Eigen::Matrix4f trs = tr * rs;
  /*
  cout << "Scale: " << scale << endl;
  cout << trs << endl;
  */


  // rotate cloud2 from landscape to portrait
  Eigen::Matrix4f tmpT3;
  tmpT3 << 0, 1, 0, 0,
          -1, 0, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
  pcl::transformPointCloud (*cloud2, *cloud2, tmpT3);
  //pcl::transformPointCloud (*k2, *k2, tmpT3);
  b2a = b2a * tmpT3.inverse();

  pcl::transformPointCloud (*cloud1, *cloud1, trs);
  pcl::transformPointCloud (*cloud2, *cloud2, b2a.inverse() * trs * b2a);
  pcl::transformPointCloud (*cloud3, *cloud3, c2a.inverse() * trs * c2a);
  pcl::transformPointCloud (*k1, *k1, trs);
  pcl::transformPointCloud (*k2, *k2, b2a.inverse() * trs);
  pcl::transformPointCloud (*k3, *k3, c2a.inverse() * trs);


  pcl::copyPointCloud(*cloud1, *cloud1d);
  pcl::transformPointCloud (*cloud2, *cloud2d, b2a);
  pcl::transformPointCloud (*cloud3, *cloud3d, c2a);
  pcl::copyPointCloud(*k1, *k1d);
  pcl::transformPointCloud (*k2, *k2d, b2a);
  pcl::transformPointCloud (*k3, *k3d, c2a);

  



  viewer->setBackgroundColor (0, 0, 0);
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());

  /*
  viewer->addPointCloud<pcl::PointXYZ> (tmp1, "Actual Box");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "Actual Box");
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp3 (new pcl::PointCloud<pcl::PointXYZ>);
  tmp3->resize(3);
  for(int i = 0;i < 3;i++)
  {
    tmp3->points[i].x = tmp2->points[i].x;
    tmp3->points[i].y = tmp2->points[i].y;
    tmp3->points[i].z = tmp2->points[i].z;
  }
  pcl::transformPointCloud (*tmp3, *tmp3, tr);
  viewer->addPointCloud<pcl::PointXYZ> (tmp3, "New Box");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "New Box");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.2, 0.2, "New Box");
  */

  viewer->addPointCloud<pcl::PointXYZRGB> (cloud1d, "Cloud A");
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud2d, "Cloud B");
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud3d, "Cloud C");
  if(key_show)
  {
    key_show = !key_show;
    toggle_keypoints();
  }
  if(key_show)
  {
    mat_show = !mat_show;
    toggle_matches();
  }
  if(col_diff)
  {
    col_diff = !col_diff;
    toggle_color();
  }
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sz, "Cloud A");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sz, "Cloud B");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sz, "Cloud C");

  viewer->initCameraParameters ();
  viewer->setCameraPosition(0, -60*sqrt(2), 27+60*sqrt(2), 0, 0, 27, 0, sqrt(2)/2, sqrt(2)/2);
  //viewer->setSize(1024, 512);
  //viewer->addCoordinateSystem (6);

  // prep virtual walkthrough
  fl.open("../dat/match/" + file1.substr(0, file1.length() - 4) + "_cam.txt");
  getline(fl, line);
  sscanf(line.c_str(), "%*s %d", &count);
  std::vector<float> av, bv, cv, dv, ev, fv;
  for(int i = 0;i < count;i++)
  {
    getline(fl, line);
    float a, b, c, d, a2, b2;
    sscanf(line.c_str(), "%f %f %f %f %f %f", &a, &b, &c, &d, &a2, &b2);
    av.push_back(12*a);
    bv.push_back(12*b);
    cv.push_back(12*c);
    dv.push_back(12*d);
    ev.push_back(12*a2);
    fv.push_back(12*b2);
  }
  fl.close();

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    if(anim)
    {
        int ct = 0;
        std::cout << "Start animation\n";
        viewer->setCameraPosition(0, -60*sqrt(2), 27+60*sqrt(2), 0, 0, 27, 0, sqrt(2)/2, sqrt(2)/2);
        // change to regular color with no keypoints
        key_show = true;
        col_diff = true;
        toggle_color();
        toggle_keypoints();
        // no fusion
        pcl::copyPointCloud(*cloud1, *cloud1d);
        pcl::copyPointCloud (*cloud2, *cloud2d);
        pcl::copyPointCloud (*cloud3, *cloud3d);
        viewer->updatePointCloud(cloud1d, "Cloud A");
        viewer->updatePointCloud(cloud2d, "Cloud B");
        viewer->updatePointCloud(cloud3d, "Cloud C");
        for(int i = 0;i < anim_frames;i++)
        {
          Eigen::Affine3f at = Eigen::Affine3f::Identity();
          at.rotate (Eigen::AngleAxisf (-M_PI/4 * i / (anim_frames - 1), Eigen::Vector3f::UnitX()));
          pcl::transformPointCloud (*cloud1, *cloud1d, at);
          pcl::transformPointCloud (*cloud2, *cloud2d, at);
          pcl::transformPointCloud (*cloud3, *cloud3d, at);
          viewer->updatePointCloud(cloud1d, "Cloud A");
          viewer->updatePointCloud(cloud2d, "Cloud B");
          viewer->updatePointCloud(cloud3d, "Cloud C");

          viewer->spinOnce();

          if(save_on)
          {
            save_frame(1, i);
            ct++;
          }
          else
            boost::this_thread::sleep (boost::posix_time::microseconds (5000));
        }
        toggle_color();
        for(int i = 0;i < anim_frames;i++)
        {
          Eigen::Affine3f at = Eigen::Affine3f::Identity();
          at.rotate (Eigen::AngleAxisf (-M_PI/4 + M_PI/4 * i / (anim_frames - 1), Eigen::Vector3f::UnitX()));
          pcl::transformPointCloud (*cloud1, *cloud1d, at);
          pcl::transformPointCloud (*cloud2, *cloud2d, at);
          pcl::transformPointCloud (*cloud3, *cloud3d, at);
          viewer->updatePointCloud(cloud1d, "Cloud A");
          viewer->updatePointCloud(cloud2d, "Cloud B");
          viewer->updatePointCloud(cloud3d, "Cloud C");

          viewer->spinOnce();

          if(save_on)
          {
            save_frame(2, i);
            ct++;
          }
          else
            boost::this_thread::sleep (boost::posix_time::microseconds (5000));
        }
        pcl::copyPointCloud(*cloud1, *cloud1d);
        pcl::copyPointCloud (*cloud2, *cloud2d);
        pcl::copyPointCloud (*cloud3, *cloud3d);
        viewer->updatePointCloud(cloud1d, "Cloud A");
        viewer->updatePointCloud(cloud2d, "Cloud B");
        viewer->updatePointCloud(cloud3d, "Cloud C");
        for(int i = 0;i < anim_frames;i++)
        {
          Eigen::Affine3f at2 = Eigen::Affine3f::Identity();
          at2.translate(Eigen::Vector3f(10*12 * i / (anim_frames - 1), 0, 0));
          Eigen::Affine3f at3 = Eigen::Affine3f::Identity();
          at3.translate(Eigen::Vector3f(-10*12 * i / (anim_frames - 1), 0, 0));
          pcl::transformPointCloud (*cloud2, *cloud2d, at2);
          pcl::transformPointCloud (*cloud3, *cloud3d, at3);
          viewer->updatePointCloud(cloud2d, "Cloud B");
          viewer->updatePointCloud(cloud3d, "Cloud C");
          viewer->setCameraPosition(0, -(60 + 60.0*i/(anim_frames-1))*sqrt(2), 27+(60 + 60.0*i/(anim_frames-1))*sqrt(2), 0, 0, 27, 0, sqrt(2)/2, sqrt(2)/2);

          viewer->spinOnce();

          if(save_on)
          {
            save_frame(3, i);
            ct++;
          }
          else
            boost::this_thread::sleep (boost::posix_time::microseconds (5000));
        }
        Eigen::Affine3f at2 = Eigen::Affine3f::Identity();
        at2.translate(Eigen::Vector3f(10*12, 0, 0));
        Eigen::Affine3f at3 = Eigen::Affine3f::Identity();
        at3.translate(Eigen::Vector3f(-10*12, 0, 0));
        pcl::copyPointCloud(*cloud1, *cloud1d);
        pcl::transformPointCloud (*cloud2, *cloud2d, at2);
        pcl::transformPointCloud (*cloud3, *cloud3d, at3);
        viewer->updatePointCloud(cloud1d, "Cloud A");
        viewer->updatePointCloud(cloud2d, "Cloud B");
        viewer->updatePointCloud(cloud3d, "Cloud C");
        pcl::copyPointCloud(*k1, *k1d);
        pcl::transformPointCloud (*k2, *k2d, at2);
        pcl::transformPointCloud (*k3, *k3d, at3);
        viewer->updatePointCloud(k1d, "Keypoints A");
        viewer->updatePointCloud(k2d, "Keypoints B");
        viewer->updatePointCloud(k3d, "Keypoints C");
        toggle_color();
        viewer->spinOnce();
        if(save_on)
        {
          save_frame(4, 0);
          ct++;
        }
        else
          boost::this_thread::sleep (boost::posix_time::microseconds (1000000));
        toggle_keypoints();
        viewer->spinOnce();
        if(save_on)
        {
          save_frame(4, 1);
          ct++;
        }
        else
          boost::this_thread::sleep (boost::posix_time::microseconds (1000000));
        toggle_matches();
        viewer->spinOnce();
        if(save_on)
        {
          save_frame(4, 2);
          ct++;
        }
        else
          boost::this_thread::sleep (boost::posix_time::microseconds (1000000));
        toggle_keypoints();
        viewer->spinOnce();
        if(save_on)
        {
          save_frame(4, 3);
          ct++;
        }
        else
          boost::this_thread::sleep (boost::posix_time::microseconds (1000000));
        toggle_color();
        viewer->spinOnce();
        if(save_on)
        {
          save_frame(4, 4);
          ct++;
        }
        else
          boost::this_thread::sleep (boost::posix_time::microseconds (1000000));
        for(int i = 0;i < anim_frames;i++)
        {
          float mod = 1.0 * i / (anim_frames - 1);
          Eigen::Matrix4f trt2 = (1 - mod) * at2.matrix() + mod * b2a;
          Eigen::Matrix4f trt3 = (1 - mod) * at3.matrix() + mod * c2a;
          pcl::transformPointCloud (*cloud2, *cloud2d, trt2);
          pcl::transformPointCloud (*cloud3, *cloud3d, trt3);
          pcl::transformPointCloud (*k2, *k2d, trt2);
          pcl::transformPointCloud (*k3, *k3d, trt3);
          viewer->updatePointCloud(cloud2d, "Cloud B");
          viewer->updatePointCloud(cloud3d, "Cloud C");
          viewer->updatePointCloud(k1d, "Keypoints A");
          viewer->updatePointCloud(k2d, "Keypoints B");
          viewer->updatePointCloud(k3d, "Keypoints C");
          toggle_matches();
          toggle_matches();
          viewer->setCameraPosition(0, -(120 - 60.0*i/(anim_frames-1))*sqrt(2), 27+(120 - 60.0*i/(anim_frames-1))*sqrt(2), 0, 0, 27, 0, sqrt(2)/2, sqrt(2)/2);

          viewer->spinOnce();

          if(save_on)
          {
            save_frame(5, i);
            ct++;
          }
          else
            boost::this_thread::sleep (boost::posix_time::microseconds (5000));
        }
        toggle_color();
        toggle_matches();
        for(int i = 0;i < anim_frames;i++)
        {
          float mod = 1.0 * i / (anim_frames - 1);
          Eigen::Matrix4f trt2 = (1 - mod) * at2.matrix() + mod * b2a;
          Eigen::Matrix4f trt3 = (1 - mod) * at3.matrix() + mod * c2a;
          pcl::transformPointCloud (*cloud2, *cloud2d, trt2);
          pcl::transformPointCloud (*cloud3, *cloud3d, trt3);
          pcl::transformPointCloud (*k2, *k2d, trt2);
          pcl::transformPointCloud (*k3, *k3d, trt3);
          viewer->updatePointCloud(cloud2d, "Cloud B");
          viewer->updatePointCloud(cloud3d, "Cloud C");
          viewer->updatePointCloud(k1d, "Keypoints A");
          viewer->updatePointCloud(k2d, "Keypoints B");
          viewer->updatePointCloud(k3d, "Keypoints C");
          toggle_matches();
          toggle_matches();
          mod = 1.0*i / (anim_frames-1);
          viewer->setCameraPosition(av[0]*mod, (-60.0*sqrt(2))*(1-mod)+bv[0]*mod, (27-60.0*sqrt(2))*(1-mod)+cv[0]*mod, dv[0]*mod, ev[0]*mod, 27*(1-mod)+fv[0]*mod, 0, 0, 1);

          viewer->spinOnce();

          if(save_on)
          {
            save_frame(6, i);
            ct++;
          }
          else
            boost::this_thread::sleep (boost::posix_time::microseconds (5000));
        }
        // Virtual Walkthrough
        for(int i = 0;i < count;i++)
        {
          viewer->setCameraPosition(av[i], bv[i], cv[i], dv[i], ev[i], fv[i], 0, 0, 1);

          viewer->spinOnce();
          
          //cout << "Frame " << i << " out of " << count << endl;
          if(save_on)
          {
            save_frame(7, i);
            ct++;
          }
          else
            boost::this_thread::sleep (boost::posix_time::microseconds (5000));
        }
        

        anim = false;
    }
    else
      viewer->spinOnce(100);

    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
