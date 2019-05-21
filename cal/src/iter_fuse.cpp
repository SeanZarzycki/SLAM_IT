

#include <iostream>
#include <opencv2/core.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>

#include <Eigen/Dense>

#include <boost/thread.hpp>

#include "Fuse.h"

using namespace cv;
using namespace std;

FuseAlg* view_fuse;

int parseHighArgument(char* arg);
int run(vector<char*> argv, Eigen::Matrix<float, 4, 4> &s2d);
void viewOne (pcl::visualization::PCLVisualizer& viewer);

int mode, thread_num;
bool inter;

Fuse_sets process_inputs(FileStorage fs)
{
    Fuse_sets sets;

    FileNode fmode = fs["filter"]["mode"];
    sets.filt_N = (int) fs["filter"]["Radius"]["n"];
    sets.filt_R = (float) fs["filter"]["Radius"]["r"];
    sets.filt_K = (int) fs["filter"]["Stat"]["K"];
    sets.filt_T = (float) fs["filter"]["Stat"]["T"];
    sets.filt_dres = (float) fs["filter"]["Down"]["res"];
    string kmode = fs["keypoints"]["mode"];
    sets.keys_dist = (float) (fs["keypoints"]["dist"]);
    sets.sift_ms = (float) (fs["keypoints"]["SIFT"]["min scale"]);
    sets.sift_no = (int) fs["keypoints"]["SIFT"]["octaves"];
    sets.sift_ns = (int) fs["keypoints"]["SIFT"]["scales"];
    sets.sift_mc = (float) fs["keypoints"]["SIFT"]["min contrast"];
    sets.sift_prct = (float) fs["keypoints"]["SIFT"]["prctile"];
    sets.harr_rad = (float) fs["keypoints"]["Harris"]["r"];
    sets.harr_tau = (float) fs["keypoints"]["Harris"]["t"];
    sets.harr_maxs = ((int) fs["keypoints"]["Harris"]["max_sup"]) > 0;
    sets.harr_ref = ((int) fs["keypoints"]["Harris"]["res"]) > 0;
    sets.norm_r = (float) fs["normals"]["r"];
    string femode = fs["features"]["mode"];
    sets.fpfh_r = (float) fs["features"]["FPFH"]["r"];
    sets.rift_r = (float) fs["features"]["RIFT"]["rad"];
    sets.rift_igr = (float) fs["features"]["RIFT"]["igr"];
    sets.pfhc_r = (float) fs["features"]["PFHC"]["r"];
    sets.corr_dist = (float) fs["correlate"]["dist"];
    sets.corr_eps = (float) fs["correlate"]["eps"];
    sets.corr_n = (int) fs["correlate"]["n"];
    
    sets.itcp_mcd = (int) fs["icp"]["mcd"];
    sets.itcp_n = (int) fs["icp"]["n"];

    // bound filter
    FileNode bounds = fs["filter"]["Bound"];
    sets.filt_lims[0] = (float) bounds["x"][0];
    sets.filt_lims[1] = (float) bounds["x"][1];
    sets.filt_lims[2] = (float) bounds["y"][0];
    sets.filt_lims[3] = (float) bounds["y"][1];
    sets.filt_lims[4] = (float) bounds["z"][0];
    sets.filt_lims[5] = (float) bounds["z"][1];

    // convert filter setup
    FileNode ftypes = fs["filter"]["types"];
    FileNodeIterator it1 = fmode.begin(), it1_end = fmode.end();
    FileNodeIterator it2 = ftypes.begin(), it2_end = ftypes.end();
    sets.filt_type.clear();
    for(;it1 != it1_end;it1++)
    {
        it2 = ftypes.begin();
        for(;it2 != it2_end;++it2)
            if(((string) *it1) == ((string) ((*it2)["name"])))
                sets.filt_type.push_back((int) ((*it2)["v"]));
    }

    // convert keypoint selection
    FileNode ktypes = fs["keypoints"]["types"];
    it2 = ktypes.begin(); it2_end = ktypes.end();
    for(;it2 != it2_end;++it2)
        if(kmode == ((string) ((*it2)["name"])))
            sets.keys_mode = (int) (*it2)["v"];

    FileNode fetypes = fs["features"]["types"];
    it2 = fetypes.begin(); it2_end = fetypes.end();
    for(;it2 != it2_end;++it2)
        if(femode == ((string) ((*it2)["name"])))
            sets.feat_mode = (int) (*it2)["v"];
    
    FileNode temp = fs["transform"]["T"][(int) fs["transform"]["select"]]["data"];
    // parse transform
    for(int i = 0;i < 4;i++)
        for(int j = 0;j < 4;j++)
            sets.randtr(i, j) = (float) temp[4*i+j];


    sets.extr_nsz = 0.01;
    sets.extr_mksz = 0.01;

    return sets;
}
int decodeVar(vector<string> ls)
{

}
void setVar(Fuse_sets sets, int index, int vali, float valf)
{
    switch(index)
    {
        case 0: // filt_type
            sets.filt_type.push_back(vali);
            break;
        case 1: // filt_N
            sets.filt_N = vali;
            break;
        case 2: // filt_R
            sets.filt_R = valf;
            break;
        case 3: // filt_K
            sets.filt_K = vali;
            break;
        case 4: // filt_T
            sets.filt_T = valf;
            break;
        case 5: // filt_dres
            sets.filt_dres = valf;
            break;
        case 6: // sift_ms
            sets.sift_ms = valf;
            break;
        case 7: // sift_no
            sets.sift_no = vali;
            break;
        case 8: // sift_ns
            sets.sift_ns = vali;
            break;
        case 9: // sift_mc
            sets.sift_mc = vali;
            break;
        case 10: // sift_prct
            sets.sift_prct = valf;
            break;
        case 11: // keys_mode
            sets.keys_mode = vali;
            break;
        case 12: // harr_rad
            sets.harr_rad = valf;
            break;
        case 13: // harr_maxs
            sets.harr_maxs = vali > 0;
            break;
        case 14: // harr_ref
            sets.harr_ref = vali > 0;
            break;
        case 15: // harr_tau
            sets.harr_tau = valf;
            break;
        case 16: // norm_r
            sets.norm_r = valf;
            break;
        case 17: // feat_mode
            sets.feat_mode = vali;
            break;
        case 18: // feat_r
            sets.fpfh_r = valf;
            break;
        case 19: // corr_eps
            sets.corr_eps = valf;
            break;
        case 20: // corr_n
            sets.corr_n = vali;
            break;
        case 21: // feat_r
            sets.rift_r = valf;
            break;
        case 22: // feat_r
            sets.rift_igr = valf;
            break;
    }
}

void run_iter(FileStorage fs)
{
    FileNode its = fs["iterate"];
    
    vector<int> idx (its.size(), 0);
    vector<int> n (its.size(), 0);
    vector<vector<float>> vals;
    vector<vector<float>> valsi;
    for(size_t i = 0;i < its.size();i++)
    {
        n[i] = (int) its[i]["n"];
        //idx[i] = decodeVar(its[i]["var"]);
        vals[i].resize(n[i]);
        valsi[i].resize(n[i]);
        for(int j = 0;j < n[i];j++)
        {
            vals[i][j] = ((float) its[i]["a"]) + (((float) its[i]["b"]) - ((float) its[i]["a"])) * j / n[i];
            valsi[i][j] = ((int) its[i]["a"]) + round((((float) its[i]["b"]) - ((float) its[i]["a"])) * j / n[i]);
        }
    }





    vector<boost::thread*> threads;
    for(int t = 0;t < thread_num;t++)
    {
        threads.push_back(new boost::thread());
    }
}

int main(int argc, char** argv)
{
    // default to normal calulation
    mode = 0;
    inter = true;

    vector<char*> pars;
    pars.push_back(argv[0]);
    for(int i = 1;i < argc;i++)
    {
        if(parseHighArgument(argv[i]) == -1)
        {
            pars.push_back(argv[i]);
        }
    }


    string filename = "../src/fuse_pars.yaml";
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    
    if(((int) fs["enables"]["fuse"]) == 0)
        pars.push_back("fuse_en=0");
    if(((int) fs["icp"]["en"]) == 1)
        pars.push_back("icp_en=1");

    Fuse_sets osets = process_inputs(fs);
    osets.use_thread = inter;

    int pass = 0;
    if(mode == 0)
    {
        view_fuse = new FuseAlg(&osets);
        Eigen::Matrix<float, 4, 4> trans;
        pass = view_fuse->run(pars, trans);
        cout << trans << endl;

          // display clouds
        if(view_fuse->disp_view)
        {
            pcl::visualization::CloudViewer viewer ("Fuse Example");
            viewer.showCloud(view_fuse->cloud1, "Cloud A");
            viewer.showCloud(view_fuse->cloud2, "Cloud B");
            if(view_fuse->keys)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr dk1 (new pcl::PointCloud<pcl::PointXYZ>);
                copyPointCloud(*view_fuse->keys1, *dk1);
                pcl::PointCloud<pcl::PointXYZ>::Ptr dk2 (new pcl::PointCloud<pcl::PointXYZ>);
                copyPointCloud(*view_fuse->keys2, *dk2);
                
                viewer.showCloud(dk1, "Keypoints A");
                viewer.showCloud(dk2, "Keypoints B");
            }
            viewer.runOnVisualizationThreadOnce(viewOne);
            
            while (!viewer.wasStopped ())
            { }
        }

        delete view_fuse;
    }
    else if(mode == 1)
    {
        run_iter(fs);

        pass = 0;
    }
    else
    {
        cout << "Unknown mode\n";
        pass = 0;
    }
    
    fs.release();


    return pass;
}





int parseHighArgument(char* arg)
{
	int option;
	//float foption;
	//char buf[1000];
	
	if(1==sscanf(arg,"mode=%d", &option))
    {
        mode = option;
        return 0;
    }
    if(1==sscanf(arg,"thread=%d", &option))
    {
        thread_num = option;
        return 0;
    }
    if(1==sscanf(arg,"inter=%d", &option))
    {
        if(option == 1)
            inter = true;
        else
            inter = false;
        return 0;
    }
    /*
    if(1==sscanf(arg,"-%s", buf))
    {
        if(strcmp(buf, "c") == 0 || strcmp(buf, "-colors") == 0)
        {

            return 0;
        }
        if(strcmp(buf, "k") == 0 || strcmp(buf, "-keys") == 0)
        {
            
            return 0;
        }
    }
    */

    return -1;
}

void viewOne (pcl::visualization::PCLVisualizer& viewer)
{
  viewer.setCameraPosition(0, 0, -2, 0, 0, 1, 0, 1, 0);

  if(view_fuse->normals)
  {
    viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(view_fuse->cloud1, view_fuse->norms1, 1, view_fuse->sets->extr_nsz, "Normals A");
    viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(view_fuse->cloud2, view_fuse->norms2, 1, view_fuse->sets->extr_nsz, "Normals B");
  }

  if(view_fuse->key_sphere)
  {
    // Draw each keypoint as a sphere
    for (size_t i = 0; i < view_fuse->keys1->size (); ++i)
    {
      // Get the point data
      const pcl::PointXYZ & p = view_fuse->keys1->points[i];

      // Pick the radius of the sphere *
      float r = 0.1 * 1;//p.scale;
      // * Note: the scale is given as the standard deviation of a Gaussian blur, so a
      //   radius of 2*p.scale is a good illustration of the extent of the keypoint

      // Generate a unique string for each sphere
      std::stringstream ss ("keypointA");
      ss << i;

      // Add a sphere at the keypoint
      viewer.addSphere (p, r, 0.0, 1.0, 1.0, ss.str ());
    }
    for (size_t i = 0; i < view_fuse->keys2->size (); ++i)
    {
      // Get the point data
      const pcl::PointXYZ & p = view_fuse->keys2->points[i];

      // Pick the radius of the sphere *
      float r = 0.1 * 1;//p.scale;
      // * Note: the scale is given as the standard deviation of a Gaussian blur, so a
      //   radius of 2*p.scale is a good illustration of the extent of the keypoint

      // Generate a unique string for each sphere
      std::stringstream ss ("keypointB");
      ss << i;

      // Add a sphere at the keypoint
      viewer.addSphere (p, r, 1.0, 1.0, 0.0, ss.str ());
    }
  }
  if(view_fuse->show_corr)
  {
    for (size_t i = 0; i < view_fuse->inlie->size (); ++i)
    {
      // Get the pair of points
      //const pcl::PointXYZ & p_left = view_fuse->keys1->points[view_fuse->corr->at(i).index_query];
      //const pcl::PointXYZ & p_right = view_fuse->keys2->points[view_fuse->corr->at(i).index_match];
      const pcl::PointXYZ & p_left = view_fuse->keys1->points[view_fuse->inlie->at(i).index_query];
      const pcl::PointXYZ & p_right = view_fuse->keys2->points[view_fuse->inlie->at(i).index_match];

      // Generate a random (bright) color
      double r = (rand() % 100);
      double g = (rand() % 100);
      double b = (rand() % 100);
      double max_channel = std::max (r, std::max (g, b));
      r /= max_channel;
      g /= max_channel;
      b /= max_channel;

      // Generate a unique string for each line
      std::stringstream ss ("line");
      ss << i;

      // Draw the line
      viewer.addLine (p_left, p_right, r, g, b, ss.str ());
      if(!view_fuse->key_sphere && !view_fuse->keys)
      {
        std::stringstream ss2;
        ss2 << "keypointA" << i;
        viewer.addSphere (p_left, view_fuse->sets->extr_mksz, 1.0, 1.0, 0.0, ss2.str ());
        std::stringstream ss3;
        ss3 << "keypointB" << i;
        viewer.addSphere (p_right, view_fuse->sets->extr_mksz, 0.0, 1.0, 1.0, ss3.str ());
      }
    }
  }

  if(view_fuse->col_diff)
  {
	  viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1, 0.2, 0.2, "Cloud A");
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 0.2, 0.2, 1, "Cloud B");
    if(view_fuse->normals)
    {
      viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 0.7, 0.35, 0.35, "Normals A");
      viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 0.35, 0.35, 0.7, "Normals B");
    }
  }
  if(view_fuse->keys)
  {
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1, 1, 0, "Keypoints A");
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 0, 1, 1, "Keypoints B");
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 10, "Keypoints A");
    viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 10, "Keypoints B");
  }

  //viewer.saveScreenshot("~/data/results/img1.png");
  //viewer.saveScreenshot("../dat/results/img1.png");
}