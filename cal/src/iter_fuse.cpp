

#include <iostream>
#include <opencv2/core.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>

#include <Eigen/Dense>

#include "Fuse_help.h"

using namespace cv;
using namespace std;

int parseHighArgument(char* arg);
int run(vector<char*> argv, Eigen::Matrix<float, 4, 4> &s2d);

int mode;

void process_inputs(FileStorage fs)
{
    FileNode fmode = fs["filter"]["mode"];
    filt_N = (int) fs["filter"]["Radius"]["n"];
    filt_R = (float) fs["filter"]["Radius"]["n"];
    filt_K = (int) fs["filter"]["Stat"]["K"];
    filt_T = (float) fs["filter"]["Stat"]["T"];
    filt_dres = (float) fs["filter"]["Down"]["res"];
    string kmode = fs["keypoints"]["mode"];
    sift_ms = (float) (fs["keypoints"]["SIFT"]["min scale"]);
    sift_no = (int) fs["keypoints"]["SIFT"]["octaves"];
    sift_ns = (int) fs["keypoints"]["SIFT"]["scales"];
    sift_mc = (float) fs["keypoints"]["SIFT"]["min contrast"];
    sift_prct = (float) fs["keypoints"]["SIFT"]["prctile"];
    harr_rad = (float) fs["keypoints"]["Harris"]["r"];
    harr_tau = (float) fs["keypoints"]["Harris"]["t"];
    harr_maxs = ((int) fs["keypoints"]["Harris"]["max_sup"]) > 0;
    harr_ref = ((int) fs["keypoints"]["Harris"]["res"]) > 0;
    norm_r = (float) fs["normals"]["r"];
    string femode = fs["features"]["mode"];
    feat_r = (float) fs["features"]["FPFH"]["r"];
    corr_eps = (float) fs["correlate"]["eps"];
    corr_n = (int) fs["correlate"]["n"];

    // convert filter setup
    FileNode ftypes = fs["filter"]["types"];
    FileNodeIterator it1 = fmode.begin(), it1_end = fmode.end();
    FileNodeIterator it2 = ftypes.begin(), it2_end = ftypes.end();
    filt_type = 0;
    for(;it1 != it1_end;it1++)
    {
        it2 = ftypes.begin();
        for(;it2 != it2_end;++it2)
            if(((string) *it1) == ((string) ((*it2)["name"])))
                filt_type += (int) ((*it2)["v"]);
    }

    // convert keypoint selection
    FileNode ktypes = fs["keypoints"]["types"];
    it2 = ktypes.begin(); it2_end = ktypes.end();
    for(;it2 != it2_end;++it2)
        if(kmode == ((string) ((*it2)["name"])))
            keys_mode = (int) (*it2)["v"];

    FileNode fetypes = fs["keypoints"]["types"];
    it2 = fetypes.begin(); it2_end = fetypes.end();
    for(;it2 != it2_end;++it2)
        if(femode == ((string) ((*it2)["name"])))
            feat_mode = (int) (*it2)["v"];
}
void run_iter(FileStorage fs)
{
    
}

int main(int argc, char** argv)
{
    // default to normal calulation
    mode = 0;

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
    cout << "Open: " << fs.open(filename, FileStorage::READ) << endl;
    
    process_inputs(fs);


    int pass = 0;
    if(mode == 0)
    {
        Eigen::Matrix<float, 4, 4> trans;
        pass = run(pars, trans);
        cout << trans << endl;
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