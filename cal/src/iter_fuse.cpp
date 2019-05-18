

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

Eigen::Matrix4f randtr;

void process_inputs(FileStorage fs)
{
    FileNode fmode = fs["filter"]["mode"];
    filt_N = (int) fs["filter"]["Radius"]["n"];
    filt_R = (float) fs["filter"]["Radius"]["r"];
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

    // bound filter
    FileNode bounds = fs["filter"]["Bound"];
    filt_lims[0] = (float) bounds["x"][0];
    filt_lims[1] = (float) bounds["x"][1];
    filt_lims[2] = (float) bounds["y"][0];
    filt_lims[3] = (float) bounds["y"][1];
    filt_lims[4] = (float) bounds["z"][0];
    filt_lims[5] = (float) bounds["z"][1];

    // convert filter setup
    FileNode ftypes = fs["filter"]["types"];
    FileNodeIterator it1 = fmode.begin(), it1_end = fmode.end();
    FileNodeIterator it2 = ftypes.begin(), it2_end = ftypes.end();
    filt_type.clear();
    for(;it1 != it1_end;it1++)
    {
        it2 = ftypes.begin();
        for(;it2 != it2_end;++it2)
            if(((string) *it1) == ((string) ((*it2)["name"])))
                filt_type.push_back((int) ((*it2)["v"]));
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
    
    FileNode temp = fs["transform"]["T"][(int) fs["transform"]["select"]]["data"];
    // parse transform
    for(int i = 0;i < 4;i++)
        for(int j = 0;j < 4;j++)
            randtr(i, j) = (float) temp[4*i+j];
}
int decodeVar(FileNode n)
{

}
void setVar(int index, int vali, float valf)
{
    switch(index)
    {
        case 0: // filt_type
            filt_type.push_back(vali);
            break;
        case 1: // filt_N
            filt_N = vali;
            break;
        case 2: // filt_R
            filt_R = valf;
            break;
        case 3: // filt_K
            filt_K = vali;
            break;
        case 4: // filt_T
            filt_T = valf;
            break;
        case 5: // filt_dres
            filt_dres = valf;
            break;
        case 6: // sift_ms
            sift_ms = valf;
            break;
        case 7: // sift_no
            sift_no = vali;
            break;
        case 8: // sift_ns
            sift_ns = vali;
            break;
        case 9: // sift_mc
            sift_mc = vali;
            break;
        case 10: // sift_prct
            sift_prct = valf;
            break;
        case 11: // keys_mode
            keys_mode = vali;
            break;
        case 12: // harr_rad
            harr_rad = valf;
            break;
        case 13: // harr_maxs
            harr_maxs = vali > 0;
            break;
        case 14: // harr_ref
            harr_ref = vali > 0;
            break;
        case 15: // harr_tau
            harr_tau = valf;
            break;
        case 16: // norm_r
            norm_r = valf;
            break;
        case 17: // feat_mode
            feat_mode = vali;
            break;
        case 18: // feat_r
            feat_r = valf;
            break;
        case 19: // corr_eps
            corr_eps = valf;
            break;
        case 20: // corr_n
            corr_n = vali;
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
        idx[i] = decodeVar(its[i]["var"]);
        vals[i].resize(n[i]);
        valsi[i].resize(n[i]);
        for(int j = 0;j < n[i];j++)
        {
            vals[i][j] = ((float) its[i]["a"]) + (((float) its[i]["b"]) - ((float) its[i]["a"])) * j / n[i];
            valsi[i][j] = ((int) its[i]["a"]) + round((((float) its[i]["b"]) - ((float) its[i]["a"])) * j / n[i]);
        }
    }
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
    fs.open(filename, FileStorage::READ);
    
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