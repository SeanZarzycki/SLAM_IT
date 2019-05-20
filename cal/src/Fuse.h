// general
#include <iostream>
#include <cmath>
#include <string>

#include <boost/thread/thread.hpp>

// misc pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>

// fusion requirements
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/filters/extract_indices.h>
#include "edge_det.h"
#include <pcl/features/fpfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/point_types_conversion.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/rift.h>
#include <pcl/correspondence.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>

#include "Fuse_help.h"


const int dist_bins = 4;
const int grad_bins = 8;
typedef pcl::Histogram<dist_bins*grad_bins> RIFT_DESC;

class FuseAlg
{
    public:
        FuseAlg(Fuse_sets* settings);
        int run(std::vector<char*> argv, Eigen::Matrix<float, 4, 4> &s2d);
        
        void key_detect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &keys, pcl::PointCloud<pcl::Normal>::Ptr &norms, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &feats_fpfh, pcl::PointCloud<RIFT_DESC>::Ptr &feats_rift, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &feats_pfhc);
        void match_fpfh(pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fs, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fd, pcl::CorrespondencesPtr cor);
        void match_rift(pcl::PointCloud<RIFT_DESC>::Ptr &fs, pcl::PointCloud<RIFT_DESC>::Ptr &fd, pcl::CorrespondencesPtr cor);
        void match_pfhc(pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fs, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fd, pcl::CorrespondencesPtr cor);
        Eigen::Matrix<float, 4, 4> register_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr &ms, pcl::PointCloud<pcl::PointXYZ>::Ptr &md, pcl::CorrespondencesPtr cor);
        int parseArgument(char* arg);

        Fuse_sets* sets;

        std::string file1, file2;
        int scan_val;
        bool col_diff, keys, key_sphere, c_disp, show_filt, show_corr, trans, ext_points, normals, disp_view;
        bool fuse_en, icp_en;

        // clouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2;
        pcl::PointCloud<pcl::PointXYZ>::Ptr keys1;
        pcl::PointCloud<pcl::PointXYZ>::Ptr keys2;
        pcl::PointCloud<pcl::Normal>::Ptr norms1;
        pcl::PointCloud<pcl::Normal>::Ptr norms2;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh1, fpfh2;
        pcl::PointCloud<RIFT_DESC>::Ptr rift1, rift2;
        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr pfhc1, pfhc2;
        pcl::CorrespondencesPtr inlie;
        pcl::CorrespondencesPtr corr;
};
