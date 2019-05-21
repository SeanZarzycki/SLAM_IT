
#include <vector>
#include <Eigen/Dense>

struct Fuse_sets
{
public:
    // default constructor
    Fuse_sets();
    // make copy of settings
    Fuse_sets copy();

    // constant values for testing
    std::vector<int> filt_type;
    std::vector<float> filt_lims;
    int filt_N;
    float filt_R;
    int filt_K;
    float filt_T;
    float filt_dres;
    int filt_keyn;
    float sift_ms;
    int sift_no;
    int sift_ns;
    int sift_mc;
    float sift_prct;
    int keys_mode;
    float keys_dist;
    float harr_rad;
    bool harr_maxs;
    bool harr_ref;
    float harr_tau;
    float norm_r;
    int feat_mode;
    float fpfh_r;
    float rift_r;
    float rift_igr;
    float pfhc_r;
    
    float corr_dist;
    float corr_eps;
    int corr_n;

    float itcp_mcd;
    int itcp_n;

    Eigen::Matrix4f randtr;

    bool use_thread;

    float extr_nsz;
    float extr_mksz;
};