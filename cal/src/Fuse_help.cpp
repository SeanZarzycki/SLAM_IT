/* Hyper Parameters for Fusion */
#include "Fuse_help.h"


Fuse_sets::Fuse_sets()
{
    /* Filter */
    // filter type
    filt_type.clear();
    // bound filter limits
    filt_lims = {-5, 5, -5, 5, -5, 5};
    // NUmber of neighbors required
    filt_N = 0;//30
    // search radius
    filt_R = 0.02f;//0.08
    // Number of neighbors to search
    filt_K = 100; // 30
    // variance threshold
    filt_T = 0.08f; // 0.08
    // downsample res
    filt_dres = 0.005f; // 0.01

    /*** Keypoint Detection ***/
    keys_mode = 1;
    // min allowed distance between keypoints
    keys_dist = 0;
    /* SIFT Detection */
    // min scale
    sift_ms = 0.1f;//0.04f;
    // number of octaves
    sift_no = 8;
    // scales per octave
    sift_ns = 10;
    // min contrast
    sift_mc = 0.0001f;
    // percentile (ie 0.5 is the top 50%)
    sift_prct = 0.5f;
    /* Harris Detection */
    // radius
    harr_rad = 0.1f;
    // non max suppression
    harr_maxs = true;
    // refinement of keypoints
    harr_ref = true;
    // threshold
    harr_tau = 0.0f;

    /* Normals Calculation */
    norm_r = 0.1;//0.075;

    /* Feature Calculation */
    feat_mode = 0;
    fpfh_r = 0.2f;//0.1
    rift_r = 0.02;
    rift_igr = 0.03;
    pfhc_r = 0.2;

    /* Correlation */
    // max distance between matched keypoints
    corr_dist = 1;
    // alignment epsilon
    corr_eps = 0.1;
    // alignment max iterations
    corr_n = 20;

    /* Iterative Closest Point */
    // max correspondence distance
    itcp_mcd = 0.005;
    // max iterations
    itcp_n = 50;

    use_thread = true;

    /* Extra Parameters */
    // normal display length
    extr_nsz = 0.01;
    // keypoint sphere size
    extr_mksz = 0.01;
}
Fuse_sets Fuse_sets::copy()
{
    Fuse_sets cp;

    cp.filt_type = filt_type;
    cp.filt_type = filt_type;
    cp.filt_N = filt_N;
    cp.filt_R = filt_R;
    cp.filt_K = filt_K;
    cp.filt_T = filt_T;
    cp.filt_dres = filt_dres;
    cp.filt_keyn = filt_keyn;
    cp.sift_ms = sift_ms;
    cp.sift_no = sift_no;
    cp.sift_ns = sift_ns;
    cp.sift_mc = sift_mc;
    cp.sift_prct = sift_prct;
    cp.keys_mode = keys_mode;
    cp.keys_dist = keys_dist;
    cp.harr_rad = harr_rad;
    cp.harr_maxs = harr_maxs;
    cp.harr_ref = harr_ref;
    cp.harr_tau = harr_tau;
    cp.norm_r = norm_r;
    cp.feat_mode = feat_mode;
    cp.fpfh_r = fpfh_r;
    cp.rift_r = rift_r;
    cp.rift_igr = rift_igr;
    cp.pfhc_r = pfhc_r;
    cp.corr_dist = corr_dist;
    cp.corr_eps = corr_eps;
    cp.corr_n = corr_n;

    cp.itcp_mcd = itcp_mcd;
    cp.itcp_n = itcp_n;

    cp.randtr = randtr;

    cp.use_thread = use_thread;

    cp.extr_nsz = extr_nsz;
    cp.extr_mksz = extr_mksz;

    return cp;
}