/* Hyper Parameters for Fusion */
#include <vector>

/* Filter */
// filter type
std::vector<int> filt_type;
// bound filter limits
std::vector<float> filt_lims = {-5, 5, -5, 5, -5, 5};
// NUmber of neighbors required
int filt_N = 0;//30
// search radius
float filt_R = 0.02f;//0.08
// Number of neighbors to search
int filt_K = 100; // 30
// variance threshold
float filt_T = 0.08f; // 0.08
// downsample res
float filt_dres = 0.005f; // 0.01

/*** Keypoint Detection ***/
int keys_mode = 1;
/* SIFT Detection */
// min scale
float sift_ms = 0.1f;//0.04f;
// number of octaves
int sift_no = 8;
// scales per octave
int sift_ns = 10;
// min contrast
int sift_mc = 0.0001f;
// percentile (ie 0.5 is the top 50%)
float sift_prct = 0.5f;
/* Harris Detection */
// radius
float harr_rad = 0.1f;
// non max suppression
bool harr_maxs = true;
// refinement of keypoints
bool harr_ref = true;
// threshold
float harr_tau = 0.0f;

/* Normals Calculation */
float norm_r = 0.1;//0.075;

/* Feature Calculation */
int feat_mode = 0;
float feat_r = 0.2f;//0.1

/* Correlation */
// alignment epsilon
float corr_eps = 0.1;
// alignment max iterations
int corr_n = 20;


/* Extra Parameters */
// normal display length
float extr_nsz = 0.01;
// keypoint sphere size
float extr_mksz = 0.01;