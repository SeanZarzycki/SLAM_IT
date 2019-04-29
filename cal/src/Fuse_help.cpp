/* Hyper Parameters for Fusion */


/* Filter */
// filter type: 1 for radius, 2 for stat, 3 for Radius after downsample, 4 for stat then radius
int filt_type = 2;
// NUmber of neighbors required
int filt_N = 4;//30
// search radius
float filt_R = 0.0075f;//0.08
// Number of neighbors to search
int filt_K = 100; // 30
// variance threshold
float filt_T = 0.08f; // 0.08
// NUmber of neighbors required
int filt_N2 = 0;//30
// search radius
float filt_R2 = 0.1f;//0.08

/* SIFT Detection */
// min scale
float sift_ms = 0.05f;//0.04f;
// number of octaves
int sift_no = 8;
// scales per octave
int sift_ns = 10;
// min contrast
int sift_mc = 0.0001f;
// percentile (ie 0.5 is the top 50%)
float sift_prct = 0.5f;

/* Normals Calculation */
float norm_r = 0.075;//0.075;

/* Feature Calculation */
float feat_r = 0.1f;//0.1

/* Correlation */
// alignment epsilon
float corr_eps = 0.1;
// alignment max iterations
int corr_n = 20;


/* Extra Parameters */
// downsample point cloud 2
bool extr_down = true;
// downsample res
float extr_dres = 0.01f; // 0.01
// normal display length
float extr_nsz = 0.01;
// keypoint sphere size
float extr_mksz = 0.01;