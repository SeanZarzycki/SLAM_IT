/* Hyper Parameters for Fusion */


/* Filter */
// filter type: 1 for radius, 2 for stat
int filt_type = 2;
// NUmber of neighbors required
int filt_N = 30;//30
// search radius
float filt_R = 0.08f;//0.05

/* SIFT Detection */
// min scale
float sift_ms = 0.04f;//0.04f;
// number of octaves
int sift_no = 6;
// scales per octave
int sift_ns = 10;
// min contrast
int sift_mc = 0.0001f;

/* Normals Calculation */
float norm_r = 0.05;//0.023;

/* Feature Calculation */
float feat_r = 0.1f;//0.06

/* Correlation */
// percentile (ie 0.5 is median)
float corr_v = 0.9;
// alignment epsilon
float corr_eps = 0.1;
// alignment max iterations
int corr_n = 10;


/* Extra Parameters */
// downsample point cloud 2
bool extr_down = false;
// downsample res
float extr_dres = 0.01f;