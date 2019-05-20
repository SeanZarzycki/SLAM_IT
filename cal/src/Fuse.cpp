

#include "Fuse.h"

using namespace std;

FuseAlg::FuseAlg(Fuse_sets* settings)
{
  sets = settings;

  cloud1 = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud2 = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
  keys1 = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>());
  keys2 = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>());
  norms1 = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>());
  norms2 = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>());
  fpfh1 = pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
  fpfh2 = pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
  rift1 = pcl::PointCloud<RIFT_DESC>::Ptr (new pcl::PointCloud<RIFT_DESC>());
  rift2 = pcl::PointCloud<RIFT_DESC>::Ptr (new pcl::PointCloud<RIFT_DESC>());
  pfhc1 = pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr (new pcl::PointCloud<pcl::PFHRGBSignature250>());
  pfhc2 = pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr (new pcl::PointCloud<pcl::PFHRGBSignature250>());
  inlie = pcl::CorrespondencesPtr (new pcl::Correspondences());
  corr = pcl::CorrespondencesPtr (new pcl::Correspondences());
}

void FuseAlg::key_detect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &keys, pcl::PointCloud<pcl::Normal>::Ptr &norms, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &feats_fpfh, pcl::PointCloud<RIFT_DESC>::Ptr &feats_rift, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &feats_pfhc)
{
  if(c_disp)
  {
    cout << "Original Size: " << cloud->points.size() << "\n";
  }
  
  string filt_name = "";
  // Preprocess filtering
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filt (new pcl::PointCloud<pcl::PointXYZRGB>);
  copyPointCloud(*cloud, *filt);
  for(size_t k = 0;k < sets->filt_type.size();k++)
  {
    if(sets->filt_type[k] == 4)
    {
      // limit bounds of point cloud
      pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::GT, sets->filt_lims[0])));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::LT, sets->filt_lims[1])));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::GT, sets->filt_lims[2])));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT, sets->filt_lims[3])));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, sets->filt_lims[4])));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, sets->filt_lims[5])));
      // build the filter
      pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
      condrem.setCondition (range_cond);
      condrem.setInputCloud(filt);
      condrem.setKeepOrganized(false);
      // apply filter
      condrem.filter(*filt);

      if(c_disp)
        cout << "After Bound Filter: " << filt->points.size() << endl;

      if(filt_name.size() > 0)
        filt_name = filt_name + "+";
      filt_name = filt_name + "Bound Filter";
    }
    if(sets->filt_type[k] == 2)
    {
      // Create the filtering object
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      sor.setInputCloud (filt);
      sor.setMeanK (sets->filt_K);
      sor.setStddevMulThresh (sets->filt_T);
      sor.filter (*filt);

      if(c_disp)
        cout << "After Statistical: " << filt->points.size() << endl;

      if(filt_name.size() > 0)
        filt_name = filt_name + "+";
      filt_name = filt_name + "Statistical Filter";
    }
    if(sets->filt_type[k] == 1)
    {
      pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
      // build the filter
      outrem.setInputCloud(filt);
      outrem.setRadiusSearch(sets->filt_R);
      outrem.setMinNeighborsInRadius(sets->filt_N);
      // apply filter
      outrem.filter (*filt);
      
      if(c_disp)
        cout << "After Radius: " << filt->points.size() << endl;

      if(filt_name.size() > 0)
        filt_name = filt_name + "+";
      filt_name = filt_name + "Radius Filter";
    }
    if(sets->filt_type[k] == 3)
    {
      // Downsample
      pcl::VoxelGrid<pcl::PointXYZRGB> sor;
      sor.setInputCloud (filt);
      sor.setLeafSize (sets->filt_dres, sets->filt_dres, sets->filt_dres);
      sor.filter (*filt);

      if(c_disp)
        cout << "After Downsample: " << filt->points.size() << endl;

      if(filt_name.size() > 0)
        filt_name = filt_name + "+";
      filt_name = filt_name + "Downsample";
    }
  }
  
  /*
  if(c_disp && sets->filt_type != 0)
  {
    cout << filt_name << " Size: " << filt->points.size() << "\n";
  }
  */

  // get normals
  //pcl::PointCloud<pcl::Normal>::Ptr norms (new pcl::PointCloud<pcl::Normal>);
  //pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
  fuse::EdgeDetection norm_est;
  norm_est.setInputCloud (filt); // possibly use raw cloud
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  norm_est.setSearchMethod (tree2);
  norm_est.setRadiusSearch (sets->norm_r);
  norm_est.compute(*norms);

  // Keypoint detection
  vector<float> sift_vals;
  if(!ext_points)
  {
    if(sets->keys_mode == 1)
    {
      // SIFT
      pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
      pcl::PointCloud<pcl::PointWithScale>::Ptr results (new pcl::PointCloud<pcl::PointWithScale>());
      sift.setSearchMethod(tree);
      sift.setScales(sets->sift_ms, sets->sift_no, sets->sift_ns);
      sift.setMinimumContrast(sets->sift_mc);
      sift.setInputCloud(filt);
      sift.compute(*results);

      if(c_disp)
      {
        if(sets->sift_prct > 0 && !ext_points)
          cout << "Original Keypoints: " << results->size() << endl;
      }

      // filter out small sift results
      sift_vals.resize(results->size());
      for(size_t j = 0;j < results->size();j++)
        sift_vals[j] = results->points[j].scale;
      std::sort(sift_vals.begin(), sift_vals.end());
      int ind = sift_vals.size() * sets->sift_prct;
      float prctile = sift_vals[ind];
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
      for(size_t j = 0;j < results->size();j++)
      {
        if(results->points[j].scale >= prctile)
          inliers->indices.push_back(j);
      }
      pcl::ExtractIndices<pcl::PointWithScale> extract;
      extract.setInputCloud (results);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*results);
      pcl::copyPointCloud (*results, *keys);
    }
    else if(sets->keys_mode == 2)
    {
      // Harris
      pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI, pcl::Normal> harr (pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI, pcl::Normal>::ResponseMethod::HARRIS, sets->harr_rad, sets->harr_tau);
      pcl::PointCloud<pcl::PointXYZI> result;
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
      harr.setSearchMethod(tree);
      harr.setNonMaxSupression(sets->harr_maxs);
      harr.setRefine(sets->harr_ref);
      harr.setInputCloud(filt);
      harr.setNormals(norms);
      harr.compute(result);
      pcl::copyPointCloud (result, *keys);
    }

    if(sets->keys_dist > 0)
    {
      // remove close points
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
      for(size_t i = 0;i < keys->size();i++)
      {
        bool check = true;
        
        for(size_t j = i+1;j < keys->size();j++)
          if(pow(keys->points[i].x - keys->points[j].x, 2) + pow(keys->points[i].y - keys->points[j].y, 2) + pow(keys->points[i].z - keys->points[j].z, 2) < pow(sets->keys_dist, 2))
            check = false;
        
        if(check)
          inliers->indices.push_back(i);
      }
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (keys);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*keys);
    }
  }
  if(c_disp)
    cout << "Keypoints: " << keys->points.size() << endl;

  // get features
  if(sets->feat_mode == 1)
  {
    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree3 (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    fpfh_est.setSearchMethod (tree3);
    fpfh_est.setRadiusSearch (sets->fpfh_r);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keys_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud (*keys, *keys_rgb);
    fpfh_est.setSearchSurface (filt);
    fpfh_est.setInputNormals (norms);  
    fpfh_est.setInputCloud (keys_rgb);
    fpfh_est.compute (*feats_fpfh);

    if(c_disp)
      cout << "FPFH Features Calculated\n";
  }
  else if(sets->feat_mode == 2)
  {
    pcl::PointCloud<pcl::IntensityGradient>::Ptr grads (new pcl::PointCloud<pcl::IntensityGradient>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clt (new pcl::PointCloud<pcl::PointXYZI>);
    PointCloudXYZRGBtoXYZI(*filt, *clt);

    pcl::IntensityGradientEstimation < pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient, pcl::common::IntensityFieldAccessor<pcl::PointXYZI> > ge;
    ge.setInputCloud(clt);
    ge.setInputNormals(norms);
    ge.setRadiusSearch(sets->rift_igr);
    ge.compute(*grads);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, RIFT_DESC> rift;
    pcl::PointCloud<pcl::PointXYZI>::Ptr keysi (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud (*keys, *keysi);
    rift.setInputCloud(keysi);
    rift.setSearchSurface(clt);
    rift.setSearchMethod(kdtree);
    rift.setInputGradient(grads);
    rift.setRadiusSearch(sets->rift_r);
    rift.setNrDistanceBins(dist_bins);
    rift.setNrGradientBins(grad_bins);

    rift.compute(*feats_rift);

    if(c_disp)
      cout << "RIFT Features Calculated\n";
  }
  else if(sets->feat_mode == 3)
  {
    pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> pfhrgb_est;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree3 (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pfhrgb_est.setSearchMethod (tree3);
    pfhrgb_est.setRadiusSearch (sets->pfhc_r);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keys_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud (*keys, *keys_rgb);
    pfhrgb_est.setSearchSurface (filt);
    pfhrgb_est.setInputNormals (norms);
    pfhrgb_est.setInputCloud (keys_rgb);
    pfhrgb_est.compute (*feats_pfhc);

    if(c_disp)
      cout << "PFHRGB Features Calculated\n";
  }

  if(show_filt)
    copyPointCloud(*filt, *cloud);
}
void FuseAlg::match_fpfh(pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fs, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fd, pcl::CorrespondencesPtr cor)
{
  pcl::Correspondences cort1;
  pcl::KdTreeFLANN<pcl::FPFHSignature33> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (fd);
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < fs->size (); ++i)
  {
    int found = descriptor_kdtree.nearestKSearch (fs->at(i), k, k_indices, k_squared_distances);
    if(found == 1)
    {
      pcl::Correspondence ct (k_indices[0], static_cast<int> (i), k_squared_distances[0]);
      cort1.push_back(ct);
    }
  }

  pcl::Correspondences cort2;
  pcl::KdTreeFLANN<pcl::FPFHSignature33> descriptor_kdtree2;
  descriptor_kdtree2.setInputCloud (fs);
  std::vector<int> k_indices2 (k);
  std::vector<float> k_squared_distances2 (k);
  for (size_t i = 0; i < fd->size (); ++i)
  {
    int found = descriptor_kdtree2.nearestKSearch (fd->at(i), k, k_indices2, k_squared_distances2);
    if(found == 1)
    {
      pcl::Correspondence ct (k_indices2[0], static_cast<int> (i), k_squared_distances2[0]);
      cort2.push_back(ct);
    }
  }

  for(size_t i = 0;i < cort1.size();i++)
    for(size_t j = 0;j < cort2.size();j++)
      if(cort1[i].index_match == cort2[j].index_query && cort1[i].index_query == cort2[j].index_match)
      {
        corr->push_back(cort1[i]);
      }

  if(c_disp)
    cout << "Initial Matches: " << cor->size() << endl;
}
void FuseAlg::match_rift(pcl::PointCloud<RIFT_DESC>::Ptr &fs, pcl::PointCloud<RIFT_DESC>::Ptr &fd, pcl::CorrespondencesPtr cor)
{
  pcl::Correspondences cort1;
  for (size_t i = 0; i < fs->size (); ++i)
  {
    int found = -1;
    float val = 1000000;
    for(size_t j = 0;j < fd->size();j++)
    {
      float tval = 0, tval2 = 0;
      for(int k = 0;k < dist_bins*grad_bins;k++)
      {
        tval += pow(fs->at(i).histogram[k] - fd->at(j).histogram[k], 2);
        tval2 += pow(fs->at(i).histogram[k], 2) + pow(fd->at(j).histogram[k], 2);
      }
      
      if(tval2 > 0 && tval / tval2 < val)
      {
        found = j;
        val = tval / tval2;
      }
    }

    if(found >= 0)
    {
      pcl::Correspondence ct (found, static_cast<int> (i), val);
      corr->push_back(ct);
    }
  }

  if(c_disp)
    cout << "Initial Matches: " << cor->size() << endl;
}
void FuseAlg::match_pfhc(pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fs, pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fd, pcl::CorrespondencesPtr cor)
{
  pcl::Correspondences cort1;
  pcl::KdTreeFLANN<pcl::PFHRGBSignature250> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (fd);
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < fs->size (); ++i)
  {
    int found = descriptor_kdtree.nearestKSearch (fs->at(i), k, k_indices, k_squared_distances);
    if(found == 1)
    {
      pcl::Correspondence ct (k_indices[0], static_cast<int> (i), k_squared_distances[0]);
      cort1.push_back(ct);
    }
  }

  pcl::Correspondences cort2;
  pcl::KdTreeFLANN<pcl::PFHRGBSignature250> descriptor_kdtree2;
  descriptor_kdtree2.setInputCloud (fs);
  std::vector<int> k_indices2 (k);
  std::vector<float> k_squared_distances2 (k);
  for (size_t i = 0; i < fd->size (); ++i)
  {
    int found = descriptor_kdtree2.nearestKSearch (fd->at(i), k, k_indices2, k_squared_distances2);
    if(found == 1)
    {
      pcl::Correspondence ct (k_indices2[0], static_cast<int> (i), k_squared_distances2[0]);
      cort2.push_back(ct);
    }
  }

  for(size_t i = 0;i < cort1.size();i++)
    for(size_t j = 0;j < cort2.size();j++)
      if(cort1[i].index_match == cort2[j].index_query && cort1[i].index_query == cort2[j].index_match)
      {
        corr->push_back(cort1[i]);
      }

  if(c_disp)
    cout << "Initial Matches: " << cor->size() << endl;
}
Eigen::Matrix<float, 4, 4> FuseAlg::register_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr &ms, pcl::PointCloud<pcl::PointXYZ>::Ptr &md, pcl::CorrespondencesPtr cor)
{
  // remove matched keypoints that are far away from each other
  pcl::CorrespondencesPtr corf;
  for(size_t i = 0;i < cor->size();i++)
  {
    float dist = std::sqrt(std::pow(ms->points[inlie->at(i).index_match].x - ms->points[inlie->at(i).index_query].x, 2) + std::pow(ms->points[inlie->at(i).index_match].y - ms->points[inlie->at(i).index_query].y, 2) + std::pow(ms->points[inlie->at(i).index_match].z - ms->points[inlie->at(i).index_query].z, 2));
    if(dist <= sets->corr_dist)
      corf->push_back(cor->at(i));
  }

  // Refine points
  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> sac;
  sac.setInputSource(ms);
  sac.setInputTarget(md);
  sac.setInlierThreshold(sets->corr_eps);
  sac.setMaximumIterations(sets->corr_n);
  sac.setInputCorrespondences(corf);
  sac.getRemainingCorrespondences(*corf, *inlie);

  // rescale clouds
  float scs = 0; // ms scale measurement
  for(size_t i = 0;i < inlie->size()-1;i++)
    for(size_t j = i+1;j < inlie->size();j++)
      scs += std::sqrt(std::pow(ms->points[inlie->at(i).index_match].x - ms->points[inlie->at(j).index_match].x, 2) + std::pow(ms->points[inlie->at(i).index_match].y - ms->points[inlie->at(j).index_match].y, 2) + std::pow(ms->points[inlie->at(i).index_match].z - ms->points[inlie->at(j).index_match].z, 2));
  float scd = 0; // md scale measurement
  for(size_t i = 0;i < inlie->size()-1;i++)
    for(size_t j = i+1;j < inlie->size();j++)
      scd += std::sqrt(std::pow(md->points[inlie->at(i).index_query].x - md->points[inlie->at(j).index_query].x, 2) + std::pow(md->points[inlie->at(i).index_query].y - md->points[inlie->at(j).index_query].y, 2) + std::pow(md->points[inlie->at(i).index_query].z - md->points[inlie->at(j).index_query].z, 2));
  // normalize
  Eigen::Affine3f rs = Eigen::Affine3f::Identity();
  float scale = scd / scs;
  // comment out to disable scaling
  //rs.scale(scale);
  cout << "Estimated Scale: " << scale << endl;
  // rescale source
  pcl::PointCloud<pcl::PointXYZ>::Ptr mt (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud (*ms, *mt, rs);

  if(c_disp)
    cout << "Matches: " << inlie->size() << "\n";

  // find transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr msr (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr mdr (new pcl::PointCloud<pcl::PointXYZ>());
  for(size_t i = 0;i < inlie->size();i++)
  {
    msr->push_back(mt->points[inlie->at(i).index_match]);
    mdr->push_back(md->points[inlie->at(i).index_query]);
  }
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans;
  Eigen::Matrix4f tr;
  trans.estimateRigidTransformation(*msr, *mdr, tr);



  return tr;
}


int FuseAlg::run(std::vector<char*> argv, Eigen::Matrix<float, 4, 4> &s2d)
{
  normals = false;
  ext_points = false;
  trans = true;
  show_corr = false;
  show_filt = false;
  c_disp = false;
  col_diff = false;
  keys = true;
  key_sphere = false;
  disp_view = true;
  fuse_en = true;
  icp_en = false;
  scan_val = 0;
  size_t i = 1;
  while(i < argv.size())
  {
      scan_val = parseArgument(argv[i]);
      i++;
  }
  if(file1.empty())
      file1 = "run3.pcd";
  if(file2.empty())
      file2 = "run4.pcd";

  // load point clouds
  pcl::PCDReader reader;
  reader.read<pcl::PointXYZRGB> ("../dat/fuse_dataset/" + file1, *cloud1);
  reader.read<pcl::PointXYZRGB> ("../dat/fuse_dataset/" + file2, *cloud2);

  if(ext_points)
  {
    pcl::PointCloud<pcl::PointXYZ> temp1;
    pcl::PointCloud<pcl::PointXYZ> temp2;
    reader.read<pcl::PointXYZ> ("../dat/match/" + file1, temp1);
    reader.read<pcl::PointXYZ> ("../dat/match/" + file2, temp2);
    copyPointCloud(temp1, *keys1);
    copyPointCloud(temp2, *keys2);
    pcl::transformPointCloud (*keys2, *keys2, sets->randtr);
  }
  pcl::transformPointCloud (*cloud2, *cloud2, sets->randtr);
  

  if(icp_en || !fuse_en)
  {
    sets->feat_mode = 0;
    s2d = sets->randtr;
    if(!fuse_en)
      sets->keys_mode = 0;
  }
  if(!sets->use_thread)
  {
    if(c_disp)
      cout << "Cloud 1\n";
    key_detect(cloud1, keys1, norms1, fpfh1, rift1, pfhc1);

    if(c_disp)
      cout << "\nCloud 2\n";
    key_detect(cloud2, keys2, norms2, fpfh2, rift2, pfhc2);
  }
  else
  {
    if(c_disp)
      cout << "Compute Keypoints and Features\n";
    bool temp = c_disp;
    c_disp = false;
    boost::thread t1(&FuseAlg::key_detect, this, boost::ref(cloud1), boost::ref(keys1), boost::ref(norms1), boost::ref(fpfh1), boost::ref(rift1), boost::ref(pfhc1));
    boost::thread t2(&FuseAlg::key_detect, this, boost::ref(cloud2), boost::ref(keys2), boost::ref(norms2), boost::ref(fpfh2), boost::ref(rift2), boost::ref(pfhc2));

    t1.join();
    t2.join();
    c_disp = temp;
    if(c_disp)
      cout << "Cloud 1 Keypoints: " << keys1->size() << "\nCloud 2 Keypoints: " << keys2->size() << endl;
  }
  if(fuse_en && !icp_en)
  {
    if(c_disp)
      cout << "\nRegister Clouds\n";
    if(sets->feat_mode == 1)
      match_fpfh(fpfh2, fpfh1, corr);
    else if(sets->feat_mode == 2)
      match_rift(rift2, rift1, corr);
    else if(sets->feat_mode == 3)
      match_pfhc(pfhc2, pfhc1, corr);
    s2d = register_clouds(keys2, keys1, corr);

    if(trans)
    {
      pcl::transformPointCloud (*cloud2, *cloud2, s2d);
      pcl::transformPointCloud (*keys2, *keys2, s2d);
    }
  }
  else if(icp_en)
  {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(keys2);
    icp.setInputTarget(keys1);
    icp.align(*keys2);
    s2d = icp.getFinalTransformation();
    if(trans)
    {
      pcl::transformPointCloud (*cloud2, *cloud2, s2d);
      //pcl::transformPointCloud (*keys2, *keys2, s2d);
    }
  }

  return 0;
}





int FuseAlg::parseArgument(char* arg)
{
	int option;
	float foption;
	char buf[1000];
	
	
  if(scan_val == 0)
  {
    if(1==sscanf(arg,"=%d", &option))
    {
        
      return 0;
    }
    if(1==sscanf(arg,"=%f", &foption))
    {
        
      return 0;
    }
    if(1==sscanf(arg,"fuse_en=%d", &option))
    {
      fuse_en = option == 1;
      return 0;
    }
    if(1==sscanf(arg,"icp_en=%d", &option))
    {
      icp_en = option == 1;
      return 0;
    }
    if(1==sscanf(arg,"-%s", buf))
    {
      if(strcmp(buf, "c") == 0 || strcmp(buf, "-colors") == 0)
      {
        col_diff = !col_diff;
        return 0;
      }
      if(strcmp(buf, "v") == 0 || strcmp(buf, "-viewer") == 0)
      {
        disp_view = !disp_view;
        return 0;
      }
      if(strcmp(buf, "k") == 0 || strcmp(buf, "-keys") == 0)
      {
        keys = !keys;
        return 0;
      }
      if(strcmp(buf, "s") == 0 || strcmp(buf, "-sphere") == 0)
      {
        key_sphere = !key_sphere;
        return 0;
      }
      if(strcmp(buf, "d") == 0 || strcmp(buf, "-console") == 0)
      {
        c_disp = !c_disp;
        return 0;
      }
      if(strcmp(buf, "f") == 0 || strcmp(buf, "-filt") == 0)
      {
        show_filt = !show_filt;
        return 0;
      }
      if(strcmp(buf, "l") == 0 || strcmp(buf, "-lines") == 0)
      {
        show_corr = !show_corr;
        return 0;
      }
      if(strcmp(buf, "t") == 0 || strcmp(buf, "-transform") == 0)
      {
        trans = !trans;
        return 0;
      }
      if(strcmp(buf, "e") == 0 || strcmp(buf, "-external") == 0)
      {
        ext_points = !ext_points;
        return 0;
      }
      if(strcmp(buf, "n") == 0 || strcmp(buf, "-normals") == 0)
      {
        normals = !normals;
        return 0;
      }
    }

    if(file1.empty())
    {
        file1 = arg;
        return 0;
    }
    if(file2.empty())
    {
        file2 = arg;
        return 0;
    }
  }/*
  else if(scan_val == 1)
  {
    sscanf(arg, "%s",buf);
    filename = buf;
    return 0;
  }*/


	printf("could not parse argument \"%s\"!!!!\n", arg);
  return 0;
}
