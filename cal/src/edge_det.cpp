

#include "edge_det.h"



template <typename PointT, typename Scalar> inline void
fuse::flipNormalTowardsViewpoint (const PointT &point, float vp_x, float vp_y, float vp_z,
                            Eigen::Matrix<Scalar, 3, 1>& normal)
{
  Eigen::Matrix <Scalar, 3, 1> vp (vp_x - point.x, vp_y - point.y, vp_z - point.z);

  // Flip the plane normal
  if (vp.dot (normal) < 0)
    normal *= -1;
};
template <typename PointT> inline void
fuse::flipNormalTowardsViewpoint (const PointT &point, float vp_x, float vp_y, float vp_z,
                            float &nx, float &ny, float &nz)
{
  // See if we need to flip any plane normals
  vp_x -= point.x;
  vp_y -= point.y;
  vp_z -= point.z;

  // Dot product between the (viewpoint - point) and the plane normal
  float cos_theta = (vp_x * nx + vp_y * ny + vp_z * nz);

  // Flip the plane normal
  if (cos_theta < 0)
  {
    nx *= -1;
    ny *= -1;
    nz *= -1;
  }
};
template<typename PointNT> inline bool
fuse::flipNormalTowardsNormalsMean ( pcl::PointCloud<PointNT> const &normal_cloud,
                                std::vector<int> const &normal_indices,
                                Eigen::Vector3f &normal)
{
    Eigen::Vector3f normal_mean = Eigen::Vector3f::Zero ();

    for (size_t i = 0; i < normal_indices.size (); ++i)
    {
        const PointNT& cur_pt = normal_cloud[normal_indices[i]];

        if (pcl::isFinite (cur_pt))
        {
        normal_mean += cur_pt.getNormalVector3fMap ();
        }
    }

    if (normal_mean.isZero ())
        return false;

    normal_mean.normalize ();

    if (normal.dot (normal_mean) < 0)
    {
        normal = -normal;
    }

    return true;
};


void
fuse::EdgeDetection::computeFeature (PointCloudOut &output)
{
  // Allocate enough space to hold the results
  // \note This resize is irrelevant for a radiusSearch ().
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  output.is_dense = true;
  // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
  if (input_->is_dense)
  {
    // Iterating over the entire index vector
    for (size_t idx = 0; idx < indices_->size (); ++idx)
    {
      if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0 ||
          !computePointEdge (*surface_, nn_indices, output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2], output.points[idx].curvature))
      {
        output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

        output.is_dense = false;
        continue;
      }

      flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                  output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2]);

    }
  }
  else
  {
    // Iterating over the entire index vector
    for (size_t idx = 0; idx < indices_->size (); ++idx)
    {
      if (!isFinite ((*input_)[(*indices_)[idx]]) ||
          this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0 ||
          !computePointEdge (*surface_, nn_indices, output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2], output.points[idx].curvature))
      {
        output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

        output.is_dense = false;
        continue;
      }

      flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                  output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2]);

    }
  }
};


template <typename Matrix, typename Vector> inline void
fuse::maxeig33 (const Matrix& mat, typename Matrix::Scalar& eigenvalue, Vector& eigenvector)
{
  typedef typename Matrix::Scalar Scalar;
  // Scale the matrix so its entries are in [-1,1].  The scaling is applied
  // only when at least one matrix entry has magnitude larger than 1.

  Scalar scale = mat.cwiseAbs ().maxCoeff ();
  if (scale <= std::numeric_limits < Scalar > ::min ())
    scale = Scalar (1.0);

  Matrix scaledMat = mat / scale;

  Vector eigenvalues;
  pcl::computeRoots (scaledMat, eigenvalues);

  bool swt;
  swt = eigenvalues(2) / eigenvalues(1) >= eigenvalues(1) / eigenvalues(0);
  
  eigenvalue = eigenvalues (swt ? 0 : 2) * scale;

  scaledMat.diagonal ().array () -= eigenvalues (2);

  Vector vec1 = scaledMat.row (0).cross (scaledMat.row (1));
  Vector vec2 = scaledMat.row (0).cross (scaledMat.row (2));
  Vector vec3 = scaledMat.row (1).cross (scaledMat.row (2));

  Scalar len1 = vec1.squaredNorm ();
  Scalar len2 = vec2.squaredNorm ();
  Scalar len3 = vec3.squaredNorm ();

  if(swt)
  {
    if (len1 >= len2 && len1 >= len3)
      eigenvector = vec1 / std::sqrt (len1);
    else if (len2 >= len1 && len2 >= len3)
      eigenvector = vec2 / std::sqrt (len2);
    else
      eigenvector = vec3 / std::sqrt (len3);
  }
  else
  {
    if (len1 <= len2 && len1 <= len3)
      eigenvector = vec1 / std::sqrt (len1);
    else if (len2 <= len1 && len2 <= len3)
      eigenvector = vec2 / std::sqrt (len2);
    else
      eigenvector = vec3 / std::sqrt (len3);
  }
};


inline void
fuse::solveLineParameters (const Eigen::Matrix3f &covariance_matrix,
                           const Eigen::Vector4f &point,
                           Eigen::Vector4f &plane_parameters, float &curvature)
{
  solveLineParameters (covariance_matrix, plane_parameters [0], plane_parameters [1], plane_parameters [2], curvature);

  plane_parameters[3] = 0;
  plane_parameters[3] = -1 * plane_parameters.dot (point);
};
inline void
fuse::solveLineParameters (const Eigen::Matrix3f &covariance_matrix,
                           float &nx, float &ny, float &nz, float &curvature)
{
  // Extract the smallest eigenvalue and its eigenvector
  EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
  maxeig33 (covariance_matrix, eigen_value, eigen_vector);

  nx = eigen_vector [0];
  ny = eigen_vector [1];
  nz = eigen_vector [2];

  // Compute the curvature surface change
  float eig_sum = covariance_matrix.coeff (0) + covariance_matrix.coeff (4) + covariance_matrix.coeff (8);
  if (eig_sum != 0)
    curvature = fabsf (eigen_value / eig_sum);
  else
    curvature = 0;
};
