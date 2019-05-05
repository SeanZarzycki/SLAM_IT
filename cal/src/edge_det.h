

#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>


namespace fuse
{
    template <typename PointT> inline void
    flipNormalTowardsViewpoint (const PointT &point, float vp_x, float vp_y, float vp_z,
                            float &nx, float &ny, float &nz);
    template <typename PointT, typename Scalar> inline void
    flipNormalTowardsViewpoint (const PointT &point, float vp_x, float vp_y, float vp_z,
                            Eigen::Matrix<Scalar, 3, 1>& normal);
    template<typename PointNT> inline bool
    flipNormalTowardsNormalsMean ( pcl::PointCloud<PointNT> const &normal_cloud,
                                 std::vector<int> const &normal_indices,
                                 Eigen::Vector3f &normal);
    template <typename Matrix, typename Vector> inline void
    maxeig33 (const Matrix& mat, typename Matrix::Scalar& eigenvalue, Vector& eigenvector);
    inline void
    solveLineParameters (const Eigen::Matrix3f &covariance_matrix,
                            float &nx, float &ny, float &nz, float &curvature);                           
    inline void
    solveLineParameters (const Eigen::Matrix3f &covariance_matrix,
                            const Eigen::Vector4f &point,
                            Eigen::Vector4f &plane_parameters, float &curvature);

    class EdgeDetection : public pcl::Feature<pcl::PointXYZRGB, pcl::Normal>
    {
        public:
            typedef pcl::PointXYZRGB PointInT;
            typedef pcl::Normal PointOutT;

            typedef boost::shared_ptr<EdgeDetection> Ptr;
            typedef boost::shared_ptr<const EdgeDetection> ConstPtr;

            using pcl::Feature<PointInT, PointOutT>::feature_name_;
            using pcl::Feature<PointInT, PointOutT>::getClassName;
            using pcl::Feature<PointInT, PointOutT>::indices_;
            using pcl::Feature<PointInT, PointOutT>::input_;
            using pcl::Feature<PointInT, PointOutT>::surface_;
            using pcl::Feature<PointInT, PointOutT>::k_;
            using pcl::Feature<PointInT, PointOutT>::search_radius_;
            using pcl::Feature<PointInT, PointOutT>::search_parameter_;
            
            typedef typename pcl::Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
            typedef typename pcl::Feature<PointInT, PointOutT>::PointCloudConstPtr PointCloudConstPtr;


            /** \brief Empty constructor. */
            EdgeDetection () 
            : vpx_ (0)
            , vpy_ (0)
            , vpz_ (0)
            , covariance_matrix_ ()
            , xyz_centroid_ ()
            , use_sensor_origin_ (true)
            {
            feature_name_ = "EdgeDetection";
            };

            /** \brief Empty destructor */
            virtual ~EdgeDetection () {}

            /** \brief Compute the Least-Squares plane fit for a given set of points, using their indices,
            * and return the estimated plane parameters together with the surface curvature.
            * \param cloud the input point cloud
            * \param indices the point cloud indices that need to be used
            * \param plane_parameters the plane parameters as: a, b, c, d (ax + by + cz + d = 0)
            * \param curvature the estimated surface curvature as a measure of
            * \f[
            * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
            * \f]
            */
            template <typename PointT> inline bool
            computePointEdge (const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices,
                                Eigen::Vector4f &plane_parameters, float &curvature)
            {
                if (indices.size () < 3 ||
                    computeMeanAndCovarianceMatrix (cloud, indices, covariance_matrix_, xyz_centroid_) == 0)
                {
                    plane_parameters.setConstant (std::numeric_limits<float>::quiet_NaN ());
                    curvature = std::numeric_limits<float>::quiet_NaN ();
                    return false;
                }
                
                // Get the plane normal and surface curvature
                solveLineParameters (covariance_matrix_, xyz_centroid_, plane_parameters, curvature);
                return true;
            }

        /** \brief Compute the Least-Squares plane fit for a given set of points, using their indices,
            * and return the estimated plane parameters together with the surface curvature.
            * \param cloud the input point cloud
            * \param indices the point cloud indices that need to be used
            * \param nx the resultant X component of the plane normal
            * \param ny the resultant Y component of the plane normal
            * \param nz the resultant Z component of the plane normal
            * \param curvature the estimated surface curvature as a measure of
            * \f[
            * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
            * \f]
            */
            inline bool
            computePointEdge (const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices,
                                float &nx, float &ny, float &nz, float &curvature)
            {
                if (indices.size () < 3 ||
                    computeMeanAndCovarianceMatrix (cloud, indices, covariance_matrix_, xyz_centroid_) == 0)
                {
                    nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN ();
                    return false;
                }

                // Get the plane normal and surface curvature
                solveLineParameters (covariance_matrix_, nx, ny, nz, curvature);
                return true;
            }

        /** \brief Provide a pointer to the input dataset
            * \param cloud the const boost shared pointer to a PointCloud message
            */
            virtual inline void 
            setInputCloud (const PointCloudConstPtr &cloud)
            {
                input_ = cloud;
                if (use_sensor_origin_)
                {
                    vpx_ = input_->sensor_origin_.coeff (0);
                    vpy_ = input_->sensor_origin_.coeff (1);
                    vpz_ = input_->sensor_origin_.coeff (2);
                }
            }
        
        /** \brief Set the viewpoint.
            * \param vpx the X coordinate of the viewpoint
            * \param vpy the Y coordinate of the viewpoint
            * \param vpz the Z coordinate of the viewpoint
            */
            inline void
            setViewPoint (float vpx, float vpy, float vpz)
            {
                vpx_ = vpx;
                vpy_ = vpy;
                vpz_ = vpz;
                use_sensor_origin_ = false;
            }

        /** \brief Get the viewpoint.
            * \param [out] vpx x-coordinate of the view point
            * \param [out] vpy y-coordinate of the view point
            * \param [out] vpz z-coordinate of the view point
            * \note this method returns the currently used viewpoint for normal flipping.
            * If the viewpoint is set manually using the setViewPoint method, this method will return the set view point coordinates.
            * If an input cloud is set, it will return the sensor origin otherwise it will return the origin (0, 0, 0)
            */
            inline void
            getViewPoint (float &vpx, float &vpy, float &vpz)
            {
                vpx = vpx_;
                vpy = vpy_;
                vpz = vpz_;
            }

        /** \brief sets whether the sensor origin or a user given viewpoint should be used. After this method, the 
            * normal estimation method uses the sensor origin of the input cloud.
            * to use a user defined view point, use the method setViewPoint
            */
            inline void
            useSensorOriginAsViewPoint ()
            {
                use_sensor_origin_ = true;
                if (input_)
                {
                    vpx_ = input_->sensor_origin_.coeff (0);
                    vpy_ = input_->sensor_origin_.coeff (1);
                    vpz_ = input_->sensor_origin_.coeff (2);
                }
                else
                {
                    vpx_ = 0;
                    vpy_ = 0;
                    vpz_ = 0;
                }
            }
        
        protected:
        /** \brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
            * setSearchSurface () and the spatial locator in setSearchMethod ()
            * \note In situations where not enough neighbors are found, the normal and curvature values are set to NaN.
            * \param output the resultant point cloud model dataset that contains surface normals and curvatures
            */
            void
            computeFeature (PointCloudOut &output);

        /** \brief Values describing the viewpoint ("pinhole" camera model assumed). For per point viewpoints, inherit
            * from NormalEstimation and provide your own computeFeature (). By default, the viewpoint is set to 0,0,0. */
            float vpx_, vpy_, vpz_;

        /** \brief Placeholder for the 3x3 covariance matrix at each surface patch. */
            EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix_;

        /** \brief 16-bytes aligned placeholder for the XYZ centroid of a surface patch. */
            Eigen::Vector4f xyz_centroid_;
        
        /** whether the sensor origin of the input cloud or a user given viewpoint should be used.*/
            bool use_sensor_origin_;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
