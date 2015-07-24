
#include "kinematic/cxy_icp_kinematic_point.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {

        template<typename _Scalar>
        cxy_icp_kinematic_point<_Scalar>::cxy_icp_kinematic_point(const cxy_config* const config_ptr
                                                                , const int& joint_idx
                                                                , const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr& kdtree
                                                                , const pcl::PointCloud<PointT>::Ptr& dataCloud)
                : config_(config_ptr)
                , joint_idx_(joint_idx)
                , kdtree_ptr_(kdtree)
                , dataCloud_(dataCloud)
        {

        }


        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::init()
        {

        }







        template<typename _Scalar>
        const _Scalar cxy_icp_kinematic_point<_Scalar>::matchPointCloud(const PointT& model
                                                                        , PointT& data
                                                                        , Eigen::Matrix< _Scalar, 3, 1>& res)
        {


            static const int K(1);
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            /// kdtreeptr_ is updated by setModelCloud
            if (nullptr == kdtree_ptr_)
                ROS_INFO("nullptr kdtree");

            if ( 0 == kdtree_ptr_->nearestKSearch (model, K, pointIdxNKNSearch, pointNKNSquaredDistance)  )
            {
                return std::nanf("");
            }

            /*if (pointNKNSquaredDistance[0] > max_correspondence_dist_square_)
            {
                return std::nanf("");
            }
            */
            /// Use Euclidean distance
            //ROS_INFO_STREAM("closet point = "<<pTmp.x<<" "<<pTmp.y<<" "<<pTmp.z);
            //ROS_INFO_STREAM(" ");
            data = ((*dataCloud_)[pointIdxNKNSearch[0]]);

            res(0) = data.x - model.x;
            res(1) = data.y - model.y;
            res(2) = data.z - model.z;
            //int signTmp = 1;//(res(0)+res(1)+res(2)) > 0 ? 1 : -1;
            //ROS_INFO_STREAM(rtmp);

            return std::sqrt(pointNKNSquaredDistance[0]);
        }


        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::computePointResidual()
        {

            /*
             * TODO add residual of other type
             */
            uint8_t ii = 0;
            if (config_->with_icp_jacobian)
            {
                point_resdual_n(0) = cxy_icp_kinematic_point::matchPointCloud(modelPoint_global_
                                                                              , dataPoint_
                                                                              , point_resdual3_);
                ++ii;
            }
            if (config_->with_collision_jacobian)
            {
                point_resdual_n(ii) = 0;
                ++ii;
            }
            if (config_->with_push_jacobian)
            {
                point_resdual_n(ii) = 0;

                ++ii;
            }
            if (config_->with_silhouette_jacobian)
            {
                point_resdual_n(ii) = 0;

                ++ii;
            }

        }

        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::computePointJacobian()
        {

        }
    }
}

template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_point<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_point<double>;

