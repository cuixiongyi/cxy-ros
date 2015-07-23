
#include "kinematic/cxy_icp_kinematic_point.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {

        template<typename _Scalar>
        cxy_icp_kinematic_point<_Scalar>::cxy_icp_kinematic_point(const cxy_config* const config_ptr,
                                                                    const int& joint_idx)
                : config_(config_ptr)
                , joint_idx_(joint_idx)
        {

        }


        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::init()
        {

        }

        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::computePointResidual()
        {

        }

        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::computePointJacobian()
        {

        }




        template<typename _Scalar>
        const _Scalar cxy_icp_kinematic_point<_Scalar>::matchPointCloud(const PointT& model
                                                                        , const pcl::KdTreeFLANN<PointT>::Ptr& kdtreeptr
                                                                        , const PointT& data
                                                                        , Eigen::Matrix< _Scalar, 3, 1>& res)
        {


            static const int K(1);
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            /// kdtreeptr_ is updated by setModelCloud
            if (nullptr == kdtreeptr)
                ROS_INFO("nullptr kdtree");

            if ( 0 == kdtreeptr->nearestKSearch (model, K, pointIdxNKNSearch, pointNKNSquaredDistance)  )
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
            res(0) = data.x - model.x;
            res(1) = data.y - model.y;
            res(2) = data.z - model.z;
            //int signTmp = 1;//(res(0)+res(1)+res(2)) > 0 ? 1 : -1;
            //ROS_INFO_STREAM(rtmp);

            return std::sqrt(pointNKNSquaredDistance[0]);
        }
    }
}

template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_point<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_point<double>;

