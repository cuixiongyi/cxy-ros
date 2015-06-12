#include "cxy_icp_rigid_func.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {
        cxy_icp_rigid_func::cxy_icp_rigid_func(pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud
                                               , pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud
                                               , pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeptr)
                : cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar, NX, NY>(7, dataCloud->size())
        {
            modelCloud_ = modelCloud;
            dataCloud_ = dataCloud;
            kdtreeptr_ = kdtreeptr;
        }

        int cxy_icp_rigid_func::operator()(ParaType const &x, ResidualType &fvec) const
        {
            std::vector<_Scalar> vPara(7);
            vPara[0] = fvec(0);
            vPara[1] = fvec(1);
            vPara[2] = fvec(2);
            vPara[3] = fvec(3);
            vPara[4] = fvec(4);
            vPara[5] = fvec(5);
            vPara[6] = fvec(6);

            for (unsigned int ii = 0; ii < dataCloud_->size(); ++ii)
            {
                pcl::PointXYZ transPoint;
                cxy_transform::Pose::composePoint((*dataCloud_)[ii], transPoint, vPara);
                Vector3 r3;
                fvec(ii, 0) = matchPointCloud(transPoint, r3);

            }

            return 1;
        }

        int cxy_icp_rigid_func::df(ParaType const &x, JacobianType &fjac) const
        {
            std::vector<_Scalar> vPara(7);
            vPara[0] = fvec(0);
            vPara[1] = fvec(1);
            vPara[2] = fvec(2);
            vPara[3] = fvec(3);
            vPara[4] = fvec(4);
            vPara[5] = fvec(5);
            vPara[6] = fvec(6);

            for (unsigned int ii = 0; ii < dataCloud_->size(); ++ii)
            {
                pcl::PointXYZ transPoint;
                cxy_transform::Pose::composePoint((*dataCloud_)[ii], transPoint, vPara);
                Vector3 r3;
                matchPointCloud(transPoint, r3);
                Matrix34f jac34(calculateJacobianKernel(vPara
                        , transPoint));
                Eigen::Matrix<_Scalar, 1, 4> jq(r3.transpose()*jac34);
                //rowJ<<r3(0), r3(1), r3(2),  jq(0), jq(1), jq(2), jq(3);
                JacobianType(ii, 0) = r3(0);
                JacobianType(ii, 1) = r3(0);
                JacobianType(ii, 2) = r3(0);
                JacobianType(ii, 3) = jq(0);
                JacobianType(ii, 4) = jq(1);
                JacobianType(ii, 5) = jq(2);
                JacobianType(ii, 6) = jq(3);

            }


            return 1;
        }

        const _Scalar cxy_icp_rigid_func::matchPointCloud(const PointT& data
                                                   , Vector3& res)
        {
            static const int K(1);
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<_Scalar> pointNKNSquaredDistance(K);
            /// kdtreeptr_ is updated by setModelCloud
            if (nullptr == kdtreeptr_)
                ROS_INFO("nullptr kdtree");

            if ( 0 == kdtreeptr_->nearestKSearch (data, K, pointIdxNKNSearch, pointNKNSquaredDistance)  )
            {
                return std::nanf("");
            }
            if (pointNKNSquaredDistance[0] > max_correspondence_dist_square_)
            {
                return std::nanf("");
            }
            /// Use Euclidean distance
            const pcl::PointXYZ& pTmp((*modelCloud_)[pointIdxNKNSearch[0]]);
            ROS_INFO_STREAM("closet point = "<<pTmp.x<<" "<<pTmp.y<<" "<<pTmp.z);
            //ROS_INFO_STREAM(" ");
            res(0) = pTmp.x - data.x;
            res(1) = pTmp.y - data.y;
            res(2) = pTmp.z - data.z;
            //int signTmp = 1;//(res(0)+res(1)+res(2)) > 0 ? 1 : -1;
            //ROS_INFO_STREAM(rtmp);

            return sqrt(pointNKNSquaredDistance[0]);
        }


        const Matrix34f cxy_icp_rigid_func::calculateJacobianKernel(const std::vector<_Scalar> &para
                                                               , const pcl::PointXYZ& a)
        {

            Eigen::Quaternionf q(para[3], para[4], para[5], para[6]);
            Matrix34f jacQuat;
            jacQuat.setZero();

            jacQuat << (2*a.z*q.y() - 2*a.y*q.z()) , (2*a.y*q.y() + 2*a.z*q.z())                ,(2*a.z*q.w() - 4*a.x*q.y() + 2*a.y*q.x()) , (2*a.z*q.x() - 4*a.x*q.z() - 2*a.y*q.w())
                    , (2*a.x*q.z() - 2*a.z*q.x()) , (2*a.x*q.y() - 2*a.z*q.w() - 4*a.y*q.x()) , (2*a.x*q.x() + 2*a.z*q.z())                 ,( 2*a.x*q.w() - 4*a.y*q.z() + 2*a.z*q.y())
                    , (2*a.y*q.x() - 2*a.x*q.y()) , (2*a.y*q.w() + 2*a.x*q.z() - 4*a.z*q.x()) , (2*a.y*q.z() - 2*a.x*q.w() - 4*a.z*q.y()) , (2*a.x*q.x() + 2*a.y*q.y());

            Eigen:Matrix44f normalJaco44;
            normalJaco44.setZero();
            normalJaco44 << q.x()*q.x()+q.y()*q.y()+q.z()*q.z(), -q.w()*q.x(), -q.w()*q.y(), -q.w()*q.z(),
                    -q.x()*q.w(), q.w()*q.w()+q.y()*q.y()+q.z()*q.z(), -q.x()*q.y(), -q.x()*q.z(),
                    -q.y()*q.w(), -q.y()*q.x(), q.w()*q.w()+q.x()*q.x()+q.z()*q.z(), -q.y()*q.z(),
                    -q.z()*q.w(), -q.z()*q.x(), -q.z()*q.y(), q.w()*q.w()+q.y()*q.y()+q.x()*q.x();
            normalJaco44 = normalJaco44 / std::pow(q.w()*q.w()+q.y()*q.y()+q.x()*q.x()+q.z()*q.z(), 1.5);

            /*
             [ 2*a.z*q.y() - 2*a.y*q.z(),             2*a.y*q.y() + 2*a.z*q.z(), 2*a.z*q.w() - 4*a.x*q.y() + 2*a.y*q.x(), 2*a.z*q.x() - 4*a.x*q.z() - 2*a.y*q.w();
             [ 2*a.x*q.z() - 2*a.z*q.x(), 2*a.x*q.y() - 2*a.z*q.w() - 4*a.y*q.x(),             2*a.x*q.x() + 2*a.z*q.z(), 2*a.x*q.w() - 4*a.y*q.z() + 2*a.z*q.y()]
             [ 2*a.y*q.x() - 2*a.x*q.y(), 2*a.y*q.w() + 2*a.x*q.z() - 4*a.z*q.x(), 2*a.y*q.z() - 2*a.x*q.w() - 4*a.z*q.y(),             2*a.x*q.x() + 2*a.y*q.y()]
             */
            return jacQuat*normalJaco44;
        }
    }
}