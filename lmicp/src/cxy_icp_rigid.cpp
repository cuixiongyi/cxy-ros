#include "cxy_icp_rigid.h"
#include "../include/cxy_transform.h"

namespace cxy {
    namespace cxy_lmicp_lib {


        cxy_icp_rigid::cxy_icp_rigid() : cxy_icp()
            {


            }
        const dataType cxy_icp_rigid::residual(const dataIdxType& dataIdx
                                               , paraVectorType const& para)
        {

            pcl::PointXYZ transPoint;
            cxy_transform::Pose::composePoint((*modelCloud_)[dataIdx], transPoint, para);
            Vector3f r3;
            return matchPointCloud(transPoint, r3);
        }
        derVectorType cxy_icp_rigid::residual_derivative(const dataIdxType& dataIdx
                                                         , const paraVectorType& para)
        {
            pcl::PointXYZ transPoint;
            cxy_transform::Pose::composePoint((*modelCloud_)[dataIdx], transPoint, para);
            Vector3f r3;
            matchPointCloud(transPoint, r3);

            //const PointT& d((*dataCloud_)[dataIdx]), &m((*modelCloud_)[modelMatchIdx[ii]]);
            //Eigen::Matrix<float, 1, 7> rowJ;
            //rowJ <<(2*d.x - 2*m.x), (2*d.y - 2*m.y), (2*d.z - 2*m.z), 0, (4*d.y*(d.z - m.z) - 4*d.z*(d.y - m.y)), (4*d.z*(d.x - m.x) - 4*d.x*(d.z - m.z)), (4*d.x*(d.y - m.y) - 4*d.y*(d.x - m.x));
            //rowJ <<(2*d.x - 2*m.x), (2*d.y - 2*m.y), (2*d.z - 2*m.z), (4*d.y*(d.z - m.z) - 4*d.z*(d.y - m.y)), (4*d.z*(d.x - m.x) - 4*d.x*(d.z - m.z)), (4*d.x*(d.y - m.y) - 4*d.y*(d.x - m.x));
            //ROS_INFO_STREAM_ONCE(rowJ);

            Matrix34f jac34(calculateJacobianKernel(pose_k
                            , d));
            Eigen::Matrix<float, 1, 4> jq(r3.transpose()*jac34);
            //rowJ<<r3(0), r3(1), r3(2),  jq(0), jq(1), jq(2), jq(3);
            derVectorType derivative(7);
            derivative[0] = r3(0);
            derivative[1] = r3(1);
            derivative[2] = r3(2);
            derivative[3] = jq(0);
            derivative[4] = jq(1);
            derivative[5] = jq(2);
            derivative[6] = jq(3);
            return derivative;
        }


            float cxy_icp_rigid::matchPointCloud()
            {
                float r(0.0);
                dataMatchIdx_.clear();
                dataMatchIdx_.reserve(data->size());
                modelMatchIdx_.clear();
                modelMatchIdx_.reserve(data->size());
                matchDistance_.clear();
                matchDistance_.reserve(data->size());

                static const int K(1);
                std::vector<int> pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);
                /// kdtreeptr_ is updated by setModelCloud
                if (nullptr == kdtreeptr_)
                    ROS_INFO("nullptr kdtree");

                for (int ii = 0; ii < dataCloud_->size(); ++ii)
                {
                    if ( 0 == kdtreeptr_->nearestKSearch ((*dataCloud_)[ii], K, pointIdxNKNSearch, pointNKNSquaredDistance)  )
                    {
                        continue;
                    }
                    if (pointNKNSquaredDistance[0] > max_correspondence_dist_square_)
                    {
                        continue;
                    }
                    dataMatchIdx_.push_back(ii);
                    modelMatchIdx_.push_back(pointIdxNKNSearch[0]);
                    /// Use Euclidean distance
                    const PointT& pTmp((*modelCloud_)[pointIdxNKNSearch[0]]), p2Tmp((*dataCloud_)[ii]);
                    //int signTmp = 1;//((pTmp.x - p2Tmp.x)+(pTmp.y - p2Tmp.y)+(pTmp.z - p2Tmp.z)) > 0 ? 1 : -1;
                    float rtmp(sqrt(pointNKNSquaredDistance[0]));
                    matchDistance.push_back(pointNKNSquaredDistance[0]);
                    //ROS_INFO_STREAM(rtmp);
                    r += rtmp;

                }

                return r;
            }

            const float cxy_icp_rigid::matchPointCloud(const PointT& data
                                                       , Eigen::Vector3f& res)
            {
                static const int K(1);
                std::vector<int> pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);
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
                const PointT& pTmp((*modelCloud_)[pointIdxNKNSearch[0]]);
                ROS_INFO_STREAM("closet point = "<<pTmp.x<<" "<<pTmp.y<<" "<<pTmp.z);
                ROS_INFO_STREAM(" ");
                res(0) = pTmp.x - data.x;
                res(1) = pTmp.y - data.y;
                res(2) = pTmp.z - data.z;
                //int signTmp = 1;//(res(0)+res(1)+res(2)) > 0 ? 1 : -1;
                //ROS_INFO_STREAM(rtmp);

                return sqrt(pointNKNSquaredDistance[0]);
            }

        inline const Matrix34f cxy_icp_rigid::calculateJacobianKernel(const std::vector<float> para
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