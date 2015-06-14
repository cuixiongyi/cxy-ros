#include "cxy_icp_rigid_func.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {
        template<typename _Scalar, int NX, int NY>
        cxy_icp_rigid_func<_Scalar,NX,NY>::cxy_icp_rigid_func(pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud
                                               , pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud
                                               , pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeptr)
                : cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar, NX, NY>(7, dataCloud->size())
        {
            modelCloud_ = modelCloud;
            dataCloud_ = dataCloud;
            kdtreeptr_ = kdtreeptr;
        }

        template<typename _Scalar, int NX, int NY>
        _Scalar cxy_icp_rigid_func<_Scalar,NX,NY>::operator()(ParaType  &x, ResidualType &fvec) const
        {
            static int ac = 0;
            //ROS_INFO_STREAM("Call f the 1   "<<++ac);
            std::vector<_Scalar> vPara(7);
            vPara[0] = x(0);
            vPara[1] = x(1);
            vPara[2] = x(2);
            vPara[3] = x(3);
            vPara[4] = x(4);
            vPara[5] = x(5);
            vPara[6] = x(6);
            //ROS_INFO_STREAM("Call f the 2   "<<ac);
            std::cout<<x(0)<<" "<<x(1)<<" "<<x(2)<<"  q= "<<x(3)<<" "<<x(4)<<" "<<x(5)<<" "<<x(6)<<std::endl;
            _Scalar res(0.0);
            for (unsigned int ii = 0; ii < dataCloud_->size(); ++ii)
            {
                pcl::PointXYZ transPoint;
                cxy_transform::Pose::composePoint((*dataCloud_)[ii], transPoint, vPara);
                Eigen::Matrix< _Scalar, 3, 1> r3;
                fvec[ii] = matchPointCloud(transPoint, r3);
                res += fvec[ii] / this->values();

            }
            //ROS_INFO_STREAM("Call f the 3    "<<ac<<" time. Residual =  "<< res);

            x[3] = vPara[3];
            x[4] = vPara[4];
            x[5] = vPara[5];
            x[6] = vPara[6];
            return res;
        }

        template<typename _Scalar, int NX, int NY>
        _Scalar cxy_icp_rigid_func<_Scalar,NX,NY>::df(ParaType  &x, JacobianType &fjac) const
        {
            std::vector<_Scalar> vPara(7);
            vPara[0] = x(0);
            vPara[1] = x(1);
            vPara[2] = x(2);
            vPara[3] = x(3);
            vPara[4] = x(4);
            vPara[5] = x(5);
            vPara[6] = x(6);

            for (unsigned int ii = 0; ii < dataCloud_->size(); ++ii)
            {
                pcl::PointXYZ transPoint;
                cxy_transform::Pose::composePoint((*dataCloud_)[ii], transPoint, vPara);
                Eigen::Matrix< _Scalar, 3, 1> r3;
                matchPointCloud(transPoint, r3);
                
                Matrix34f jac34(calculateJacobianKernel(vPara
                                                        , (*dataCloud_)[ii]));
                if (ii == 700)
                {
                    //std::cout<<ii<<" = "<<(*dataCloud_)[ii].x<<"  "<<(*dataCloud_)[ii].y<<"  "<<(*dataCloud_)[ii].z<<std::endl;
                    std::cout<<ii<<" = "<<jac34(0)<<"  "<<jac34(1)<<"  "<<jac34(2)<<std::endl;
                    std::cout<<ii<<" = "<<vPara[0]<<"  "<<vPara[1]<<"  "<<vPara[2]<<"  "<<vPara[3]<<"  "<<vPara[4]<<"  "<<vPara[5]<<"  "<<vPara[6]<<std::endl;
                }
                Eigen::Matrix<_Scalar, 1, 4> jq(r3.transpose()*jac34);
                //rowJ<<r3(0), r3(1), r3(2),  jq(0), jq(1), jq(2), jq(3);
                fjac(ii, 0) = r3(0, 0);
                fjac(ii, 1) = r3(1, 0);
                fjac(ii, 2) = r3(2, 0);
                fjac(ii, 3) = jq(0);
                fjac(ii, 4) = jq(1);
                fjac(ii, 5) = jq(2);
                fjac(ii, 6) = jq(3);
                    
                    //std::cout<<ii<<" = "<<r3(0, 0)<<"  "<<r3(1, 0)<<"  "<<r3(2, 0)<<"  "<<jq(0, 0)<<"  "<<jq(0,1)<<"  "<<jq(0,2)<<"  "<<jq(0,3)<<std::endl;
                    
            }
            x[3] = vPara[3];
            x[4] = vPara[4];
            x[5] = vPara[5];
            x[6] = vPara[6];

            return 1;
        }

        template<typename _Scalar, int NX, int NY>
        const _Scalar cxy_icp_rigid_func<_Scalar,NX,NY>::matchPointCloud(const PointT& data
                                                   , Eigen::Matrix< _Scalar, 3, 1>& res) const
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

            /*if (pointNKNSquaredDistance[0] > max_correspondence_dist_square_)
            {
                return std::nanf("");
            }
            */
            /// Use Euclidean distance
            const pcl::PointXYZ& pTmp((*modelCloud_)[pointIdxNKNSearch[0]]);
            //ROS_INFO_STREAM("closet point = "<<pTmp.x<<" "<<pTmp.y<<" "<<pTmp.z);
            //ROS_INFO_STREAM(" ");
            res(0) = pTmp.x - data.x;
            res(1) = pTmp.y - data.y;
            res(2) = pTmp.z - data.z;
            //int signTmp = 1;//(res(0)+res(1)+res(2)) > 0 ? 1 : -1;
            //ROS_INFO_STREAM(rtmp);

            return sqrt(pointNKNSquaredDistance[0]);
        }


        template<typename _Scalar, int NX, int NY>
        const Eigen::Matrix< _Scalar, 3, 4> cxy_icp_rigid_func<_Scalar,NX,NY>::calculateJacobianKernel(const std::vector<_Scalar> &para
                                                               , const pcl::PointXYZ& a) const
        {

            Eigen::Quaternionf q(para[3], para[4], para[5], para[6]);
            Matrix34f jacQuat;
            jacQuat.setZero();

            jacQuat << (2*a.z*q.y() - 2*a.y*q.z()) , (2*a.y*q.y() + 2*a.z*q.z())                ,(2*a.z*q.w() - 4*a.x*q.y() + 2*a.y*q.x()) , (2*a.z*q.x() - 4*a.x*q.z() - 2*a.y*q.w())
                    , (2*a.x*q.z() - 2*a.z*q.x()) , (2*a.x*q.y() - 2*a.z*q.w() - 4*a.y*q.x()) , (2*a.x*q.x() + 2*a.z*q.z())                 ,( 2*a.x*q.w() - 4*a.y*q.z() + 2*a.z*q.y())
                    , (2*a.y*q.x() - 2*a.x*q.y()) , (2*a.y*q.w() + 2*a.x*q.z() - 4*a.z*q.x()) , (2*a.y*q.z() - 2*a.x*q.w() - 4*a.z*q.y()) , (2*a.x*q.x() + 2*a.y*q.y());

            Matrix44f normalJaco44;
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

template class cxy::cxy_lmicp_lib::cxy_icp_rigid_func<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_rigid_func<double>;
