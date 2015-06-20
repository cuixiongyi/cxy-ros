#pragma once
#include "optimization/cxy_cost_func_abstract.h"
#include "cxy_transform.h"
#include "cxy_debug.h"
#include <cstdlib>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/kdtree/kdtree_flann.h>
#include "pcl_ros/transforms.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {


        template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
        class cxy_icp_arti_func : public cxy_optimization::Cxy_Cost_Func_Abstract< _Scalar, NX, NY>
        {

            typedef pcl::PointXYZ PointT;
            typedef pcl::PointCloud<PointT>    PointCloud;
            typedef pcl::PointCloud<PointT>::Ptr    PointCloudPtr;
            typedef pcl::PointCloud<PointT>::ConstPtr    PointCloudConstPtr;
            typedef Eigen::Matrix< _Scalar, 3, 4> Matrix34f;
            typedef Eigen::Matrix< _Scalar, 3, 7> Matrix37f;
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 7> MatrixX7f;
            typedef Eigen::Matrix< _Scalar, 7, 7> Matrix7f;
            typedef Eigen::Matrix< _Scalar, 7, 1> Vector7f;
            typedef Eigen::Matrix< _Scalar, 6, 6> Matrix6f;
            typedef Eigen::Matrix< _Scalar, 6, 1> Vector6f;
            typedef Eigen::Matrix< _Scalar, 3, 1> Vector3;
            typedef Eigen::Matrix< _Scalar, 4, 4> Matrix44f;
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;

            public:
                enum {
                    ParaAtCompileTime = NX,
                    DataAtCompileTime = NY
                };

                //: all of this is what you need in the derived function

                typedef Eigen::Matrix<_Scalar,ParaAtCompileTime,1> ParaType;
                typedef Eigen::Matrix<_Scalar,DataAtCompileTime,1> ResidualType;
                typedef Eigen::Matrix<_Scalar,DataAtCompileTime,ParaAtCompileTime> JacobianType;

                cxy_icp_arti_func(pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud
                                                   , pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud
                                                   , pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeptr)
                : cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar, NX, NY>(2, dataCloud->size())
                {
                    modelCloud_ = modelCloud;
                    dataCloud_ = dataCloud;
                    kdtreeptr_ = kdtreeptr;
                }

                _Scalar operator()(ParaType & x, ResidualType& fvec) const
                {
                    /// test manifold start
                    if (0)
                    {
                        manifold();
                    }
                    /// test manifold end

                    static int ac = 0;
                    //ROS_INFO_STREAM("Call f the 1   "<<++ac);
                    std::vector<_Scalar> vPara(7);
                    vPara[0] = 0.0;
                    vPara[1] = .0;
                    vPara[2] = .0;
                    vPara[3] = x(0);
                    vPara[4] = x(1);
                    vPara[5] = .0;
                    vPara[6] = .0;
                    
                    //ROS_INFO_STREAM("Call f the 2   "<<ac);
                    //std::cout<<x(0)<<" "<<x(1)<<" "<<x(2)<<"  q= "<<x(3)<<" "<<x(4)<<" "<<x(5)<<" "<<x(6)<<std::endl;
                    _Scalar res(0.0);
                    cxy_transform::Pose<_Scalar> pose;
                    pose.t()(0) = 0.0;
                    pose.t()(1) = 0.0;
                    pose.t()(2) = 0.0;
                    pose.q().w() = x(0);
                    pose.q().x() = x(1);
                    pose.q().y() = 0.0;
                    pose.q().z() = 0.0;
                    pose.normalize();
                    for (unsigned int ii = 0; ii < dataCloud_->size(); ++ii)
                    {
                        pcl::PointXYZ transPoint;
                        pose.composePoint((*dataCloud_)[ii], transPoint);
                        Eigen::Matrix< _Scalar, 3, 1> r3;
                        fvec[ii] = matchPointCloud(transPoint, r3);
                        res += fvec[ii] / this->values();

                    }
                    //ROS_INFO_STREAM("Call f the 3    "<<ac<<" time. Residual =  "<< res);

                    x(0) = pose.q().w();
                    x(1) = pose.q().x();
                    ROS_INFO_STREAM("Residual =  "<<x(0)<< "  "<<x(1));
                    if (1)
                    {
                        static std::ofstream fout("/home/xiongyi/repo/gradiant.txt");
                        fout<<x(0)<<" "<<x(1)<<" "<<res<<std::endl;
                    }
                    return res;
                }

                _Scalar df(ParaType & x, JacobianType& fjac) const
                {
                    std::vector<_Scalar> vPara(7);
                    vPara[0] = 0.0;
                    vPara[1] = .0;
                    vPara[2] = .0;
                    vPara[3] = x(0);
                    vPara[4] = x(1);
                    vPara[5] = .0;
                    vPara[6] = .0;
                    

                    /*cxy_transform::Pose pose;
                    pcl::PointCloud<PointXYZ>::Ptr transPoint(new pcl::PointCloud<PointXYZ>);
                    pose.rotateByAxis(cxy_transform::Axis::X_axis, x(0));
                    pose.composePoint(dataCloud_, transPoint);
                    Eigen::Matrix< _Scalar, 3, 1> r3;
                    for (unsigned int ii = 0; ii < dataCloud_->size(); ++ii)
                    {
                        fvec[ii] = matchPointCloud(transPoint[ii], r3);
                        res += fvec[ii] / this->values();
*/
                    cxy_transform::Pose<_Scalar> pose;
                    pose.t()(0) = 0.0;
                    pose.t()(1) = 0.0;
                    pose.t()(2) = 0.0;
                    pose.q().w() = x(0);
                    pose.q().x() = x(1);
                    pose.q().y() = 0.0;
                    pose.q().z() = 0.0;
                    pose.normalize();
                    vPara[0] = 0.0;
                    vPara[1] = .0;
                    vPara[2] = .0;
                    vPara[3] = pose.q().w();
                    vPara[4] = pose.q().x();
                    vPara[5] = .0;
                    vPara[6] = .0;
                    for (unsigned int ii = 0; ii < dataCloud_->size(); ++ii)
                    {
                        pcl::PointXYZ transPoint;
                        pose.composePoint((*dataCloud_)[ii], transPoint);
                        Eigen::Matrix< _Scalar, 3, 1> r3;
                        matchPointCloud(transPoint, r3);


                        //Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> r3;
                        
                        Matrix34f jac34(calculateJacobianKernel(vPara
                                                                , (*dataCloud_)[ii]));
                        if (ii == 700)
                        {
                            //std::cout<<ii<<" = "<<(*dataCloud_)[ii].x<<"  "<<(*dataCloud_)[ii].y<<"  "<<(*dataCloud_)[ii].z<<std::endl;
                            //std::cout<<ii<<" = "<<jac34(0)<<"  "<<jac34(1)<<"  "<<jac34(2)<<std::endl;
                            //std::cout<<ii<<" = "<<vPara[0]<<"  "<<vPara[1]<<"  "<<vPara[2]<<"  "<<vPara[3]<<"  "<<vPara[4]<<"  "<<vPara[5]<<"  "<<vPara[6]<<std::endl;
                        }
                        ROS_INFO_STREAM(jac34);
                        ROS_INFO(" ");
                        Eigen::Matrix<_Scalar, 1, 4> jq(r3.transpose()*jac34);
                        //rowJ<<r3(0), r3(1), r3(2),  jq(0), jq(1), jq(2), jq(3);
                        /*fjac(ii, 0) = r3(0);
                        fjac(ii, 1) = r3(1);
                        fjac(ii, 2) = r3(2);
                        fjac(ii, 3) = jq(0);
                        fjac(ii, 4) = jq(1);
                        fjac(ii, 5) = jq(2);
                        fjac(ii, 6) = jq(3);*/
                        fjac(ii, 0) = jq(0);
                        fjac(ii, 1) = jq(1);

                            
                        if (ii == 20)
                            std::cout<<"dev = "<<jq(0, 0)<<"  "<<jq(0,1)<<"  "<<jq(0,2)<<"  "<<jq(0,3)<<std::endl;
                            
                    }
                    x(0) = pose.q().w();
                    x(1) = pose.q().x();
                    //std::cout<<"wx = "<<x(0)<<"  "<<x(1)<<"  "<<std::endl;
                    return 1;
                }

                const _Scalar matchPointCloud(const PointT& data
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

                    return std::sqrt(pointNKNSquaredDistance[0]);
                }


                const Eigen::Matrix< _Scalar, 3, 4> calculateJacobianKernel(const std::vector<_Scalar> &para
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
                void manifold()
                {

                        std::ofstream fout("/home/xiongyi/repo/manifold.txt");
                        cxy_transform::Pose<_Scalar> pose;
                        const _Scalar delta = 1.0;
                        int counter1(0);
                        while (1)
                        {
                            counter1++;
                            pose.rotateByAxis(cxy_transform::Axis::X_axis, delta);
                            std::vector<_Scalar> vPara(7);
                            _Scalar res(0.0);
                            vPara[0] = 0.0;
                            vPara[1] = .0;
                            vPara[2] = .0;
                            vPara[3] = pose.q().w();
                            vPara[4] = pose.q().x();
                            vPara[5] = .0;
                            vPara[6] = .0;
                            pose.normalize();
                            for (unsigned int ii = 0; ii < dataCloud_->size(); ++ii)
                            {
                                pcl::PointXYZ transPoint;
                                pose.composePoint((*dataCloud_)[ii], transPoint);
                                Eigen::Matrix< _Scalar, 3, 1> r3;
                                fvec[ii] = matchPointCloud(transPoint, r3);
                                res += fvec[ii] / this->values();

                            }
                            fout<<pose.q().w()<<" "<<pose.q().x()<<" "<<res<<std::endl;
                            if (counter1 >= 361)
                                std::exit(1);
                        }
                    }
                    return ;
            private:
                pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_;
                pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud_;
                pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeptr_;

        };
    }

}