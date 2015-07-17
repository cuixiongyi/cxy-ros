//
// Created by xiongyi on 6/7/15.
//
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/kdtree/kdtree_flann.h>
#include "pcl_ros/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/io/pcd_io.h>

#include "geometric_shapes/shape_operations.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"

#include "ros/ros.h"
#include "cxy_transform.h"
#include "optimization/cxy_cost_func_abstract.h"
#include "auto_ptr.h"
//#include <vnl/vnl_vector.h>
//#include <vnl/vnl_matrix.h>

namespace cxy
{
    namespace cxy_lmicp_lib
    {
        namespace ctrs = cxy_transform;



        template<typename _Scalar, int _MinimizerType>
        class cxy_icp {
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
            typedef Eigen::Vector3f             Vector3f;
            typedef Eigen::Matrix< _Scalar, 4, 4> Matrix44f;


        public:
            cxy_icp() :  hasSetModelCloud_(false)
                        , hasSetDataCloud_(false)
                        , func_(nullptr)
            {

            }

            ~cxy_icp()
            {
                if (func_ != nullptr)
                delete func_;
            }


            // should be override in rigid or articulate
            bool setModelCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr model)
            {
                if (nullptr == model || model->size() < 10)
                {
                    return false;
                }
                hasSetModelCloud_ = true;
                //pcl::PointCloud<pcl::PointXYZ>::Ptr tmp = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(*model);
                //translateToCenter(tmp);
                //modelCloud_ = tmp;
                modelCloud_ = model;

                kdtreeptr_ = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>);
                kdtreeptr_->setInputCloud(modelCloud_);
                return true;

            }

            inline bool setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr data)
            {
                hasSetDataCloud_ = true;
                dataCloud_ = data;
            }


            // There are 3 layers of minimization class
            // The 1st layer is the cxy_icp, is abstract, should not be changed
            // 2nd layer is cxy_icp_rigid, deal with cost function
            // 3rd layer is cxy_icp_rigid_lm, deal with minimization interface

            //: This function belong to 1st layer
            _Scalar icp_run(Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> &x)
            {
                if ( ! hasSetModelCloud_ || ! hasSetDataCloud_)
                {
                    return -1.0;
                }
                icp_prepare_cost_function();
                _Scalar result = icp_minimization(x);

                
                return result;
            }


            //: This function belong to 2nd layer, initialize specific cost function
            virtual int icp_prepare_cost_function() = 0;

            //: This function belong to 3rd layer, using optimization interface
            virtual _Scalar icp_minimization(Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> &x) = 0;

            // match dataCloud_ and modelCloud_, store the result in member variables
            // should be override in rigid or articulate
            //virtual dataType translatePointCloud() = 0;



            //bool setModelCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr model);

        private:
            

        protected:
            
            double transformation_epsilon_, euclidean_fitness_epsilon_, max_correspondence_dist_, max_correspondence_dist_square_;

            bool hasSetModelCloud_, hasSetDataCloud_;
            cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar>*  func_;


            pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud_;
            pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeptr_;

            // store matchPointCloud result
            std::vector<int> modelMatchIdx_;
            std::vector<float> matchDistance_;
            std::vector<Eigen::Vector3f> residual_;


        };
    }
}

