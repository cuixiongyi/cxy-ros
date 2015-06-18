#pragma once

#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/kdtree/kdtree_flann.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/approximate_voxel_grid.h"
#include "pcl/filters/filter.h"
#include <pcl/io/ply_io.h>

#include <ros/console.h>

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include "geometric_shapes/shape_operations.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"

#include "limits"
#include "boost/lexical_cast.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry> 

#include <utility>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <stdlib.h>


#include "visualization_msgs/MarkerArray.h"
#include "cxy_transform.h"


namespace cxy_lmicp
{
    typedef float _Scalar ;
    using namespace cxy;
    namespace gm = geometry_msgs;
    namespace E = Eigen;
    namespace ctrs = cxy_transform;

    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT>    PointCloud;
    typedef pcl::PointCloud<PointT>::Ptr    PointCloudPtr;
    typedef pcl::PointCloud<PointT>::ConstPtr    PointCloudConstPtr;
    typedef E::Matrix< float, 3, 4> Matrix34f;
    typedef E::Matrix< float, 3, 7> Matrix37f;
    typedef E::Matrix< float, E::Dynamic, 7> MatrixX7f;
    typedef E::Matrix< float, 7, 7> Matrix7f;
    typedef E::Matrix< float, 7, 1> Vector7f;
    typedef E::Matrix< float, 6, 6> Matrix6f;
    typedef E::Matrix< float, 6, 1> Vector6f;

    typedef E::Matrix< float, 4, 4> Matrix44f;

    enum class Model {DOOR_BODY, DOOR_FRAME, DRILL, ROBOTIQ_HAND, DEBRIS, VALVE};
    enum class RotationAxis {X_AXIS, Y_AXIS, Z_AXIS};

    class ModelProperties
    {
        public:
            Model                       model_name_;
            std::vector<std::string>    model_stl_file_list_;
            std::vector<RotationAxis>   orientation_offset_axis_list_;
            std::vector<double>         orientation_offset_angle_list_;

            ModelProperties() {}
            ~ModelProperties() {}
    };

    class LM_ICP
    {
        protected:
        	
            bool initialise();
            void publishModel(const E::Matrix4d transform);
            void publishModelCloud(const PointCloudPtr &cloud);
            void poseToMatrix(gm::Pose &pose, E::Matrix4d &mat);
            const float searchMatchPoints(PointCloudPtr data
                                            , std::vector<int>& dataMatchIdx
                                            , std::vector<int>& modelMatchIdx
                                            , std::vector<float>& matchDistance);
            
            const float computeResidual(PointCloudPtr data
                                            , ctrs::Pose<_Scalar> &pose
                                            , std::vector<int>& dataMatchIdx
                                            , std::vector<int>& modelMatchIdx
                                            , std::vector<float>& matchDistance);
            
            // return L-2 norm residual and res is residual on xyz axis
            inline  const float searchMatchPoints(const PointT& data
                                                        , E::Vector3f& res);
            void calculateResidual(PointCloudPtr data
                                        , const std::vector<int>& dataMatchIdx
                                        , const std::vector<int>& modelMatchIdx
                                        , std::vector<E::Vector3f>& residual);




            ros::NodeHandle nh_, pnh_;
            ros::Publisher pub_model_, pub_model_pointcloud_, pub_data_pointcloud_, pub_result_;
            int max_iterations_;
            double transformation_epsilon_, euclidean_fitness_epsilon_, max_correspondence_dist_, max_correspondence_dist_square_;

            /// modelcloud related
            pcl::KdTreeFLANN<PointT>::Ptr kdtreeptr_;
            std::vector<PointCloudPtr> modelCloudVec_;
            PointCloudConstPtr modelCloud_;
            bool isNewModel_;

            /// dataCloud related
            std::vector<PointCloudPtr> dataCloudVec_;
            PointCloudPtr dataCloud_;
            std::shared_ptr<std::vector<float> > matchDistancefPtr_;
        public:
            LM_ICP();
            ~LM_ICP();

            ctrs::Pose<_Scalar> lmicp(const PointCloudPtr data
                            , const PointCloudPtr model
                            , const ctrs::Pose<_Scalar> guess = ctrs::Pose<_Scalar>());
            void initGuess(ctrs::Pose<_Scalar>& guess);
            bool setModelCloud(PointCloudConstPtr model);
            inline const Matrix34f calculateJacobianKernel(const ctrs::Pose<_Scalar>& pose_k
                                                    , const E::Vector3f& res
                                                    , const PointT& p);

            
            void calculateJacobian();
            void calculateLevenbergMarquardt(const PointCloudPtr data
                                    , const std::vector<int>& dataMatchIdx
                                    , const std::vector<int>& modelMatchIdx
                                    , const ctrs::Pose<_Scalar>& pose_k
                                    , const std::vector<E::Vector3f>& residual
                                    , ctrs::Pose<_Scalar>& pose_k1);



            ctrs::Pose<_Scalar> lmicpNumerical(const PointCloudPtr data
                                    , const PointCloudPtr model
                                    , const ctrs::Pose<_Scalar> guess = ctrs::Pose<_Scalar>());
            void calculateLevenbergMarquardtNumerical(const PointCloudPtr data
                                    , const std::vector<int>& dataMatchIdx
                                    , const std::vector<int>& modelMatchIdx
                                    , const ctrs::Pose<_Scalar>& pose_k
                                    , const std::vector<E::Vector3f>& residual
                                    , ctrs::Pose<_Scalar>& pose_k1);
            inline const E::Matrix<float,1,7> calculateJacobianKernelNumerical(const ctrs::Pose<_Scalar>& pose_k
                                                    , const PointT& d
                                                    , const PointT& m
                                                    , float& l2norm
                                                    , E::Vector3f res);
            void calculateCenter(const PointCloudConstPtr data
                                    , Eigen::Vector4f& center);
            void translateToCenter(PointCloudPtr& data);
                                    

            void publish(const PointCloudConstPtr& data, const ros::Publisher& pub);
        ;
         
    };
}
