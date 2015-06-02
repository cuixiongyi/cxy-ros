#pragma once

#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/approximate_voxel_grid.h"
#include "pcl/filters/filter.h"

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

#include "chrono"

#include "perception_common/SampleCADModels.h"
#include "perception_common/MultisensePointCloud.h"
#include "visualization_msgs/MarkerArray.h"

namespace perception_model_based_detection
{
    typedef pcl::PointXYZ                       multisensePoint;
    typedef pcl::PointCloud<multisensePoint>    multisenseCloud;
    typedef pcl::PointXYZ                       modelPoint;
    typedef pcl::PointCloud<modelPoint>         modelCloud;

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

    class ModelBasedDetection
    {
        protected:
            ros::NodeHandle                     nh_, pnh_;
            perception_common::SamplingParams   sampling_params_;
            perception_common::SampleCADModels  *model_sampler_;
            Model                               model_to_track_;
            std::vector<std::string>            model_filenames_;
            modelCloud::Ptr                     sampled_prefiltered_model_cloud_;
            modelCloud::Ptr                     sampled_prefiltered_model_cloud_normals_;
            tf::TransformListener               transform_listener_;
            tf::TransformBroadcaster            transform_broadcaster_;
            std::string                         root_frame_id_;
            std::string                         estimate_frame_id_;
            std::string                         camera_frame_id_;
            ros::Publisher                      robotiq_model_publisher_;
            ros::Publisher                      model_cloud_publisher_;
            double                              initZ_;
            std::vector<double>                 vGuessZList_;
            std::vector<Eigen::Matrix4d>        vGuessPoseList_;
            int                                 iGuessZNumber_;
            double                              dGuessZ_;
            double                              dGuessZInterval_;
            Eigen::Matrix4d                     initTransform_;
            void filterCloud(modelCloud::Ptr &cloud_input, modelCloud::Ptr &cloud_filtered,
                             modelCloud::Ptr &cloud_normals);
            bool icp(modelCloud::Ptr &input,
                     multisenseCloud::Ptr &target,
                     Eigen::Matrix4d &guess,
                     double &score,
                     Eigen::Matrix4d &transform);
            void publishMarker(const Eigen::Matrix4d transform);
            void publishEstimateTransform(Eigen::Matrix4d &m_transform);
            void publishModelCloud(modelCloud::Ptr &cloud);
            void poseToMatrix(geometry_msgs::Pose &pose, Eigen::Matrix4d &mat);
            geometry_msgs::PoseStamped guessPose_;
        public:
            ModelBasedDetection(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
            ModelBasedDetection();
            ~ModelBasedDetection();

            void setGuessList();
            bool initialise();
            void setModelToTrack(Model model_to_track);
            void setRootFrameId(std::string root_frame_id);
            void setCameraFrameId(std::string camera_frame_id);
            void setEstimateFrameId(std::string estimate_frame_id);

            bool poseGuess(geometry_msgs::PoseStamped &link_pose) ;

            bool setGuessPose(const geometry_msgs::PoseStamped &input);
            bool setGuessPose(const Eigen::Matrix4d &input);
            bool startTracking(multisenseCloud::Ptr &target_cloud, double &score, Eigen::Matrix4d &transform);

            static void rotateOrientation(Eigen::Matrix3d &rot_mat,
                                   const RotationAxis &rot_axis, const double &rot_angle_in_radians);
            static void rotateOrientation(Eigen::Quaterniond &rot_quaternion,
                                   const RotationAxis &rot_axis, const double &rot_angle_in_radians);

            void setGuessZ(const double z); 
            void setGuessZInterval(const double zInterval); 
            const double getGuessZ() const; 
            const double getGuessZInterval() const;
            bool coarseSearch_;
    };
}
