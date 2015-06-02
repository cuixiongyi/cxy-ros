#include "ros/ros.h"
#include "perception_model_based_detection/model_based_detection.h"
#include "perception_common/global.h"
#include "perception_common/MultisensePointCloud.h"

class ModelBasedObjectDetector : public perception_model_based_detection::ModelBasedDetection
{
    public:
        ModelBasedObjectDetector(ros::NodeHandle nh, ros::NodeHandle pnh);
        ~ModelBasedObjectDetector();

        bool poseGuess(geometry_msgs::PoseStamped &link_pose);
};

typedef ModelBasedObjectDetector*   ModelBasedObjectDetectorPtr;

bool convertPointCloudTypes(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                            pcl::PointCloud<drc_perception::LaserPoint>::Ptr &cloud_in);
bool poseToMatrix(geometry_msgs::PoseStamped pose, Eigen::Matrix4d &matrix);
bool matrixToPose(std::string frame, Eigen::Matrix4d transform, geometry_msgs::PoseStamped &pose);
bool updatePoseWithTransform(geometry_msgs::PoseStamped &pose, Eigen::Matrix4d transform);
