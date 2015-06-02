#include "perception_model_based_detection/test_mbd.h"

ModelBasedObjectDetector::ModelBasedObjectDetector(ros::NodeHandle nh, ros::NodeHandle pnh)
    : perception_model_based_detection::ModelBasedDetection(nh, pnh)
{
    // Constructor code goes here
}

ModelBasedObjectDetector::~ModelBasedObjectDetector()
{
    // Destructor code goes here
}

bool ModelBasedObjectDetector::poseGuess(geometry_msgs::PoseStamped &link_pose)
{
    ros::Time                   time = ros::Time::now();
    tf::StampedTransform        stamped_tf;
    geometry_msgs::PoseStamped  pose_model;

    try
    {
        transform_listener_.waitForTransform("left_camera_optical_frame", "model_guess_frame", time, ros::Duration(3.0));
        transform_listener_.lookupTransform("left_camera_optical_frame", "model_guess_frame", time, stamped_tf);

        Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
        //perception_model_based_detection::ModelBasedDetection::rotateOrientation(q, perception_model_based_detection::RotationAxis::X_AXIS, -M_PI_2);

        pose_model.header.frame_id = "model_guess_frame";
        pose_model.header.stamp = time;
        pose_model.pose.position.x = 0.0;
        pose_model.pose.position.y = 0.0;
        pose_model.pose.position.z = 0.0;
        pose_model.pose.orientation.w = q.w();
        pose_model.pose.orientation.x = q.x();
        pose_model.pose.orientation.y = q.y();
        pose_model.pose.orientation.z = q.z();

        transform_listener_.transformPose("left_camera_optical_frame", pose_model, link_pose);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN_STREAM("Transform error: " << ex.what());
        return false;
    }
    catch (...)
    {
        ROS_WARN("Error in test_mbd.cpp : poseGuess()");
        return false;
    }

    return true;
}

bool convertPointCloudTypes(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                            pcl::PointCloud<drc_perception::LaserPoint>::Ptr &cloud_in)
{
    if (cloud_out == NULL)
        cloud_out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    if (cloud_in == NULL)
        return false;

    for (int i = 0; i < cloud_in->points.size(); i++)
    {
        pcl::PointXYZ   temp_pt;

        temp_pt.x = cloud_in->points[i].x;
        temp_pt.y = cloud_in->points[i].y;
        temp_pt.z = cloud_in->points[i].z;

        cloud_out->points.push_back(temp_pt);
    }

    ROS_INFO_STREAM("Cloud size - " << cloud_out->points.size());

    return true;
}

bool poseToMatrix(geometry_msgs::PoseStamped pose, Eigen::Matrix4d &matrix)
{
    Eigen::Quaternion<double> q(pose.pose.orientation.w, pose.pose.orientation.x,
                                pose.pose.orientation.y, pose.pose.orientation.z);
    Eigen::Matrix3d m = q.toRotationMatrix();

    matrix << m(0,0), m(0,1), m(0,2), pose.pose.position.x,
              m(1,0), m(1,1), m(1,2), pose.pose.position.y,
              m(2,0), m(2,1), m(2,2), pose.pose.position.z,
              0,      0,      0,      1;

  return true;
}

bool matrixToPose(std::string frame, Eigen::Matrix4d transform, geometry_msgs::PoseStamped &pose)
{
    Eigen::Matrix3d m;
    m << transform(0,0), transform(0,1), transform(0,2),
         transform(1,0), transform(1,1), transform(1,2),
         transform(2,0), transform(2,1), transform(2,2);

    // check orthonormal
    double det = (m(0,0)*m(1,1)*m(2,2)) + (m(0,1)*m(1,2)*m(2,0)) + (m(0,2)*m(1,0)*m(2,1)) -
                 (m(0,2)*m(1,1)*m(2,0)) - (m(0,1)*m(1,0)*m(2,2)) - (m(0,0)*m(1,2)*m(2,1));

    if ((det >= 1.25) || (det <= -1.25))
        return false;
    else
    {
        Eigen::Quaternion<double> q (m);
        //geometry_msgs::PoseStamped guess_pose;
        pose.header.frame_id = frame;
        pose.pose.position.x = transform(0,3);
        pose.pose.position.y = transform(1,3);
        pose.pose.position.z = transform(2,3);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        return true;
    }
}

bool updatePoseWithTransform(geometry_msgs::PoseStamped &pose, Eigen::Matrix4d transform)
{
    Eigen::Matrix4d final, initial;

    poseToMatrix(pose, initial);

    final = transform * initial;

    matrixToPose("left_camera_optical_frame", final, pose);

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_mbd");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    drc_perception::LaserPointCloud::Ptr    laser_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr     laser_cloud_xyz_ptr;
    Eigen::Matrix4d                         transform;
    double                                  score;

    ModelBasedObjectDetectorPtr             model_based_detector;
    drc_perception::MultisensePointCloud    multisense_point_cloud(nh);

    perception_model_based_detection::Model model;

    ros::Publisher                          model_desired_pose_publisher;
    geometry_msgs::PoseStamped              frame_centre_pose;

    geometry_msgs::PoseStamped              frame_centre_pose1;
    tf::TransformListener listener;
    tf::StampedTransform itransform;

    geometry_msgs::PoseStamped              door_body_handle_pose;


    ROS_WARN_STREAM(argc);

    if (argc > 1)
    {
        if (std::strcmp(argv[1], "door_body") == 0)
        {
            ROS_WARN_STREAM("door_body");
            model = perception_model_based_detection::Model::DOOR_BODY;
            model_desired_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/model_based_detection/door_body_handle", 3);
        }
        else if (std::strcmp(argv[1], "door_frame") == 0)
        {
            ROS_WARN_STREAM("door_frame");
            model = perception_model_based_detection::Model::DOOR_FRAME;
            model_desired_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/model_based_detection/frame_centre_top", 3);
        }
        else if (std::strcmp(argv[1], "drill") == 0)
        {
            ROS_WARN_STREAM("drill");
            model = perception_model_based_detection::Model::DRILL;
        }
        else if (std::strcmp(argv[1], "robotiq_hand") == 0)
            model = perception_model_based_detection::Model::ROBOTIQ_HAND;
        else
            model = perception_model_based_detection::Model::DOOR_BODY;
    }

    model_based_detector = new ModelBasedObjectDetector(nh, pnh);
    model_based_detector->setModelToTrack(model);
    model_based_detector->setCameraFrameId("left_camera_optical_frame");
    model_based_detector->setEstimateFrameId("estimated_model_frame");
    model_based_detector->initialise();

    while (ros::ok())
    {
        ros::spinOnce();
        if (multisense_point_cloud.giveLaserCloud(laser_cloud_ptr) == true)
        {
            ROS_INFO_ONCE("In main loop!");
            if (laser_cloud_ptr == NULL)
            {
                ROS_INFO_ONCE("Received null cloud. Waiting for valid cloud.");
                continue;
            }

            ROS_INFO_ONCE("Received cloud.");
            ROS_INFO_STREAM_ONCE("Cloud size - " << laser_cloud_ptr->points.size());

            if (convertPointCloudTypes(laser_cloud_xyz_ptr, laser_cloud_ptr) == true)
            {
                ROS_INFO_ONCE("Converted cloud!");
                model_based_detector->startTracking(laser_cloud_xyz_ptr, score, transform);
                if (model == perception_model_based_detection::Model::DOOR_FRAME)
                {
                    frame_centre_pose.header.frame_id = "model_guess_frame";
                    // Coordinate of the centre of the top of the door frame (Xianchao)
                    frame_centre_pose.pose.position.x = 0.067/2;
                    frame_centre_pose.pose.position.y = 1.067/2;
                    frame_centre_pose.pose.position.z = 2.108;
                    frame_centre_pose.pose.orientation.w = 1.0;
                    frame_centre_pose.pose.orientation.x = 0.0;
                    frame_centre_pose.pose.orientation.y = 0.0;
                    frame_centre_pose.pose.orientation.z = 0.0;

                    updatePoseWithTransform(frame_centre_pose, transform);
                    
                    try{
                        // listener.lookupTransform("left_camera_optical_frame", "l_foot",ros::Time(0), itransform);
                        listener.transformPose("l_foot", frame_centre_pose, frame_centre_pose1);
                        }
                    catch (tf::TransformException &ex) {
                        ROS_ERROR("%s",ex.what());
                        ros::Duration(1.0).sleep();
                        continue;
                    }

                    frame_centre_pose1.header.stamp = ros::Time::now();
                    model_desired_pose_publisher.publish(frame_centre_pose1);
                }
                else if (model == perception_model_based_detection::Model::DOOR_BODY)
                {
                    // 0.0 0.8 1.05
                    door_body_handle_pose.header.frame_id = "model_guess_frame";
                    door_body_handle_pose.pose.position.x = 0.8;
                    door_body_handle_pose.pose.position.y = 0.0;
                    door_body_handle_pose.pose.position.z = 1.05;
                    door_body_handle_pose.pose.orientation.w = 0.70711;
                    //door_body_handle_pose.pose.orientation.w = 0.25882;
                    //door_body_handle_pose.pose.orientation.w = 0.5;
                    door_body_handle_pose.pose.orientation.x = 0.0;
                    door_body_handle_pose.pose.orientation.y = 0.0;
                    door_body_handle_pose.pose.orientation.z = -0.70711;
                    //door_body_handle_pose.pose.orientation.z = 0.86603;
                    //door_body_handle_pose.pose.orientation.z = 0.96593;


                    updatePoseWithTransform(door_body_handle_pose, transform);
                    door_body_handle_pose.header.stamp = ros::Time::now();
                    model_desired_pose_publisher.publish(door_body_handle_pose);
                }
            }
        }
    }

    return 0;
}

