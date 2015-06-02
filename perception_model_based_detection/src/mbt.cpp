#pragma once
#include "perception_model_based_detection/mbt.h"


namespace cxy_mbt
{



    
    mbt::mbt() :
             //nh_(mbt_Config::getInstance()->nh_)
         pnh_("~" )
        , model_based_detector_(new perception_model_based_detection::ModelBasedDetection())
        //, callback_((ros::NodeHandle)nh_, (ros::NodeHandle)pnh_)
        //, callback_(mbt_Config::getInstance()->nh_, mbt_Config::getInstance()->pnh_)
        , action_(mbt_Config::getInstance()->action_name_)
    {

        return;
    }

    void mbt::start()
    {
        //nh_ =mbt_Config::getInstance()->nh_; 
        //pnh_ =mbt_Config::getInstance()->pnh_; 
        perception_model_based_detection::Model model;
        tf::Transform                                  tftransform;
        geometry_msgs::PoseStamped              frame_centre_pose;
        geometry_msgs::PoseStamped              frame_centre_pose1;
        tf::TransformListener listener;
        tf::StampedTransform itransform;
        geometry_msgs::PoseStamped              door_body_handle_pose;
        
        if (mbt_Config::getInstance()->model_type_== "Valve")
            model_ = perception_model_based_detection::Model::VALVE;
        else if (mbt_Config::getInstance()->model_type_== "Debris")
            model_ = perception_model_based_detection::Model::DEBRIS;
        
        Eigen::Matrix4d                         transform;
        double                                  score;
        // set tf frame
            
        sub_guesspose_ = nh_.subscribe("/grip_torus_GUI", 1, &VisualServo_Callback::callback_GuessPose, &callback_);
        //sub_cam_ = nh_.subscribe("/depth_non_filtered", 1, &VisualServo_Callback::callback_DepthCamera, &callback_);
        sub_visualServoState_ = nh_.subscribe("/mbt_visualservostate", 1, &VisualServo_Callback::callback_VisualServoState, &callback_);
        model_desired_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/model_based_detection/debris", 3);
        pub_stl_marker_ = nh_.advertise<visualization_msgs::Marker>("/valve_model", 0);
       
        // init model based tracking
        model_based_detector_->setModelToTrack(model_);
        model_based_detector_->setCameraFrameId(mbt_Config::getInstance()->mbt_frame_id_);
        model_based_detector_->setEstimateFrameId(mbt_Config::getInstance()->mbt_frame_id_);
        model_based_detector_->initialise();

        
        frame_centre_pose.header.frame_id = mbt_Config::getInstance()->mbt_frame_id_;
        frame_centre_pose.pose.position.x = 0;
        frame_centre_pose.pose.position.y = 0;
        frame_centre_pose.pose.position.z = 0;
        frame_centre_pose.pose.orientation.w = 1.0;
        frame_centre_pose.pose.orientation.x = 0.0;
        frame_centre_pose.pose.orientation.y = 0.0;
        frame_centre_pose.pose.orientation.z = 0.0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr rawPointCloud;
        /// loop at 15 Hz
        ros::Rate rate(mbt_Config::getInstance()->main_loop_rate_);
        Eigen::Matrix4d lastTransform = Eigen::Matrix4d::Identity() ;
        lastTransform(2,3) = 0.3;
        while (ros::ok())
        {
            ros::spinOnce();
            if (! parseGoal((int)mbt_Config::getInstance()->action_goal_.grab)) 
                continue;
            if ( ! checkPointCloudUpdate())
                continue;
            if ((bool)mbt_Config::getInstance()->isnew_guesspose_)
            {
                ROS_INFO_STREAM("mbt::start New Guess pose set    "<<mbt_Config::getInstance()->isnew_guesspose_);
                model_based_detector_->setGuessPose(mbt_Config::getInstance()->newpose_);
            }
            else
            {
                ROS_INFO_ONCE("mbt::start use last time's transformation");
                model_based_detector_->setGuessPose(lastTransform );
            }
            mbt_Config::getInstance()->isnew_depth_ = false;
            pcl::PointCloud<pcl::PointXYZ>::Ptr pctmp(mbt_Config::getInstance()->newdepth_);
            if (!mbt_Config::getInstance()->newdepth_ )
            {
                rate.sleep();
                rate.sleep();
                continue;
            }
            lastTransform = detect(pctmp);
            mbt_Config::getInstance()->isnew_guesspose_= false;
            mbt_Config::getInstance()->transform_ = lastTransform;
            mbt_Config::getInstance()->newtime_transform_ = ros::Time::now();
            // spin at main_loop_rate_  Hz
            ROS_INFO_ONCE("mbt::start tracking finished");
            ros::spinOnce();
            rate.sleep();
        
        }
        return;
    }
    
    Eigen::Matrix4d mbt::detect(pcl::PointCloud<pcl::PointXYZ>::Ptr rawInput)
    {
        Eigen::Matrix4d                         transform;
        double                                  score;
        //pcl::PointCloud< pcl::PointXYZ>::Ptr plPtr(rawInput);
            //static interactive_marker::InteractiveMarkerServer markerServer("valve_model_server");
        ROS_INFO_STREAM("rawInput " << rawInput->size());
        model_based_detector_->startTracking(rawInput, score, transform);
        //ROS_INFO_STREAM("transform " << transform);
        /*frame_centre_pose.header.frame_id = mbt_Config::getInstance()->mbt_frame_id_;
                // Coordinate of the centre of the top of the door frame (Xianchao)
        frame_centre_pose.pose.position.x = 0;
        frame_centre_pose.pose.position.y = 0;
        frame_centre_pose.pose.position.z = 0;
        frame_centre_pose.pose.orientation.w = 1.0;
        frame_centre_pose.pose.orientation.x = 0.0;
        frame_centre_pose.pose.orientation.y = 0.0;
        frame_centre_pose.pose.orientation.z = 0.0;
        //updatePoseWithTransform(frame_centre_pose, transform);
        while(true)
        {
            try
            {
                listener_.lookupTransform(mbt_Config::getInstance()->mbt_frame_id_,"l_palm" ,ros::Time(0), itransform);
                listener_.transformPose("l_palm", frame_centre_pose, frame_centre_pose1);
                break;
              }
            catch (tf::TransformException &ex) 
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(0.2).sleep();
         
            }
        }*/

        return transform;


    }

    bool mbt::parseGoal(const int goal_grab) 
    {
        if (0 == goal_grab)
        {
            ROS_INFO_ONCE("mbt::parseGoal Goal == STOP");
            sub_cam_.shutdown();
            mbt_Config::getInstance()->cam_init_count_ = 0;
            mbt_Config::getInstance()->isnew_guesspose_ = false;

            return false;
        }
        // Left hand
        if (1 == goal_grab)
        {
            ROS_INFO_ONCE("mbt::parseGoal Goal == Left hand");
            sub_cam_= nh_.subscribe("/depth_non_filtered", 1, &VisualServo_Callback::callback_DepthCamera, &callback_);
        }

        // Right hand
        if (2 == goal_grab)
        {
            ROS_INFO_ONCE("mbt::parseGoal Goal == Right hand");
            sub_cam_= nh_.subscribe("/right_hand/depth_non_filtered", 1, &VisualServo_Callback::callback_DepthCamera, &callback_);
        }
        return true;
    }
    bool mbt::checkPointCloudUpdate()
    {
        if ( ! mbt_Config::getInstance()->isnew_depth_ )
            return false;
        double update_rate(10e10);//1/(mbt_Config::getInstance()->main_loop_rate_-7));
        double now = ros::Time::now().toSec();
        double last = mbt_Config::getInstance()->newtime_depth_.toSec();
        if (now - last > update_rate)
        {
            ROS_INFO_STREAM("mbt::checkPointCloudUpdate overtime "<<now - last);
            return false;
        }

        return true;
    }


    bool mbt::updatePoseWithTransform(geometry_msgs::PoseStamped &pose, Eigen::Matrix4d transform)
    {
        Eigen::Matrix4d final, initial;

        mbt_Config::poseToMatrix(pose, initial);

        final = transform * initial;

        mbt_Config::matrixToPose("left_camera_optical_frame", transform, pose);

        return true;
    }



   

}

