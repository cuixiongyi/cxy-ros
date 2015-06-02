#include "perception_model_based_detection/model_based_detection.h"

namespace perception_model_based_detection
{
    ModelBasedDetection::ModelBasedDetection() : pnh_("~")

    //ros::NodeHandle node_handle, ros::NodeHandle private_node_handle
    {
    //    nh_ = node_handle;
    //    pnh_ = private_node_handle;
    }
ModelBasedDetection::ModelBasedDetection(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)

    //
    {
        nh_ = node_handle;
        pnh_ = private_node_handle;
    }
    ModelBasedDetection::~ModelBasedDetection()
    {
        // Destructor code goes here
    }

    bool ModelBasedDetection::initialise()
    {
        initTransform_= Eigen::Matrix4d::Identity();
        if (model_to_track_ == Model::ROBOTIQ_HAND)
        {
            model_filenames_.push_back("package://perception_model_based_detection/models/robotiq/s-model/visual/GRIPPER_OPEN_FINGER_1.stl");
            model_filenames_.push_back("package://perception_model_based_detection/models/robotiq/s-model/visual/GRIPPER_OPEN_FINGER_2.stl");
            model_filenames_.push_back("package://perception_model_based_detection/models/robotiq/s-model/visual/GRIPPER_OPEN_FINGER_3.stl");
            model_filenames_.push_back("package://perception_model_based_detection/models/robotiq/s-model/visual/GRIPPER_OPEN_PALM.stl");
        }
        else if (model_to_track_ == Model::DOOR_BODY)
            model_filenames_.push_back("package://perception_model_based_detection/models/door/door.stl");
        else if (model_to_track_ == Model::DOOR_FRAME)
            model_filenames_.push_back("package://perception_model_based_detection/models/door/frame.stl");
        else if (model_to_track_ == Model::DRILL)
            model_filenames_.push_back("package://perception_model_based_detection/models/drill/cordless_drill.stl");
        else if (model_to_track_ == Model::VALVE)
        {
            initTransform_(1,1) = 0;
            initTransform_(1,2) = -1;
            initTransform_(2,1) = 1;
            initTransform_(2,2) = 0;

            model_filenames_.push_back("package://perception_model_based_detection/models/vrc_valve_a.dae");
        }
        else if (model_to_track_ == Model::DEBRIS)
        {

        }

        model_sampler_ = new perception_common::SampleCADModels(nh_, pnh_);
        pnh_.getParam("initZ", initZ_);
        sampling_params_.number_of_points = 2000;
        sampling_params_.sample_type = perception_common::SampleType::RANDOM;
        sampling_params_.step_size = 0.005;

        sampled_prefiltered_model_cloud_ = modelCloud::Ptr(new modelCloud);
        sampled_prefiltered_model_cloud_normals_ = modelCloud::Ptr(new modelCloud);

        model_sampler_->meshesToPointCloud(model_filenames_, *sampled_prefiltered_model_cloud_,
                                          *sampled_prefiltered_model_cloud_normals_, sampling_params_);

        pcl::transformPointCloud(*sampled_prefiltered_model_cloud_ , *sampled_prefiltered_model_cloud_ , initTransform_);
        initTransform_ = Eigen::Matrix4d::Identity();
        robotiq_model_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("estimated_model_", 5);
        model_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("model_cloud_", 5);
    
        coarseSearch_ = true;
        iGuessZNumber_ = 5;
        dGuessZ_ = initZ_;
        dGuessZInterval_ = 0.07;
        vGuessZList_.push_back(dGuessZ_+-3*dGuessZInterval_ );
        vGuessZList_.push_back(dGuessZ_+-2*dGuessZInterval_ );
        vGuessZList_.push_back(dGuessZ_+-1*dGuessZInterval_ );
        vGuessZList_.push_back(dGuessZ_+ 0*dGuessZInterval_ );
        vGuessZList_.push_back(dGuessZ_+ 1*dGuessZInterval_ );
        vGuessZList_.push_back(dGuessZ_+ 2*dGuessZInterval_ );
        vGuessZList_.push_back(dGuessZ_+ 3*dGuessZInterval_ );
    }

    bool ModelBasedDetection::setGuessPose(const Eigen::Matrix4d &input)
    {
        //guessPose_ = link_;
        //coarseSearch_ = false;
            //ROS_INFO_STREAM("xtmp<<"  "<<ytmp<<"  "<<ztmp");
        vGuessPoseList_.clear();
        double startT = -0.05, endT = 0.05, searchInterval = 0.04;
        if (coarseSearch_)
        {
            startT = -0.2;
            endT = 0.2;
            searchInterval = 0.06;
        }
        const double xStart(startT), xEnd(endT),
                  yStart(startT), yEnd(endT),
                  zStart(startT), zEnd(endT);
        const double xcenter = input(0,3);
        const double ycenter = input(1,3);
        const double zcenter = input(2,3);
        Eigen::Matrix4d guess_mat = Eigen::Matrix4d::Identity(); 
        //ROS_INFO_STREAM(xStart+xcenter<<"  "<<xEnd+xcenter);
        for (double xtmp = xStart+xcenter; xtmp < xEnd+xcenter; xtmp+=searchInterval)
        {
        for (double ytmp = yStart+ycenter; ytmp < yEnd+ycenter; ytmp+=searchInterval)
        {
        for (double ztmp = zStart+zcenter; ztmp < zEnd+zcenter; ztmp+=searchInterval)
        {
            guess_mat(0,3) = xtmp;
            guess_mat(1,3) = ytmp;
            guess_mat(2,3) = ztmp;
            //ROS_INFO_STREAM(xtmp<<"  "<<ytmp<<"  "<<ztmp);
            vGuessPoseList_.push_back(guess_mat); 
        }
        }
        }    
        
        coarseSearch_ = false;
        return true;
    }
    bool ModelBasedDetection::setGuessPose(const geometry_msgs::PoseStamped &input)
    {
        guessPose_ = input;
        vGuessPoseList_.clear();
        double startT = -0.05, endT = 0.05, searchInterval = 0.04;
        if (coarseSearch_)
        {
            startT = -0.2;
            endT = 0.2;
            searchInterval = 0.06;
        }
        const double xStart(startT), xEnd(endT),
                  yStart(startT), yEnd(endT),
                  zStart(startT), zEnd(endT);
        const double xcenter = input.pose.position.x;
        const double ycenter = input.pose.position.y;
        const double zcenter = input.pose.position.z;
        Eigen::Matrix4d guess_mat = Eigen::Matrix4d::Identity(); 
        for (double xtmp = xStart+xcenter; xtmp < xEnd+xcenter; xtmp+=searchInterval)
        {
        for (double ytmp = yStart+ycenter; ytmp < yEnd+ycenter; ytmp+=searchInterval)
        {
        for (double ztmp = zStart+zcenter; ztmp < zEnd+zcenter; ztmp+=searchInterval)
        {
            guess_mat(0,3) = xtmp;
            guess_mat(1,3) = ytmp;
            guess_mat(2,3) = ztmp;
            vGuessPoseList_.push_back(guess_mat); 
        }
        }
        }    
        coarseSearch_ = false;
        return true;
    }
    void ModelBasedDetection::setModelToTrack(Model model_to_track)
    {
        model_to_track_ = model_to_track;
    }

    void ModelBasedDetection::setRootFrameId(std::string root_frame_id)
    {
        root_frame_id_ = root_frame_id;
    }

    void ModelBasedDetection::setCameraFrameId(std::string camera_frame_id)
    {
        camera_frame_id_ = camera_frame_id;
    }

    void ModelBasedDetection::setEstimateFrameId(std::string estimate_frame_id)
    {
        estimate_frame_id_ = estimate_frame_id;
    }

    void ModelBasedDetection::filterCloud(modelCloud::Ptr &cloud_input, modelCloud::Ptr &cloud_filtered,
                                    modelCloud::Ptr &cloud_normals)
    {
       // ROS_INFO("Filtering point cloud.");

        modelCloud::Ptr  vis_input, vis_normals, trans_input, trans_normals, filtered_normals;

        vis_input = modelCloud::Ptr(new modelCloud);
        vis_normals = modelCloud::Ptr(new modelCloud);
        trans_input = modelCloud::Ptr(new modelCloud);
        trans_normals = modelCloud::Ptr(new modelCloud);
        filtered_normals = modelCloud::Ptr(new modelCloud);

        //model_sampler_->filterPointsWithInnerNormals(cloud_input, cloud_normals, vis_input, vis_normals);

        vis_input = cloud_input;
        vis_normals = cloud_normals;

       //ROS_INFO("Filtering 1 done.");

        // Transform to the pose of the guess from the Forward Kinematics using the poseGuess()
        geometry_msgs::PoseStamped  link_pose;
        poseGuess(link_pose);

        Eigen::Matrix4d     guess_mat;
        poseToMatrix(link_pose.pose, guess_mat);
        guess_mat = Eigen::Matrix4d::Identity();

        guess_mat(2,3) = initZ_;
        // Get the value of the estimate_frame
        pcl::transformPointCloud(*vis_input, *trans_input, guess_mat);
        pcl::transformPointCloud(*vis_normals, *trans_normals, guess_mat);

        trans_input->header.frame_id = camera_frame_id_;
        trans_normals->header.frame_id = camera_frame_id_;

        Eigen::Vector3d  to_origin;
        Eigen::Vector3d  origin(0.0, 0.0, 0.0);

        to_origin(0) = link_pose.pose.position.x;
        to_origin(1) = link_pose.pose.position.y;
        to_origin(2) = link_pose.pose.position.z;

        to_origin(0) = 0.0; 
        to_origin(1) = 0.0; 
        to_origin(2) = 0.0; 
//        for (int i = 0; i < trans_input->points.size(); i++)
//        {
//            origin(0) = origin(0) + trans_input->points[i].x;
//            origin(1) = origin(1) + trans_input->points[i].y;
//            origin(2) = origin(2) + trans_input->points[i].z;
//        }

//        origin = origin / trans_input->points.size();
//        to_origin = to_origin - origin;

        model_sampler_->filterOccludedPoints(trans_input, cloud_filtered, trans_normals, filtered_normals, to_origin);
        //cloud_filtered = trans_input;
//        publishModelCloud(cloud_filtered);
    }

    void ModelBasedDetection::setGuessList()
    {
        //iGuessZNumber_ = 5;
        //dGuessZ_ = initZ_;
        //dGuessZInterval_ = 0.08;
        vGuessZList_.clear();
        vGuessZList_.push_back(dGuessZ_-2*dGuessZInterval_ );
        vGuessZList_.push_back(dGuessZ_-1*dGuessZInterval_ );
        vGuessZList_.push_back(dGuessZ_+ 0*dGuessZInterval_ );
        vGuessZList_.push_back(dGuessZ_+ 1*dGuessZInterval_ );
        vGuessZList_.push_back(dGuessZ_+ 2*dGuessZInterval_ );
    }

bool ModelBasedDetection::icp(modelCloud::Ptr &input,
                            modelCloud::Ptr &target,
                            Eigen::Matrix4d &guess,
                            double &score,
                            Eigen::Matrix4d &transform)
    {
        multisenseCloud final;
        int             max_iterations;
        double          euclidean_fitness_epsilon;
        double          max_correspondence_dist;
        double          transformation_epsilon;

        pcl::IterativeClosestPoint<multisensePoint, modelPoint> icp;

        //multisenseCloud::Ptr    filtered_cloud(new multisenseCloud);
        //pcl::ApproximateVoxelGrid<multisensePoint> approximate_voxel_filter;
        //approximate_voxel_filter.setLeafSize(0.01, 0.01, 0.01);
        //approximate_voxel_filter.setInputCloud(target);
        //approximate_voxel_filter.filter(*filtered_cloud);

        icp.setInputSource(input);
        //icp.setInputTarget(filtered_cloud);
        icp.setInputTarget(target);

        //ROS_INFO_STREAM("Input Header - " << input->header.frame_id << " Size: " << input->points.size());
        //ROS_INFO_STREAM("Target Header - " << target->header.frame_id << " Size: " << target->points.size());
//        //ROS_INFO_STREAM("Filtered cloud size: " << filtered_cloud->points.size());

        pnh_.param<int>("max_iterations", max_iterations, 1000);
        pnh_.param<double>("transformation_epsilon", transformation_epsilon, 1e-8);
        pnh_.param<double>("euclidean_fitness_epsilon", euclidean_fitness_epsilon, 0.1);
        pnh_.param<double>("max_correspondence_dist", max_correspondence_dist, 0.2); // 0.2

        icp.setMaximumIterations(max_iterations);
        icp.setTransformationEpsilon(transformation_epsilon);
        icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
        icp.setMaxCorrespondenceDistance(max_correspondence_dist);
        
        Eigen::Matrix4d guess_mat =initTransform_; //Eigen::Matrix4d::Identity();// initTransform_; 
        Eigen::Matrix4d transform_min;
        
        double scoremin = 9999;
        /*
          guess_mat(1,1) = 0;
        guess_mat(1,2) = -1;
        guess_mat(2,1) = 1;
        guess_mat(2,2) = 0;
        */
    for (int ii = 0; ii < vGuessPoseList_.size(); ++ii)
    {
        icp.align(final, vGuessPoseList_[ii].cast<float>());
            transform = ((Eigen::Matrix4f) icp.getFinalTransformation()).cast<double>();
            score = icp.getFitnessScore();

            if (score < scoremin)
            {
                scoremin = score;
                transform_min = transform;
            }
        }
        //ROS_INFO_STREAM("ICP has converged: " << icp.hasConverged() << ", " << icp.getFitnessScore());
        transform = transform_min;
        double xcenter = transform(0,3);
        double ycenter = transform(1,3);
        double zcenter = transform(2,3);
        //xcenter = transform(0,3);
        //ycenter = transform(1,3);
        transform = initTransform_; 
        transform(0,3) = xcenter;
        transform(1,3) = ycenter;
        transform(2,3) = zcenter;
        icp.align(final, transform.cast<float>());
        transform = ((Eigen::Matrix4f) icp.getFinalTransformation()).cast<double>();
        score = icp.getFitnessScore();
        ROS_INFO_STREAM("score " << score);
        return true;
    }

    bool ModelBasedDetection::startTracking(multisenseCloud::Ptr &target_ptr, double &score, Eigen::Matrix4d &transform)
    {
        bool                        tracking_result;
        modelCloud::Ptr             icp_input_ptr;
        geometry_msgs::PoseStamped  id;
        Eigen::Matrix4d             mat_id;

        transform = Eigen::Matrix4d::Identity();
        icp_input_ptr = modelCloud::Ptr(new modelCloud);

        filterCloud(sampled_prefiltered_model_cloud_, icp_input_ptr, sampled_prefiltered_model_cloud_normals_);

       // ROS_INFO_STREAM("Finished filtering." << std::endl);

        ROS_INFO_STREAM("Filtered cloud properties - Size: " << sampled_prefiltered_model_cloud_->points.size());

        //publishModelCloud(icp_input_ptr);

        id.header.frame_id      = camera_frame_id_;
        id.pose.position.x      = 0.0;
        id.pose.position.y      = 0.0;
        id.pose.position.z      = initZ_;
        id.pose.orientation.w   = 1.0;
        id.pose.orientation.x   = 0.0;
        id.pose.orientation.y   = 0.0;
        id.pose.orientation.z   = 0.0;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr trans_input(new pcl::PointCloud<pcl::PointXYZ>);
       
       
        tracking_result = icp(sampled_prefiltered_model_cloud_, target_ptr, mat_id, score, transform);
        
        Eigen::Matrix4d guess_mat = initTransform_; 
        double scoremin = 9999;
        dGuessZ_ = transform(2,3);
        guess_mat(2,3) = dGuessZ_;       


        pcl::transformPointCloud(*sampled_prefiltered_model_cloud_, *target_ptr, transform);
        
        //ROS_INFO_STREAM("next time Z guess " << dGuessZ_); 
        dGuessZInterval_ = 0.04;
        setGuessList();
        //transform = transform*guess_mat;
        publishModelCloud(target_ptr);
        publishMarker(transform);
        publishEstimateTransform(transform);

        return tracking_result;
    }

    bool ModelBasedDetection::poseGuess(geometry_msgs::PoseStamped &link_pose) 
    { return false;};
    
    void ModelBasedDetection::publishMarker(const Eigen::Matrix4d transform)
    {
        if (robotiq_model_publisher_.getNumSubscribers() < 1)
            return;

        visualization_msgs::MarkerArray model;
        visualization_msgs::Marker      marker;
        Eigen::Matrix3d m;
        m << transform(0,0), transform(0,1), transform(0,2),
         transform(1,0), transform(1,1), transform(1,2),
         transform(2,0), transform(2,1), transform(2,2);

        Eigen::Quaternion<double> q(m);
        for (int i = 0; i < model_filenames_.size(); i++)
        {

            //rotateOrientation(q, RotationAxis::X_AXIS, -M_PI_2);

            marker.header.frame_id = estimate_frame_id_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "estimated_" + estimate_frame_id_;
            marker.id = i;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = transform(0,3);
            marker.pose.position.y = transform(1,3);
            marker.pose.position.z = transform(2,3);
            marker.pose.orientation.w = q.w();
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration();

            marker.mesh_resource = model_filenames_.at(i);
            model.markers.push_back(marker);
        }

        robotiq_model_publisher_.publish(model);
    }

    void ModelBasedDetection::publishEstimateTransform(Eigen::Matrix4d &m_transform)
    {
        Eigen::Matrix3d         m_rot;
        tf::StampedTransform    stamped_tf;

        // Transform to the pose of the guess from the Forward Kinematics using the poseGuess()
        geometry_msgs::PoseStamped  link_pose;
        poseGuess(link_pose);

        Eigen::Matrix4d     guess_mat;
        poseToMatrix(link_pose.pose, guess_mat);

        m_transform = m_transform * guess_mat;

        m_rot << m_transform(0, 0), m_transform(0, 1), m_transform(0, 2),
                 m_transform(1, 0), m_transform(1, 1), m_transform(1, 2),
                 m_transform(2, 0), m_transform(2, 1), m_transform(2, 2);

        Eigen::Quaterniond      q(m_rot);

        stamped_tf.frame_id_        = camera_frame_id_;
        stamped_tf.child_frame_id_  = estimate_frame_id_;
        stamped_tf.stamp_ = ros::Time::now();

        //ROS_INFO_STREAM("camera_: " << camera_frame_id_);
        //ROS_INFO_STREAM("estimate: " << estimate_frame_id_);

        stamped_tf.setOrigin(tf::Vector3(m_transform(0,3), m_transform(1, 3), m_transform(2, 3)));
        stamped_tf.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        /*ROS_INFO_STREAM("Position - " << m_transform(0, 3) << " " << m_transform(1, 3) << " " <<
                        m_transform(2, 3) << std::endl << "Quaternions - " << q.x() << " " << q.y()
                        << " " << q.z() << " " << q.w());
        */
        //transform_broadcaster_.sendTransform(stamped_tf);
    }

    void ModelBasedDetection::publishModelCloud(modelCloud::Ptr &cloud)
    {
        sensor_msgs::PointCloud2    model_cloud_msg;

        pcl::toROSMsg(*cloud, model_cloud_msg);
        model_cloud_msg.header.frame_id = camera_frame_id_;
        model_cloud_msg.header.stamp = ros::Time::now();

        model_cloud_publisher_.publish(model_cloud_msg);
    }

    void ModelBasedDetection::poseToMatrix(geometry_msgs::Pose &pose, Eigen::Matrix4d &mat)
    {
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z);
        Eigen::Matrix3d m = q.toRotationMatrix();

        mat << m(0, 0), m(0, 1), m(0, 2), pose.position.x,
               m(1, 0), m(1, 1), m(1, 2), pose.position.y,
               m(2, 0), m(2, 1), m(2, 2), pose.position.z,
               0,       0,       0,       1;
    }

    void ModelBasedDetection::rotateOrientation(Eigen::Matrix3d &rot_mat,
                                          const RotationAxis &rot_axis, const double &rot_angle_in_radians)
    {
        Eigen::Matrix3d rotation;
        Eigen::Vector3d     rot_axis_vec;

        if (rot_axis == RotationAxis::X_AXIS)
            rot_axis_vec = Eigen::Vector3d(1.0, 0.0, 0.0);
        else if (rot_axis == RotationAxis::Y_AXIS)
            rot_axis_vec = Eigen::Vector3d(0.0, 1.0, 0.0);
        else if (rot_axis == RotationAxis::Z_AXIS)
            rot_axis_vec = Eigen::Vector3d(0.0, 0.0, 1.0);

        rotation = Eigen::AngleAxisd(rot_angle_in_radians, rot_axis_vec);
        rot_mat = rot_mat * rotation;
    }

    void ModelBasedDetection::rotateOrientation(Eigen::Quaterniond &rot_quaternion,
                                          const RotationAxis &rot_axis, const double &rot_angle_in_radians)
    {
        Eigen::Quaterniond  rotation;
        Eigen::Vector3d     rot_axis_vec;

        if (rot_axis == RotationAxis::X_AXIS)
            rot_axis_vec = Eigen::Vector3d(1.0, 0.0, 0.0);
        else if (rot_axis == RotationAxis::Y_AXIS)
            rot_axis_vec = Eigen::Vector3d(0.0, 1.0, 0.0);
        else if (rot_axis == RotationAxis::Z_AXIS)
            rot_axis_vec = Eigen::Vector3d(0.0, 0.0, 1.0);


        rotation = Eigen::AngleAxisd(rot_angle_in_radians, rot_axis_vec);
        rot_quaternion = rot_quaternion * rotation;
    }
void ModelBasedDetection::setGuessZ(const double z) 
    {
        dGuessZ_ = z;
    }
void ModelBasedDetection::setGuessZInterval(const double zInterval) 
    {
        dGuessZInterval_ = zInterval;
    }
const double ModelBasedDetection::getGuessZ() const 
    {
        return dGuessZ_;
    }
const double ModelBasedDetection::getGuessZInterval() const 
    {
        return dGuessZInterval_;
    }

}
