#include "../include/cxy_icp_rigid_lm.h"

namespace cxy {
    namespace cxy_lmicp_lib {
/*
        cxy_icp_rigid_lm::cxy_icp_rigid_lm() : cxy_icp_rigid()
        {
            ;

        }

        dataType cxy_icp_rigid_lm::icp_run(pcl::PointCloud<pcl::PointXYZ>::Ptr data, cxy_transform::Pose &outPose)
    {
        if ( ! hasSetModelCloud_)
    {
        return -1.0;
    }
    dataCloud_ = data;

    ctrs::Pose guessTmp(0);

    ctrs::Pose pose_inc(0);
    ctrs::Pose pose_tmp(0);

    dataVectorType            para(7);
    std::vector <dataIdxType> idxVector;
    idxVector.
    reserve(dataCloud_
    ->

    size()

    );
    for (
    unsigned int ii = 0;
    ii<dataCloud_->

    size();

    ii++)
{
    idxVector.
    push_back(ii);
}
para[3] = 1.0;
para[4] = 0.0001;
para[5] = 0.0001;
para[6] = 0.0001;
publish(dataCloud_, pub_data_pointcloud_
);
publish(modelCloud_, pub_model_pointcloud_
);
}

dlib::solve_least_squares_lm<dlib::objective_delta_stop_strategy
                            , float
                            , dataVectorType
                            , dataIdxVectorType
                            , dataVectorType>
                            (dlib::objective_delta_stop_strategy(1e-7).be_verbose()
                            , cxy_icp_rigid::residual
                            , cxy_icp_rigid::residual_derivative
                            , idxVector
                            , para);

cxy_transform::Pose pose;
pose.t()(0) = para[0];
pose.t()(1) = para[1];
pose.t()(2) = para[2];
pose.q().w() = para[3];
pose.q().x() = para[4];
pose.q().y() = para[5];
pose.q().z() = para[6];
pcl::PointCloud<pcl::PointXYZ>::Ptr transDataCloud;

pose.composePoint(dataCloud_, transDataCloud);

publish(transDataCloud, pub_data_pointcloud_);
publish(modelCloud_, pub_model_pointcloud_);

            return 0.0;

        }

void cxy_icp_rigid_lm::publish(const PointCloudConstPtr& data, const ros::Publisher& pub)
{
    sensor_msgs::PointCloud2    cloud_msg;
    //ROS_INFO_STREAM(data->size());
    pcl::toROSMsg(*data, cloud_msg);
    cloud_msg.header.frame_id = "icp";
    cloud_msg.header.stamp = ros::Time::now();

    //pub_depth = nhTmpe.advertise<sensor_msgs::PointCloud2>("depth_point_cloud_visualservo", 2);
    pub.publish(cloud_msg);
    return;
}
 */
    }
}