#include "optimization/cxy_cost_func_example.h"
#include <unsupported/Eigen/NonLinearOptimization>
#include "Eigen/Core"
#include <iostream>
#include "cxy_icp_rigid.h"
#include "cxy_transform.h"
#include "cxy_icp_arti.h"
#include "../../../../../../../usr/include/pcl-1.7/pcl/point_cloud.h"
#include "../include/cxy_transform.h"
#include "../include/cxy_icp_kinematic_node.h"
#include "../include/cxy_icp_kinematic_chain.h"
#include "../../../../../../../usr/include/c++/4.8/bits/stl_bvector.h"
//#include "main.h"

using namespace  cxy;



using namespace cxy_optimization;
using namespace Eigen;
using namespace cxy_lmicp_lib;

typedef pcl::PointXYZ PointT;

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPlyFile(std::string name);

void publish(const pcl::PointCloud<PointT>::Ptr& data, const ros::Publisher& pub);

void initKinematicChain(std::vector<cxy_lmicp_lib::cxy_icp_kinematic_node<float> >& kin_nodes, int chain_number);

int main(int argc, char *argv[])
{
      int n = 3, info;
    ros::init(argc,argv, "lmicp");

    pcl::PointCloud<PointT>::Ptr data(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr model(new pcl::PointCloud<PointT>);

ros::Publisher pub_model_, pub_model_pointcloud_, pub_data_pointcloud_, pub_result_;
            ros::NodeHandle nh_, pnh_;

            pub_model_ = nh_.advertise<visualization_msgs::MarkerArray>("model", 5);
            pub_data_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("data_pointcloud", 5);
            pub_model_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("model_pointcloud", 5);
            pub_result_ = nh_.advertise<sensor_msgs::PointCloud2>("data_result", 5);


    std::vector<cxy_lmicp_lib::cxy_icp_kinematic_node<float>> kin_nodes;
    std::vector<int> kc_root_list;
    kc_root_list.push_back(-1);
    kc_root_list.push_back(0);
    initKinematicChain(kin_nodes, 2);
    cxy_lmicp_lib::cxy_icp_kinematic_chain<float> kc;
    std::shared_ptr<std::vector<cxy_icp_kinematic_node<float>>> kc_nodes_ptr = std::make_shared<std::vector<cxy_icp_kinematic_node<float>>>(kin_nodes);
    kc.setKinematicNodes(kc_nodes_ptr);
    kc.setKinematicRootList(kc_root_list);

    Eigen::Matrix< float, Eigen::Dynamic, 1> x;

    x.resize(2);
    x.setZero();
    x(0) = 0.0;
    x(1) = 0.0;
    

    // do the computation
    //cxy_lmicp_lib::cxy_icp_arti<float, 2> arti_icp;
    //lmicp.setModelCloud(data);
    //lmicp.icp_run(x);
    char c;
    float x2(40);
    //pcl::PointCloud<PointT>::Ptr transPoint(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr transPoint = kc.getFullModelCloud(x);

//    lmicp.setDataCloud(transPoint);
    while (1)
    {
        std::cin>>c;
        if ('b' == c)
          break;
        if ('t' == c)
        {
            pcl::PointCloud<PointT>::Ptr resultPoint(kc.getFullModelCloud(x));

            publish(data, pub_data_pointcloud_);
            publish(resultPoint, pub_model_pointcloud_);
            //std::cin>>x2;
            std::cout<<resultPoint->size()<<std::endl;
            x(1) = x2;
          continue;
        }
        if ('r' == c)
        {

          pcl::PointCloud<PointT>::Ptr resultPoint(new pcl::PointCloud<PointT>);

          publish(data, pub_data_pointcloud_);
          publish(resultPoint, pub_model_pointcloud_);
          continue;
        }
        if ('l' == c)
        {

          continue;
        }


    }



      std::cout<<x<<std::endl;
}

void initKinematicChain(std::vector<cxy_lmicp_lib::cxy_icp_kinematic_node<float> >& kin_nodes, int chain_number)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr data(new pcl::PointCloud<pcl::PointXYZ>);
    const int pointSize = 200;
    const float delta_X = 0.05;
    const float delta_Y = 0.025;
    const float delta_Z = -0.025;
    data->points.reserve(pointSize);
    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;
    for (int ii = 0; ii < pointSize; ++ii)
    {
        data->push_back(PointT(X, Y, Z));
        X += delta_X;
        if ( X >= 0.3)
        {
            Z += delta_Z;
            X = 0.0;
        }
    }
    for (int ii = 0; ii < chain_number; ++ii)
    {
        cxy_lmicp_lib::cxy_icp_kinematic_node<float> kcTmp;
        kcTmp.pose_.t()(2) = Z;
        kcTmp.setRotateAxis(cxy_transform::Axis::X_axis);
        kcTmp.setModelCloud(data);
        kin_nodes.push_back(kcTmp);
    }
}


void publish(const pcl::PointCloud<PointT>::Ptr& data, const ros::Publisher& pub)
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



// test rotateByAxis
/*
char c;
      cxy_transform::Pose<float> pose;
      while (1)
      {
        std::cin>>c;
        if ('n' == c)
            break;
        pose.rotateByAxis(cxy_transform::Axis::X_axis, 30.0);
        pcl::PointCloud<PointT>::Ptr transPoint(new pcl::PointCloud<PointT>);
        pose.composePoint(data, transPoint);

        publish(data, pub_data_pointcloud_);
        publish(transPoint, pub_model_pointcloud_);
      }
*/
