#include "optimization/cxy_cost_func_example.h"
#include <unsupported/Eigen/NonLinearOptimization>
#include "Eigen/Core"
#include <iostream>
#include "cxy_icp_rigid.h"
#include "cxy_transform.h"
#include "cxy_icp_arti.h"
//#include "main.h"

using namespace  cxy;



using namespace cxy_optimization;
using namespace Eigen;

typedef pcl::PointXYZ PointT;

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPlyFile(std::string name);

void publish(const pcl::PointCloud<PointT>::Ptr& data, const ros::Publisher& pub);

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
/*
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFile("bun045.ply",mesh);
    pcl::fromROSMsg(mesh.cloud, *data);
    pcl::io::loadPolygonFile("bun090.ply",mesh);
    pcl::fromROSMsg(mesh.cloud, *model);
*/
    //std::ifstream fin_tar("bun045.ply");
    //std::ifstream fin_mod("bun090.ply");

    data = loadPlyFile("/home/xiongyi/repo/bun000.ply");
    if (0)
    {
        model = loadPlyFile("/home/xiongyi/repo/bun045.ply");
    }
    else
    {
        for (int i = 0; i < data->points.size(); ++i)
        {
            model->push_back(pcl::PointXYZ(data->points[i].x, data->points[i].y+0.01, data->points[i].z+0.01));
        }
    }


    Eigen::Matrix< float, Eigen::Dynamic, 1> x;
    Eigen::Matrix< float, Eigen::Dynamic, 1> x2;
    /* the following starting values provide a rough fit. */
    x.resize(7);
    x.setZero();
    x(3) = 1.0;
    x(4) = 0.01;
    x(5) = 0.01;
    x(6) = 0.01;

    //x(1) = 0.01;

    // do the computation
    cxy_lmicp_lib::cxy_icp_rigid<float, 1> lmicp;
    lmicp.setModelCloud(data);
    //lmicp.icp_run(x);
    char c;
    pcl::PointCloud<PointT>::Ptr transPoint(new pcl::PointCloud<PointT>);

    cxy_transform::Pose<float> pose;
    pose.rotateByAxis(cxy_transform::Axis::X_axis, 30.0);

    pose.composePoint(data, transPoint);
    lmicp.setDataCloud(transPoint);
    cxy_transform::Pose<float> t1_0;
    while (1)
    {
        std::cin>>c;
        if ('b' == c)
            break;
        if ('t' == c)
        {
            pose.composePoint(data, transPoint);
            std::cout<<"true = "<<pose.q().w()<<"  "<<pose.q().x()<<std::endl;
            publish(data, pub_data_pointcloud_);
            publish(transPoint, pub_model_pointcloud_);
            continue;
        }
        if ('r' == c)
        {
            lmicp.icp_run(x);
            cxy_transform::Pose<float> pose2;
            pose2.q().w() = x(0);
            pose2.q().x() = x(1);
            std::cout<<"result = "<<x(0)<<"  "<<x(1)<<std::endl;
            pcl::PointCloud<PointT>::Ptr resultPoint(new pcl::PointCloud<PointT>);
            pose2.composePoint(transPoint, resultPoint);
            publish(data, pub_data_pointcloud_);
            publish(resultPoint, pub_model_pointcloud_);
            continue;
        }
        

        while (1)
        {
            float in;
            cxy_transform::Pose<float> pose2;
            std::cin>>in;
            pose2.rotateByAxis(cxy_transform::Axis::X_axis, in);

            pcl::PointCloud<PointT>::Ptr resultPoint(new pcl::PointCloud<PointT>);
            pose2.composePoint(transPoint, resultPoint);
            std::cout<<"result = "<<pose2.q().w()<<"  "<<pose2.q().x()<<std::endl;
            publish(data, pub_data_pointcloud_);
            publish(resultPoint, pub_model_pointcloud_);
        }
    }



    std::cout<<x<<std::endl;
}


pcl::PointCloud<PointT>::Ptr loadPlyFile(std::string name)
{


    pcl::PointCloud<PointT>::Ptr pointcloud(new pcl::PointCloud<PointT>);
    std::ifstream fin(name);
    ROS_INFO_STREAM(fin.is_open());
    std::string line;
    long int count(0);
    while (std::getline(fin, line))
    {
        //std::getline(fin, line, '\n');
        std::size_t pos(line.find("element vertex"));
//        ROS_INFO_STREAM(line);
        if ( std::string::npos != pos)
        {
            std::string tmp(line.begin()+pos+14, line.end());
            count = atol(tmp.c_str());
            //ROS_INFO_STREAM("read count  "<< "  "<<tmp << ".");
            ROS_INFO_STREAM(line <<std::endl);
            continue;
        }
        pos = line.find("end_header");
        if ( std::string::npos != pos)
        {
            //ROS_INFO("read end");
            break;
        }

    }
    std::cout<<"PointCloud Num   "<< count<<std::endl;
    pointcloud->reserve(count);
    for (int i = 0; i < count; ++i)
    {
        /* code */
        PointT p;
        fin >> p.x >> p.y >> p.z;
        pointcloud->push_back(p);
        //ROS_INFO_STREAM("points  "<< "  "<<p.x);

    }
    return pointcloud;
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