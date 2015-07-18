#include <unsupported/Eigen/NonLinearOptimization>
#include "Eigen/Core"
#include <iostream>

#include "../../../../../../../usr/include/pcl-1.7/pcl/point_cloud.h"

#include "optimization/cxy_cost_func_example.h"
#include "utility/cxy_transform.h"
#include "kinematic/cxy_icp_kinematic_joint.h"
#include "kinematic/cxy_icp_kinematic_chain.h"
#include "energy/cxy_icp_rigid.h"
#include "energy/cxy_icp_arti_ik.h"
#include "utility/cxy_transform.h"
//#include "../../../../../../../usr/include/c++/4.8/bits/stl_bvector.h"
//#include "main.h"

using namespace  cxy;



using namespace cxy_optimization;
using namespace Eigen;
using namespace cxy_lmicp_lib;

typedef pcl::PointXYZ PointT;

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPlyFile(std::string name);

void publish(const pcl::PointCloud<PointT>::Ptr& data, const ros::Publisher& pub);

void initKinematicChain(std::vector<cxy_lmicp_lib::cxy_icp_kinematic_joint<float> >& kin_nodes, int chain_number);

pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud(float& Z);

pcl::PointCloud<PointT>::Ptr loadPlyFile(std::string name);



pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud(float& Z_io)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr data(new pcl::PointCloud<pcl::PointXYZ>);
    if (0)
    {
        srand(0);
        data = loadPlyFile("/home/xiongyi/repo/bun000.ply");
        pcl::PointCloud<PointT>::Ptr data_tmp(new pcl::PointCloud<PointT>);
        {
            const int& size = data->size();
            for (int ii = 0; ii < 2000; ++ii)
            {
                const int ramdon_idx = rand() % size;
                data_tmp->push_back((*data)[ramdon_idx]);
                if ((*data)[ramdon_idx].y > Z_io)
                {
                    Z_io = (*data)[ramdon_idx].y;
                }
            }
        }
        data = data_tmp;
    }
    else
    {
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
        Z_io = Z;
    }
    return data;
}
void initKinematicChain(std::vector<cxy_lmicp_lib::cxy_icp_kinematic_joint<float> >& kin_nodes, int chain_number)
{
pcl::PointCloud<pcl::PointXYZ>::Ptr data(new pcl::PointCloud<pcl::PointXYZ>);
float Z = 0.0;
data = getPointCloud(Z);
for (int ii = 0; ii < chain_number; ++ii)
{
cxy_lmicp_lib::cxy_icp_kinematic_joint<float> kcTmp;
if ( ii == 0)
{

ROS_INFO_STREAM("Z0 test = "<<kcTmp.pose_.t()(2));
kcTmp.pose_.t()(1) = 0.0;

/* code */
}
else
{
kcTmp.pose_.t()(2) = Z;
ROS_INFO_STREAM("Z = "<<Z);
}
kcTmp.setRotateAxis(cxy_transform::Axis::X_axis_rotation);
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
