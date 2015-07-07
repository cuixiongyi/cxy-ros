#include "optimization/cxy_cost_func_example.h"
#include <unsupported/Eigen/NonLinearOptimization>
#include "Eigen/Core"
#include <iostream>
#include "cxy_icp_rigid.h"
#include "cxy_transform.h"
#include "cxy_icp_arti.h"
#include "cxy_icp_arti_one.h"
#include "../../../../../../../usr/include/pcl-1.7/pcl/point_cloud.h"
#include "../include/cxy_transform.h"
#include "../include/cxy_icp_kinematic_node.h"
#include "../include/cxy_icp_kinematic_chain.h"
//#include "../../../../../../../usr/include/c++/4.8/bits/stl_bvector.h"
//#include "main.h"

using namespace  cxy;



using namespace cxy_optimization;
using namespace Eigen;
using namespace cxy_lmicp_lib;

typedef pcl::PointXYZ PointT;

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPlyFile(std::string name);

void publish(const pcl::PointCloud<PointT>::Ptr& data, const ros::Publisher& pub);

void initKinematicChain(std::vector<cxy_lmicp_lib::cxy_icp_kinematic_node<float> >& kin_nodes, int chain_number);

pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud(float& Z);

pcl::PointCloud<PointT>::Ptr loadPlyFile(std::string name);

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
    //kc_root_list.push_back(0);
    initKinematicChain(kin_nodes, 1);
    cxy_lmicp_lib::cxy_icp_kinematic_chain<float> kc;
    std::shared_ptr<std::vector<cxy_icp_kinematic_node<float>>> kc_nodes_ptr = std::make_shared<std::vector<cxy_icp_kinematic_node<float>>>(kin_nodes);
    kc.setKinematicNodes(kc_nodes_ptr);
    kc.setKinematicRootList(kc_root_list);

    
    Eigen::Matrix< float, Eigen::Dynamic, 1> x;

    x.resize(1);
    x.setZero();
    x(0) = 20.0;
    //x(1) = 40.0;


    // do the computation
    //cxy_lmicp_lib::cxy_icp_arti<float, 2> arti_icp;
    //lmicp.setModelCloud(data);
    //lmicp.icp_run(x);
    char c;
    float x2(20);
    //pcl::PointCloud<PointT>::Ptr transPoint(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr transPoint = kc.getFullModelCloud_World(x);

//    lmicp.setDataCloud(transPoint);
    cxy_lmicp_lib::cxy_icp_arti_one<float, 2> one_icp;
    while (1)
    {
        std::cin>>c;
        if ('b' == c)
          break;
        if ('t' == c)
        {
            std::cin>>x2;
            x(0) = x2;
            transPoint = kc.getFullModelCloud_World(x);

            publish(transPoint, pub_data_pointcloud_);
            //publish(resultPoint, pub_model_pointcloud_);
            //std::cout<<resultPoint->size()<<std::endl;
          continue;
        }
        /// articulate parameter optimization using CXY_LevenbergMarquate
        if ('r' == c)
        {

          pcl::PointCloud<PointT>::Ptr resultPoint(new pcl::PointCloud<PointT>);
            x(0) = 0.1;
            //x(1) = 0.0;


          cxy::cxy_lmicp_lib::cxy_icp_arti<float, 1> arti_icp;
          std::shared_ptr<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> kc_ptr = std::make_shared<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> (kc);
          arti_icp.setKinematicChain(kc_ptr);
          arti_icp.setDataCloud(transPoint);
          arti_icp.icp_run(x);
          resultPoint = kc.getFullModelCloud_World(x);
          publish(transPoint, pub_data_pointcloud_);
          publish(resultPoint, pub_model_pointcloud_);
          continue;
        }
        /// articulate parameter optimization using Eigen LM
        if ('l' == c)
        {
            pcl::PointCloud<PointT>::Ptr resultPoint(new pcl::PointCloud<PointT>);
            x(0) = 0.1;
            //x(1) = 0.0;


          cxy::cxy_lmicp_lib::cxy_icp_arti<float, 2> arti_icp;
          std::shared_ptr<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> kc_ptr = std::make_shared<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> (kc);
          arti_icp.setKinematicChain(kc_ptr);
          arti_icp.setDataCloud(transPoint);
          arti_icp.icp_run(x);
          resultPoint = kc.getFullModelCloud_World(x);
          publish(transPoint, pub_data_pointcloud_);
          publish(resultPoint, pub_model_pointcloud_);
          continue;
        }
        /// articulate parameter optimization using Eigen LM
        /// convergence test
        if ('k' == c)
        {
            pcl::PointCloud<PointT>::Ptr resultPoint(new pcl::PointCloud<PointT>);
            x(0) = 0.1;
            //x(1) = 0.0;


          cxy::cxy_lmicp_lib::cxy_icp_arti<float, 2> arti_icp;
          std::shared_ptr<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> kc_ptr = std::make_shared<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> (kc);
          arti_icp.setKinematicChain(kc_ptr);

          /// draw convergence map
          std::ofstream fout("/home/xiongyi/repo/arti-manifold-convergence.txt");
          float theta_tmp = -180.0;
          while (1)
          {
            const float delta = 6.0;
            float counter1(-180);
            x(0) = theta_tmp;
            transPoint = kc.getFullModelCloud_World(x);
            arti_icp.setDataCloud(transPoint);
          
              while (1)
              {

                  x(0) = counter1;
                  fout<<x(0)<<"  "<<theta_tmp<<"  "<<arti_icp.icp_run(x)<<std::endl;
                  if (counter1 >= 174)
                      break;
                  counter1 += delta;

              } 
            if (theta_tmp >= 174)
              return 1;
            theta_tmp += delta;
          } 
          resultPoint = kc.getFullModelCloud_World(x);
          publish(transPoint, pub_data_pointcloud_);
          publish(resultPoint, pub_model_pointcloud_);
          continue;
        }
        /// one parameter optimization
        if ('o' == c)
        {
          float z = 0.0;
          static pcl::PointCloud<PointT>::Ptr data = getPointCloud(z);
          pcl::PointCloud<PointT>::Ptr resultPoint(new pcl::PointCloud<PointT>);
          /*pcl::PointCloud<PointT>::Ptr transPoint(new pcl::PointCloud<PointT>);
          cxy_transform::Pose<float> pose;
          float theta_tmp = -20.0;
          pose.rotateByAxis(cxy_transform::Axis::X_axis_rotation, theta_tmp);
          pose.composePoint(data, transPoint);
          */
          one_icp.setModelCloud(data);
          one_icp.setDataCloud(transPoint);
          Eigen::Matrix< float, Eigen::Dynamic, 1> x;
          x.resize(1);
          x(0) = 0.0;
          
          /// draw convergence map
          /*std::ofstream fout("/home/xiongyi/repo/one-para-manifold-convergence.txt");
          theta_tmp = -180;
          while (1)
          {
            const float delta = 6.0;
            float counter1(-180);
            cxy_transform::Pose<float> pose;
            pose.rotateByAxis(cxy_transform::Axis::X_axis_rotation, theta_tmp);
            pose.composePoint(data, transPoint);
            one_icp.setDataCloud(transPoint);
              while (1)
              {

                  x(0) = counter1;
                  fout<<x(0)<<"  "<<theta_tmp<<"  "<<one_icp.icp_run(x)<<std::endl;
                  if (counter1 >= 174)
                      break;
                  counter1 += delta;

              } 
              theta_tmp += delta;
              if (theta_tmp >= 174)
                return 1;
          }    */      
          one_icp.icp_run(x);
          //pose = cxy_transform::Pose<float>::rotateByAxis_fromIdentity(cxy_transform::Axis::X_axis_rotation, x(0));
          
          
          //pose.composePoint(transPoint, resultPoint);
          publish(transPoint, pub_data_pointcloud_);
          publish(resultPoint, pub_result_);
          continue;
        }


    }



      std::cout<<x<<std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud(float& Z_io)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr data(new pcl::PointCloud<pcl::PointXYZ>);
  if (1)
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
            if ((*data)[ramdon_idx].z > Z_io)
            {
              Z_io = (*data)[ramdon_idx].z;
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
void initKinematicChain(std::vector<cxy_lmicp_lib::cxy_icp_kinematic_node<float> >& kin_nodes, int chain_number)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr data(new pcl::PointCloud<pcl::PointXYZ>);
  float Z = 0.0;
  data = getPointCloud(Z);  
    for (int ii = 0; ii < chain_number; ++ii)
    {
        cxy_lmicp_lib::cxy_icp_kinematic_node<float> kcTmp;
        if ( ii != 0)
        {
            kcTmp.pose_.t()(2) = Z;
            /* code */
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

// test rotateByAxis
/*
char c;
      cxy_transform::Pose<float> pose;
      while (1)
      {
        std::cin>>c;
        if ('n' == c)
            break;
        pose.rotateByAxis(cxy_transform::Axis::X_axis_rotation, 30.0);
        pcl::PointCloud<PointT>::Ptr transPoint(new pcl::PointCloud<PointT>);
        pose.composePoint(data, transPoint);

        publish(data, pub_data_pointcloud_);
        publish(transPoint, pub_model_pointcloud_);
      }
*/
