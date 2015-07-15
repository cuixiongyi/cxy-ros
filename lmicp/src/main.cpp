#include "optimization/cxy_cost_func_example.h"
#include <unsupported/Eigen/NonLinearOptimization>
#include "Eigen/Core"
#include <iostream>
#include "cxy_icp_rigid.h"
#include "cxy_transform.h"
#include "cxy_icp_arti.h"
#include "cxy_icp_arti_ik.h"
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
    kc_root_list.push_back(0);
    kc_root_list.push_back(1);
    initKinematicChain(kin_nodes, 3);
    cxy_lmicp_lib::cxy_icp_kinematic_chain<float> kc;
    std::shared_ptr<std::vector<cxy_icp_kinematic_node<float>>> kc_nodes_ptr = std::make_shared<std::vector<cxy_icp_kinematic_node<float>>>(kin_nodes);
    kc.setKinematicNodes(kc_nodes_ptr);
    kc.setKinematicRootList(kc_root_list);

    
    Eigen::Matrix< float, Eigen::Dynamic, 1> x;

    x.resize(3);
    x.setZero();
    x(0) = Deg2Rad(20.0);
    x(1) = Deg2Rad(10.0);
    x(1) = Deg2Rad(10.0);


    // do the computation
    //cxy_lmicp_lib::cxy_icp_arti<float, 2> arti_icp;
    //lmicp.setModelCloud(data);
    //lmicp.icp_run(x);
    char c;
    float x2(20);
    //pcl::PointCloud<PointT>::Ptr transPoint(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr transPoint = kc.getFullModelCloud_World(x);

//    lmicp.setDataCloud(transPoint);
    while (1)
    {
        std::cin>>c;
        if ('b' == c)
          break;
        if ('t' == c)
        {
            std::cin>>x2;
            x(0) = Deg2Rad(x2);
            transPoint = kc.getFullModelCloud_World(x);

            publish(transPoint, pub_data_pointcloud_);
            //publish(resultPoint, pub_model_pointcloud_);
            //std::cout<<resultPoint->size()<<std::endl;
          continue;
        }
        if ('x' == c)
        {
            std::cin>>x2;
            x(0) = Deg2Rad(x2);
            continue;
        }
        /// articulate parameter optimization using CXY_LevenbergMarquate
        if ('r' == c)
        {

          pcl::PointCloud<PointT>::Ptr resultPoint(new pcl::PointCloud<PointT>);
            x(0) = 0.0;
            //x(1) = 0.0;


          cxy::cxy_lmicp_lib::cxy_icp_arti_ik<float, 1> arti_icp;
          std::shared_ptr<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> kc_ptr = std::make_shared<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> (kc);
          arti_icp.setKinematicChain(kc_ptr);
          arti_icp.setDataCloud(transPoint);
          arti_icp.icp_run(x);
          resultPoint = kc.getFullModelCloud_World(x);
          publish(transPoint, pub_data_pointcloud_);
          publish(resultPoint, pub_result_);
          continue;
        }
        /// articulate parameter optimization using Eigen LM
        if ('l' == c)
        {
            pcl::PointCloud<PointT>::Ptr resultPoint(new pcl::PointCloud<PointT>);
            //x(0) = 0.0;
            //x(1) = 0.0;
            /*
          Eigen::Matrix< float, 3, 1> rotation_axis(1.0, 0.0, 0.0);
          Eigen::Matrix< float, 3, 1> axis_out(1.0, 0.0, 0.0);
          cxy_transform::Pose<float> pose;
          float degtmp = Deg2Rad(90.0);
          pose.rotateByAxis(cxy_transform::Axis::Y_axis_rotation, degtmp);
          pose.composeDirectionVector(rotation_axis, axis_out);
          ROS_INFO_STREAM("q "<<pose.q().w()<<"  "<<pose.q().x()<<"  "<<pose.q().y()<<"  "<<pose.q().z()<<"  ");
          ROS_INFO_STREAM("inpute "<<rotation_axis);
          ROS_INFO_STREAM("output "<<axis_out);
          */
          cxy::cxy_lmicp_lib::cxy_icp_arti_ik<float, 2> arti_icp;
          std::shared_ptr<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> kc_ptr = std::make_shared<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> (kc);
          arti_icp.setKinematicChain(kc_ptr);
          arti_icp.setDataCloud(transPoint);
          ROS_INFO_STREAM("res = "<<arti_icp.icp_run(x));
          
          resultPoint = kc.getFullModelCloud_World(x);
          publish(transPoint, pub_data_pointcloud_);
          publish(resultPoint, pub_result_);
          continue;
        }
        if ('m' == c)
        {
            pcl::PointCloud<PointT>::Ptr resultPoint(new pcl::PointCloud<PointT>);
            //x(0) = 0.0;
            //x(1) = 0.0;
           
          cxy::cxy_lmicp_lib::cxy_icp_arti_ik<float, 2> arti_icp;
          std::shared_ptr<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> kc_ptr = std::make_shared<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> (kc);
          arti_icp.setKinematicChain(kc_ptr);
          arti_icp.setDataCloud(transPoint);
          arti_icp.icp_manifold();
          resultPoint = kc.getFullModelCloud_World(x);
          publish(transPoint, pub_data_pointcloud_);
          publish(resultPoint, pub_result_);
          continue;
        }
        /// articulate parameter optimization using Eigen LM
        /// convergence test
        if ('k' == c)
        {
            pcl::PointCloud<PointT>::Ptr resultPoint(new pcl::PointCloud<PointT>);
            x(0) = 0.0;
            //x(1) = 0.0;


          cxy::cxy_lmicp_lib::cxy_icp_arti_ik<float, 2> arti_icp;
          std::shared_ptr<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> kc_ptr = std::make_shared<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> (kc);
          arti_icp.setKinematicChain(kc_ptr);

          /// draw convergence map
          std::ofstream fout("/home/xiongyi/repo/arti-manifold-convergence.txt");
          const float delta = Deg2Rad(10.0);
          float theta_tmp = Deg2Rad(0.0);
          while (1)
          {
            float counter1(Deg2Rad(0.0));
            x(0) = theta_tmp;
            transPoint = kc.getFullModelCloud_World(x);
            arti_icp.setDataCloud(transPoint);
          
              while (1)
              {

                  x(0) = counter1;
                  fout<<counter1<<"  "<<theta_tmp<<"  "<<arti_icp.icp_run(x)<<" "<<x(0)<<std::endl;
                  std::cout<<counter1<<"  "<<theta_tmp<<"  "<<arti_icp.icp_run(x)<<" "<<x(0)<<std::endl;
                  if (counter1 >= Deg2Rad(360))
                      break;
                  counter1 += delta;

              } 
            if (theta_tmp >= Deg2Rad(360))
              return 1;
            theta_tmp += delta;
          } 
          resultPoint = kc.getFullModelCloud_World(x);
          publish(transPoint, pub_data_pointcloud_);
          publish(resultPoint, pub_result_);
          continue;
        }
        if ('d' == c)
        {
            pcl::PointCloud<PointT>::Ptr resultPoint(new pcl::PointCloud<PointT>);

          cxy::cxy_lmicp_lib::cxy_icp_arti_ik<float, 2> arti_icp;
          std::shared_ptr<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> kc_ptr = std::make_shared<cxy_lmicp_lib::cxy_icp_kinematic_chain<float>> (kc);
          arti_icp.setKinematicChain(kc_ptr);
          Eigen::Matrix< float, Eigen::Dynamic, 1> x_true;

          x_true.resize(3);
          x_true.setZero();
          
          
          /// draw convergence map
          //std::ofstream fout("/home/xiongyi/repo/arti-manifold-convergence.txt");
          const float delta = Deg2Rad(10.0);
          const float start_angle = Deg2Rad(-120.0);
          x_true(0) = start_angle;
          x_true(1) = Deg2Rad(-120.0);
          x_true(2) = Deg2Rad(10.0);
          x(0) = start_angle + Deg2Rad(0.0);
          x(1) = start_angle + Deg2Rad(0.0) + Deg2Rad(-10.0);
          x(2) = Deg2Rad(10.0) + Deg2Rad(0.0);
          while (1)
          {
            x_true(0) += delta;
            x_true(1) += 0.7*delta;
            x_true(2) += 0.5*delta;
            //if (x_true(1) <= Deg2Rad(120))
            if (0)
              return 1;
            transPoint = kc.getFullModelCloud_World(x_true);
            arti_icp.setDataCloud(transPoint);
            float res = arti_icp.icp_run(x);
            //x(0) = counter1;
            //fout<<counter1<<"  "<<theta_tmp<<"  "<<arti_icp.icp_run(x)<<" "<<x(0)<<std::endl;
            std::cout<<res<<"  "<<Rad2Deg(x(0)-x_true(0))<<" "<<Rad2Deg(x(1)-x_true(1))<<std::endl;

            
            resultPoint = kc.getFullModelCloud_World(x);
            for (int ii = 0; ii <resultPoint->size(); ++ii)
            {
              (*resultPoint)[ii].y += 0.25;
            }
            publish(transPoint, pub_data_pointcloud_);
            publish(resultPoint, pub_result_);
            ros::Duration d = ros::Duration(0.5);
            d.sleep();

          } 
          
          continue;
        }

    }



      std::cout<<x<<std::endl;
}

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
void initKinematicChain(std::vector<cxy_lmicp_lib::cxy_icp_kinematic_node<float> >& kin_nodes, int chain_number)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr data(new pcl::PointCloud<pcl::PointXYZ>);
  float Z = 0.0;
  data = getPointCloud(Z);  
    for (int ii = 0; ii < chain_number; ++ii)
    {
        cxy_lmicp_lib::cxy_icp_kinematic_node<float> kcTmp;
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
