#include <memory>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

//#include "common/cxy_config.h"
#include "common/cxy_common.h"
#include "utility/cxy_transform.h"
#include "kinematic/cxy_icp_kinematic.h"
#include "tracker/cxy_tracker.h"
#include "utility/cxy_publisher.h"

using namespace cxy;

int main(int argc, char  *argv[])
{
    ros::init(argc, argv, "my_node_name");
    ros::NodeHandle nh;

	//cxy::config("common/config");
    std::shared_ptr<const cxy::cxy_config> config_ptr = cxy::cxy_config::getConfig();
    cxy::cxy_config::unserialize();
    cxy::cxy_tracker<float> tracker(config_ptr.get());
    cxy::cxy_tracker<float> data(config_ptr.get());
    //cxy::cxy_kinematic::cxy_icp_kinematic<float> kin_data(config_ptr.get());
    Eigen::Matrix< float, Eigen::Dynamic, 1> x_data(cxy_config::joint_DoFs);
    x_data.setZero();

    // tf frame
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", cxy_config::rviz_frame_name_));

    /*
     * use synthesized data test tracking
     */
    pcl::PointCloud<PointT>::Ptr dataCloud;
    pcl::PointCloud<PointT>::Ptr modelCloud;
    char key;
    const float trans_inc = 1;
    float rot_inc = Deg2Rad(2.0);

    //rot_inc = 2.5;
    srand(cxy_config::random_seed);
    while (1)
    {
        std::cin>>key;
        if ('q' == key)
            return 0;
        if ('t' == key)
        {
            data.setX(x_data);
            dataCloud = data.getVisibleModelCloud();

            tracker.setDataCloud(dataCloud);
            tracker.runOptimization();
            //std::cout<<"result = "<<std::endl<<Rad2Deg(tracker.getX())<<std::endl;
            //std::cout<<"true = "<<std::endl<<Rad2Deg(x_data)<<std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr fullCloud (new pcl::PointCloud<pcl::PointXYZ>);
            modelCloud = tracker.getVisibleModelCloud(fullCloud);

            //cxy_publisher::publishDataPoint(dataCloud);
            cxy_publisher::publishDataPoint(dataCloud);
            cxy_publisher::publishModelPoint(modelCloud);
            cxy_publisher::publishMeshModel(tracker.getModelMarkerArray());

        }

        if ('r' == key)
        {
            //x_data(1) += trans_inc;
            //x_data(2) += trans_inc;
            //x_data(4) += rot_inc;
            //x_data(5) += rot_inc;
            //x_data(7) += rot_inc;
            for (int ii = 6; ii < cxy_config::joint_DoFs; ++ii)
            {
                //x_data(ii) += Deg2Rad(rand() % 5);
                x_data(ii) += rot_inc;
            }
            for (int ii = 0; ii < 3; ++ii)
            {
                //x_data(ii) += rand() % 5;
            }


        }
        if ('p' == key)
        {
            data.setX(x_data);
            dataCloud = data.getVisibleModelCloud();

            cxy_publisher::publishDataPoint(dataCloud);
            cxy_publisher::publishModelPoint(modelCloud);
            cxy_publisher::publishMeshModel(tracker.getModelMarkerArray());

        }
        if ('a' == key)
        {
            const int times = 60;
            int sign = 1;
            for (int jj = 0; jj < times; ++jj)
            {

                if (jj > times /2)
                    sign = -1;
                for (int ii = 6; ii < cxy_config::joint_DoFs; ++ii)
                {
                    //x_data(ii) += Deg2Rad(rand() % 5);
                    x_data(ii) += sign * rot_inc;
                }
                data.setX(x_data);
                dataCloud = data.getVisibleModelCloud();

                tracker.setDataCloud(dataCloud);
                tracker.runOptimization();
                //std::cout<<"result = "<<std::endl<<Rad2Deg(tracker.getX())<<std::endl;
                //std::cout<<"true = "<<std::endl<<Rad2Deg(x_data)<<std::endl;
                pcl::PointCloud<pcl::PointXYZ>::Ptr fullCloud (new pcl::PointCloud<pcl::PointXYZ>);
                modelCloud = tracker.getVisibleModelCloud(fullCloud);

                //cxy_publisher::publishDataPoint(dataCloud);
                for (int ii = 0; ii < dataCloud->size(); ++ii)
                {
                    dataCloud->points[ii].x += 110;
                }
                cxy_publisher::publishDataPoint(dataCloud);
                cxy_publisher::publishModelPoint(modelCloud);
                cxy_publisher::publishMeshModel(tracker.getModelMarkerArray());
            }



        }
    }

}

