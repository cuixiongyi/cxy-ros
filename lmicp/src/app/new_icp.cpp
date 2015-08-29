#include <memory>
#include "ros/ros.h"

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
    /*
     * use synthesized data test tracking
     */
    pcl::PointCloud<PointT>::Ptr dataCloud;
    pcl::PointCloud<PointT>::Ptr modelCloud;
    char key;
    const float trans_inc = 0.1;
    const float rot_inc = 2.0;
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
            std::cout<<tracker.runOptimization()<<std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr fullCloud (new pcl::PointCloud<pcl::PointXYZ>);
            modelCloud = tracker.getVisibleModelCloud(fullCloud);

            //cxy_publisher::publishDataPoint(dataCloud);
            cxy_publisher::publishDataPoint(dataCloud);
            cxy_publisher::publishModelPoint(modelCloud);

        }
        if ('r' == key)
        {
            x_data(0) += trans_inc;
            //x_data(4) += rot_inc;
        }
        if ('p' == key)
        {
            cxy_publisher::publishDataPoint(dataCloud);
            cxy_publisher::publishModelPoint(modelCloud);

        }
    }

}

