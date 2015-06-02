
#include "perception_model_based_detection/mbt_action.h"
#include "perception_model_based_detection/mbt_config.h"
namespace cxy_mbt
{

    
    mbt_Action::mbt_Action(std::string action_name) :
    //ros::NodeHandle nh, ros::NodeHandle pnh
        action_name_(action_name),
        nh_(""),
        as_(nh_, action_name, boost::bind(&cxy_mbt::mbt_Action::executeCB, this, _1), false)
        //
    {
        ROS_INFO("mbt_Action::mbt_Action Start action");
        as_.start();
    }

    void mbt_Action::executeCB(const perception_msgs::visual_servo_LRGoalConstPtr &goal)
    {
        static bool success = false;
        ROS_INFO("mbt_Action::executeCB Start action");
        mbt_Config::getInstance()->action_goal_ = *goal;         
        ros::Time ttmp = mbt_Config::getInstance()->newtime_transform_;
        if ( ! mbt_Config::isSynchronized(ttmp))
            return ;
        Eigen::Matrix4d transform = mbt_Config::getInstance()->transform_;
        if (as_.isPreemptRequested() || ! ros::ok())
        {
            ROS_INFO("action preempted");
            as_.setPreempted();
            result_.done = false; 
        }
        if (transform (2,3) < 0.1)
        {
            success = true;
            ROS_INFO("action successssssssssssss"); 
            
        }
        /// if success reset variables
        if (success)
        {
            //success = false; 
            result_.done = true; 
            as_.setSucceeded(result_);
            mbt_Config::getInstance()->isnew_guesspose_ = false;
            mbt_Config::getInstance()->isnew_visualservostate_ = false;
        }
        return ;
    }


}
