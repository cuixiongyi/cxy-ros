
#include "perception_model_based_detection/mbt_callback.h"
namespace cxy_mbt
{
VisualServo_Callback::VisualServo_Callback() :
                        camerInitFrameCount_Max(3)
//ros::NodeHandle nh, ros::NodeHandle pnh
{
    
  pub_depth = nh_.advertise<sensor_msgs::PointCloud2>("depth_point_cloud_visualservo", 2);
//   listener; 
    //nh_ = nh;
    //pnh_ = pnh;
}
VisualServo_Callback::~VisualServo_Callback()
//ros::NodeHandle nh, ros::NodeHandle pnh
{
//   listener; 
    //nh_ = nh;
    //pnh_ = pnh;
}
void VisualServo_Callback::callback_GuessPose(const geometry_msgs::PoseStamped::ConstPtr pose)
{
    geometry_msgs::PoseStamped poseTmp;
    listener.transformPose("l_foot", *pose, poseTmp);

    /*
    while(1)     
    {
        try
        {
            listener.lookupTransform(mbt_Config::getInstance()->mbt_frame_id_, pose->frame_id,ros::Time(0), itransform);
            listener.transformPose(mbt_Config::getInstance()->mbt_frame_id_, pose, poseTmp);
            break;
        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.1).sleep();
        }
    }
    */
    mbt_Config::getInstance()->newtime_guesspose_ = ros::Time::now();
    mbt_Config::getInstance()->newpose_ = poseTmp;
    mbt_Config::getInstance()->isnew_guesspose_= true;
}
void VisualServo_Callback::callback_DepthCamera(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input)
{
    static tf::TransformBroadcaster                       br;
    static tf::Transform                                  tftransform;
    static tf::TransformListener                    lis;
    Eigen::Quaternion<double> q1(Eigen::AngleAxisd(-3.14/2, Eigen::Vector3d(1,0,0)));
    Eigen::Quaternion<double> q2(Eigen::AngleAxisd(-3.14/2, Eigen::Vector3d(0,0,1)));
    Eigen::Quaternion<double> q3(Eigen::AngleAxisd(-1.919, Eigen::Vector3d(0,1,0)));
    //aff.rotate();
    //aff.rotate(Eigen::AngleAxisd(-3.14/2, Eigen::Vector3d(0,1,0)));
    //aff.rotate(Eigen::AngleAxisd(1.919, Eigen::Vector3d(0,0,1)));
    q1 = q3*q2*q1;
    tftransform.setOrigin(tf::Vector3(0.04,0.0,0.1));
    std::vector< double > qeig{q1.x(), q1.y(), q1.z(),q1.w()};

    tftransform.setRotation(tf::Quaternion(qeig[0],qeig[1],qeig[2],qeig[3]));
    const static std::string l_hand_frame = "l_palm";
    const static std::string l_depth_cam_frame (mbt_Config::getInstance()->mbt_frame_id_);
    const int camerInitFrameCount_(mbt_Config::getInstance()->cam_init_count_);
    br.sendTransform(tf::StampedTransform(tftransform, ros::Time::now(), l_hand_frame, l_depth_cam_frame));
    

  if (camerInitFrameCount_ < camerInitFrameCount_Max)
  {
    mbt_Config::getInstance()->cam_init_count_++;
    return;
  }
  if (input == NULL || input->size() < 80 || std::isnan(input->size()))
  {
    ROS_INFO("Depth camera input invalid");
    return;
  }
    tf::StampedTransform transform;
    //Eigen::Matrix4d trans = Eigen::Matrix4d
    while(true)
    {
      try{
          lis.lookupTransform (l_depth_cam_frame, l_hand_frame, ros::Time(0), transform);
          break;
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
       }
   }
  std::vector<double> vq{(double)transform.getRotation().w(),(double)transform.getRotation().x(),(double)transform.getRotation().y(),(double)transform.getRotation().z()};
  Eigen::Quaternion< double > q(vq[0], vq[1], vq[2],vq[3]);
  Eigen::Matrix3d m(q.matrix());
  Eigen::Vector3d v(transform.getOrigin().getX(),transform.getOrigin().getX(),transform.getOrigin().getX());
  Eigen::Matrix4d trans;
  trans << m(0, 0), m(0, 1), m(0, 2), v(0),
                 m(1, 0), m(1, 1), m(1, 2), v(1),
                 m(2, 0), m(2, 1), m(2, 2), v(2),
                 0,       0,        0,      1;
    trans << 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0,       0,        0,      1;
    pcl::PointCloud<PointT>::Ptr target_ptr = boost::make_shared<pcl::PointCloud<PointT> >();
  pcl::transformPointCloud(*input, *target_ptr, trans);
  
    //pcl::fromROSMsg (*input, cloud_in);
    pcl::PointCloud<PointT>::Ptr cloud_cut = boost::make_shared<pcl::PointCloud<PointT> >();
    pcl::PointCloud<PointT>::Ptr tmpcloud = boost::make_shared<pcl::PointCloud<PointT> >();
    pcl::PointCloud<PointT>::Ptr cloud_out = boost::make_shared<pcl::PointCloud<PointT> >();
    
    for (int ii = 0; ii < target_ptr->width; ++ii)
    {
      for (int jj = 0; jj < target_ptr->height; ++jj)
      {
        tmpcloud->push_back(target_ptr->points[target_ptr->width*jj+ii]);
      }
    }
    tmpcloud->height = target_ptr->width;
    tmpcloud->width = target_ptr->height;
    target_ptr = tmpcloud;

    sensor_msgs::PointCloud2    cloud_msg;
  pcl::toROSMsg(*target_ptr, cloud_msg);
  cloud_msg.header.frame_id = l_hand_frame;
  cloud_msg.header.stamp = ros::Time::now();
  
  pub_depth.publish(cloud_msg);

    //publishTFFrame("l_palm");
    pcl::PointCloud<pcl::PointXYZ>::Ptr rawPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //convertPointCloudTypes(rawPointCloud, input);
    if(target_ptr == NULL || target_ptr->size()<10) 
    {
        ROS_INFO_ONCE("empty pointcloud from depth camera");
        return;
    }
    

    /*
    ROS_INFO_ONCE("Received cloud.");
    ROS_INFO_STREAM_ONCE("Cloud size - " << rawPointCloud->points.size());
    */
    
    mbt_Config::getInstance()->isnew_depth_ = true;
    mbt_Config::getInstance()->newtime_depth_ = ros::Time::now();
    mbt_Config::getInstance()->newdepth_ = target_ptr;

    

    return;
}

void VisualServo_Callback::callback_VisualServoState(const std_msgs::Int16::ConstPtr& state)
{
    mbt_Config::getInstance()->newtime_visualservostate_ = ros::Time::now();
    mbt_Config::getInstance()->isnew_visualservostate_ = true;
    mbt_Config::getInstance()->newvisualservostate_ = state->data;
    return;
}

void VisualServo_Callback::publishTFFrame(std::string name)
{
    //tftransform.setOrigin(tf::Vector3(0.0,0.0,0.0));
    //tftransform.setRotation(tf::Quaternion(1.0,0.0,0.0,0.0));
    //br_.sendTransform(tf::StampedTransform(tftransform, ros::Time::now(), name, mbt_Config::getInstance()->mbt_frame_id_));

}

bool VisualServo_Callback::convertPointCloudTypes(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud_in)
{
    if (cloud_out == NULL)
        cloud_out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    if (cloud_in == NULL)
        return false;
    for (int i = 0; i < cloud_in->points.size(); i++)
    {
        pcl::PointXYZ   temp_pt;
        temp_pt.x = cloud_in->points[i].x;
        temp_pt.y = cloud_in->points[i].y;
        temp_pt.z = cloud_in->points[i].z;
        cloud_out->points.push_back(temp_pt);
    }
    //ROS_INFO_STREAM("Cloud size - " << cloud_out->points.size());
    return true;
}






}
