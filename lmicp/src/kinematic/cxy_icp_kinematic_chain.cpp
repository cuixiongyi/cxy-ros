#include "kinematic/cxy_icp_kinematic_chain.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {
        template<typename _Scalar>
        cxy_icp_kinematic_chain<_Scalar>::cxy_icp_kinematic_chain(const std::shared_ptr<cxy_config>& config)
        {
            config_ = config;
        }

        template<typename _Scalar>
        pcl::PointCloud<pcl::PointXYZ>::Ptr cxy_icp_kinematic_chain<_Scalar>::getFullModelCloud_World(Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x)
        {
            CXY_ASSERT(1);
            CXY_ASSERT(x.rows() == config_->joint_number_);
            unsigned int pointCloudSize = 0;
            for (int ii = 0; ii < x.rows(); ++ii)
            {
                pointCloudSize += joints_[ii]->getModelCloud()->size();

            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud (new pcl::PointCloud<pcl::PointXYZ>);
            transCloud->reserve(pointCloudSize);
            for (int ii = 0; ii < x.rows(); ++ii)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZ>);
                cxy_transform::Pose<_Scalar> pose;
                tmpCloud = getOneModelCloud_World(x, ii, pose);
                //ROS_INFO_STREAM("w = "<<pose.q().w()<<" z = "<<pose.t()(2));
                for (int jj = 0; jj < tmpCloud->size(); ++jj)
                {
                    transCloud->push_back((*tmpCloud)[jj]);
                    //ROS_INFO_STREAM((*kc_nodes_)[joint].modelCloud_[ii].x<<" "<<(*kc_nodes_)[joint].modelCloud_[ii].y<<"  "<<[ii].z);
                    /* code */
                }
                //ROS_INFO_STREAM(" ");
            }
            return transCloud;

        }

        template<typename _Scalar>
        pcl::PointCloud<pcl::PointXYZ>::Ptr cxy_icp_kinematic_chain<_Scalar>::getOneModelCloud_World(
                                            Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x
                                          , const int& joint
                                          , cxy_transform::Pose<_Scalar>& pose
                                          , cxy_transform::Pose<_Scalar>& pose_parent )
        {
            CXY_ASSERT(x.rows() == config_->joint_number_);
            CXY_ASSERT(x.rows() >= joint);

            //cxy_transform::Pose<_Scalar> pose_world;
            getKinematicPose2World(x, joint, pose, pose_parent);
            //ROS_INFO_STREAM("pose World: "<<joint<<" "<<pose_world.t()(0)<<" "<<pose_world.t()(1)<<" "<<pose_world.t()(2)<<" "<<pose_world.q().x()<<" "<<pose_world.q().y()<<" "<<pose_world.q().z()<<" "<<pose_world.q().w());

            pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud (new pcl::PointCloud<pcl::PointXYZ>);
            pose.composePoint(joints_[ii]->getModelCloud(), transCloud);
            return transCloud;
        }


        template<typename _Scalar>
        pcl::PointCloud<pcl::PointXYZ>::Ptr cxy_icp_kinematic_chain<_Scalar>::getOneModelCloud_World(
                                            Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x
                                          , const int& joint
                                          , cxy_transform::Pose<_Scalar>& pose)
                                        
        {
            cxy_transform::Pose<_Scalar> pose_parent;
            return getOneModelCloud_World(x, joint, pose, pose_parent);
        }


        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::getKinematicPose2World(
                                              Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x
                                            , const int& joint
                                            , cxy_transform::Pose<_Scalar>& pose
                                            , cxy_transform::Pose<_Scalar>& pose_parent)
        {
            CXY_ASSERT(x.rows() == config_->joint_number_);
            CXY_ASSERT(x.rows() >= joint);

            if ( -1 == joints_[joint]->getParent())
            {
                //pose = cxy_transform::Pose<_Scalar>();
                pose = cxy_transform::Pose<_Scalar>::rotateByAxis_fromIdentity(joints_[joint]->getJointType(), x(joint), joints_[joint]->getPose());
                //ROS_INFO_STREAM("getKinematicPose2World: "<<joint<<" "<<pose.t()(0)<<" "<<pose.t()(1)<<" "<<pose.t()(2)<<" "<<pose.q().x()<<" "<<pose.q().y()<<" "<<pose.q().z()<<" "<<pose.q().w());
                pose_parent = cxy_transform::Pose<_Scalar>();
                return;
            }
            else
            {
                getKinematicPose2World(x, joints_[joint]->getParent(), pose, pose_parent);
            }

            cxy_transform::Pose<_Scalar> poseTmp = cxy_transform::Pose<_Scalar>::rotateByAxis_fromIdentity(joints_[joint]->getJointType(), x(joint), joints_[joint]->getPose());
            cxy_transform::Pose<_Scalar> pose_out;

            /// Problem
            pose.composePose(poseTmp, pose_out);
            pose_parent = pose;
            pose = pose_out;
            //ROS_INFO_STREAM("getKinematicPose2World: "<<joint<<" "<<pose.t()(0)<<" "<<pose.t()(1)<<" "<<pose.t()(2)<<" "<<pose.q().x()<<" "<<pose.q().y()<<" "<<pose.q().z()<<" "<<pose.q().w());

            return;
        }

        /*
        template<typename _Scalar>
        std::shared_ptr<std::vector<cxy_icp_kinematic_joint<_Scalar>>>& cxy_icp_kinematic_chain<_Scalar>::getKinematicChainNodes()
        {
            return kc_nodes_;
        }*/




        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::constructKinematicChain()
        {
            joints_.reserve(config_->joint_number_);

            for (int ii = 0; ii < config_->joint_number_; ++ii)
            {
                joints_.push_back(std::make_shared<cxy_icp_kinematic_joint<_Scalar>>(config_, ii));
                joints_[ii]->init();
            }
        }

    }
}


template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_chain<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_chain<double>;
