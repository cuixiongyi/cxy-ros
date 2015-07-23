#include "kinematic/cxy_icp_kinematic_chain.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {
        template<typename _Scalar>
        cxy_icp_kinematic_chain<_Scalar>::cxy_icp_kinematic_chain(const cxy_config* const config)
        : config_(config)
        {
            constructKinematicChain();
        }

        template<typename _Scalar>
        cxy_icp_kinematic_chain<_Scalar>::~cxy_icp_kinematic_chain()
        {

        }

        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::setJointPara(const MatrixX1& x)
        {
            x_ = x;
        }

        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::updateJoints()
        {
            // assuming x_ has been updated
            for (int ii = 0; ii < config_->joint_number_; ++ii)
            {

                if (syc_tryUpdateJointList(ii))
                {
                    cxy_transform::Pose<_Scalar> tmp;
                    cxy_transform::Pose<_Scalar> tmp_parent;
                    getKinematicPose2World(ii, tmp, tmp_parent);

                    syc_setJointUptoDate(ii, tmp);
                }

            }
        }


        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::updateModelPoints()
        {
            points_ = std::make_shared<std::vector<cxy_icp_kinematic_point>>(modelCloud_->size());

            for (int ii = 0; ii < modelCloud_->size(); ++ii)
            {

                /*
                 * TODO pointJointIdx_ need to be assigned when modelCloud generation
                 */
                points_->push_back(cxy_icp_kinematic_point(config_, pointJointIdx_[ii]));
                (*points_)[ii].modelPoint_global_ = (*modelCloud_)[ii];
                (*points_)[ii].dataPoint_ = (*dataCloud_)[ii];


                (*points_)[ii].point_resdual1_ = cxy_icp_kinematic_point::matchPointCloud((*points_)[ii].modelPoint_global_
                                                        , kdtreeptr_
                                                        , (*points_)[ii].dataPoint_
                                                        , (*points_)[ii].point_resdual3_);
                (*points_)[ii].jacobian_.resize(config_->kinematic_ptr_->)
            }
        }

        template<typename _Scalar>
        bool cxy_icp_kinematic_chain<_Scalar>::syc_tryUpdateJointList(const int &joint)
        {
            std::lock_guard<std::mutex> lock(kinematic_chain_lock);
            if ( Update_Status::NotUptoDate == joint_sync_list[joint])
            {
                joint_sync_list[joint] = Update_Status::BeingProcessed;
                return true;
            }
            return false;
        }

        /*
        template<typename _Scalar>
        pcl::PointCloud<pcl::PointXYZ>::Ptr cxy_icp_kinematic_chain<_Scalar>::getFullModelCloud_World()
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

                }
                //ROS_INFO_STREAM(" ");
            }
            return transCloud;

        }

        template<typename _Scalar>
        pcl::PointCloud<pcl::PointXYZ>::Ptr cxy_icp_kinematic_chain<_Scalar>::getOneModelCloud_World(
                                           const int& joint
                                          , cxy_transform::Pose<_Scalar>& pose
                                          , cxy_transform::Pose<_Scalar>& pose_parent )
        {

            //cxy_transform::Pose<_Scalar> pose_world;
            getKinematicPose2World(joint, pose, pose_parent);
            //ROS_INFO_STREAM("pose World: "<<joint<<" "<<pose_world.t()(0)<<" "<<pose_world.t()(1)<<" "<<pose_world.t()(2)<<" "<<pose_world.q().x()<<" "<<pose_world.q().y()<<" "<<pose_world.q().z()<<" "<<pose_world.q().w());

            pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud (new pcl::PointCloud<pcl::PointXYZ>);
            pose.composePoint(joints_[joint]->getModelCloud(), transCloud);
            return transCloud;
        }


        template<typename _Scalar>
        pcl::PointCloud<pcl::PointXYZ>::Ptr cxy_icp_kinematic_chain<_Scalar>::getOneModelCloud_World(
                                          const int& joint
                                          , cxy_transform::Pose<_Scalar>& pose)
                                        
        {
            cxy_transform::Pose<_Scalar> pose_parent;
            return getOneModelCloud_World(x, joint, pose, pose_parent);
        }

        */


        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::getKinematicPose2World(
                                            const int& joint
                                            , cxy_transform::Pose<_Scalar>& pose
                                            , cxy_transform::Pose<_Scalar>& pose_parent)
        {

            if (syc_isJointUptoDate(joint))
            {
                pose = joints_[joint]->getPose();
                pose_parent = cxy_transform::Pose<_Scalar>();
                return ;
            }

            if ( -1 == joints_[joint]->getParent())
            {
                cxy_transform::Pose<_Scalar> ptmp;
                if (cxy_transform::Axis::Six_DoF == joints_[joint]->getJointType())
                {
                    pose.rotateByAxis(cxy::cxy_transform::Axis::X_axis_rotation, Deg2Rad(x_(3)));
                    pose.rotateByAxis(cxy::cxy_transform::Axis::Y_axis_rotation, Deg2Rad(x_(4)));
                    pose.rotateByAxis(cxy::cxy_transform::Axis::Z_axis_rotation, Deg2Rad(x_(5)));
                    pose.t()(0) = x_(0);
                    pose.t()(1) = x_(1);
                    pose.t()(2) = x_(2);
                }
                pose_parent = cxy_transform::Pose<_Scalar>();
                return;
            }
            else
            {

                getKinematicPose2World(joints_[joint]->getParent(), pose, pose_parent);
            }

            cxy_transform::Pose<_Scalar> pose_fix = joints_[joint]->getOriginPose();
            pose_fix.rotatefromFix(joints_[joint]->getJointType(), x_(joint), pose);

            return;
        }

        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr data)
        {
            dataTime = cxy_sync();
            hasSetDataCloud_ = true;
            dataCloud_ = data;
            kdtreeptr_ = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>);
            kdtreeptr_->setInputCloud(dataCloud_);
        }


        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::constructKinematicChain()
        {
            joints_.reserve(config_->joint_number_);
            joint_sync_list.reserve(config_->joint_number_);
            for (int ii = 0; ii < config_->joint_number_; ++ii)
            {
                joints_.push_back(std::make_shared<cxy_icp_kinematic_joint<_Scalar>>(config_, ii));
                joints_[ii]->init();
                joint_sync_list.push_back(Update_Status::NotUptoDate);
            }
        }

        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::syc_resetSyncList()
        {
            std::lock_guard<std::mutex> lock(kinematic_chain_lock);
            for (int ii = 0; ii < joint_sync_list.size(); ++ii)
            {
                joint_sync_list[ii] = Update_Status::NotUptoDate;
            }
        }

        template<typename _Scalar>
        bool cxy_icp_kinematic_chain<_Scalar>::syc_isJointUptoDate(const int &joint)
        {
            std::lock_guard<std::mutex> lock(kinematic_chain_lock);
            if (Update_Status::UptoDate == joint_sync_list[joint])
            {
                return true;

            }
            return false;
        }

        template<typename _Scalar>
        bool cxy_icp_kinematic_chain<_Scalar>::syc_setJointUptoDate(const int & joint, const cxy_transform::Pose<_Scalar>& pose)
        {
            std::lock_guard<std::mutex> lock(kinematic_chain_lock);
            if (Update_Status::BeingProcessed != joint_sync_list[joint])
            {
                /// before syc_setJointUptoDate, the joint has to be marked as BeingProcessed,
                // with try syc_tryUpdateJointList()
                throw std::runtime_error("set sync to non registered process");
                return false;
            }

            joint_sync_list[joint] = Update_Status::UptoDate;
            joints_[joint]->setPose(pose);
            return true;
        }




    }
}


template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_chain<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_chain<double>;
