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
        void cxy_icp_kinematic_chain<_Scalar>::updateJoints(const MatrixX1& x)
        {
            CXY_ASSERT(x.rows() == cxy_config::joint_DoFs);

            x_ = x;

            // assuming x_ has been updated
            // setJointPara has been called
            int jtmp = 0;
            for (int ii = 0; ii < cxy_config::joint_DoFs; )
            {
                joints_[jtmp]->setTheta(x.data() + ii);
            }
            for (int ii = 0; ii < cxy_config::joint_number_; ++ii)
            {
                if (syc_tryUpdateJointList(ii))
                {
                    cxy_transform::Pose<_Scalar> tmp;
                    getKinematicPose2World(ii, tmp);

                    syc_setJointUptoDate(ii, tmp);

                }

            }
        }


        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::updateModelPoints()
        {
            //points_ = std::make_shared<std::vector<cxy_icp_kinematic_point<_Scalar>*>>(modelCloud_->size());

            // ii is for each joint
            for (int ii = 0; ii < modelCloud_engin_.size(); ++ii)
            {
                /// this points are in global coordinate
                pcl::PointCloud<PointT>::Ptr points = modelCloud_engin_[ii]->getVisibleCloud(joints_[ii]->getPose());

                for (int jj = 0; jj < points->size(); ++jj)
                {
                    /*
                     * TODO use object_pool to allocate memory
                     */
                    points_.push_back(std::make_shared<cxy_icp_kinematic_point<_Scalar>>(config_, kdtreeptr_, dataCloud_, (joints_[ii]).get(), this));
                    auto lastPoint = points_.end()-1;
                    (*lastPoint)->modelPoint_global_ = (*points)[ii];
                }


            }
        }


        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::getResidual(MatrixX1& residual)
        {
            long rows, cols;
            config_->getJacobianSize(rows, cols);
            if (rows != residual.rows())
            {
                residual.resize(rows, 1);
            }

            int row = 0;
            for (int ii = 0; ii < points_.size(); ++ii)
            {
                /*
                 * TODO add other jacobian residual
                 */


                row = ii*config_->n_num_;
                points_[ii]->computePointResidual(row, residual);
            }

        }

        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::getJacobian(MatrixXX& jacobian)
        {
            long rows, cols;
            config_->getJacobianSize(rows, cols);
            if (rows != jacobian.rows()
                    || cols != jacobian.cols())
            {
                jacobian.resize(rows, cols);
            }

            int row = 0;
            for (int ii = 0; ii < points_.size(); ++ii)
            {
                row = ii*config_->n_num_;
                jacobian.row(1);
                points_[ii]->computePointJacobian(jacobian.block(row, 0, cxy_config::n_num_, jacobian.cols()));

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
            CXY_ASSERT(x_.rows() == config_->joint_number_);
            unsigned int pointCloudSize = 0;
            for (int ii = 0; ii < x_.rows(); ++ii)
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
                                          , cxy_transform::Pose<_Scalar>& pose)
        {

            //cxy_transform::Pose<_Scalar> pose_world;
            getKinematicPose2World(joint, pose, pose_parent);
            //ROS_INFO_STREAM("pose World: "<<joint<<" "<<pose_world.t()(0)<<" "<<pose_world.t()(1)<<" "<<pose_world.t()(2)<<" "<<pose_world.q().x()<<" "<<pose_world.q().y()<<" "<<pose_world.q().z()<<" "<<pose_world.q().w());

            pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud (new pcl::PointCloud<pcl::PointXYZ>);
            pose.composePoint(joints_[joint]->getModelCloud(), transCloud);
            return transCloud;
        }

*/



        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::getKinematicPose2World(
                                            const int& joint
                                            , cxy_transform::Pose<_Scalar>& pose)

        {
            if (syc_isJointUptoDate(joint))
            {
                pose = joints_[joint]->getPose();
                return ;
            }
            cxy_transform::Pose<_Scalar> pose_parent;
            const _Scalar* const& x {joints_[joint]->getTheta()};
            const cxy_transform::Axis jointType = joints_[joint]->getJointType();

            if ( -1 == joints_[joint]->getParentIdx())
            {
                if (cxy_transform::Axis::Six_DoF == jointType)
                {
                    pose.rotateByAxis(cxy::cxy_transform::Axis::X_axis_rotation, Deg2Rad(x[3]));
                    pose.rotateByAxis(cxy::cxy_transform::Axis::Y_axis_rotation, Deg2Rad(x[4]));
                    pose.rotateByAxis(cxy::cxy_transform::Axis::Z_axis_rotation, Deg2Rad(x[5]));
                    pose.t()(0) = x[0];
                    pose.t()(1) = x[1];
                    pose.t()(2) = x[2];

                }
                else if (cxy_transform::Axis::X_axis_rotation == jointType || cxy_transform::Axis::Y_axis_rotation == jointType || cxy_transform::Axis::Z_axis_rotation == jointType )
                {
                    pose = cxy_transform::Pose<_Scalar>();
                    pose.rotateByAxis(jointType, Deg2Rad(x[0]));


                }

                return;
            }
            else
            {
                while ( ! syc_isJointUptoDate(joints_[joint]->getParentIdx()))
                {
                    std::this_thread::yield();
                }
                getKinematicPose2World(joints_[joint]->getParentIdx(), pose_parent);
            }

            const cxy_transform::Pose<_Scalar>& pose_fix = joints_[joint]->getOriginPose();
            cxy_transform::Pose<_Scalar> tmp;
            pose_parent.composePose(pose_fix, tmp);
            tmp.composePose(cxy_transform::Pose<_Scalar>::rotateByAxis_fromIdentity(jointType, Deg2Rad(x[0])), pose);


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
            modelCloud_engin_.reserve(config_->joint_number_);
            for (int ii = 0; ii < config_->joint_number_; ++ii)
            {
                joints_.push_back(
                        std::make_shared<cxy_icp_kinematic_joint<_Scalar>>(config_, ii, this));
                joints_[ii]->init();
                joint_sync_list.push_back(Update_Status::NotUptoDate);
            }

            // Set child list for each joint
            {
                std::vector<std::vector<int >> jointChildList {cxy_config::joint_number_};
                jointChildList.resize(cxy_config::joint_number_);
                for (int ii = 0; ii < cxy_config::joint_number_; ++ii)
                {
                    int idx = ii;
                    int parent = cxy_config::joint_config_[idx].joint_parent;
                    if (-1 == parent)
                    {
                        joints_[ii]->setParent(nullptr);
                        joints_[ii]->setHierarchy(0);

                        continue;
                    }
                    else
                    {
                        joints_[ii]->setParent(joints_[parent].get());
                        /// set Hierarchy for each joint
                        /// +1 from its parent joint
                        joints_[ii]->setHierarchy(joints_[parent]->getHierarchy()+1);

                    }
                    while (-1 != parent)
                    {
                        jointChildList[parent].push_back(ii);

                        idx = parent;
                        parent = cxy_config::joint_config_[idx].joint_parent;

                    }

                }
                for (int ii = 0; ii < cxy_config::joint_number_; ++ii)
                {
                    std::vector<const cxy_icp_kinematic_joint<_Scalar>*> childList(jointChildList[ii].size());
                    /*
                     * setChildList
                     * ChildList the 1st element is the joint itself
                     */
                    childList.push_back(joints_[ii].get());

                    for (int jj = 0; jj < jointChildList[ii].size(); ++jj)
                    {
                        childList.push_back(joints_[jointChildList[ii][jj]].get());

                    }
                    joints_[ii]->setChildList(childList);

                }
            }

            // set modelCloud for each joint
            for (int ii = 0; ii < config_->joint_number_; ++ii)
            {
                modelCloud_engin_.push_back(std::make_shared<cxy_modelCloud_engin>(joints_[ii]->joint_info_.model_filename));
            }

            /// set Joint Parameter Start Idx
            int idx = 0;
            for (int ii = 0; ii < config_->joint_number_; ++ii)
            {
                joints_[ii]->setParaStartIdx(idx);
                idx += joints_[ii]->getNumDoF();
            }
            //
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
