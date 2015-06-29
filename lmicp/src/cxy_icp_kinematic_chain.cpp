#include "cxy_icp_kinematic_chain.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {
        template<typename _Scalar>
        cxy_icp_kinematic_chain<_Scalar>::cxy_icp_kinematic_chain()
        {

        }

        template<typename _Scalar>
        pcl::PointCloud<pcl::PointXYZ>::Ptr cxy_icp_kinematic_chain<_Scalar>::getFullModelCloud_World(const Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x)
        {
            CXY_ASSERT(1);
            CXY_ASSERT(x.rows() == kc_nodes_->size());
            CXY_ASSERT(x.rows() == kc_root_list_.size());
            unsigned int pointCloudSize = 0;
            for (int ii = 0; ii < x.rows(); ++ii)
            {
                pointCloudSize += (*kc_nodes_)[ii].modelCloud_->size();

            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud (new pcl::PointCloud<pcl::PointXYZ>);
            transCloud->reserve(pointCloudSize);
            for (int ii = 0; ii < x.rows(); ++ii)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZ>);
                cxy_transform::Pose<_Scalar> pose;
                tmpCloud = getOneModelCloud_World(x, ii, pose);
                for (int jj = 0; jj < tmpCloud->size(); ++jj)
                {
                    transCloud->push_back((*tmpCloud)[jj]);
                    //ROS_INFO_STREAM((*kc_nodes_)[joint].modelCloud_[ii].x<<" "<<(*kc_nodes_)[joint].modelCloud_[ii].y<<"  "<<[ii].z);
                    /* code */
                }

            }
            return transCloud;

        }



        template<typename _Scalar>
        pcl::PointCloud<pcl::PointXYZ>::Ptr cxy_icp_kinematic_chain<_Scalar>::getOneModelCloud_World(
                                            const Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x
                                          , const int& joint
                                          , cxy_transform::Pose<_Scalar>& pose)
                                        
        {
            CXY_ASSERT(x.rows() == kc_nodes_->size());
            CXY_ASSERT(x.rows() == kc_root_list_.size());
            CXY_ASSERT(x.rows() >= joint);

            //cxy_transform::Pose<_Scalar> pose_world;
            getKinematicPose2World(x, joint, pose);
            //ROS_INFO_STREAM("pose World: "<<joint<<" "<<pose_world.t()(0)<<" "<<pose_world.t()(1)<<" "<<pose_world.t()(2)<<" "<<pose_world.q().x()<<" "<<pose_world.q().y()<<" "<<pose_world.q().z()<<" "<<pose_world.q().w());

            pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud (new pcl::PointCloud<pcl::PointXYZ>);
            pose.composePoint((*kc_nodes_)[joint].modelCloud_, transCloud);
            return transCloud;
        }

        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::getKinematicPose2World(
                                              const Eigen::Matrix<_Scalar, Eigen::Dynamic, 1>& x
                                            , const int& joint
                                            , cxy_transform::Pose<_Scalar>& pose)
        {
            CXY_ASSERT(x.rows() == kc_nodes_->size());
            CXY_ASSERT(x.rows() == kc_root_list_.size());
            CXY_ASSERT(x.rows() >= joint);

            if ( -1 == kc_root_list_[joint])
            {
                pose = cxy_transform::Pose<_Scalar>();
                pose = cxy_transform::Pose<_Scalar>::rotateByAxis_fromIdentity((*kc_nodes_)[joint].rotateAxis_, x(joint), (*kc_nodes_)[joint].pose_);
                //ROS_INFO_STREAM("getKinematicPose2World: "<<joint<<" "<<pose.t()(0)<<" "<<pose.t()(1)<<" "<<pose.t()(2)<<" "<<pose.q().x()<<" "<<pose.q().y()<<" "<<pose.q().z()<<" "<<pose.q().w());
                return;
            }
            else
            {
                getKinematicPose2World(x, kc_root_list_[joint], pose);
            }

            cxy_transform::Pose<_Scalar> poseTmp = cxy_transform::Pose<_Scalar>::rotateByAxis_fromIdentity((*kc_nodes_)[joint].rotateAxis_, x(joint), (*kc_nodes_)[joint].pose_);
            cxy_transform::Pose<_Scalar> pose_out;
            pose.composePose(poseTmp, pose_out);
            pose = pose_out;
            //ROS_INFO_STREAM("getKinematicPose2World: "<<joint<<" "<<pose.t()(0)<<" "<<pose.t()(1)<<" "<<pose.t()(2)<<" "<<pose.q().x()<<" "<<pose.q().y()<<" "<<pose.q().z()<<" "<<pose.q().w());

            return;
        }

        /*
        template<typename _Scalar>
        std::shared_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>>& cxy_icp_kinematic_chain<_Scalar>::getKinematicChainNodes()
        {
            return kc_nodes_;
        }*/

        template<typename _Scalar>
        const std::shared_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>>& cxy_icp_kinematic_chain<_Scalar>::getKinematicChainNodes() const
        {
            return kc_nodes_;
        }

        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::setKinematicNodes(std::shared_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>> kin_nodes)
        {
            CXY_ASSERT(nullptr != kin_nodes );
            kc_nodes_ = kin_nodes;
            return;
        }

        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::setKinematicRootList(std::vector<int> &list)
        {
            CXY_ASSERT( 0 != list.size() );

            kc_root_list_ = list;
            return;

        }


    }
}


template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_chain<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_chain<double>;
