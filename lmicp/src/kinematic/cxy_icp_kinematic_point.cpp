
#include "kinematic/cxy_icp_kinematic_point.h"

#include "kinematic/cxy_icp_kinematic_joint.h"
#include "kinematic/cxy_icp_kinematic_chain.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {

        template<typename _Scalar>
        cxy_icp_kinematic_point<_Scalar>::cxy_icp_kinematic_point(const cxy_config* const config_ptr
                                                                , const pcl::KdTreeFLANN<PointT>::Ptr& kdtree
                                                                , const pcl::PointCloud<PointT>::Ptr& dataCloud
                                                                , const cxy_icp_kinematic_joint<_Scalar>* const joint
                                                                , const cxy_icp_kinematic_chain<_Scalar>* kc_ptr)


        : config_(config_ptr)
                , joint_(joint)
                , kdtree_ptr_(kdtree)
                , dataCloud_(dataCloud)
                , kc_ptr_(kc_ptr)
        {

        }


        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::init()
        {

        }







        template<typename _Scalar>
        const _Scalar& cxy_icp_kinematic_point<_Scalar>::matchPointCloud(const PointT& model
                                                                        , PointT& data
                                                                        , Eigen::Matrix< _Scalar, 3, 1>& res)
        {


            static const int K(1);
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            /// kdtreeptr_ is updated by setModelCloud
            if (nullptr == kdtree_ptr_)
                ROS_INFO("nullptr kdtree");

            if ( 0 == kdtree_ptr_->nearestKSearch (model, K, pointIdxNKNSearch, pointNKNSquaredDistance)  )
            {
                return std::nanf("");
            }

            /*if (pointNKNSquaredDistance[0] > max_correspondence_dist_square_)
            {
                return std::nanf("");
            }
            */
            /// Use Euclidean distance
            //ROS_INFO_STREAM("closet point = "<<pTmp.x<<" "<<pTmp.y<<" "<<pTmp.z);
            //ROS_INFO_STREAM(" ");
            data = ((*dataCloud_)[pointIdxNKNSearch[0]]);

            res(0) = data.x - model.x;
            res(1) = data.y - model.y;
            res(2) = data.z - model.z;
            //int signTmp = 1;//(res(0)+res(1)+res(2)) > 0 ? 1 : -1;
            //ROS_INFO_STREAM(rtmp);

            return std::sqrt(pointNKNSquaredDistance[0]);
        }

        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::computePointResidual(const int& rows, MatrixX1& res)
        {

            /*
             * TODO add residual of other type
             */
            uint8_t ii = rows;
            if (config_->with_icp_jacobian)
            {
                res(ii) = cxy_icp_kinematic_point::matchPointCloud(modelPoint_global_
                                                                              , dataPoint_
                                                                              , point_resdual3_);
                ++ii;
            }
            if (config_->with_collision_jacobian)
            {
                res(ii) = 0;
                ++ii;
            }
            if (config_->with_push_jacobian)
            {
                res(ii) = 0;

                ++ii;
            }
            if (config_->with_silhouette_jacobian)
            {
                res(ii) = 0;

                ++ii;
            }

        }

        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::computePointJacobian(const int& rows, MatrixXX& jac)
        {
            {
                int rows_true, cols_true;
                config_->getJacobianSize(rows_true, cols_true);
                CXY_ASSERT(rows_true > rows && cols_true == config_->joint_DoFs);

            }

            /*
             * TODO add jacobian of other type
             */
            uint8_t ii = rows;
            if (config_->with_icp_jacobian)
            {

                compute_icp_jacobian(ii, jac);
                ++ii;
            }
            if (config_->with_collision_jacobian)
            {
                compute_collision_jacobian(ii, jac);

                ++ii;
            }
            if (config_->with_push_jacobian)
            {
                compute_push_jacobian(ii, jac);

                ++ii;
            }
            if (config_->with_silhouette_jacobian)
            {
                compute_silhouette_jacobian(ii, jac);

                ++ii;
            }

        }


        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::compute_icp_jacobian(const int& rows, MatrixXX& jac)
        {

            /*
             * the 1st element of jointRelationList_ is it self
             */
            const std::vector<int>& jointParentList (cxy_icp_kinematic_joint<_Scalar>::getJointRelationList(this->joint_idx_));
            for (int ii = 0; ii < jointParentList.size(); ++ii)
            {
                const cxy_icp_kinematic_joint<_Scalar>& joint(kc_ptr_->getJoint(jointParentList[ii]));

                if (cxy_transform::Axis::Six_DoF == joint.getJointType())
                    continue;

                Eigen::Matrix< _Scalar, 3, 1> rotation_axis;
                Eigen::Matrix< _Scalar, 3, 1> tmp(modelPoint_global_.x - joint.getPose().t()(0), modelPoint_global_.y - joint.getPose().t()(1), modelPoint_global_.z - joint.getPose().t()(2));

                Eigen::Matrix< _Scalar, 3, 1> fix_axis(0, 0, 0);
                if (cxy_transform::Axis::X_axis_rotation == joint.getJointType())
                {
                    fix_axis(0) = 1;
                    joint.getPose().composeDirectionVector(fix_axis, rotation_axis);
                }
                if (cxy_transform::Axis::Y_axis_rotation == joint.getJointType())
                {
                    fix_axis(1) = 1;
                    joint.getPose().composeDirectionVector(fix_axis, rotation_axis);
                }
                if (cxy_transform::Axis::Z_axis_rotation == joint.getJointType())
                {
                    fix_axis(2) = 1;
                    joint.getPose().composeDirectionVector(fix_axis, rotation_axis);
                }

                Eigen::Matrix< _Scalar, 3, 1> cross = rotation_axis.cross(tmp);
                //Eigen::Matrix< _Scalar, 3, 1> cross_norm = cross / (std::sqrt(cross(0)*cross(0)+cross(1)*cross(1)+cross(2)*cross(2)));
                Eigen::Matrix< _Scalar, 3, 1>& cross_norm = cross;
                const float jac_step_scale = 0.3;
                const float r3_length = 1; //std::sqrt(point_resdual3_(0)*r3(0)+r3(1)*r3(1)+r3(2)*r3(2));
                float scale0 = r3_length * jac_step_scale * cross_norm(0);
                float scale1 = r3_length * jac_step_scale * cross_norm(1);
                float scale2 = r3_length * jac_step_scale * cross_norm(2);
                if (std::isnan(scale0))
                    scale0 = 0.0;
                if (std::isnan(scale1))
                    scale1 = 0.0;
                if (std::isnan(scale2))
                    scale2 = 0.0;
                if (std::abs(point_resdual3_(0)+scale0) + std::abs(point_resdual3_(1)+scale1) + std::abs(point_resdual3_(2)+scale2) > std::abs(point_resdual3_(0)) + std::abs(point_resdual3_(1)) + std::abs(point_resdual3_(2)))
                {
                    jac(rows, cxy_config::jointParaIdx_[jointParentList[ii]]) =  -std::abs(cross(2) + cross(1) + cross(0));
                    /*
                    if (jj == 20)
                        ROS_INFO_STREAM("reversed");
                        */
                }
                else
                {
                    jac(rows, cxy_config::jointParaIdx_[jointParentList[ii]]) =  std::abs(cross(2) + cross(1) + cross(0));
                    //fjac(jj, 0) = -fjac(jj, 0);
                }
            }

        }

        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::compute_collision_jacobian(const int& rows, MatrixXX& jac)
        {

        }

        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::compute_push_jacobian(const int& rows, MatrixXX& jac)
        {

        }

        template<typename _Scalar>
        void cxy_icp_kinematic_point<_Scalar>::compute_silhouette_jacobian(const int& rows, MatrixXX& jac)
        {

        }
    }
}

template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_point<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_point<double>;

