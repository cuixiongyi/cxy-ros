
#include "kinematic/cxy_icp_kinematic_joint.h"

#include "kinematic/cxy_icp_kinematic_chain.h"

namespace cxy
{
	namespace cxy_lmicp_lib
	{

        template<typename _Scalar>
        std::vector<std::vector<int >> cxy_icp_kinematic_joint<_Scalar>::jointRelationList_;

        template<typename _Scalar>
        cxy_icp_kinematic_joint<_Scalar>::cxy_icp_kinematic_joint(const cxy_config* const config_ptr
                                                                  , const int& joint_idx
                                                                  , const cxy_icp_kinematic_chain<_Scalar>* kc_ptr)
        : joint_info_(config_->joint_config_[joint_idx])
            , config_(config_ptr)
            , kc_ptr_(kc_ptr)
            , joint_idx_(joint_idx)
            , DoF_(joint_info_.DoF)
            , originPose_(joint_info_.t[0], joint_info_.t[1], joint_info_.t[2]
                          , joint_info_.r[0], joint_info_.r[1], joint_info_.r[2])
        {
            theta_ = new _Scalar [DoF_];
        }

        template<typename _Scalar>
        cxy_icp_kinematic_joint<_Scalar>::~cxy_icp_kinematic_joint()
        {
            delete[] theta_;
        }


        template<typename _Scalar>
        void cxy_icp_kinematic_joint<_Scalar>::init()
        {

        }


/*
        template<typename _Scalar>
        cxy_transform::Pose& cxy_icp_kinematic_joint<_Scalar>::getPose()
        {
            return pose_;
        }

        template<typename _Scalar>
        const cxy_transform::Pose& cxy_icp_kinematic_joint<_Scalar>::getPose() const
        {
            return pose_;
        }
*/



        template<typename _Scalar>
        void cxy_icp_kinematic_joint<_Scalar>:: updateJointRelation()
        {
            jointRelationList_.resize(cxy_config::joint_number_);

            for (int ii = 0; ii < cxy_config::joint_number_; ++ii)
            {
                /*
                 * the 1st element of jointRelationList_ is it self
                 */
                jointRelationList_[ii].push_back(ii);

            }
            for (int ii = 0; ii < cxy_config::joint_number_; ++ii)
            {
                int parent = cxy_config::joint_config_[ii].joint_parent;

                if (-1 == cxy_config::joint_config_[ii].joint_parent)
                {
                    jointRelationList_[ii].push_back(-1);

                    continue;
                }

                jointRelationList_[ii].push_back(parent);

                parent = cxy_config::joint_config_[parent].joint_parent;
                while (1)
                {
                    jointRelationList_[ii].push_back(parent);
                    if (-1 == parent)
                        break;
                    parent = cxy_config::joint_config_[parent].joint_parent;

                }
            }

            return;
        }

        template<typename _Scalar>
        const std::vector<int >& cxy_icp_kinematic_joint<_Scalar>::getJointRelationList(const int& joint)
        {
            return jointRelationList_[joint];
        }



    }
}

template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_joint<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_kinematic_joint<double>;

