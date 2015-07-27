
#include "kinematic/cxy_icp_kinematic_joint.h"

namespace cxy
{
	namespace cxy_lmicp_lib
	{

        template<typename _Scalar>
        cxy_icp_kinematic_joint<_Scalar>::cxy_icp_kinematic_joint(const cxy_config* const config_ptr
                                                                  , const int& joint_idx
                                                                  , const cxy_icp_kinematic_chain<_Scalar>* kc_ptr)
        : joint_info_(config_->joint_config_[joint_idx])
            , config_(config_ptr)
            , kc_ptr_(kc_ptr)
        {

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
        const Joint_Relation&& cxy_icp_kinematic_joint<_Scalar>:: getJointRelation(const int& joint_a, const int& joint_b)
        {
            CXY_ASSERT(joint_a > 0 && joint_a < cxy_config::joint_number_);
            CXY_ASSERT(joint_a > 0 && joint_a < cxy_config::joint_number_);

            return static_cast<Joint_Relation>(jointRelationMatrix_(joint_a, joint_b));
        }

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
                for (int jj = 0; jj < cxy_config::joint_number_; ++jj)
                {
                    jointRelationMatrix_(ii,jj) = -1;

                }
            }
            for (int ii = 0; ii < cxy_config::joint_number_; ++ii)
            {
                int parent = cxy_config::joint_config_[ii].joint_parent;

                if (-1 == cxy_config::joint_config_[ii].joint_parent)
                {
                    jointRelationList_[ii].push_back(-1);
                    for (int jj = 0; jj < cxy_config::joint_number_; ++jj)
                    {
                        if (ii == jj)
                        {
                            jointRelationMatrix_(ii,jj) = static_cast<std::int8_t>(Joint_Relation::No_Relation);

                        }
                        jointRelationMatrix_(ii,jj) = static_cast<std::int8_t>(Joint_Relation::Parent);
                        jointRelationMatrix_(jj,ii) = static_cast<std::int8_t>(Joint_Relation::Child);

                    }
                    continue;
                }

                jointRelationMatrix_(parent,ii) = static_cast<std::int8_t>(Joint_Relation::Immediate_Parent);
                jointRelationMatrix_(ii,parent) = static_cast<std::int8_t>(Joint_Relation::Immediate_Child);
                jointRelationList_[ii].push_back(parent);

                parent = cxy_config::joint_config_[parent].joint_parent;
                while (1)
                {
                    jointRelationList_[ii].push_back(parent);
                    if (-1 == parent)
                        break;
                    jointRelationMatrix_(parent,ii) = static_cast<std::int8_t>(Joint_Relation::Parent);
                    jointRelationMatrix_(ii,parent) = static_cast<std::int8_t>(Joint_Relation::Child);
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

