#pragma once
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
        std::auto_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>>& cxy_icp_kinematic_chain<_Scalar>::getKinematicChainNodes()
        {
            return kc_nodes_;
        }
        template<typename _Scalar>
        const std::auto_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>>& cxy_icp_kinematic_chain<_Scalar>::getKinematicChainNodes() const
        {
            return kc_nodes_;
        }
        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::setKinematicNodes(const std::auto_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>> kin_nodes)
        {
            CXY_ASSERT(nullptr == kin_nodes );
            kc_nodes_ = kin_nodes;
            return;
        }
        template<typename _Scalar>
        void cxy_icp_kinematic_chain<_Scalar>::setKinematicRootList(const std::vector<int> &list)
        {
            CXY_ASSERT( 0 == list.size() );

            kc_root_list_ = list;
            return;

        }


    }
}