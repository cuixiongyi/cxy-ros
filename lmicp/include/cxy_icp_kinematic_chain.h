#pragma once
#include "cxy_transform.h"
#include "cxy_debug.h"
#include "cxy_icp_kinematic_node.h"
#include "common/cxy_common.h"

#include <cstdlib>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <memory>

namespace cxy
{
    namespace cxy_lmicp_lib
    {
        template<typename _Scalar>
        class cxy_icp_kinematic_chain
        {

        private:

            std::auto_ptr<std::vector<cxy_icp_kinematic_node<_Scalar>>> kc_nodes_;



        public:
            cxy_icp_kinematic_chain();

            pcl::PointCloud<pcl::PointXYZ>::Ptr getTransCloud();

            std::vector<cxy_icp_kinematic_node<_Scalar>>& getKinematicChain() {return kc_nodes_;}
            const std::vector<cxy_icp_kinematic_node<_Scalar>>& getKinematicChain() const {return kc_nodes_;}
            void setKinematicNodes(std::vector<cxy_icp_kinematic_node<_Scalar>> kin_nodes) {kc_nodes_ = kin_nodes;};



        };


    }
}