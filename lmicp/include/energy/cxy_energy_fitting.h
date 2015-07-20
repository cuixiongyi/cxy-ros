//
// Created by xiongyi on 7/20/15.
//
#pragma once

#include <Eigen/Core>
#include <iostream>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "cxy_tracker_forward_declaration.h"
#include "utility/cxy_transform.h"
#include "optimization/cxy_cost_func_abstract.h"


namespace cxy
{
namespace cxy_lmicp_lib
{

    template<typename _Scalar>
    class cxy_energy_fitting
    {
    public:
        typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixX1;
        typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixXX;

        cxy_energy_fitting();
        ~cxy_energy_fitting();

        static void computeJacobian(const cxy_icp_kinematic_point* point_);

    };

}
}

