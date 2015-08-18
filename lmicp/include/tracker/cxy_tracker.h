#pragma once

#include "optimization/cxy_cost_func_abstract.h"
#include "optimization/cxy_nonlinear_minimizer.h"

#include "common/cxy_config.h"
#include "kinematic/cxy_icp_kinematic.h"
#include <unsupported/Eigen/NonLinearOptimization>

//#include "kinematic/"

namespace cxy
{
    template<typename _Scalar>
    class cxy_tracker : cxy_optimization::Cxy_Cost_Func_Abstract< _Scalar>
    {

        typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixX1;
        typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixXX;

        cxy_tracker(const cxy_config *const );
        ~cxy_tracker();

        virtual _Scalar operator()(MatrixX1 & x, MatrixX1& fvec) const;
        virtual _Scalar df(MatrixX1 & x, MatrixXX& fjac) const;

        void runOptimization();

        mutable cxy_kinematic::cxy_icp_kinematic<_Scalar> kinematic_;

        inline MatrixX1 const& getX_() {return x_;}
    private:
        const cxy_config* const config_;

        MatrixX1 x_;
        //MatrixXX jaco_;
    };

}