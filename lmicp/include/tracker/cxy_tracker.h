#pragma once
#include <unsupported/Eigen/NonLinearOptimization>

#include "optimization/cxy_cost_func_kinematic.h"

#include "common/cxy_config.h"
#include "kinematic/cxy_icp_kinematic.h"

//#include "kinematic/"

namespace cxy
{
    template<typename _Scalar>
    class cxy_tracker
    {

        typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixX1;
        typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixXX;
    public:
        cxy_tracker(const cxy_config *const );
        ~cxy_tracker();

        void runOptimization();

        mutable cxy_kinematic::cxy_icp_kinematic<_Scalar> kinematic_;

        inline MatrixX1 const& getX_() {return x_;}
    private:
        const cxy_config* const config_;

        MatrixX1 x_;
        //MatrixXX jaco_;
    };

}