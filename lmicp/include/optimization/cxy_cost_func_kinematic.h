#pragma once

#include "optimization/cxy_cost_func_abstract.h"
#include <vector>
#include "Eigen/Core"
#include "cxy_tracker_forward_declaration.h"

namespace cxy
{

    namespace cxy_optimization
    {
        template<typename _Scalar>
        class cxy_cost_func_kinematic : public Cxy_Cost_Func_Abstract< _Scalar, Eigen::Dynamic, Eigen::Dynamic>
        {
        public:

            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixX1;
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixXX;



            cxy_cost_func_kinematic(int const&, int const&, cxy_kinematic::cxy_icp_kinematic<_Scalar>* const&);

            virtual _Scalar operator()(MatrixX1 & x, MatrixX1& res) const;


            virtual _Scalar df(MatrixX1 & x, MatrixXX& jac) const;

        private:
            cxy_kinematic::cxy_icp_kinematic<_Scalar>* const& kinematic_;
        };

    }
}