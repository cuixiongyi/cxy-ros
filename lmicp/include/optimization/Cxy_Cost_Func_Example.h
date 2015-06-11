#pragma once

#include "Cxy_Cost_Func_Abstract.h"
#include <vector>
#include "Eigen/core.h"

namespace cxy
{

    namespace cxy_optimization
    {
        //: From Eigen unsupported/test/NonLinearOptimization.cpp
        template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
        struct Cxy_Cost_Func_Example : Cxy_Cost_Func_Abstract< _Scalar, NX, NY>

        {
            Cxy_Cost_Func_Example(void): Cxy_Cost_Func_Abstract<_Scalar>(3,15) {}
            int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
            {
                double tmp1, tmp2, tmp3;
                static const double y[15] = {1.4e-1, 1.8e-1, 2.2e-1, 2.5e-1, 2.9e-1, 3.2e-1, 3.5e-1,
                                             3.9e-1, 3.7e-1, 5.8e-1, 7.3e-1, 9.6e-1, 1.34, 2.1, 4.39};

                for (int i = 0; i < values(); i++)
                {
                    tmp1 = i+1;
                    tmp2 = 16 - i - 1;
                    tmp3 = (i>=8)? tmp2 : tmp1;
                    fvec[i] = y[i] - (x[0] + tmp1/(x[1]*tmp2 + x[2]*tmp3));
                }
                return 0;
            }

            int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const
            {
                double tmp1, tmp2, tmp3, tmp4;
                for (int i = 0; i < values(); i++)
                {
                    tmp1 = i+1;
                    tmp2 = 16 - i - 1;
                    tmp3 = (i>=8)? tmp2 : tmp1;
                    tmp4 = (x[1]*tmp2 + x[2]*tmp3); tmp4 = tmp4*tmp4;
                    fjac(i,0) = -1;
                    fjac(i,1) = tmp1*tmp2/tmp4;
                    fjac(i,2) = tmp1*tmp3/tmp4;
                }
                return 0;
            }
        };

    }
}