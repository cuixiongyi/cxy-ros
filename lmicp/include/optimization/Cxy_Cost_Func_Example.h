#pragma once

#include "Cxy_Cost_Func_Abstract.h"
#include <vector>
#include "Eigen/Core"

namespace cxy
{

    namespace cxy_optimization
    {
        //: From Eigen unsupported/test/NonLinearOptimization.cpp
        template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
        class Cxy_Cost_Func_Example : public Cxy_Cost_Func_Abstract< _Scalar, NX, NY>
        {
        public:
            enum {
                InputsAtCompileTime = NX,
                ValuesAtCompileTime = NY
            };

            //: all of this is what you need in the derived function

            typedef Eigen::Matrix<_Scalar,InputsAtCompileTime,1> InputType;
            typedef Eigen::Matrix<_Scalar,ValuesAtCompileTime,1> ValueType;
            typedef Eigen::Matrix<_Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

            Cxy_Cost_Func_Example(void): Cxy_Cost_Func_Abstract<_Scalar, NX, NY>(3,15) {}
            int operator()(InputType const& x, ValueType& fvec) const
            {
                _Scalar tmp1, tmp2, tmp3;
                static const _Scalar y[15] = {1.4e-1, 1.8e-1, 2.2e-1, 2.5e-1, 2.9e-1, 3.2e-1, 3.5e-1,
                                             3.9e-1, 3.7e-1, 5.8e-1, 7.3e-1, 9.6e-1, 1.34, 2.1, 4.39};

                for (int i = 0; i < this->values(); i++)
                {
                    tmp1 = i+1;
                    tmp2 = 16 - i - 1;
                    tmp3 = (i>=8)? tmp2 : tmp1;
                    fvec[i] = y[i] - (x[0] + tmp1/(x[1]*tmp2 + x[2]*tmp3));
                }
                return 0;
            }

            int df(InputType const& x, JacobianType& fjac) const
            {
                _Scalar tmp1, tmp2, tmp3, tmp4;
                for (int i = 0; i < this->values(); i++)
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