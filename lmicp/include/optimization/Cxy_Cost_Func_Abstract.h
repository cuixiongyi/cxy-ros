#pragma once

#include "Eigen/core.h"

namespace cxy
{

    namespace cxy_optimization
    {

        //: From Eigen unsupported/test/NonLinearOptimization.cpp
        // The reason do not use class here is that the function is not attached to class
        // So the function is not a member function, which can not be registered without knowing the class
        template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
        struct Cxy_Cost_Func_Abstract
        {
            typedef _Scalar Scalar;
            enum {
                InputsAtCompileTime = NX,
                ValuesAtCompileTime = NY
            };
            typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
            typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
            typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

            const int m_inputs, m_values;

            Cxy_Cost_Func_Abstract() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
            Cxy_Cost_Func_Abstract(int inputs, int values) : m_inputs(inputs), m_values(values) {}

            int inputs() const { return m_inputs; }
            int values() const { return m_values; }

            // you should define that in the subclass :
            //  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
        };

    }
}
