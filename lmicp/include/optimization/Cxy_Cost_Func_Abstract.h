#pragma once


#include "Eigen/Core"

namespace cxy
{

    namespace cxy_optimization
    {

        //: From Eigen unsupported/test/NonLinearOptimization.cpp
        // The reason do not use class here is that the function is not attached to class
        // So the function is not a member function, which can not be registered without knowing the class
        // you should define that in the subclass :
        //  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
        //
        template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
        class Cxy_Cost_Func_Abstract
        {
        public:
            enum {
                InputsAtCompileTime = NX,
                ValuesAtCompileTime = NY
            };
            typedef Eigen::Matrix<_Scalar,InputsAtCompileTime,1> InputType;
            typedef Eigen::Matrix<_Scalar,ValuesAtCompileTime,1> ValueType;
            typedef Eigen::Matrix<_Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;
        public:

             const int m_inputs, m_values;

            Cxy_Cost_Func_Abstract() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
            Cxy_Cost_Func_Abstract(int inputs, int values) : m_inputs(inputs), m_values(values) {}

            constexpr int inputs() const { return m_inputs; }
            constexpr int values() const { return m_values; }

            // you should define that in the subclass :
            //  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
        };


    }
}
