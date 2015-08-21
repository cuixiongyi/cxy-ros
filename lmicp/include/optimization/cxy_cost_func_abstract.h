#pragma once


#include "Eigen/Core"
namespace cxy
{

    namespace cxy_optimization
    {

        
        //: From Eigen unsupported/test/NonLinearOptimization.cpp
        // The reason do not use class here is that the function is not attached to class
        // So the function is not a member function, which can not be registered without knowing the class
        // what need to be done is in the Cxy_Cost_Func_Example.h
        //
        template<typename _Scalar, int NX=Eigen::Dynamic, int NY=Eigen::Dynamic>
        class Cxy_Cost_Func_Abstract
        {
        public:
            enum {
                ParaAtCompileTime = NX,
                DataAtCompileTime = NY
            };
            typedef Eigen::Matrix<_Scalar,Eigen::Dynamic,1> ParaType;
            typedef Eigen::Matrix<_Scalar,Eigen::Dynamic,1> ResidualType;
            typedef Eigen::Matrix<_Scalar,Eigen::Dynamic,Eigen::Dynamic> JacobianType;

            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixX1;
            typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixXX;

        public:

             const int nPara_, nData_;

            Cxy_Cost_Func_Abstract() : nPara_(ParaAtCompileTime), nData_(DataAtCompileTime) {}
            Cxy_Cost_Func_Abstract(int inputs, int values) : nPara_(inputs), nData_(values) {}

            constexpr int inputs() const { return nPara_; }
            constexpr int values() const { return nData_; }

            // you should define that in the subclass :
            virtual _Scalar operator()(MatrixX1 & x, MatrixX1& fvec) const = 0;
            virtual _Scalar df(MatrixX1 & x, MatrixXX& fjac) const = 0;


            //  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
        };


    }
}
template class cxy::cxy_optimization::Cxy_Cost_Func_Abstract<float>;
template class cxy::cxy_optimization::Cxy_Cost_Func_Abstract<double>;

