#pragma once
#include "Eigen/Core"

namespace cxy {
    namespace cxy_optimization {
        //: The function type FunctorType need have 4 functions implemented
        // operator(), fd(), inputs(), values()
        // more details in the file cxy_cost_func_example.h
        template<typename FunctorType, typename Scalar>
        class cxy_nonlinear_minimizer
        {
            typedef Eigen::Matrix< Scalar, Eigen::Dynamic, 1 > FVectorType;
            typedef Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic > JacobianType;
            enum class minimizer_state : int
            {
                NotStarted = -1,
            };
        protected:
            FunctorType& func_;
            const unsigned int nPara_;
            const unsigned int nData_;
            FVectorType rf;
            JacobianType jf;
        public:
            cxy_nonlinear_minimizer(FunctorType &_functor)
                : func_(_functor), nPara_(_functor.inputs()), nData_(_functor.values()) 
                {
                    rf.resize(nData_, 1);
                    jf.resize(nData_, nPara_);
                }

            virtual Scalar minimizeOneStep(FVectorType& x) = 0;
            virtual Scalar minimize(FVectorType& x) = 0;
            


            


        };

    }
}