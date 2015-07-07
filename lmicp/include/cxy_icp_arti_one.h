#pragma once

#include "cxy_icp.h"
#include "cxy_transform.h"
#include "cxy_icp_arti_one_func.h"
#include <unsupported/Eigen/NonLinearOptimization>
#include "optimization/cxy_nonlinear_minimizer_LM.h"

namespace cxy {
    namespace cxy_lmicp_lib {
        template<typename _Scalar, int _MinimizerType>
        class cxy_icp_arti_one : public cxy_icp<_Scalar, _MinimizerType>
        {

        public:
            cxy_icp_arti_one() : cxy_icp<_Scalar, _MinimizerType>() {}
            virtual int icp_prepare_cost_function()
            {
                cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar>* tmp = new cxy_icp_arti_one_func<_Scalar>(this->modelCloud_, this->dataCloud_, this->kdtreeptr_);
                this->func_ = tmp;
                return 1;
            }

            _Scalar icp_minimization(Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> &x)
            {
                cxy_optimization::cxy_nonlinear_method state(static_cast<cxy_optimization::cxy_nonlinear_method>(_MinimizerType));
                switch (state)
                {
                    case cxy_optimization::cxy_nonlinear_method::CXY_LEVENBERG_MARQUARDT :  
                    {

                        cxy_optimization::cxy_nonlinear_minimizer_LM<cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar>, _Scalar > lm(*this->func_);
                        return lm.minimize(x);
                    }
                    case cxy_optimization::cxy_nonlinear_method::EIGEN_MINPACK :  
                    {
                        
                        Eigen::LevenbergMarquardt <cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar>, _Scalar > lm2(*this->func_);
                        lm2.lmder1(x);
                        Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> res_tmp;
                        return (*(this->func_))(x,res_tmp);
                        //lm2.lmder1(x);
                        //lm2.lmder1(x);
                        //lm2.lmder1(x);
                    }


                }
                //cxy_optimization::cxy_nonlinear_minimizer_LM<cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar>, _Scalar > lm(*this->func_);
                //return lm.minimize(x);

           
            }


        protected:


        };
    }
}