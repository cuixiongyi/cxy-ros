#pragma once
#include "Eigen/Core"
#include "cxy_nonlinear_minimizer.h"
#include "cxy_debug.h"

namespace cxy {
    namespace cxy_optimization {
        //: The function type FunctorType need have 4 functions implemented
        // operator(), fd(), inputs(), values()
        // more details in the file cxy_cost_func_example.h
        template<typename FunctorType, typename Scalar>
        class cxy_nonlinear_minimizer_LM : public cxy_nonlinear_minimizer<FunctorType, Scalar>
        {
            typedef Eigen::Matrix< Scalar, Eigen::Dynamic, 1 > FVectorType;
            typedef Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic > JacobianType;
            enum class minimizer_state : int
            {
                NotStarted = -1,
            };
        private:
            const Scalar sigma_;
        public:
            cxy_nonlinear_minimizer_LM(FunctorType &_functor)
                : cxy_nonlinear_minimizer<FunctorType, Scalar>(_functor)
                 , sigma_(0.02)
                {
                    this->rf.resize(this->nData_, 1);
                    this->jf.resize(this->nData_, this->nPara_);
                }

            virtual Scalar minimizeOneStep(FVectorType& x)
            {
                const int iter = 2;
                Scalar lambda(0.0);
                //ROS_INFO_STREAM("x1 = "<<x.rows()<<std::endl<<x);
                
                /* code */
                JacobianType jacTjac(this->nPara_, this->nPara_);
                JacobianType jac_right(this->nPara_, 1);
                JacobianType jac_left(this->nPara_, this->nPara_);
                JacobianType j_dia(this->nPara_, this->nPara_);

                jacTjac.setZero();
                jac_right.setZero();
                j_dia.setZero();
                jac_left.setZero();

                this->func_(x, this->rf);
                this->func_.df(x, this->jf);

                jacTjac = this->jf.transpose()*this->jf;
                for (int i = 0; i < this->nData_; ++i)
                {
                    Scalar& r = this->rf(i); 
                    this->rf(i) = r < sigma_ ? r*r : 2*sigma_*std::abs(r)-sigma_*sigma_;
                }
                jac_right = this->jf.transpose()*this->rf;

                for (int ii = 0; ii < this->nPara_; ++ii)
                {

                    j_dia(ii,ii) = jacTjac(ii,ii);

                }
                Scalar resdiual(std::numeric_limits<Scalar>::max());
                Scalar lambdaTmp;
                FVectorType result_Pose(this->nPara_, 1);
                for (int i = 0; i < iter; ++i)
                {
                    result_Pose.setZero();
                    lambda += Scalar(1.0) / iter;
                    jac_left = -(jacTjac+lambda*j_dia);
                    //ROS_INFO_STREAM(jac_left);
                    jac_left = jac_left.inverse();
                    //ROS_INFO_STREAM(jac_left);
                    result_Pose = jac_left * jac_right;
                    //ROS_INFO_STREAM("result_Pose   "<<result_Pose);
                    result_Pose = x + result_Pose;

                    Scalar tmp (this->func_(result_Pose, this->rf));
                    ROS_INFO_STREAM("result = "<<tmp);

                    if (resdiual > tmp)
                    {
                        resdiual = tmp;
                        lambdaTmp = lambda;
                    }
                }

                    jac_left = jacTjac+lambdaTmp*j_dia;
                    //ROS_INFO_STREAM(jac_left);
                    jac_left = jac_left.inverse();
                    //ROS_INFO_STREAM(jac_left);
                    result_Pose = jac_left * jac_right;
                    //CXY_ASSERT(result_Pose.rows() == this->nPara_);
                    //assert(x.rows() != this->nPara_);
                    //ROS_INFO_STREAM("x = "<<x);
                    ROS_INFO_STREAM("resultFinal = "<<resdiual);
                    ROS_INFO_STREAM("  ");
                    x = x + result_Pose;


                ROS_INFO_STREAM("lambda = "<<lambdaTmp<< "  Res =  "<< resdiual);
                return resdiual;
            }

            virtual Scalar minimize(FVectorType& x)
            {
                const Scalar stop(0.00001);
                Scalar resdiual;
                Scalar resdiualLast(std::numeric_limits<Scalar>::max());
                while (1)
                {
                    resdiual = minimizeOneStep(x);
                    if (resdiualLast - resdiual < stop)
                        break;
                    resdiualLast = resdiual;
                }   
                return resdiual;             
            }
            


            


        };

    }
}