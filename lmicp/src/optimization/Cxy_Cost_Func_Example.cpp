#include "optimization/Cxy_Cost_Func_Example.h"
#include <unsupported/Eigen/NonLinearOptimization>
#include "Eigen/Core"
#include <iostream>
//#include "main.h"

using namespace  cxy;



using namespace cxy_optimization;
using namespace Eigen;
int main()
        {
            int n = 3, info;



            Eigen::VectorXf x;

            /* the following starting values provide a rough fit. */
            x.setConstant(n, 1.);

            // do the computation
            Cxy_Cost_Func_Example<float>                functor;
//            std::cout<<functor::NX;
            Eigen::LevenbergMarquardt <Cxy_Cost_Func_Example<float>, float > lm(functor);

            info = lm.lmder1(x);

            // check return value
            //VERIFY_IS_EQUAL(info, 1);
            //VERIFY_IS_EQUAL(lm.nfev, 6);
            //VERIFY_IS_EQUAL(lm.njev, 5);

            // check norm
            //VERIFY_IS_APPROX(lm.fvec.blueNorm(), 0.09063596);

            // check x
            VectorXd x_ref(n);
            x_ref << 0.08241058, 1.133037, 2.343695;
            std::cout<<x_ref<<std::endl<<x<<std::endl;
            //VERIFY_IS_APPROX(x, x_ref);
        }
