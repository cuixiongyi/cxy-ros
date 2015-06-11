#include "optimization/Cxy_Cost_Func_Example.h"
#include <unsupported/Eigen/NonLinearOptimization>
#include "Eigen/core.h"
#include "Eigen/main.h"

namespace cxy
{

    namespace cxy_optimization
    {
        using namespace Eigen;

        int main()
        {
            int n = 3, info;

            Eigen::VectorXd x;

            /* the following starting values provide a rough fit. */
            x.setConstant(n, 1.);

            // do the computation
            Cxy_Cost_Func_Example                      functor;
            Eigen::LevenbergMarquardt <Cxy_Cost_Func_Example> lm(functor);
            info = lm.lmder1(x);

            // check return value
            VERIFY_IS_EQUAL(info, 1);
            VERIFY_IS_EQUAL(lm.nfev, 6);
            VERIFY_IS_EQUAL(lm.njev, 5);

            // check norm
            VERIFY_IS_APPROX(lm.fvec.blueNorm(), 0.09063596);

            // check x
            Cxy_Cost_Func_ExampleVectorXd x_ref(n);
            x_ref << 0.08241058, 1.133037, 2.343695;
            VERIFY_IS_APPROX(x, x_ref);
        }
    }
}