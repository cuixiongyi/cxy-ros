#include "tracker/cxy_tracker.h"


namespace cxy
{

    template<typename _Scalar>
    cxy_tracker<_Scalar>::cxy_tracker(const cxy_config *const config)
    : config_(config)
    , kinematic_(config)
    {

    }

    template<typename _Scalar>
    cxy_tracker<_Scalar>::~cxy_tracker()
    {

    }

    template<typename _Scalar>
    void cxy_tracker<_Scalar>::runOptimization()
    {
        Eigen::LevenbergMarquardt <cxy_tracker<_Scalar>, _Scalar > lm2(*this);
        _Scalar res = lm2.lmder1(x_);

        return;
    }

    template<typename _Scalar>
    _Scalar cxy_tracker<_Scalar>::operator()(MatrixX1 & x, MatrixX1& fvec) const
    {
        kinematic_.computeResidual(x_, fvec);
    }

    template<typename _Scalar>
    _Scalar cxy_tracker<_Scalar>::df(MatrixX1 & x, MatrixXX& fjac) const
    {
        kinematic_.computeJacobian(x, fjac);
    }



}