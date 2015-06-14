#pragma once
#include "Eigen/Core"

namespace cxy 
{
    namespace cxy_optimization 
    {
        template<typename Scalar>
    	class cxy_terminate_criteria
    	{
    	private:
    		Scalar factor;
	        unsigned int maxIteration;   // maximum number of function evaluation
	        Scalar ftol;
	        Scalar xtol;
	        Scalar gtol;
	        Scalar epsfcn;
    	public:
    		cxy_terminate_criteria()
				            : factor(Scalar(100.))
				            , maxfev(400)
				            , ftol(std::sqrt(Eigen::NumTraits<Scalar>::epsilon()))
				            , xtol(std::sqrt(Eigen::NumTraits<Scalar>::epsilon()))
				            , gtol(Scalar(0.))
				            , epsfcn(Scalar(0.)) {}
        
    };

    	};

    }
}