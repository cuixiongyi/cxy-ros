#pragma once

#include "Cxy_Cost_Func_Abstract.h"
#include <vector>

namespace cxy
{

    namespace cxy_optimization
    {
        //
        template<class Return, class Argument>
        class Cxy_Cost_Func_Vector : public Cxy_Cost_Func_Abstract<Return, Argument, std::vector<Return>, std::vector<Return>>
        {
        public:
            //: assuming std::vector as list and parameter and derivative is the same type with Return
            virtual Return residual(Argument const& data, std::vector<Return> const& p) = 0;

            virtual std::vector<Return> residual_derivative(Argument const& data, std::vector<Return> const& p) = 0;

            virtual ~Cxy_Cost_Func_Vector() {}

        private:
            virtual Cxy_Cost_Func_Vector( const Cxy_Cost_Func_Abstract& other ) = delete;
            virtual Cxy_Cost_Func_Vector &operator=(const Cxy_Cost_Func_Abstract &) = delete;


        };

    }
}