#pragma once


namespace cxy
{

    namespace cxy_optimization
    {

        template<class Return, class Argument, class Parameter, class Derivative>
        class Cxy_Cost_Func_Abstract
        {
        public:

            //: This cost function is inspired by dlib and vnl library
            // The idea is to use Inheritance to pass the residual and derivative function
            // The benefit is that member data could be used in the residual and derivative computation.
            // Because there are case that addition information is needed, such as find a nearest neighbor
            // Return is either float or double
            // Argument is data point, such as pcl::PointXYZ
            // Parameter is a list for parameters, such as std::vector<Return>
            // Derivative is a list for derivative returned by function: residual_derivative, such as std::vector<Return>
            virtual Return residual(Argument const& data, Parameter const& p) = 0;

            virtual Derivative residual_derivative(Argument const& data, Parameter const& p) = 0;

            virtual ~Cxy_Cost_Func_Abstract() {}

        private:
            virtual Cxy_Cost_Func_Abstract( const Cxy_Cost_Func_Abstract& other ) = delete;
            virtual Cxy_Cost_Func_Abstract &operator=(const Cxy_Cost_Func_Abstract &) = delete;

        };

    }
}
