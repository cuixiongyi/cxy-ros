
#include "cxy_icp_arti.h"

namespace cxy {
    namespace cxy_lmicp_lib {

        template<typename _Scalar, int _MinimizerType>
        bool cxy_icp_arti<_Scalar, _MinimizerType>::setKinematicChain(std::shared_ptr<cxy_icp_kinematic_chain<_Scalar>> kc)
        {

            hasSetKC_ = true;
            //pcl::PointCloud<pcl::PointXYZ>::Ptr tmp = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(*model);
            //translateToCenter(tmp);
            //modelCloud_ = tmp;
            kc_ = kc;
            /*
            for (int ii = 0; ii < (*kc_).size(); ++ii)
            {
                CXY_ASSERT(nullptr == modelCloud_[ii]);
                if (nullptr == modelCloud_[ii] || modelCloud_[ii]->size() < 5)
                {
                    return false;
                }
            }
            */
            return true;

        }

        template<typename _Scalar, int _MinimizerType>
        inline bool cxy_icp_arti<_Scalar, _MinimizerType>::setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr data)
        {
            hasSetDataCloud_ = true;
            dataCloud_ = data;
            kdtreeptr_ = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>);
            kdtreeptr_->setInputCloud(dataCloud_);
            
        }


        // There are 3 layers of minimization class
        // The 1st layer is the cxy_icp, is abstract, should not be changed
        // 2nd layer is cxy_icp_rigid, deal with cost function
        // 3rd layer is cxy_icp_rigid_lm, deal with minimization interface

        //: This function belong to 1st layer
    template<typename _Scalar, int _MinimizerType>
        _Scalar cxy_icp_arti<_Scalar, _MinimizerType>::icp_run(Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> &x)
        {
            if ( ! hasSetKC_ || ! hasSetDataCloud_)
            {
                return -1.0;
            }
            icp_prepare_cost_function();
            _Scalar result = icp_minimization(x);

            
            return result;
        }            


    template<typename _Scalar, int _MinimizerType>
        cxy_icp_arti<_Scalar, _MinimizerType>::cxy_icp_arti() :  hasSetKC_(false)
                , hasSetDataCloud_(false)
                , func_(nullptr)
        {

        }
        template<typename _Scalar, int _MinimizerType>
        cxy_icp_arti<_Scalar, _MinimizerType>::~cxy_icp_arti()
        {
            if (func_ != nullptr)
                delete func_;
        }

    template<typename _Scalar, int _MinimizerType>
        int cxy_icp_arti<_Scalar, _MinimizerType>::icp_prepare_cost_function()
        {
            //func_ = std::make_shared<cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar>>(kc_->size(), kc_, this->dataCloud_, this->kdtreeptr_);
            return 1;
        }

    template<typename _Scalar, int _MinimizerType>
        _Scalar cxy_icp_arti<_Scalar, _MinimizerType>::icp_minimization(Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> &x)
        {
            cxy_optimization::cxy_nonlinear_method state(static_cast<cxy_optimization::cxy_nonlinear_method>(_MinimizerType));
            if ( cxy_optimization::cxy_nonlinear_method::EIGEN_MINPACK == state)
            {

                for (int ii = 0; ii < kc_->size(); ii++)
                {
                    func_ = std::make_shared<cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar>>(1, kc_, this->dataCloud_, this->kdtreeptr_, ii, x);

                    Eigen::LevenbergMarquardt <cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar>, _Scalar > lm2(*func_);
                    Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> x_joint;
                    x_joint.resize(1);
                    x_joint(0) = x(ii);
                    lm2.lmder1(x_joint);
                }
                
            }
           
            //cxy_optimization::cxy_nonlinear_minimizer_LM<cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar>, _Scalar > lm(*this->func_);
            //return lm.minimize(x);
        }


    }
}

template class cxy::cxy_lmicp_lib::cxy_icp_arti<float, 1>;
template class cxy::cxy_lmicp_lib::cxy_icp_arti<float, 2>;
template class cxy::cxy_lmicp_lib::cxy_icp_arti<double, 1>;
template class cxy::cxy_lmicp_lib::cxy_icp_arti<double, 2>;
