#pragma once

#include "cxy_icp_arti.h"

namespace cxy {
    namespace cxy_lmicp_lib {

    template<typename _Scalar, int _MinimizerType>
        bool setModelCloud(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> model)
        {
            
            hasSetModelCloud_ = true;
            //pcl::PointCloud<pcl::PointXYZ>::Ptr tmp = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(*model);
            //translateToCenter(tmp);
            //modelCloud_ = tmp;
            modelCloud_ = model;
            for (int ii = 0; ii < modelCloud_.size(); ++ii)
            {
                /* code */
                CXY_ASSERT(nullptr == modelCloud_[ii])
                if (nullptr == modelCloud_[ii] || modelCloud_[ii]->size() < 5)
                {
                    return false;
                }
            }

            return true;

        }

        template<typename _Scalar, int _MinimizerType>
        inline bool setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr data)
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
        _Scalar icp_run(Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> &x)
        {
            if ( ! hasSetModelCloud_ || ! hasSetDataCloud_)
            {
                return -1.0;
            }
            icp_prepare_cost_function();
            _Scalar result = icp_minimization(x);

            
            return result;
        }            


    template<typename _Scalar, int _MinimizerType>
        cxy_icp_arti()
        {
            
        }

    template<typename _Scalar, int _MinimizerType>
        int icp_prepare_cost_function()
        {
            cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar>* tmp = new cxy_icp_arti_func<_Scalar>(this->modelCloud_, this->dataCloud_, this->kdtreeptr_);
            this->func_ = tmp;
            return 1;
        }

    template<typename _Scalar, int _MinimizerType>
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
                    return lm2.lmder1(x);
                }


            }
            //cxy_optimization::cxy_nonlinear_minimizer_LM<cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar>, _Scalar > lm(*this->func_);
            //return lm.minimize(x);
        }


    }
}
