#pragma once

#include "cxy_icp.h"
#include "cxy_transform.h"
#include "optimization/Cxy_Cost_Func_Abstract.h"

namespace cxy {
    namespace cxy_lmicp_lib {

        class cxy_icp_rigid : public cxy_icp
        {

        public:
            cxy_icp_rigid();


            const Matrix34f calculateJacobianKernel(const std::vector<float> &para
                                                                          , const pcl::PointXYZ& a);

        protected:

            virtual dataType matchPointCloud();

            virtual const dataType matchPointCloud(const pcl::PointXYZ& data
                                                , Eigen::Vector3f& res);

            virtual dataType residual(dataIdxType const& dataIdx
                                            , dataVectorType const& para);
            virtual dataVectorType residual_derivative(dataIdxType const& dataIdx
                                                      , dataVectorType const &para);
        };
    }
}