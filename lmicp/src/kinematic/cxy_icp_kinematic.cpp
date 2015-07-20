#include "kinematic/cxy_icp_kinematic.h"

namespace cxy
{
namespace cxy_lmicp_lib
{
    template<typename _Scalar>
    cxy_icp_kinematic<_Scalar>::cxy_icp_kinematic(std::shared_ptr<const cxy_config> config_ptr)
    : config_(config_ptr)
    {
        kc_ = std::make_shared<cxy_icp_kinematic_chain>(config_);
        matrix_cols_J_ = config_->joint_DoFs;

    }

    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::computeJacobian(Eigen::Matrix< float, Eigen::Dynamic, Eigen::Dynamic>& jac)
    {

    }


    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr data)
    {
        kc_->setDataCloud(data);

        setMatrixSize(data->points.size());

    }

    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::setMatrixSize(const int& point_size)
    {
        matrix_rows_ = point_size;
        matrix_rows_J_ = point_size * config_->n_num_;
    }

    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::updateModel()
    {

    }

}
}