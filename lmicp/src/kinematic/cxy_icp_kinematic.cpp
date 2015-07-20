#include "kinematic/cxy_icp_kinematic.h"

namespace cxy
{
namespace cxy_lmicp_lib
{
    cxy_icp_kinematic::cxy_icp_kinematic(std::shared_ptr<cxy_config> config_ptr)
    {
        config_ = config_ptr;
        kc_ = std::make_shared<cxy_icp_kinematic_chain>(config_);
        matrix_cols_J_ = config_->joint_DoFs;

    }

    void cxy_icp_kinematic::computeJacobian(Eigen::Matrix< float, Eigen::Dynamic, Eigen::Dynamic>& jac)
    {

    }


    void cxy_icp_kinematic::setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr data)
    {
        kc_->setDataCloud(data);

        setMatrixSize(data->points.size());

    }

    void cxy_icp_kinematic::setMatrixSize(const int& point_size)
    {
        matrix_rows_ = point_size;
        matrix_rows_J_ = point_size * config_->n_num_;
    }

    void updateModel()
    {

    }

}
}