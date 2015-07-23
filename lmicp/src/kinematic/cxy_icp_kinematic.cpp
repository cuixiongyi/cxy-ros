#include "kinematic/cxy_icp_kinematic.h"

namespace cxy
{
namespace cxy_lmicp_lib
{
    template<typename _Scalar>
    cxy_icp_kinematic<_Scalar>::cxy_icp_kinematic(const cxy_config* const config_ptr)
    : config_(config_ptr)
    {
        kc_ = std::make_shared<cxy_icp_kinematic_chain>(config_);
        matrix_cols_Jac_ = config_->joint_DoFs;

    }

    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::computeResidual(const MatrixX1& x, MatrixX1& res)
    {


        updateJointModel(x);

        kc_->updateModelPoints();

    }

    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::computeJacobian(const MatrixX1& x, MatrixXX& jac)
    {

        updateJointModel(x);

    }



    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr data)
    {
        kc_->setDataCloud(data);

        setMatrixSize(data->points.size());

    }


    void cxy_icp_kinematic<_Scalar>::updateJointModel(const MatrixX1& joint_para)
    {
        kc_->setJointPara(joint_para);
        kc_->updateJoints();

        /*
         * TODO update CAD model and get visible points on each joint
         * TODO pointJointIdx_ also need to be assigned

         */
        matrix_model_point_size_ = ;
        kc_->getFullModelCloud_World();

        /*
         * TODO for now asumming the model points are stored in modelCloud_
         */
        kc_->updateModelPoints();
    }


    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::setMatrixSize(const int& point_size)
    {
        matrix_model_point_size_ = point_size;
        matrix_rows_Jac_ = point_size * config_->n_num_;
    }


    template<typename _Scalar>
    cxy_icp_kinematic<_Scalar>::~cxy_icp_kinematic()
    {

    }

}
}