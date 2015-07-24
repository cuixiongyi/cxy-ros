#include "kinematic/cxy_icp_kinematic.h"

namespace cxy
{
namespace cxy_lmicp_lib
{
    template<typename _Scalar>
    cxy_icp_kinematic<_Scalar>::cxy_icp_kinematic(const cxy_config* const config_ptr)
    : config_(config_ptr)
    {
        kc_ = std::make_shared<cxy_icp_kinematic_chain>(config_ptr);


    }

    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::computeResidual(const MatrixX1& x, MatrixX1& res)
    {


        updateJointModel(x);

        kc_->updateModelPoints();

        kc_->getResidual();
    }

    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::computeJacobian(const MatrixX1& x, MatrixXX& jac)
    {

        updateJointModel(x);

        kc_->updateModelPoints();
        int rows, cols;
        config_->getJacobianSize(rows, cols);
        (*points_)[ii]->jacobian_.resize(config_->n_num_, cols);
    }



    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr data)
    {
        kc_->setDataCloud(data);



    }


    void cxy_icp_kinematic<_Scalar>::updateJointModel(const MatrixX1& joint_para)
    {
        kc_->setJointPara(joint_para);
        kc_->updateJoints();

        /*
         * TODO update CAD model and get visible points on each joint
         * TODO pointJointIdx_ also need to be assigned

         */
        config_->setModelPointNum();
        kc_->getFullModelCloud_World();

        /*
         * TODO for now asumming the model points are stored in modelCloud_
         */
        kc_->updateModelPoints();
    }




    template<typename _Scalar>
    cxy_icp_kinematic<_Scalar>::~cxy_icp_kinematic()
    {

    }

}
}