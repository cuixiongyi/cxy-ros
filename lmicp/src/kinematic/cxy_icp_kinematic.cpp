#include "kinematic/cxy_icp_kinematic.h"

namespace cxy
{
namespace cxy_lmicp_lib
{
    template<typename _Scalar>
    cxy_icp_kinematic<_Scalar>::cxy_icp_kinematic(const cxy_config* const config_ptr)
    : config_(config_ptr)
    {
        kc_ = std::make_shared<cxy_icp_kinematic_chain<_Scalar>>(config_ptr);
        std::call_once(joint_Parent_init, cxy_icp_kinematic_joint<_Scalar>::updateJointRelation);

    }

    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::computeResidual(const MatrixX1& x, MatrixX1& res)
    {


        updateJointModel(x);

        kc_->updateModelPoints();

        kc_->getResidual(res);
    }

    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::computeJacobian(const MatrixX1& x, MatrixXX& jac)
    {

        updateJointModel(x);

        kc_->updateModelPoints();

        //points_[ii]->jacobian_.resize(config_->n_num_, cols);

        kc_->getJacobian(jac);
    }



    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr data)
    {
        kc_->setDataCloud(data);



    }

    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::updateJointModel(const MatrixX1& joint_para)
    {
        kc_->setJointPara(joint_para);
        kc_->updateJoints();

        /*
         * TODO update CAD model and get visible points on each joint
         * TODO pointJointIdx_ also need to be assigned

         */
        int point_num = 1;
        config_->setModelPointNum(point_num);
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