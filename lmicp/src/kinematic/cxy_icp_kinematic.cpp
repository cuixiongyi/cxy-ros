#include "kinematic/cxy_icp_kinematic.h"

namespace cxy
{
namespace cxy_kinematic
{
    template<typename _Scalar>
    std::once_flag cxy_icp_kinematic<_Scalar>::joint_Parent_init;

    template<typename _Scalar>
    cxy_icp_kinematic<_Scalar>::cxy_icp_kinematic(const cxy_config *const config_ptr)
    : config_(config_ptr)
    {
        kc_ = std::make_shared<cxy_icp_kinematic_chain<_Scalar>>(config_ptr);
//        std::call_once(joint_Parent_init, cxy_icp_kinematic_joint<_Scalar>::updateJointRelation);

    }


    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::computeResidual(const MatrixX1& x, MatrixX1& res)
    {


        updateJointModel(x);

        //kc_->updateModelPoints();

        kc_->getResidual(res);
    }

    template<typename _Scalar>
    void cxy_icp_kinematic<_Scalar>::computeJacobian(const MatrixX1& x, MatrixXX& jac)
    {

        updateJointModel(x);

        //kc_->updateModelPoints();


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
        CXY_ASSERT(joint_para.rows() == cxy_config::joint_DoFs);
        kc_->updateJoints(joint_para);

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
template class cxy::cxy_kinematic::cxy_icp_kinematic<float>;
template class cxy::cxy_kinematic::cxy_icp_kinematic<double>;
