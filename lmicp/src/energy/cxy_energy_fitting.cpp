#include "energy/cxy_energy_fitting.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {

        template<typename _Scalar>
        cxy_energy_fitting<_Scalar>::cxy_energy_fitting()
        {

        }

        template<typename _Scalar>
        cxy_energy_fitting<_Scalar>::~cxy_energy_fitting()
        {
        }

        static void cxy_energy_fitting<_Scalar>::computeJacobian(const cxy_icp_kinematic_point*& point_)
        {

            transCloud = kc_->getOneModelCloud_World(x_full_, joint_, pose, para_pose_parent);

            Eigen::Matrix< _Scalar, 3, 1> cross = rotation_axis.cross(tmp);
            Eigen::Matrix< _Scalar, 3, 1> cross_norm = cross / (std::sqrt(cross(0)*cross(0)+cross(1)*cross(1)+cross(2)*cross(2)));
            const float jac_step_scale = 0.3;
            const float r3_length = std::sqrt(r3(0)*r3(0)+r3(1)*r3(1)+r3(2)*r3(2));
            float scale0 = r3_length * jac_step_scale * cross_norm(0);
            float scale1 = r3_length * jac_step_scale * cross_norm(1);
            float scale2 = r3_length * jac_step_scale * cross_norm(2);
            if (std::isnan(scale0))
                scale0 = 0.0;
            if (std::isnan(scale1))
                scale1 = 0.0;
            if (std::isnan(scale2))
                scale2 = 0.0;
            if (std::abs(r3(0)+scale0) + std::abs(r3(1)+scale1) + std::abs(r3(2)+scale2) > std::abs(r3(0)) + std::abs(r3(1)) + std::abs(r3(2)))
            {
                fjac(jj, 0) =  -std::abs(cross(2) + cross(1) + cross(0));
                if (jj == 20)
                    ROS_INFO_STREAM("reversed");
            }
            else
            {
                fjac(jj, 0) =  std::abs(cross(2) + cross(1) + cross(0));
                //fjac(jj, 0) = -fjac(jj, 0);
            }

            jacS += fjac(jj, 0);
        }


    }
}
