#include "kinematic/cxy_icp_kinematic.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {
        cxy_icp_kinematic::cxy_icp_kinematic(std::shared_ptr<cxy_config> config_ptr)
        {
            config_ = config_ptr;
            kc_ = std::make_shared<cxy_icp_kinematic_chain>(config_);
            kc_->constructKinematicChain();
        }


    }
}