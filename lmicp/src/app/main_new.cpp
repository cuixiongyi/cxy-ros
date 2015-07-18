#include <memory>

#include "common/cxy_config.h"
#include "kinematic/cxy_icp_kinematic.h"


int main(int argc, char const *argv[])
{
	//cxy::config("common/config");
    std::shared_ptr<cxy::cxy_config> config_ptr = std::make_shared<cxy::cxy_config>("/home/xiongyi/cxy_workspace/src/cxyros/lmicp/include/common/config");
    config_ptr->unserialize();

    cxy::cxy_lmicp_lib::cxy_icp_kinematic kin(config_ptr);



	return 0;
}