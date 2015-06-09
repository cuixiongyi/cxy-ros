#include "cxy_lmicp.h"

namespace cxy {
    namespace cxy_lmicp_lib {

        cxy_lmicp::cxy_lmicp() : cxy_icp()
        {
            func_ = new cxy_icp_func_rigid;

        }
    }
}