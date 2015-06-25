#include "cxy_icp_kinematic_node.h"


namespace cxy
{
    namespace cxy_lmicp_lib
    {
		template<typename _Scalar>
		cxy_icp_kinematic_node::cxy_icp_kinematic_node()
		{
			
		}
				
		template<typename _Scalar>
		pcl::PointCloud<pcl::PointXYZ>::Ptr cxy_icp_kinematic_node::getTransCloud()
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp;
			pose_.composePoint()
		}
				

	}
}