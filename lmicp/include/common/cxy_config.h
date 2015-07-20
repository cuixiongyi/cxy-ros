#pragma once

#include <string>
#include <sstream>
#include <iostream>

#include "utility/cxy_transform.h"
#include "common/serialization.h"
#include "common/cxy_debug.h"
#include "common/cxy_joint_info.h"

namespace cxy
{

	class cxy_config : public serialization
	{
	public:

		cxy_config& operator=(const cxy_config&) = delete;	// Disallow copying
		cxy_config(const cxy_config&) = delete;
		cxy_config() = delete;
		cxy_config(std::string filename);
		~cxy_config();

        void parseJoints();

        cxy_transform::Axis parseJointType(const std::string&);


		virtual void serialize();

		virtual void unserialize();


				
		std::string filename_;
		bool isOpen_ = {false};
		int joint_number_ = {0};
		std::vector<cxy_joint_info> joint_config_;

        int joint_DoFs = {0};

        // this is the number of jacobian type used
        int n_num_ = {1};

        bool with_icp_jacobian = {true};
        float icp_jaclbian_weight = {10};

        bool with_collision_jacobian = {false};
        float collision_jacobian_weight = {10};

        bool with_silhouette_jacobian = {false};
        float silhouette_jacobian_weight = {10};

        bool with_push_jacobian = {false};
        float push_jacobian_weight = {10};

	public:
		inline const bool& isOpen()
		{
			return isOpen_;
		}
	};
}