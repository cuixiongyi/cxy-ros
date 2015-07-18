#pragma once

#include <string>
#include <sstream>
#include <iostream>

#include "utility/cxy_transform.h"
#include "common/serialization.h"
#include "common/cxy_debug.h"
#include "common/joint_info.h"

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


				
	private:
		std::string filename_;
		bool isOpen_ = {false};
		int joint_number_ = {0};
		std::vector<joint_info> joint_config_;

	public:
		inline const bool& isOpen()
		{
			return isOpen_;
		}
	};
}