#pragma once

#include <string>
#include <sstream>
#include <iostream>

#include "utility/cxy_transform.h"
#include "common/serialization.h"
#include "common/cxy_debug.h"

namespace cxy
{
    struct cxy_config_joint
    {
        int joint_idx = {-1};
        int joint_parent = {-1};

        cxy_transform::Axis jointType;
        std::string model_filename;
        float t[3];
        float r[3];

    };
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

		int joint_number_ = {0};
		std::vector<cxy_config_joint> kinematic_config_;
	};
}