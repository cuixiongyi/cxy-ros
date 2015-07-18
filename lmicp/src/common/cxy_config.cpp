
#include "common/cxy_config.h"

namespace cxy
{

	cxy_config::cxy_config(std::string filename) : 
					serialization(filename)
					, filename_{filename}
		{};
	cxy_config::~cxy_config() {};
	
	void cxy_config::serialize()
			{}
	void cxy_config::unserialize()
	{
		if (fin_.is_open())
			fin_.close();
		fin_.open(filename_);
		//std::cout<<filename_<<std::endl;
		if (! fin_.is_open())
			throw std::runtime_error("open file fail");
		std::string line;
        int8_t lineStatus = 0;
		while (this->getline(fin_, line, lineStatus))
		{
            if (-2 == lineStatus)
            {
                std::cout<<"comment : "<<line<<std::endl;
                continue;

            }
			std::stringstream iss(line);
			std::string var_name;
			iss >> var_name;


            if ( "joint_number" == var_name)
			{
				iss >> joint_number_;
                parseJoints();
			}
		}		
	}

    void cxy_config::parseJoints()
    {
        std::string line;
        int8_t lineStatus = 0;
        kinematic_config_.reserve(joint_number_);

        for (int ii = 0; ii < joint_number_; ++ii)
        {
            if ( ! this->getline(fin_, line, lineStatus))
                throw std::runtime_error("config file incomplete, no joint info found");
            if ( -2 == lineStatus)
            {
                --ii;
                continue;
            }

            std::stringstream iss(line);
            cxy_config_joint kj;

            // joint index
            iss>>kj.joint_idx;
            CXY_ASSERT(kj.joint_idx == ii);
            //std::cout<<"joint line: "<<line<<std::endl;

            // joint parent
            iss>>kj.joint_parent;
            CXY_ASSERT(kj.joint_parent < ii && kj.joint_parent >= -1 );

            // joint joint type
            std::string jointType;
            iss>>jointType;

            kj.jointType = parseJointType(jointType);
            CXY_ASSERT(kj.jointType != cxy_transform::Axis::error_code);

            // joint model file name
            iss>>kj.model_filename;

            // the second line
            while (1)
            {
                if (!this->getline(fin_, line, lineStatus))
                    throw std::runtime_error("config file incomplete, no joint info found");

                if (0 == lineStatus)
                {
                    break;
                }

            }
            //std::stringstream ss2(line);
            iss.clear();
            iss.str(line);
            //std::cout<<"iss: "<<line<<std::endl;
            iss>>kj.t[0]>>kj.t[1]>>kj.t[2]>>kj.r[0]>>kj.r[1]>>kj.r[2];

            kinematic_config_.push_back(kj);
        }
    }
    cxy_transform::Axis cxy_config::parseJointType(const std::string& str)
    {
        if ("X_axis_rotation" == str)
            return cxy_transform::Axis::X_axis_rotation;
        if ("Y_axis_rotation" == str)
            return cxy_transform::Axis::Y_axis_rotation;
        if ("Z_axis_rotation" == str)
            return cxy_transform::Axis::Z_axis_rotation;
        if ("X_axis_translation" == str)
            return cxy_transform::Axis::X_axis_translation;
        if ("Y_axis_translation" == str)
            return cxy_transform::Axis::Y_axis_translation;
        if ("Z_axis_translation" == str)
            return cxy_transform::Axis::Z_axis_translation;
        if ("Six_DoF" == str)
            return cxy_transform::Axis::Six_DoF;

        return cxy_transform::Axis::error_code;
    }

}