
#include "common/cxy_config.h"

namespace cxy
{
     std::string cxy_config::filename_ = {"/home/xiongyi/cxy_workspace/src/cxyros/lmicp/include/common/config"};
      bool cxy_config::isOpen_ = {false};
      int cxy_config::joint_number_ = {0};
      std::vector<cxy_joint_info> cxy_config::joint_config_;
     std::vector<int> cxy_config::jointParaIdx_;

      int cxy_config::joint_DoFs = {0};

      bool cxy_config::with_icp_jacobian = {true};
      float cxy_config::icp_jaclbian_weight = {10};

      bool cxy_config::with_collision_jacobian = {false};
      float cxy_config::collision_jacobian_weight = {10};

      bool cxy_config::with_silhouette_jacobian = {false};
      float cxy_config::silhouette_jacobian_weight = {10};

      bool cxy_config::with_push_jacobian = {false};
      float cxy_config::push_jacobian_weight = {10};
     std::ifstream cxy_config::fin_;


	cxy_config::cxy_config()
		{
            //kinematic_ptr_ = std::make_shared<cxy_lmicp_lib::cxy_icp_kinematic<float>>(this);
        };
	cxy_config::~cxy_config()
    {

    }
	
	void cxy_config::serialize()
    {

    }

    std::shared_ptr<const cxy_config> cxy_config::getConfig()
    {
        return std::make_shared<const cxy_config>();
    }

    void cxy_config::unserialize() 
	{
        if (isOpen_)
            return;
		if (fin_.is_open())
			fin_.close();
		fin_.open(filename_);
		//std::cout<<filename_<<std::endl;
		if (! fin_.is_open())
			throw std::runtime_error("open file fail");
		std::string line;
        int8_t lineStatus = 0;
		while (getline(fin_, line, lineStatus))
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
                joint_DoFs = 0;
                jointParaIdx_.resize(joint_number_);
                parseJoints();
			}

		}
#ifdef CXY_JACO_TYPE_FITTING
    with_icp_jacobian = true;
    icp_jaclbian_weight = 1.0;
#else
        with_icp_jacobian = false;
#endif
#ifdef CXY_JACO_TYPE_COLLISION
    with_collision_jacobian = true;
    collision_jacobian_weight = 1.0;
#else
        with_collision_jacobian = false;
#endif
#ifdef CXY_JACO_TYPE_PUSH
    with_push_jacobian = true;
    push_jacobian_weight = 1.0;
#else
        with_push_jacobian = false;
#endif
#ifdef CXY_JACO_TYPE_SILHOUETTE
    with_silhouette_jacobian = true;
    silhouette_jacobian_weight = 1.0;
#else
        with_silhouette_jacobian = false;
#endif
        isOpen_ = true;
        std::cout<<"config test"<<with_collision_jacobian<<"  "<<collision_jacobian_weight;

	}

    void cxy_config::parseJoints()
    {
        std::string line;
        int8_t lineStatus = 0;
        joint_config_.reserve(joint_number_);
        int jointParaIdxCount = 0;
        for (int ii = 0; ii < joint_number_; ++ii)
        {
            if ( ! getline(fin_, line, lineStatus))
                throw std::runtime_error("config file incomplete, no joint info found");
            if ( -2 == lineStatus)
            {
                --ii;
                continue;
            }

            std::stringstream iss(line);
            cxy_joint_info kj;

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

            if (cxy_transform::Axis::Six_DoF == kj.jointType)
                kj.DoF = 6;
            else
                kj.DoF = 1;
            jointParaIdx_[ii] = jointParaIdxCount;
            jointParaIdxCount += kj.DoF;
            // joint model file name
            iss>>kj.model_filename;

            // the second line
            while (1)
            {
                if (!getline(fin_, line, lineStatus))
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

            joint_config_.push_back(kj);
        }
    }
    cxy_transform::Axis cxy_config::parseJointType(const std::string& str)
    {

        if ("X_axis_rotation" == str)
        {
            ++joint_DoFs;
            return cxy_transform::Axis::X_axis_rotation;
        }
        if ("Y_axis_rotation" == str)
        {
            ++joint_DoFs;
            return cxy_transform::Axis::Y_axis_rotation;

        }
        if ("Z_axis_rotation" == str)
        {
            ++joint_DoFs;
            return cxy_transform::Axis::Z_axis_rotation;
        }
        if ("X_axis_translation" == str)
        {
            ++joint_DoFs;
            return cxy_transform::Axis::X_axis_translation;
        }
        if ("Y_axis_translation" == str)
        {
            ++joint_DoFs;
            return cxy_transform::Axis::Y_axis_translation;
        }
        if ("Z_axis_translation" == str)
        {
            ++joint_DoFs;
            return cxy_transform::Axis::Z_axis_translation;
        }
        if ("Six_DoF" == str)
        {
            joint_DoFs += 6;
            return cxy_transform::Axis::Six_DoF;
        }

        return cxy_transform::Axis::error_code;
    }

}