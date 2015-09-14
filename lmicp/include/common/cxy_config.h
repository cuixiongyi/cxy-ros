#pragma once

#include <string>
#include <sstream>
#include <iostream>
#include <boost/preprocessor/arithmetic/inc.hpp>

#include "utility/cxy_transform.h"
#include "common/serialization.h"
#include "common/cxy_debug.h"
#include "common/cxy_joint_info.h"
#include "cxy_tracker_forward_declaration.h"

/*
 * determine n_num_ (the jacobian types) used, at compile time
 */
#ifdef CXY_JACO_TYPE_FITTING
    #define CXY_JACO_TYPE_COUNTING_1    1
#else
    #define CXY_JACO_TYPE_COUNTING_1    0
#endif

#ifdef CXY_JACO_TYPE_COLLISION
    #define CXY_JACO_TYPE_COUNTING_2    BOOST_PP_INC(CXY_JACO_TYPE_COUNTING_1)
#else
    #define CXY_JACO_TYPE_COUNTING_2    CXY_JACO_TYPE_COUNTING_1
#endif

#ifdef CXY_JACO_TYPE_PUSH
    #define CXY_JACO_TYPE_COUNTING_3    BOOST_PP_INC(CXY_JACO_TYPE_COUNTING_2)
#else
    #define CXY_JACO_TYPE_COUNTING_3    CXY_JACO_TYPE_COUNTING_2
#endif

#ifdef CXY_JACO_TYPE_SILHOUETTE
    #define CXY_JACO_TYPE_COUNTING_4    BOOST_PP_INC(CXY_JACO_TYPE_COUNTING_3)
#else
    #define CXY_JACO_TYPE_COUNTING_4    CXY_JACO_TYPE_COUNTING_3
#endif

#ifdef CXY_JACO_TYPE_JOINTLIMIT
    #define CXY_JACO_TYPE_COUNTING_5    BOOST_PP_INC(CXY_JACO_TYPE_COUNTING_4)
#else
#define CXY_JACO_TYPE_COUNTING_5    CXY_JACO_TYPE_COUNTING_4
#endif
/*
#ifdef CXY_JACO_TYPE_SILHOUETTE
    #define CXY_JACO_TYPE_COUNTING_4    BOOST_PP_INC(CXY_JACO_TYPE_COUNTING_3)
#else
#define CXY_JACO_TYPE_COUNTING_4    CXY_JACO_TYPE_COUNTING_3
#endif
*/
#define CXY_JACO_TYPE_COUNT_FINAL       CXY_JACO_TYPE_COUNTING_5


namespace cxy
{

	class cxy_config
	{

	public:

		cxy_config& operator=(const cxy_config&) = delete;	// Disallow copying
		cxy_config(const cxy_config&) = delete;
		///cxy_config() = delete;
		cxy_config();
		~cxy_config();

        //cxy_config& operator=(const cxy_config&) = delete;
        //cxy_config(const cxy_config&) = delete;
        static void parseJoints();

        static cxy_transform::Axis parseJointType(const std::string&);

        static std::shared_ptr<const cxy_config> getConfig();

		virtual void serialize();

		static void unserialize();


        //std::shared_ptr<cxy_lmicp_lib::cxy_icp_kinematic<float>> kinematic_ptr_;


        static std::string filename_;
        static std::string filePrefix_;
        static bool isOpen_ ;
        static unsigned int joint_number_ ;
        static std::vector<cxy_joint_info> joint_config_;

        static unsigned int random_seed;

        static int joint_DoFs ;
        static std::vector<int> jointParaIdx_;
        // this is the number of jacobian type used
        static constexpr int n_num_ = {CXY_JACO_TYPE_COUNT_FINAL};

        static bool withJacobianEffectChild;
        static int withJacobianEffectChild_HierarchyMax;
        static bool with_icp_jacobian ;
        static float icp_jaclbian_weight;

        static bool with_collision_jacobian ;
        static float collision_jacobian_weight ;

        static bool with_silhouette_jacobian;
        static float silhouette_jacobian_weight;

        static bool with_push_jacobian;
        static float push_jacobian_weight;

	public:
        static inline const bool& isOpen()
		{
			return isOpen_;
		}


        inline void getJacobianSize(long& rows, long& cols) const
        {
            rows = model_Point_num * n_num_;
            cols = joint_DoFs;
        }

    private:
        mutable int model_Point_num = {0};
        static std::ifstream fin_;


        static bool getline(std::istream& is, std::string& line, int8_t& lineStatus)
        {
            lineStatus = 0;

            bool result = std::getline(is, line);
            if (false == result)
            {
                lineStatus = -1;
                return false;
            }
            if (line.find("#") != std::string::npos)
            {
                lineStatus = -2;
                return true;
            }
            if ("" == line)
            {
                lineStatus = -2;
                return true;
            }
            return true;
        }
    public:
        static const std::string rviz_frame_name_ ;
    };
}