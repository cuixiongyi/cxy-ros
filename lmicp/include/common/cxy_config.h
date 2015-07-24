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

#define CXY_JACO_TYPE_COUNT_FINAL       CXY_JACO_TYPE_COUNTING_4


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

        static std::shared_ptr<const cxy_config> getConfig();

		virtual void serialize();

		virtual void unserialize();


        std::shared_ptr<cxy_lmicp_lib::cxy_icp_kinematic> kinematic_ptr_;


        static std::string filename_;
        static bool isOpen_ = {false};
        static int joint_number_ = {0};
        static std::vector<cxy_joint_info> joint_config_;

        static int joint_DoFs = {0};

        // this is the number of jacobian type used
        static constexpr int n_num_ = {CXY_JACO_TYPE_COUNT_FINAL};

        static bool with_icp_jacobian = {true};
        static float icp_jaclbian_weight = {10};

        static bool with_collision_jacobian = {false};
        static float collision_jacobian_weight = {10};

        static bool with_silhouette_jacobian = {false};
        static float silhouette_jacobian_weight = {10};

        static bool with_push_jacobian = {false};
        static float push_jacobian_weight = {10};

	public:
        static inline const bool& isOpen()
		{
			return isOpen_;
		}

        inline const int& getModelPointNum() const { return model_Point_num;}
        inline void setModelPointNum(const int& num) const { model_Point_num = num;}

        inline void getJacobianSize(int& rows, int& cols) const
        {
            rows = model_Point_num * n_num_;
            cols = joint_DoFs;
        }

    private:
        mutable int model_Point_num = {0};


    };
}