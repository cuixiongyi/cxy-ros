#pragma once
#include <cstdint>
#include <ros/time.h>

#include "common/cxy_debug.h"

namespace cxy
{
		
	class cxy_sync
	{
	public:
		cxy_sync(const int32_t&, const int32_t&);
		cxy_sync();
		~cxy_sync();
		static void setInnerLoopThrush(const int32_t&, const int32_t&);


		uint_fast64_t timeDifference(const cxy_sync&);
		bool isInsync_Inner_Loop(const cxy_sync&);



		inline void setSyncNow()
			{ 
				ros::Time now = ros::Time::now();
				sec_ = now.sec;
				nsec_ = now.nsec;
			}
		inline bool isInsync_Inner_Loop()
			{
				return isInsync_Inner_Loop(cxy_sync());
			}
			
		static const uint64_t nsec2secmagnitude_ = {1000000000};
		static int32_t in_sync_inner_loop_thrush_sec_;
		static int32_t in_sync_inner_loop_thrush_nsec_;  /// 10 ms
		static uint64_t in_sync_inner_loop_thrush_nsec_sum_;

	private:
		int32_t sec_ {0};
		int32_t nsec_ {0};

	};
}

