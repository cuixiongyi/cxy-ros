#include "utility/cxy_sync.h"

namespace cxy
{

	void cxy_sync::setInnerLoopThrush(const int32_t& sec, const int32_t& nsec)
	{
		CXY_ASSERT(sec > 0 && nsec > 0);
		in_sync_inner_loop_thrush_sec_ = sec;
		in_sync_inner_loop_thrush_nsec_ = nsec;
		in_sync_inner_loop_thrush_nsec_sum_ = sec*nsec2secmagnitude_ + nsec;
		return;
	}

	cxy_sync::cxy_sync()
	{
		ros::Time now = ros::Time::now();
		sec_ = now.sec;
		nsec_ = now.nsec;
	}

	cxy_sync::cxy_sync(const int32_t& sec, const int32_t& nsec)
	{
		sec_ = sec;
		nsec_ = nsec;
	}

	cxy_sync::~cxy_sync()
	{

	}

	bool cxy_sync::isInsync_Inner_Loop(const cxy_sync& query)
	{
		// assuming query is the most recent
		uint_fast64_t diff = timeDifference(query);

		if (diff > in_sync_inner_loop_thrush_nsec_sum_)
			return false;

		return true;
	}
		

	uint_fast64_t cxy_sync::timeDifference(const cxy_sync& query)
	{
		int32_t sec_diff = query.sec_ - sec_;
		
		uint_fast64_t nsec_diff = sec_diff*nsec2secmagnitude_ + query.nsec_ - nsec_;

		return nsec_diff;
	}

}