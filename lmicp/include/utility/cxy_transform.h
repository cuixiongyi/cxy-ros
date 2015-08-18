
#ifndef CXY_TRANSFORM
#define CXY_TRANSFORM

#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <iostream>

#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "ros/ros.h"
#include <cstdint>

#include "common/cxy_common.h"
namespace cxy
{
namespace cxy_transform
{

enum Axis : uint8_t
		{
            error_code      = 0,
			X_axis_rotation = 1,
			Y_axis_rotation = 2,
			Z_axis_rotation = 3,
			X_axis_translation = 4,
			Y_axis_translation = 5,
			Z_axis_translation = 6,
	        Six_DoF            = 7,
		};
		
    template <typename _Scalar>
	class Pose
	{
		typedef Eigen::Matrix<_Scalar, 3, 1> Vector;
		typedef Eigen::Quaternion<_Scalar> Quaternoin;
		
	private:
		mutable Eigen::Quaternion<_Scalar> q_;
		Eigen::Matrix<_Scalar, 3, 1>  t_;
		mutable bool bhasNormalized_;
	public:
		// constructor with nan asigned
		Pose();

		// constructor with Identity asigned 
		// a doesn't matter
		Pose(int a);

		Pose(const _Scalar& tx
			, const _Scalar& ty
			, const _Scalar& tz
			, const _Scalar& rx
			, const _Scalar& ry
			, const _Scalar& rz);


		~Pose();

		//: The inpute takes the rotating axis and the angle in radian (!!!!! no degree)
		//
		void rotateByAxis(Axis axis, const _Scalar & degree);

		static void rotateByAxis(const Axis& axis, const _Scalar & degree, Quaternoin& q_out);


        //: The inpute takes the rotating axis and the angle in radian (!!!!! no degree)
        static Pose<_Scalar> rotateByAxis_fromIdentity(const Axis & axis, const _Scalar & degree);



        static void composePoint(const pcl::PointXYZ& in_p, pcl::PointXYZ& out_p, std::vector<_Scalar> & para);

		void composePoint(const Vector& in_p, Vector &out_p) const;

		void composePoint(const Vector& in_p, pcl::PointXYZ &out_p) const;

		void composePoint(const pcl::PointXYZ& in_p, pcl::PointXYZ &out_p) const;

		void composePoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud) const;

		void composeDirectionVector(const Vector& in_p, Vector &out_p) const;
		void composeDirectionVector(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_p,  pcl::PointCloud<pcl::PointXYZ>::Ptr& out_p) const;

		void inverseComposePoint(const Vector& in_p, Vector &out_p) const;

		// p = p1*p2 == p1.composePose(p2, p)
		void composePose(const Pose&p2, Pose &out_p) const;
		
		inline void normalize() const { bhasNormalized_ = true; q_.normalize(); return;}
		inline const Vector& t() const {return t_;}
		inline Vector& t() {return t_;}
		inline const Quaternoin& q() const {return q_;}
		inline Quaternoin& q() { bhasNormalized_ = false; return q_;}


		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}
}

#endif  //#ifndef
