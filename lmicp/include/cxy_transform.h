#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <iostream>

#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "ros/ros.h"


namespace cxy_transform
{
    namespace E = Eigen;

	class Pose
	{
	private:
		E::Quaternion<double> q_;
		E::Vector3d t_;
	public:
		// constructor with nan asigned
		Pose();
		// constructor with Identity asigned 
		// a doesn't matter
		Pose(int a);

		~Pose();
		static void composePoint(const pcl::PointXYZ& in_p, pcl::PointXYZ& out_p, std::vector<float> & para);
		static void composePoint(const pcl::PointXYZ& in_p, pcl::PointXYZ& out_p, std::vector<double> & para);
		void composePoint(const E::Vector3d& in_p, E::Vector3d &out_p);
		void composePoint(const E::Vector3d& in_p, pcl::PointXYZ &out_p);
		void composePoint(const pcl::PointXYZ& in_p, pcl::PointXYZ &out_p);
		void composePoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud);

		void inverseComposePoint(const E::Vector3d& in_p, E::Vector3d &out_p);

		// p = p1*p2 == p1.composePose(p2, p)
		void composePose(const Pose& in_p, Pose &out_p);
		void normalize();
		const E::Vector3d& t() const {return t_;};
		E::Vector3d& t() {return t_;};
		const E::Quaterniond& q() const {return q_;};
		E::Quaterniond& q() {return q_;};
	};
}