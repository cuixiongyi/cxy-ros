#include "cxy_transform.h"



namespace cxy_transform
{


		Pose::Pose() :
				 t_(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN())
				, q_(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN())
		{

		}

		Pose::Pose(int a) :
				 t_(0.0, 0.0, 0.0)
				, q_(1.0, 0.0, 0.0, 0.0)
		{

		}
		Pose::~Pose()
		{

		}
		void Pose::composePoint(const E::Vector3d& in_p, E::Vector3d &out_p)
		{
			/*q_.normalize();
			out_p = q_.matrix()*in_p + t_;
			*/
			q_.normalize();
			out_p(0) = in_p(0)+t_(0) + 2*(-(q_.y()*q_.y()+q_.z()*q_.z())*in_p(0) + (q_.x()*q_.y()-q_.w()*q_.z())*in_p(1) + (q_.w()*q_.y()+q_.x()*q_.z())*in_p(2));
			out_p(1) = in_p(1)+t_(1) + 2*((q_.w()*q_.z()+q_.x()*q_.y())*in_p(0) - (q_.x()*q_.x()+q_.z()*q_.z())*in_p(1) + (q_.y()*q_.z()-q_.w()*q_.x())*in_p(2));
			out_p(2) = in_p(2)+t_(2) + 2*((q_.x()*q_.z()-q_.w()*q_.y())*in_p(0) + (q_.w()*q_.x()+q_.y()*q_.z())*in_p(1) - (q_.x()*q_.x()+q_.y()*q_.y())*in_p(2));
			
			return;
		}
		void Pose::composePoint(const pcl::PointXYZ& in_p, pcl::PointXYZ& out_p, std::vector<float> & para)
		{
			E::Vector3d in(in_p.x, in_p.y, in_p.z) ,out;
			Pose p;
			p.t()(0) = para[0];
			p.t()(1) = para[1];
			p.t()(2) = para[2];
			p.q().w() = para[3];
			p.q().x() = para[4];
			p.q().y() = para[5];
			p.q().z() = para[6];
			p.composePoint(in, out);
			out_p.x = out(0);
			out_p.y = out(1);
			out_p.z = out(2);
			para[3] = p.q().w();
			para[4] = p.q().x();
			para[5] = p.q().y();
			para[6] = p.q().z();
		}

void Pose::composePoint(const pcl::PointXYZ& in_p, pcl::PointXYZ& out_p, std::vector<double> & para)
		{
			E::Vector3d in(in_p.x, in_p.y, in_p.z) ,out;
			Pose p;
			p.t()(0) = para[0];
			p.t()(1) = para[1];
			p.t()(2) = para[2];
			p.q().w() = para[3];
			p.q().x() = para[4];
			p.q().y() = para[5];
			p.q().z() = para[6];
			p.composePoint(in, out);
			out_p.x = out(0);
			out_p.y = out(1);
			out_p.z = out(2);
			para[3] = p.q().w();
			para[4] = p.q().x();
			para[5] = p.q().y();
			para[6] = p.q().z();
		}
		void Pose::composePoint(const E::Vector3d& in_p, pcl::PointXYZ &out_p)
		{
			E::Vector3d tmp;
			composePoint(in_p, tmp);
			out_p.x = tmp(0);
			out_p.y = tmp(1);
			out_p.z = tmp(2);
		}
		void Pose::composePoint(const pcl::PointXYZ& in_p, pcl::PointXYZ &out_p)
		{
			E::Vector3d in(in_p.x, in_p.y, in_p.z) ,out;
			composePoint(in, out);
			out_p.x = out(0);
			out_p.y = out(1);
			out_p.z = out(2);

		}

		void Pose::composePoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud)
		{
			if ( nullptr == in_cloud)
			{
				ROS_INFO("error Pose::composePoint empty input");
				return;
			}
			
			out_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
			out_cloud->reserve(in_cloud->size());
			E::Vector3d inPoint;
			E::Vector3d outPoint;
			for (size_t ii = 0; ii < in_cloud->points.size(); ++ii)
			{
				inPoint(0) = in_cloud->points[ii].x;
				inPoint(1) = in_cloud->points[ii].y;
				inPoint(2) = in_cloud->points[ii].z;
				composePoint(inPoint, outPoint);
				out_cloud->push_back(pcl::PointXYZ(outPoint(0), outPoint(1), outPoint(2)));
			}

			return;
		}

		void Pose::inverseComposePoint(const E::Vector3d& in_p, E::Vector3d &out_p)
		{
			double xtmp((in_p(0)-t_(0)));
			double ytmp((in_p(1)-t_(1)));
			double ztmp((in_p(2)-t_(2)));
			out_p(0) = xtmp + 2*(-(q_.y()*q_.y()+q_.z()*q_.z())*xtmp + (q_.x()*q_.y()-q_.w()*q_.z())*ytmp + (q_.w()*q_.y()+q_.x()*q_.z())*ztmp);
			out_p(1) = ytmp + 2*((q_.w()*q_.z()+q_.x()*q_.y())*xtmp - (q_.x()*q_.x()+q_.z()*q_.z())*ytmp + (q_.y()*q_.z()-q_.w()*q_.x())*ztmp);
			out_p(2) = ztmp + 2*((q_.x()*q_.z()-q_.w()*q_.y())*xtmp + (q_.w()*q_.x()+q_.y()*q_.z())*ytmp - (q_.x()*q_.x()+q_.y()*q_.y())*ztmp);
			return;
		}


		void Pose::composePose(const Pose& in_pose, Pose &out_p)
		{
			// p1 := this 
			// p2 := in_pose
			composePoint(in_pose.t_, out_p.t_);
			const E::Quaterniond& q1(q_);
			const E::Quaterniond& q2(in_pose.q_);
			out_p.q_.w() = q1.w()*q2.w() - q1.x()*q2.x() - q1.y()*q2.y() - q1.z()*q2.z();
			out_p.q_.x() = q1.w()*q2.x() + q2.w()*q1.x() + q1.y()*q2.z() - q1.z()*q2.y();
			out_p.q_.y() = q1.w()*q2.y() + q2.w()*q1.y() + q1.z()*q2.x() - q1.x()*q2.z();
			out_p.q_.z() = q1.w()*q2.z() + q2.w()*q1.z() + q1.x()*q2.y() - q1.y()*q2.x();
			return;
		}

		void Pose::normalize()
		{
			q_.normalize();
			return;
		}

	
}
