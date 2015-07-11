#include "cxy_transform.h"

namespace cxy
{
namespace cxy_transform
{


    template <typename _Scalar>
	Pose<_Scalar>:: Pose() : t_(0.0, 0.0, 0.0), q_(1.0, 0.0, 0.0, 0.0), bhasNormalized_(false) {}

	// constructor with Identity asigned 
	// a doesn't matter
    template <typename _Scalar>
	Pose<_Scalar>:: Pose(int a) : t_(0.0, 0.0, 0.0), q_(1.0, 0.0, 0.0, 0.0), bhasNormalized_(false) {}

    template <typename _Scalar>
	Pose<_Scalar>:: ~Pose() {};

	//: The inpute takes the rotating axis and the angle in radian (!!!!! no degree)
	//
    template <typename _Scalar>
	void Pose<_Scalar>:: rotateByAxis(Axis axis, _Scalar & degree)
	{
		int n(0);
		_Scalar theta(Rad2Deg(degree));
		
		if (degree > _Scalar(180))
		{
			n = (degree-180) / _Scalar(360);
			theta = degree - (n+1)*_Scalar(360);
		}
		else if (degree < _Scalar(-180))
		{
			n = std::abs( (degree+180) / _Scalar(360));
			theta = degree + (n+1)*_Scalar(360);
		}
		
        //degree = Deg2Rad(theta);
		//std::cout<<theta<<std::endl;
		_Scalar radian = Deg2Rad(theta);
		_Scalar x = std::sin( radian / _Scalar(2.0) );
		_Scalar w = std::cos( radian / _Scalar(2.0) );
		Quaternoin q1(q_);
		Quaternoin q2(w, 0, 0, 0);
		switch (axis)
		{
			case X_axis_rotation : q2.x() = x; break;
			case Y_axis_rotation : q2.y() = x; break;
			case Z_axis_rotation : q2.z() = x; break;
		}
		q1.normalize();
		q2.normalize();
		q_.w() = q1.w()*q2.w() - q1.x()*q2.x() - q1.y()*q2.y() - q1.z()*q2.z();
		q_.x() = q1.w()*q2.x() + q2.w()*q1.x() + q1.y()*q2.z() - q1.z()*q2.y();
		q_.y() = q1.w()*q2.y() + q2.w()*q1.y() + q1.z()*q2.x() - q1.x()*q2.z();
		q_.z() = q1.w()*q2.z() + q2.w()*q1.z() + q1.x()*q2.y() - q1.y()*q2.x();

	}
	//: The inpute takes the rotating axis and the angle in radian (!!!!! no degree)
    template <typename _Scalar>
    Pose<_Scalar> Pose<_Scalar>::rotateByAxis_fromIdentity(const Axis & axis, _Scalar & degree, const Pose& p_org)
    {

        int n(0);
		_Scalar theta(Rad2Deg(degree));
        if (degree > _Scalar(180))
        {
            n = (degree-180) / _Scalar(360);
            theta = degree - (n+1)*_Scalar(360);
        }
        else if (degree < _Scalar(-180))
        {
            n = std::abs( (degree+180) / _Scalar(360));
            theta = degree + (n+1)*_Scalar(360);
        }
        //degree = Deg2Rad(theta);
        //std::cout<<theta<<std::endl;
        _Scalar radian = Deg2Rad(theta);
        _Scalar x = std::sin( radian / _Scalar(2.0) );
        _Scalar w = std::cos( radian / _Scalar(2.0) );
        Quaternoin q1(p_org.q());
        Quaternoin q2(w, 0, 0, 0);
        Pose pose(p_org);
		switch (axis)
		{
			case X_axis_rotation : q2.x() = x; break;
			case Y_axis_rotation : q2.y() = x; break;
			case Z_axis_rotation : q2.z() = x; break;
		}
		q1.normalize();
		q2.normalize();
		//Eigen::Matrix<_Scalar, 3, 1>  t_tmp(0.0,0.0,0.0);
		//p_org.composePoint(t_tmp, pose.t_);

		pose.q().w() = q1.w()*q2.w() - q1.x()*q2.x() - q1.y()*q2.y() - q1.z()*q2.z();
		pose.q().x() = q1.w()*q2.x() + q2.w()*q1.x() + q1.y()*q2.z() - q1.z()*q2.y();
		pose.q().y() = q1.w()*q2.y() + q2.w()*q1.y() + q1.z()*q2.x() - q1.x()*q2.z();
		pose.q().z() = q1.w()*q2.z() + q2.w()*q1.z() + q1.x()*q2.y() - q1.y()*q2.x();
		return pose;

    }

    template <typename _Scalar>
	void Pose<_Scalar>:: composePoint(const pcl::PointXYZ& in_p, pcl::PointXYZ& out_p, std::vector<_Scalar> & para)
	{
		Vector in(in_p.x, in_p.y, in_p.z) ,out;
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
    template <typename _Scalar>
	void Pose<_Scalar>:: composePoint(const Vector& in_p, Vector &out_p) const
	{
		if ( ! bhasNormalized_)
		{
			normalize();
		}
		out_p(0) = in_p(0)+t_(0) + 2*(-(q_.y()*q_.y()+q_.z()*q_.z())*in_p(0) + (q_.x()*q_.y()-q_.w()*q_.z())*in_p(1) + (q_.w()*q_.y()+q_.x()*q_.z())*in_p(2));
		out_p(1) = in_p(1)+t_(1) + 2*((q_.w()*q_.z()+q_.x()*q_.y())*in_p(0) - (q_.x()*q_.x()+q_.z()*q_.z())*in_p(1) + (q_.y()*q_.z()-q_.w()*q_.x())*in_p(2));
		out_p(2) = in_p(2)+t_(2) + 2*((q_.x()*q_.z()-q_.w()*q_.y())*in_p(0) + (q_.w()*q_.x()+q_.y()*q_.z())*in_p(1) - (q_.x()*q_.x()+q_.y()*q_.y())*in_p(2));
		
	}

    template <typename _Scalar>
	void Pose<_Scalar>:: composePoint(const Vector& in_p, pcl::PointXYZ &out_p) const
	{
		Vector tmp;
		composePoint(in_p, tmp);
		out_p.x = tmp(0);
		out_p.y = tmp(1);
		out_p.z = tmp(2);
	}

    template <typename _Scalar>
	void Pose<_Scalar>:: composePoint(const pcl::PointXYZ& in_p, pcl::PointXYZ &out_p) const
	{
		Vector in(in_p.x, in_p.y, in_p.z) ,out;
		composePoint(in, out);
		out_p.x = out(0);
		out_p.y = out(1);
		out_p.z = out(2);
	}

    template <typename _Scalar>
	void Pose<_Scalar>:: composePoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud) const
	{
		if ( nullptr == in_cloud)
		{
			ROS_INFO("error Pose::composePoint empty input");
			return;
		}
		
		out_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		out_cloud->reserve(in_cloud->size());
		Vector inPoint;
		Vector outPoint;
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

    template <typename _Scalar>
	void Pose<_Scalar>:: composeDirectionVector(const Vector& in_p, Vector &out_p) const
			{
				if ( ! bhasNormalized_)
				{
					normalize();
				}
				out_p(0) = in_p(0) + 2*(-(q_.y()*q_.y()+q_.z()*q_.z())*in_p(0) + (q_.x()*q_.y()-q_.w()*q_.z())*in_p(1) + (q_.w()*q_.y()+q_.x()*q_.z())*in_p(2));
				out_p(1) = in_p(1) + 2*((q_.w()*q_.z()+q_.x()*q_.y())*in_p(0) - (q_.x()*q_.x()+q_.z()*q_.z())*in_p(1) + (q_.y()*q_.z()-q_.w()*q_.x())*in_p(2));
				out_p(2) = in_p(2) + 2*((q_.x()*q_.z()-q_.w()*q_.y())*in_p(0) + (q_.w()*q_.x()+q_.y()*q_.z())*in_p(1) - (q_.x()*q_.x()+q_.y()*q_.y())*in_p(2));
				
			}
    template <typename _Scalar>
	void Pose<_Scalar>:: inverseComposePoint(const Vector& in_p, Vector &out_p) const
	{
		if ( ! bhasNormalized_)
		{
			normalize();

		}
		double xtmp((in_p(0)-t_(0)));
		double ytmp((in_p(1)-t_(1)));
		double ztmp((in_p(2)-t_(2)));
		out_p(0) = xtmp + 2*(-(q_.y()*q_.y()+q_.z()*q_.z())*xtmp + (q_.x()*q_.y()-q_.w()*q_.z())*ytmp + (q_.w()*q_.y()+q_.x()*q_.z())*ztmp);
		out_p(1) = ytmp + 2*((q_.w()*q_.z()+q_.x()*q_.y())*xtmp - (q_.x()*q_.x()+q_.z()*q_.z())*ytmp + (q_.y()*q_.z()-q_.w()*q_.x())*ztmp);
		out_p(2) = ztmp + 2*((q_.x()*q_.z()-q_.w()*q_.y())*xtmp + (q_.w()*q_.x()+q_.y()*q_.z())*ytmp - (q_.x()*q_.x()+q_.y()*q_.y())*ztmp);
		return ;
	}

	// p = p1*p2 == p1.composePose(p2, p)
    template <typename _Scalar>
	void Pose<_Scalar>:: composePose(const Pose& in_pose, Pose &out_p) const
	{
		if ( ! in_pose.bhasNormalized_)
		{
			in_pose.normalize();
		}
		if ( ! bhasNormalized_)
		{
			normalize();
		}
		composePoint(in_pose.t_, out_p.t_);
		const Quaternoin& q1(q_);
		const Quaternoin& q2(in_pose.q_);
		out_p.q_.w() = q1.w()*q2.w() - q1.x()*q2.x() - q1.y()*q2.y() - q1.z()*q2.z();
		out_p.q_.x() = q1.w()*q2.x() + q2.w()*q1.x() + q1.y()*q2.z() - q1.z()*q2.y();
		out_p.q_.y() = q1.w()*q2.y() + q2.w()*q1.y() + q1.z()*q2.x() - q1.x()*q2.z();
		out_p.q_.z() = q1.w()*q2.z() + q2.w()*q1.z() + q1.x()*q2.y() - q1.y()*q2.x();
		out_p.bhasNormalized_ = false;
		return;
	}

}
}
template class cxy::cxy_transform::Pose<float>;
template class cxy::cxy_transform::Pose<double>;
