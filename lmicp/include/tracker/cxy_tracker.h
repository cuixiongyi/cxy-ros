#pragma once
#include <unsupported/Eigen/NonLinearOptimization>

#include "optimization/cxy_cost_func_kinematic.h"

#include "common/cxy_config.h"
#include "kinematic/cxy_icp_kinematic.h"

//#include "kinematic/"

namespace cxy
{
    template<typename _Scalar>
    class cxy_tracker
    {

        typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> MatrixX1;
        typedef Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixXX;
    public:
        cxy_tracker(const cxy_config *const );
        ~cxy_tracker();

        _Scalar runOptimization();

        mutable cxy_kinematic::cxy_icp_kinematic<_Scalar> kinematic_;

        inline MatrixX1 const& getX() const {return x_;}
        inline void setX(MatrixX1 const& x) {x_ = x;}


        inline pcl::PointCloud<pcl::PointXYZ>::Ptr const& getDataCloud() const
        {
            return dataCloud_;
        }

        inline void setDataCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr const& dataCloud)
        {
            cxy_tracker::dataCloud_ = dataCloud;
            kinematic_.setDataCloud(dataCloud);
        }

        inline pcl::PointCloud<pcl::PointXYZ>::Ptr const& getVisibleModelCloud()
        {
            visibleModelCloud_ = kinematic_.getVisibleModelCloud(x_);
            return visibleModelCloud_;
        }

        inline pcl::PointCloud<pcl::PointXYZ>::Ptr const& getVisibleModelCloud
                (pcl::PointCloud<pcl::PointXYZ>::Ptr & fullCloud )
        {
            visibleModelCloud_ = kinematic_.getVisibleModelCloud(x_, fullCloud);

            return visibleModelCloud_;
        }
    private:
        const cxy_config* const config_;

        MatrixX1 x_;
        pcl::PointCloud<PointT>::Ptr dataCloud_;
        pcl::PointCloud<PointT>::Ptr visibleModelCloud_;

    };

}