#include "utility/cxy_modelCloud_engin.h"



namespace cxy
{

    cxy_modelCloud_engin::cxy_modelCloud_engin(const std::string& filename )
    : modelCloud_(new pcl::PointCloud<PointT>)
     , modelCloud_normal_(new pcl::PointCloud<PointT>)
    {
        SamplingParams para(3000, 0.005, SampleType::RANDOM);
        if (! cad_helper_.meshToPointCloud(filename, *modelCloud_, *modelCloud_normal_, para))
            throw std::runtime_error("CAD model import failed");


    }

    cxy_modelCloud_engin::~cxy_modelCloud_engin()
    {

    }

    const pcl::PointCloud<pcl::PointXYZ>::Ptr&
                        cxy_modelCloud_engin::getModelCloud()
    {
        return modelCloud_;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    cxy_modelCloud_engin::getModelCloud(cxy_transform::Pose<float> const& pose)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr retCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pose.composePoint(modelCloud_, retCloud);
        return retCloud;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    cxy_modelCloud_engin::getModelCloud(cxy_transform::Pose<double> const& pose)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr retCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pose.composePoint(modelCloud_, retCloud);
        return retCloud;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr
    cxy_modelCloud_engin::getVisibleCloud( cxy_transform::Pose<float> const& pose)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr retCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr retCloud_normal(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud_normal(new pcl::PointCloud<pcl::PointXYZ>);

        //pose.composePoint(modelCloud_, transCloud);
        //pose.composeDirectionVector(modelCloud_normal_, transCloud_normal);

        //cad_helper_.filterOccludedPoints(transCloud, retCloud, transCloud_normal, retCloud_normal, Eigen::Vector3d(1.0, 0.0, 0.0));

        // TODO this is a hack
        // get visible point isn't working
        // just return fullPointCloud
        retCloud = getModelCloud(pose);
        return retCloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    cxy_modelCloud_engin::getVisibleCloud(const cxy_transform::Pose<double>& pose)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr retCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr retCloud_normal(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud_normal(new pcl::PointCloud<pcl::PointXYZ>);
        pose.composePoint(modelCloud_, transCloud);
        pose.composeDirectionVector(modelCloud_normal_, transCloud_normal);

        cad_helper_.filterOccludedPoints(transCloud, retCloud, transCloud_normal, retCloud_normal, Eigen::Vector3d(1.0, 0.0, 0.0));

        return retCloud;
    }









}