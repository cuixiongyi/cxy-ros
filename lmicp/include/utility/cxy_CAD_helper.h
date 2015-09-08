#pragma once

#include "ros/ros.h"
#include "geometric_shapes/shape_operations.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "geometric_shapes/shapes.h"
#include "shape_msgs/Mesh.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "XmlRpcValue.h"

namespace cxy
{
    // Only RANDOM and GRID are present as of now
    // BUG: GRID is not working as expected, please ue RANDOM
    // TO-DO: Poisson Disc, Jitter and Rotated Grid
    enum class SampleType {RANDOM, GRID, POISSON_DISC, JITTER, ROTATED_GRID};

    class SamplingParams
    {
    public:
        int         number_of_points;
        float       step_size;
        SampleType  sample_type;

        SamplingParams(int number_of_points, float step_size, SampleType sample_type);
        SamplingParams() {}
        ~SamplingParams() {}
    };

    class cxy_CAD_helper
    {
    protected:

        void shapeToPointCloud(shapes::Mesh &shape
                               , sensor_msgs::PointCloud &cloud
                               , pcl::PointCloud<pcl::PointXYZ> &normals
                               , const SamplingParams &params);

        bool stlToShape(const std::string &filename
                        , shapes::Mesh &shape);

        geometry_msgs::Point32  sampleRandomPoint(geometry_msgs::Point v1, geometry_msgs::Point v2,
                geometry_msgs::Point v3);
        void sampleGridPoints(geometry_msgs::Point v1, geometry_msgs::Point v2,
                geometry_msgs::Point v3, std::list<geometry_msgs::Point32> &samples,
                float step_size);


    public:
        cxy_CAD_helper();
        ~cxy_CAD_helper();


        bool meshToPointCloud(const std::string &filename_model,
                pcl::PointCloud<pcl::PointXYZ> &cloud_out,
                pcl::PointCloud<pcl::PointXYZ> &normals,
                const SamplingParams &params);

        void getNormal(
                geometry_msgs::Point const& v1
                , geometry_msgs::Point const& v2
                , geometry_msgs::Point const& v3
                , geometry_msgs::Vector3 & normal);
        double findArea(
                geometry_msgs::Point const& v1
                , geometry_msgs::Point const& v2
                , geometry_msgs::Point const& v3
                , float const& scale);

        bool filterPointsWithInnerNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &in_normals,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &out_normals);
        bool filterOccludedPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &output,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr &normals,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &output_normals,
                Eigen::Vector3d&& to_origin);
        //bool filterOccluded2(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
        //                     pcl::PointCloud<pcl::PointXYZ>::Ptr &output,
        //                     Eigen::Vector3d to_origin);
    };
}