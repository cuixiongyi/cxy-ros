#include "cxy_icp_rigid.h"

namespace cxy {
    namespace cxy_lmicp_lib {


            cxy_icp_rigid() : cxy_icp()
            {
                func_ = new cxy_icp_func_rigid;


            }


            float cxy_icp_rigid::matchPointCloud()
            {
                float r(0.0);
                dataMatchIdx_.clear();
                dataMatchIdx_.reserve(data->size());
                modelMatchIdx_.clear();
                modelMatchIdx_.reserve(data->size());
                matchDistance_.clear();
                matchDistance_.reserve(data->size());

                static const int K(1);
                std::vector<int> pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);
                /// kdtreeptr_ is updated by setModelCloud
                if (nullptr == kdtreeptr_)
                    ROS_INFO("nullptr kdtree");

                for (int ii = 0; ii < dataCloud_->size(); ++ii)
                {
                    if ( 0 == kdtreeptr_->nearestKSearch ((*dataCloud_)[ii], K, pointIdxNKNSearch, pointNKNSquaredDistance)  )
                    {
                        continue;
                    }
                    if (pointNKNSquaredDistance[0] > max_correspondence_dist_square_)
                    {
                        continue;
                    }
                    dataMatchIdx_.push_back(ii);
                    modelMatchIdx_.push_back(pointIdxNKNSearch[0]);
                    /// Use Euclidean distance
                    const PointT& pTmp((*modelCloud_)[pointIdxNKNSearch[0]]), p2Tmp((*data)[ii]);
                    //int signTmp = 1;//((pTmp.x - p2Tmp.x)+(pTmp.y - p2Tmp.y)+(pTmp.z - p2Tmp.z)) > 0 ? 1 : -1;
                    float rtmp(sqrt(pointNKNSquaredDistance[0]));
                    matchDistance.push_back(pointNKNSquaredDistance[0]);
                    //ROS_INFO_STREAM(rtmp);
                    r += rtmp;

                }

                return r;
            }

            const float cxy_icp_rigid::matchPointCloud(const PointT& data
                                                        , Eigen::Vector3f& res)
            {
                static const int K(1);
                std::vector<int> pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);
                /// kdtreeptr_ is updated by setModelCloud
                if (nullptr == kdtreeptr_)
                    ROS_INFO("nullptr kdtree");

                if ( 0 == kdtreeptr_->nearestKSearch (data, K, pointIdxNKNSearch, pointNKNSquaredDistance)  )
                {
                    return std::nanf("");
                }
                if (pointNKNSquaredDistance[0] > max_correspondence_dist_square_)
                {
                    return std::nanf("");
                }
                /// Use Euclidean distance
                const PointT& pTmp((*modelCloud_)[pointIdxNKNSearch[0]]);
                ROS_INFO_STREAM("closet point = "<<pTmp.x<<" "<<pTmp.y<<" "<<pTmp.z);
                ROS_INFO_STREAM(" ");
                res(0) = pTmp.x - data.x;
                res(1) = pTmp.y - data.y;
                res(2) = pTmp.z - data.z;
                //int signTmp = 1;//(res(0)+res(1)+res(2)) > 0 ? 1 : -1;
                //ROS_INFO_STREAM(rtmp);

                return sqrt(pointNKNSquaredDistance[0]);
            }
    }
}