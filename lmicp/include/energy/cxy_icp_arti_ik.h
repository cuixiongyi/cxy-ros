#pragma once

#include "cxy_transform.h"
#include "cxy_icp_kinematic_chain.h"
#include "cxy_icp_arti_ik_func.h"
#include <unsupported/Eigen/NonLinearOptimization>
#include "optimization/cxy_nonlinear_minimizer_LM.h"
#include <memory>
#include "cxy_debug.h"

namespace cxy {
    namespace cxy_lmicp_lib {
        template<typename _Scalar, int _MinimizerType>
        class cxy_icp_arti_ik
        {

        public:
            bool setKinematicChain(std::shared_ptr<cxy_icp_kinematic_chain<_Scalar>> kc);


            bool setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr data);


            // There are 3 layers of minimization class
            // The 1st layer is the cxy_icp, is abstract, should not be changed
            // 2nd layer is cxy_icp_rigid, deal with cost function
            // 3rd layer is cxy_icp_rigid_lm, deal with minimization interface

            //: This function belong to 1st layer
            _Scalar icp_run(Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> &x);


            //: This function belong to 2nd layer, initialize specific cost function
            int icp_prepare_cost_function();

            int icp_manifold();
            //: This function belong to 3rd layer, using optimization interface
            _Scalar icp_minimization(Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> &x);

            // match dataCloud_ and modelCloud_, store the result in member variables
            // should be override in rigid or articulate
            //virtual dataType translatePointCloud() = 0;


            cxy_icp_arti_ik();
            ~cxy_icp_arti_ik();




            //bool setModelCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr model);

        private:
            

        protected:
            
            double transformation_epsilon_, euclidean_fitness_epsilon_, max_correspondence_dist_, max_correspondence_dist_square_;

            bool hasSetKC_, hasSetDataCloud_;
            std::shared_ptr<cxy_lmicp_lib::cxy_icp_arti_ik_func<_Scalar>>  func_;


            pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud_;
            pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeptr_;
            std::shared_ptr<cxy_icp_kinematic_chain<_Scalar>> kc_;




        protected:


        };
    }
}
