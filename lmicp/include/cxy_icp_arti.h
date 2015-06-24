#pragma once

#include "cxy_transform.h"
#include "cxy_icp_arti_func.h"
#include <unsupported/Eigen/NonLinearOptimization>
#include "optimization/cxy_nonlinear_minimizer_LM.h"
#include "cxy_debug.h"

namespace cxy {
    namespace cxy_lmicp_lib {
        template<typename _Scalar, int _MinimizerType>
        class cxy_icp_arti
        {

            bool setModelCloud(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> model);


            inline bool setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr data);


            // There are 3 layers of minimization class
            // The 1st layer is the cxy_icp, is abstract, should not be changed
            // 2nd layer is cxy_icp_rigid, deal with cost function
            // 3rd layer is cxy_icp_rigid_lm, deal with minimization interface

            //: This function belong to 1st layer
            _Scalar icp_run(Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> &x);


            //: This function belong to 2nd layer, initialize specific cost function
            int icp_prepare_cost_function();

            //: This function belong to 3rd layer, using optimization interface
            _Scalar icp_minimization(Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> &x);

            // match dataCloud_ and modelCloud_, store the result in member variables
            // should be override in rigid or articulate
            //virtual dataType translatePointCloud() = 0;



            //bool setModelCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr model);

        private:
            

        protected:
            
            double transformation_epsilon_, euclidean_fitness_epsilon_, max_correspondence_dist_, max_correspondence_dist_square_;

            bool hasSetModelCloud_, hasSetDataCloud_;
            cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar>*  func_;


            pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud_;
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> modelCloud_;
            pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeptr_;
            std::vector<int> kinematicChain_JointRoot_;
            // store matchPointCloud result
            


        public:
            cxy_icp_arti();
            int icp_prepare_cost_function();

            _Scalar icp_minimization(Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> &x);


        protected:


        };
    }
}
