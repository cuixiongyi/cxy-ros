#include "cxy_icp_arti_func.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {


    template<typename _Scalar>
    cxy_icp_arti_func<_Scalar>::cxy_icp_arti_func(int nPara
                    , std::shared_ptr<cxy_icp_kinematic_chain<_Scalar>> kc
                    , pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud
                    , pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeptr
                    , int joint
                    , Eigen::Matrix<_Scalar, Eigen::Dynamic, 1> x_full
                    )
                    : cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar, Eigen::Dynamic, Eigen::Dynamic>(nPara, kc->getModelCloud(joint)->size())
    {
        dataCloud_ = dataCloud;
        kdtreeptr_ = kdtreeptr;
        kc_ = kc;
        joint_ = joint;
        x_full_ = x_full;
        ROS_INFO_STREAM("fcun_init size =  "<<kc->getModelCloud(joint)->size());
    }

    template<typename _Scalar>
    _Scalar cxy_icp_arti_func<_Scalar>::operator()(ParaType & x, ResidualType& fvec) const
    {
        /// test manifold start
        if (0)
        {
            this->manifold();
        }
        /// test manifold end
        _Scalar res(0.0);
        int counter = 0;
        x_full_(joint_) = x(0);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud;
        cxy_transform::Pose<_Scalar> pose;
        transCloud = kc_->getOneModelCloud_World(x_full_, joint_, pose);
        fvec.resize(transCloud->size(), 1);
        for (unsigned int jj = 0; jj < transCloud->size(); ++jj)
        {
            //pose.composePoint((*transCloud)[ii], transPoint);
            Eigen::Matrix< _Scalar, 3, 1> r3;
            fvec[jj] = matchPointCloud((*transCloud)[jj], r3);
            res += fvec[jj];
            ++counter;

        }
        
        res = res / counter;
        static int ac = 0;
        //ROS_INFO_STREAM("Call f the 1   "<<++ac);


        
        ROS_INFO_STREAM("Call f the 3    "<<ac++<<" time. Residual =  "<< res);


        //ROS_INFO_STREAM("theta =  "<<x(0));
        if (0)
        {
            static std::ofstream fout("/home/xiongyi/repo/gradiant.txt");
            fout<<pose.q().w()<<" "<<pose.q().x()<<" "<<res<<std::endl;
        }
        return res;
    }

    template<typename _Scalar>
    _Scalar cxy_icp_arti_func<_Scalar>::df(ParaType & x, JacobianType& fjac) const
    {
        _Scalar res(0.0);
        int counter = 0;
        x_full_(joint_) = x(0);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud;
        cxy_transform::Pose<_Scalar> pose;
        cxy_transform::Pose<_Scalar> para_pose_parent;
        transCloud = kc_->getOneModelCloud_World(x_full_, joint_, pose, para_pose_parent);
        fjac.resize(transCloud->size(), 1);
        for (unsigned int jj = 0; jj < transCloud->size(); ++jj)
        {
            //pose.composePoint((*transCloud)[ii], transPoint);
            Eigen::Matrix< _Scalar, 3, 1> r3;
            matchPointCloud((*transCloud)[jj], r3);
            
            //Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> r3;
            //ROS_INFO_STREAM("r3 = "<<r3);
            Matrix jac31(calculateJacobianKernel(x
                                                , (*transCloud)[jj]
                                                , pose
                                                , para_pose_parent)); //(*dataCloud_)[ii]));
            if (jj == 700)
            {
                //std::cout<<ii<<" = "<<(*dataCloud_)[ii].x<<"  "<<(*dataCloud_)[ii].y<<"  "<<(*dataCloud_)[ii].z<<std::endl;
                //std::cout<<ii<<" = "<<jac34(0)<<"  "<<jac34(1)<<"  "<<jac34(2)<<std::endl;
                //std::cout<<ii<<" = "<<vPara[0]<<"  "<<vPara[1]<<"  "<<vPara[2]<<"  "<<vPara[3]<<"  "<<vPara[4]<<"  "<<vPara[5]<<"  "<<vPara[6]<<std::endl;
            }
            Matrix header(2,1);
            header<<r3(1), r3(2);

            //ROS_INFO_STREAM("jac34 = "<<jac34);
            //ROS_INFO_STREAM("para_pose_parent =  "<<para_pose_parent.q().w()<<" "<<para_pose_parent.q().x());

            Matrix jq(-r3.transpose()*jac31);
            jq *= 2;
            //ROS_INFO_STREAM("jacobian = "<<jq);
            //ROS_INFO(" ");

            //rowJ<<r3(0), r3(1), r3(2),  jq(0), jq(1), jq(2), jq(3);
            /*fjac(ii, 0) = r3(0);
            fjac(ii, 1) = r3(1);
            fjac(ii, 2) = r3(2);
            fjac(ii, 3) = jq(0);
            fjac(ii, 4) = jq(1);
            fjac(ii, 5) = jq(2);
            fjac(ii, 6) = jq(3);*/
            fjac(jj, 0) = jq(0);
            //fjac(ii, 1) = jq(1);


            //if (jj == 20)
            //    std::cout<<" cols = "<<jq.cols()<<" rows = "<<jq.rows()<<"  j = "<<jq<<std::endl;
            //std::cout<<"dev = "<<jq(0, 0)<<"  "<<jq(0,1)<<"  "<<jq(0,2)<<"  "<<jq(0,3)<<std::endl;

        }
        //x(0) = pose.q().w();
        //x(1) = pose.q().x();
        //std::cout<<"wx = "<<x(0)<<"  "<<x(1)<<"  "<<std::endl;
        return 1;
    }


    template<typename _Scalar>
    const _Scalar cxy_icp_arti_func<_Scalar>::matchPointCloud(const PointT& model
                                                            , Eigen::Matrix< _Scalar, 3, 1>& res) const
    {
        static const int K(1);
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        /// kdtreeptr_ is updated by setModelCloud
        if (nullptr == kdtreeptr_)
            ROS_INFO("nullptr kdtree");

        if ( 0 == kdtreeptr_->nearestKSearch (model, K, pointIdxNKNSearch, pointNKNSquaredDistance)  )
        {
            return std::nanf("");
        }

        /*if (pointNKNSquaredDistance[0] > max_correspondence_dist_square_)
        {
            return std::nanf("");
        }
        */
        /// Use Euclidean distance
        const pcl::PointXYZ& data((*dataCloud_)[pointIdxNKNSearch[0]]);
        //ROS_INFO_STREAM("closet point = "<<pTmp.x<<" "<<pTmp.y<<" "<<pTmp.z);
        //ROS_INFO_STREAM(" ");
        res(0) = data.x - model.x;
        res(1) = data.y - model.y;
        res(2) = data.z - model.z;
        //int signTmp = 1;//(res(0)+res(1)+res(2)) > 0 ? 1 : -1;
        //ROS_INFO_STREAM(rtmp);

        return std::sqrt(pointNKNSquaredDistance[0]);
    }


    template<typename _Scalar>

    const Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> cxy_icp_arti_func<_Scalar>::calculateJacobianKernel(
                                                const Eigen::Matrix< _Scalar, Eigen::Dynamic, 1>& x
                                            , const pcl::PointXYZ& a
                                            , const cxy_transform::Pose<_Scalar> &para_pose
                                            , const cxy_transform::Pose<_Scalar> &para_pose_parent
                                            ) const
    {
        

        cxy_transform::Pose<_Scalar> poseTmp = cxy_transform::Pose<_Scalar>::rotateByAxis_fromIdentity((*kc_->getKinematicChainNodes())[joint_].rotateAxis_, x_full_(joint_), cxy_transform::Pose<_Scalar>());
        const Eigen::Quaternion<_Scalar>& q_theta(poseTmp.q());
        // compute jacobian including kinematic chain
        const Eigen::Quaternion<_Scalar>& q_w_norm(para_pose.q());
        // quaternion transfer 3D points term
        Matrix jac_dqwX_dqw(3,4);
        // quaternion normalization term
        Matrix jac_dqw_norm_dqw_unnorm(4,4);
        /// quaternion composePose term
        Matrix jac_dqw_unnorm_dqtheta(4,2);
        /// quaternion theta 
        Matrix JacTheata(2,1);

        {
            const Eigen::Quaternion<_Scalar>& q(q_w_norm);
            jac_dqwX_dqw << (a.z*q.y() - a.y*q.z()) , (a.y*q.y() + a.z*q.z())                ,(a.z*q.w() - 2*a.x*q.y() + a.y*q.x()) , (a.z*q.x() - 2*a.x*q.z() - a.y*q.w())
                        , (a.x*q.z() - a.z*q.x()) , (a.x*q.y() - a.z*q.w() - 2*a.y*q.x()) , (a.x*q.x() + a.z*q.z())                 ,( a.x*q.w() - 2*a.y*q.z() + a.z*q.y())
                        , (a.y*q.x() - a.x*q.y()) , (a.y*q.w() + a.x*q.z() - 2*a.z*q.x()) , (a.y*q.z() - a.x*q.w() - 2*a.z*q.y()) , (a.x*q.x() + a.y*q.y());

        }
        /// Normalization term
        {
            cxy_transform::Pose<_Scalar> pose_out;
            para_pose_parent.composePose(poseTmp, pose_out);
            // unnormalized Quaternion
            const Eigen::Quaternion<_Scalar>& q(pose_out.q());

            jac_dqw_norm_dqw_unnorm << q.x()*q.x()+q.y()*q.y()+q.z()*q.z(), -q.w()*q.x(), -q.w()*q.y(), -q.w()*q.z(),
                    -q.x()*q.w(), q.w()*q.w()+q.y()*q.y()+q.z()*q.z(), -q.x()*q.y(), -q.x()*q.z(),
                    -q.y()*q.w(), -q.y()*q.x(), q.w()*q.w()+q.x()*q.x()+q.z()*q.z(), -q.y()*q.z(),
                    -q.z()*q.w(), -q.z()*q.x(), -q.z()*q.y(), q.w()*q.w()+q.y()*q.y()+q.x()*q.x();
            jac_dqw_norm_dqw_unnorm = jac_dqw_norm_dqw_unnorm / std::pow(q.w()*q.w()+q.y()*q.y()+q.x()*q.x()+q.z()*q.z(), 1.5);
        }
        /// quaternion composePose term
        {
            const Eigen::Quaternion<_Scalar>& q(para_pose_parent.q());
            jac_dqw_unnorm_dqtheta<<q.w(), -q.x()
                                  , q.x(), q.w()
                                  , q.y(), q.z()
                                  , q.z(), -q.y();

        }
        {
            JacTheata<< -std::sin(Deg2Rad(x(0)/2)), std::cos(Deg2Rad(x(0)/2));

        }
        return jac_dqwX_dqw*jac_dqw_norm_dqw_unnorm*jac_dqw_unnorm_dqtheta*JacTheata;
        //return jac_dqwX_dqw*jac_dqw_norm_dqw_unnorm*jac_dqw_unnorm_dqtheta*JacTheata;
    }

    template<typename _Scalar>
    void cxy_icp_arti_func<_Scalar>::manifold() const
    {

        std::ofstream fout("/home/xiongyi/repo/manifold.txt");
        //cxy_transform::Pose<_Scalar> pose;
        const int delta = 8.0;
        int counter1(0);
        while (1)
        {
            /*
            cxy_transform::Pose<_Scalar> pose;

            counter1 += delta;
            pose.rotateByAxis(cxy_transform::Axis::X_axis_rotation, counter1);
            std::vector<_Scalar> vPara(7);
            _Scalar res(0.0);
            _Scalar jac(0.0);
            vPara[0] = 0.0;
            vPara[1] = .0;
            vPara[2] = .0;
            vPara[3] = pose.q().w();
            vPara[4] = pose.q().x();
            vPara[5] = .0;
            vPara[6] = .0;
            pose.normalize();

            for (unsigned int ii = 0; ii < dataCloud_->size(); ++ii)
            {
                pcl::PointXYZ transPoint;
                pose.composePoint((*dataCloud_)[ii], transPoint);
                Eigen::Matrix< _Scalar, 3, 1> r3;
                res += matchPointCloud(transPoint, r3);
                Matrix jac34(calculateJacobianKernel(vPara
                        , (*dataCloud_)[ii]));//transPoint)); //(*dataCloud_)[ii]));

                Matrix header(1,2);
                header<<r3(1), r3(2);

                //ROS_INFO_STREAM("jac34 = "<<jac34);
                //ROS_INFO(" ");

                Matrix jq(-header*jac34);
                jac += jq(0);

            }
            res = res / this->values();
            jac = jac / this->values();
            std::cout<<counter1<<" "<<pose.q().w()<<" "<<pose.q().x()<<std::endl;
            fout<<counter1<<"  "<<pose.q().w()<<" "<<pose.q().x()<<" "<<res<<" "<<jac<<std::endl;
            if (counter1 >= 361)
                std::exit(1);
            */
        }
        return ;

    }

}

}
template class cxy::cxy_lmicp_lib::cxy_icp_arti_func<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_arti_func<double>;