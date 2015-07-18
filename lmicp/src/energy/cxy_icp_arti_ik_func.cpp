#include "energy/cxy_icp_arti_ik_func.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {


    template<typename _Scalar>
    cxy_icp_arti_ik_func<_Scalar>::cxy_icp_arti_ik_func(int nPara
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
        //ROS_INFO_STREAM("fcun_init size =  "<<kc->getModelCloud(joint)->size());
    }

    template<typename _Scalar>
    _Scalar cxy_icp_arti_ik_func<_Scalar>::operator()(ParaType & x, ResidualType& fvec) const
    {
        
        _Scalar res(0.0);
        x_full_(joint_) = x(0);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud;
        cxy_transform::Pose<_Scalar> pose;
        transCloud = kc_->getOneModelCloud_World(x_full_, joint_, pose);
        fvec.resize(transCloud->size(), 1);
        for (unsigned int jj = 0; jj < transCloud->size(); ++jj)
        {
            pcl::PointXYZ transPoint;
            //pose.composePoint((*transCloud)[ii], transPoint);
            Eigen::Matrix< _Scalar, 3, 1> r3;
            fvec[jj] = matchPointCloud((*transCloud)[jj], r3);
            res += fvec[jj];

        }
        
        res = res / transCloud->size();
        static int ac = 0;
        //ROS_INFO_STREAM("Call f the 1   "<<++ac);


        
        //ROS_INFO_STREAM("Call f the  "<<ac++<<"  time. Residual =  "<< res);


        ROS_INFO_STREAM("theta =  "<<Rad2Deg(x(0))<< "  res = "<<res);
        if (0)
        {
            static std::ofstream fout("/home/xiongyi/repo/gradiant.txt");
            fout<<pose.q().w()<<" "<<pose.q().x()<<" "<<res<<std::endl;
        }
        return res;
    }

    template<typename _Scalar>
    _Scalar cxy_icp_arti_ik_func<_Scalar>::df(ParaType & x, JacobianType& fjac) const
    {
        _Scalar res(0.0);
        int counter = 0;
        x_full_(joint_) = x(0);
        //std::cout<<" x = "<<x(0)<<std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud;
        cxy_transform::Pose<_Scalar> pose;
        cxy_transform::Pose<_Scalar> para_pose_parent;
        transCloud = kc_->getOneModelCloud_World(x_full_, joint_, pose, para_pose_parent);
        //ROS_INFO_STREAM(pose.q().w()<<" "<<pose.q().x()<<" "<<pose.q().y()<<" "<<pose.q().z());
        //ROS_INFO_STREAM(para_pose_parent.q().w()<<" "<<para_pose_parent.q().x()<<" "<<para_pose_parent.q().y()<<" "<<para_pose_parent.q().z());
        fjac.resize(transCloud->size(), 1);
        //std::cout<<"df()"<<std::endl;
        Eigen::Matrix< _Scalar, 3, 1> rotation_axis_in(1.0, 0.0, 0.0);
        Eigen::Matrix< _Scalar, 3, 1> rotation_axis(1.0, 0.0, 0.0);
        pose.composeDirectionVector(rotation_axis_in, rotation_axis);
        //ROS_INFO_STREAM("rotation_axis = "<<rotation_axis);
        float jacS = 0.0;
        for (unsigned int jj = 0; jj < transCloud->size(); ++jj)
        {
            pcl::PointXYZ transPoint;
            //pose.composePoint((*transCloud)[ii], transPoint);
            Eigen::Matrix< _Scalar, 3, 1> r3;
            //std::cout<<matchPointCloud((*transCloud)[jj], r3)<<std::endl;
            matchPointCloud((*transCloud)[jj], r3);
            
            //Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> r3;
            //ROS_INFO_STREAM("r3 = "<<r3);
            /*
            Matrix jac31(calculateJacobianKernel(x
                                                , (*transCloud)[jj]
                                                , pose
                                                , para_pose_parent)); //(*dataCloud_)[ii]));
           
            */
            
            Eigen::Matrix< _Scalar, 3, 1> jac;
            Eigen::Matrix< _Scalar, 3, 1> tmp((*transCloud)[jj].x - pose.t()(0), (*transCloud)[jj].y - pose.t()(1), (*transCloud)[jj].z - pose.t()(2));
            //ROS_INFO_STREAM("jac34 = "<<jac34);
            //ROS_INFO(" ");

            //Matrix jq(-jac34.transpose()*header);
            //jq *= 2;
            //fjac(jj, 0) = _Scalar(r3.transpose()*cross);
            
            //r3 = -r3;
            /*
            for (int ii = 0; ii < 3; ++ii)
            {
                if (r3(ii) < 0)
                    r3(ii) = - r3(ii);
            }
            */
            
            Eigen::Matrix< _Scalar, 3, 1> cross = rotation_axis.cross(tmp);
            Eigen::Matrix< _Scalar, 3, 1> cross_norm = cross / (std::sqrt(cross(0)*cross(0)+cross(1)*cross(1)+cross(2)*cross(2)));
            const float jac_step_scale = 0.3;
            const float r3_length = std::sqrt(r3(0)*r3(0)+r3(1)*r3(1)+r3(2)*r3(2));
            float scale0 = r3_length * jac_step_scale * cross_norm(0);
            float scale1 = r3_length * jac_step_scale * cross_norm(1);
            float scale2 = r3_length * jac_step_scale * cross_norm(2);
            if (std::isnan(scale0))
                scale0 = 0.0;
            if (std::isnan(scale1))
                scale1 = 0.0;
            if (std::isnan(scale2))
                scale2 = 0.0;
            if (std::abs(r3(0)+scale0) + std::abs(r3(1)+scale1) + std::abs(r3(2)+scale2) > std::abs(r3(0)) + std::abs(r3(1)) + std::abs(r3(2)))
            {
                fjac(jj, 0) =  -std::abs(cross(2) + cross(1) + cross(0));
                if (jj == 20)
                   ROS_INFO_STREAM("reversed");
            }
            else
            {
                fjac(jj, 0) =  std::abs(cross(2) + cross(1) + cross(0));
                //fjac(jj, 0) = -fjac(jj, 0);
            }

            jacS += fjac(jj, 0);
            //fjac(jj, 0) = 2*(r3(0)*cross(0) + r3(1)*cross(1) + r3(2)*cross(2));
            //ROS_INFO_STREAM("cross = "<<cross<< " res = "<<r3);
            //ROS_INFO_STREAM("jac = "<<fjac(jj, 0));
            
            //std::cout<<"jac = "<<jq(0)<<std::endl;
            //fjac(ii, 1) = jq(1);


            if (jj == 20)
            {
                ROS_INFO_STREAM("r3 = "<<r3);
                ROS_INFO_STREAM("cross = "<<cross_norm);
                ROS_INFO_STREAM("cross scaled = "<<scale0<<"  "<<scale1<<"  "<<scale2<<"  ");
                ROS_INFO_STREAM("res+jac = "<<std::abs(r3(0)+scale0) + std::abs(r3(1)+scale1) + std::abs(r3(2)+scale2)<< " abs(res) =  "<< std::abs(r3(0)) + std::abs(r3(1)) + std::abs(r3(2)));
                ROS_INFO_STREAM("fjac(jj, 0) = "<<fjac(jj, 0));
            }
            //    std::cout<<" cols = "<<jq.cols()<<" rows = "<<jq.rows()<<"  j = "<<jq<<std::endl;
            //std::cout<<"dev = "<<jq(0, 0)<<"  "<<jq(0,1)<<"  "<<jq(0,2)<<"  "<<jq(0,3)<<std::endl;

        }
        jacS = jacS / transCloud->size();
        //ROS_INFO_STREAM("jac Sum = " << jacS);
        //std::exit(0);

        //x(0) = pose.q().w();
        //x(1) = pose.q().x();
        //std::cout<<"wx = "<<x(0)<<"  "<<x(1)<<"  "<<std::endl;
        return 1;
    }


    template<typename _Scalar>
    const _Scalar cxy_icp_arti_ik_func<_Scalar>::matchPointCloud(const PointT& model
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
    const Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> cxy_icp_arti_ik_func<_Scalar>::calculateJacobianKernel(
                                                Eigen::Matrix< _Scalar, Eigen::Dynamic, 1>& x
                                            , const pcl::PointXYZ& a
                                            , const cxy_transform::Pose<_Scalar> &para_pose
                                            , const cxy_transform::Pose<_Scalar> &para_pose_parent
                                            ) const
    {
        

        cxy_transform::Pose<_Scalar> poseTmp = cxy_transform::Pose<_Scalar>::rotateByAxis_fromIdentity((*kc_->getKinematicChainNodes())[joint_].rotateAxis_, x(0), cxy_transform::Pose<_Scalar>());
        const Eigen::Quaternion<_Scalar>& q_theta(poseTmp.q());
        // compute jacobian including kinematic chain
        const Eigen::Quaternion<_Scalar>& q_w_norm(para_pose.q());

        const int n(2);
        Eigen::Quaternionf q(para_pose.q());
                    Matrix jacQuat;
                    jacQuat.resize(2, n);
                    jacQuat.setZero();
                    const _Scalar Y_Qw_QxAz = -q.x()*a.z;
                    const _Scalar Y_Qx_QxAy_QwAz = -2*q.x()*a.y-q.w()*a.z;
                    const _Scalar Z_Qw_QxAy = q.x()*a.y;
                    const _Scalar Z_Qx_QwAy_QxAz = -2*q.x()*a.z+q.w()*a.y;
                    jacQuat << Y_Qw_QxAz, Y_Qx_QxAy_QwAz,
                               Z_Qw_QxAy, Z_Qx_QwAy_QxAz;
                    Matrix normalJaco44;
                    normalJaco44.resize(n, n);
                    normalJaco44.setZero();
                    normalJaco44 << q.x()*q.x(), -q.w()*q.x(), 
                            -q.x()*q.w(), q.w()*q.w();
                            
                    normalJaco44 = normalJaco44 / std::pow(q.w()*q.w()+q.x()*q.x(), 1.5);
                    Matrix JacTheata(2,1);
                    JacTheata<< -std::sin(Deg2Rad(x(0)/2)), std::cos(Deg2Rad(x(0)/2));

                    /*
                     [ 2*a.z*q.y() - 2*a.y*q.z(),             2*a.y*q.y() + 2*a.z*q.z(), 2*a.z*q.w() - 4*a.x*q.y() + 2*a.y*q.x(), 2*a.z*q.x() - 4*a.x*q.z() - 2*a.y*q.w();
                     [ 2*a.x*q.z() - 2*a.z*q.x(), 2*a.x*q.y() - 2*a.z*q.w() - 4*a.y*q.x(),             2*a.x*q.x() + 2*a.z*q.z(), 2*a.x*q.w() - 4*a.y*q.z() + 2*a.z*q.y()]
                     [ 2*a.y*q.x() - 2*a.x*q.y(), 2*a.y*q.w() + 2*a.x*q.z() - 4*a.z*q.x(), 2*a.y*q.z() - 2*a.x*q.w() - 4*a.z*q.y(),             2*a.x*q.x() + 2*a.y*q.y()]
                     */
                     assert(jacQuat.cols() == normalJaco44.rows() && normalJaco44.cols() == JacTheata.rows());
                    return jacQuat*JacTheata;

        //return jac_dqwX_dqw*jac_dqw_unnorm_dqtheta*JacTheata;
        //return jac_dqwX_dqw*jac_dqw_norm_dqw_unnorm*jac_dqw_unnorm_dqtheta*JacTheata;
        //return jac_dqwX_dqw*jac_dqw_norm_dqw_unnorm*jac_dqw_unnorm_dqtheta*JacTheata;
    }

    template<typename _Scalar>
    void cxy_icp_arti_ik_func<_Scalar>::manifold() const
    {

        Eigen::Matrix< _Scalar, Eigen::Dynamic, 1> x(1);
        std::ofstream fout("/home/xiongyi/repo/manifold.txt");
        std::ofstream foutjac("/home/xiongyi/repo/manifold_jac.txt");
        //cxy_transform::Pose<_Scalar> pose;
        const _Scalar delta = Deg2Rad(5.0);
        _Scalar counter1(Deg2Rad(-0.0));
        while (1)
        {
            x(0) = counter1;
            _Scalar res(0.0);
            _Scalar jacS(0.0);
            x_full_(joint_) = x(0);
            pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud;
            cxy_transform::Pose<_Scalar> pose;
            cxy_transform::Pose<_Scalar> para_pose_parent;
            transCloud = kc_->getOneModelCloud_World(x_full_, joint_, pose, para_pose_parent);
            Eigen::Matrix< _Scalar, 3, 1> rotation_axis(1.0, 0.0, 0.0);
            for (unsigned int jj = 0; jj < transCloud->size(); ++jj)
            {
                pcl::PointXYZ transPoint;
                //pose.composePoint((*transCloud)[ii], transPoint);
                Eigen::Matrix< _Scalar, 3, 1> r3;
                res += matchPointCloud((*transCloud)[jj], r3);
                
            
                Eigen::Matrix< _Scalar, 3, 1> jac;
                //ROS_INFO_STREAM("jac34 = "<<jac34);
                //ROS_INFO(" ");
            r3 = -r3;
            /*
            for (int ii = 0; ii < 3; ++ii)
            {
                if (r3(ii) > 0)
                    r3(ii) = - r3(ii);
            }   
            */
                //Matrix jq(-jac34.transpose()*header);
                //jq *= 2;
                Eigen::Matrix< _Scalar, 3, 1> cross = rotation_axis.cross(r3);
                //fjac(jj, 0) = _Scalar(r3.transpose()*cross);
                jacS += cross(0) + cross(1) + cross(2);
                //foutjac<<cross(0) + cross(1) + cross(2)<<" ";
            }
            foutjac<<std::endl;
            res = res / this->values();
            jacS = jacS / this->values();
            fout<<counter1<<" "<<res<<" "<<jacS<<std::endl;
            std::cout<<counter1<<" "<<std::endl;
            counter1 += delta;
            if (counter1 >= Deg2Rad(360))
                std::exit(1);

        }
        return ;

    }

}

}
template class cxy::cxy_lmicp_lib::cxy_icp_arti_ik_func<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_arti_ik_func<double>;