#include "cxy_icp_arti_func.h"

namespace cxy
{
    namespace cxy_lmicp_lib
    {


    template<typename _Scalar>
    cxy_icp_arti_func<_Scalar>::cxy_icp_arti_func(int nPara
                    , std::auto_ptr<cxy_icp_kinematic_chain<_Scalar>> kc
                    , pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud
                    , pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeptr
                    )
                    : cxy_optimization::Cxy_Cost_Func_Abstract<_Scalar, Eigen::Dynamic, Eigen::Dynamic>(nPara, dataCloud->size())
    {
        dataCloud_ = dataCloud;
        kdtreeptr_ = kdtreeptr;
        kc_ = kc;
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

        static int ac = 0;
        //ROS_INFO_STREAM("Call f the 1   "<<++ac);


        //ROS_INFO_STREAM("Call f the 2   "<<ac);
        //std::cout<<x(0)<<" "<<x(1)<<" "<<x(2)<<"  q= "<<x(3)<<" "<<x(4)<<" "<<x(5)<<" "<<x(6)<<std::endl;
        _Scalar res(0.0);
        cxy_transform::Pose<_Scalar> pose;
        pose.rotateByAxis(cxy_transform::Axis::X_axis, x(0));
        pose.normalize();
        std::vector<_Scalar> vPara(7);
        vPara[0] = 0.0;
        vPara[1] = .0;
        vPara[2] = .0;
        vPara[3] = pose.q().w();
        vPara[4] = pose.q().x();
        vPara[5] = .0;
        vPara[6] = .0;

        for (unsigned int ii = 0; ii < dataCloud_->size(); ++ii)
        {
            pcl::PointXYZ transPoint;
            pose.composePoint((*dataCloud_)[ii], transPoint);
            Eigen::Matrix< _Scalar, 3, 1> r3;
            fvec[ii] = matchPointCloud(transPoint, r3);
            res += fvec[ii];

        }
        res = res / this->values();
        ROS_INFO_STREAM("Call f the 3    "<<ac++<<" time. Residual =  "<< res);


        ROS_INFO_STREAM("theta =  "<<x(0));
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
        cxy_transform::Pose<_Scalar> pose;
        pose.rotateByAxis(cxy_transform::Axis::X_axis, x(0));
        pose.normalize();
        std::vector<_Scalar> vPara(7);
        vPara[0] = x(0);
        vPara[1] = .0;
        vPara[2] = .0;
        vPara[3] = pose.q().w();
        vPara[4] = pose.q().x();
        vPara[5] = .0;
        vPara[6] = .0;

        for (unsigned int ii = 0; ii < dataCloud_->size(); ++ii)
        {
            pcl::PointXYZ transPoint;
            pose.composePoint((*dataCloud_)[ii], transPoint);
            Eigen::Matrix< _Scalar, 3, 1> r3;
            matchPointCloud(transPoint, r3);


            //Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> r3;
            //ROS_INFO_STREAM("r3 = "<<r3);
            Matrix jac34(calculateJacobianKernel(vPara
                    , transPoint)); //(*dataCloud_)[ii]));
            if (ii == 700)
            {
                //std::cout<<ii<<" = "<<(*dataCloud_)[ii].x<<"  "<<(*dataCloud_)[ii].y<<"  "<<(*dataCloud_)[ii].z<<std::endl;
                //std::cout<<ii<<" = "<<jac34(0)<<"  "<<jac34(1)<<"  "<<jac34(2)<<std::endl;
                //std::cout<<ii<<" = "<<vPara[0]<<"  "<<vPara[1]<<"  "<<vPara[2]<<"  "<<vPara[3]<<"  "<<vPara[4]<<"  "<<vPara[5]<<"  "<<vPara[6]<<std::endl;
            }
            Matrix header(2,1);
            header<<r3(1), r3(2);

            //ROS_INFO_STREAM("jac34 = "<<jac34);
            //ROS_INFO(" ");

            Matrix jq(-jac34.transpose()*header);
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
            fjac(ii, 0) = jq(0);
            //fjac(ii, 1) = jq(1);


            if (ii == 20)
                std::cout<<" cols = "<<jq.cols()<<" rows = "<<jq.rows()<<"  j = "<<jq<<std::endl;
            //std::cout<<"dev = "<<jq(0, 0)<<"  "<<jq(0,1)<<"  "<<jq(0,2)<<"  "<<jq(0,3)<<std::endl;

        }
        //x(0) = pose.q().w();
        //x(1) = pose.q().x();
        //std::cout<<"wx = "<<x(0)<<"  "<<x(1)<<"  "<<std::endl;
        return 1;
    }


    template<typename _Scalar>
    const _Scalar cxy_icp_arti_func<_Scalar>::matchPointCloud(const PointT& data
                                    , Eigen::Matrix< _Scalar, 3, 1>& res) const
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

        /*if (pointNKNSquaredDistance[0] > max_correspondence_dist_square_)
        {
            return std::nanf("");
        }
        */
        /// Use Euclidean distance
        const pcl::PointXYZ& pTmp((*modelCloud_)[pointIdxNKNSearch[0]]);
        //ROS_INFO_STREAM("closet point = "<<pTmp.x<<" "<<pTmp.y<<" "<<pTmp.z);
        //ROS_INFO_STREAM(" ");
        res(0) = pTmp.x - data.x;
        res(1) = pTmp.y - data.y;
        res(2) = pTmp.z - data.z;
        //int signTmp = 1;//(res(0)+res(1)+res(2)) > 0 ? 1 : -1;
        //ROS_INFO_STREAM(rtmp);

        return std::sqrt(pointNKNSquaredDistance[0]);
    }


    template<typename _Scalar>

    const Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic> cxy_icp_arti_func<_Scalar>::calculateJacobianKernel(
                                        const std::vector<_Scalar> &para
                                        , const pcl::PointXYZ& a) const
    {
        const int n(2);
        //ROS_INFO_STREAM("JacoIn "<<para[0]<<"  "<<para[3]<<"  "<<para[4]);
        Eigen::Quaternionf q(para[3], para[4], 0.0, 0.0);
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
        JacTheata<< -std::sin(Deg2Rad(para[0]/2)), std::cos(Deg2Rad(para[0]/2));

        /*
         [ 2*a.z*q.y() - 2*a.y*q.z(),             2*a.y*q.y() + 2*a.z*q.z(), 2*a.z*q.w() - 4*a.x*q.y() + 2*a.y*q.x(), 2*a.z*q.x() - 4*a.x*q.z() - 2*a.y*q.w();
         [ 2*a.x*q.z() - 2*a.z*q.x(), 2*a.x*q.y() - 2*a.z*q.w() - 4*a.y*q.x(),             2*a.x*q.x() + 2*a.z*q.z(), 2*a.x*q.w() - 4*a.y*q.z() + 2*a.z*q.y()]
         [ 2*a.y*q.x() - 2*a.x*q.y(), 2*a.y*q.w() + 2*a.x*q.z() - 4*a.z*q.x(), 2*a.y*q.z() - 2*a.x*q.w() - 4*a.z*q.y(),             2*a.x*q.x() + 2*a.y*q.y()]
         */
        assert(jacQuat.cols() == normalJaco44.rows() && normalJaco44.cols() == JacTheata.rows());
        return jacQuat*JacTheata;
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
            cxy_transform::Pose<_Scalar> pose;

            counter1 += delta;
            pose.rotateByAxis(cxy_transform::Axis::X_axis, counter1);
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

        }
        return ;

    }

}

}
template class cxy::cxy_lmicp_lib::cxy_icp_arti_func<float>;
template class cxy::cxy_lmicp_lib::cxy_icp_arti_func<double>;