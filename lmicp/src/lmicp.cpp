#include "lmicp.h"

namespace cxy_lmicp
{
    LM_ICP::LM_ICP() : 
                pnh_("~")
                , nh_("")
                , isNewModel_(false)

    {

        pnh_.param<int>("max_iterations", max_iterations_, 1000);
        pnh_.param<double>("transformation_epsilon", transformation_epsilon_, 1e-8);
        pnh_.param<double>("euclidean_fitness_epsilon", euclidean_fitness_epsilon_, 0.1);
        pnh_.param<double>("max_correspondence_dist", max_correspondence_dist_, 0.07); // 0.2
        max_correspondence_dist_square_ = max_correspondence_dist_*max_correspondence_dist_;

        pub_model_ = nh_.advertise<visualization_msgs::MarkerArray>("model", 5);
        pub_data_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("data_pointcloud", 5);
        pub_model_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("model_pointcloud", 5);
        pub_result_ = nh_.advertise<sensor_msgs::PointCloud2>("data_result", 5);
    }

    LM_ICP::~LM_ICP()
    {
        // Destructor code goes here
    }

    bool LM_ICP::initialise()
    {
       

    }

ctrs::Pose<_Scalar> LM_ICP::lmicp(const PointCloudPtr data, const PointCloudPtr model, const ctrs::Pose<_Scalar> guess)
    {
        if ( ! isNewModel_)
        {
            return ctrs::Pose<_Scalar>();
        }
        dataCloud_ = boost::make_shared<PointCloud>(*data);
        ctrs::Pose<_Scalar> guessTmp(guess);
        //initGuess(guessTmp);
        guessTmp = ctrs::Pose<_Scalar>(0);
        ctrs::Pose<_Scalar>& pose_base(guessTmp);
        PointCloudPtr transDataCloud;
        PointCloudPtr transDataCloud2;
        //pose_base.composePoint(data, transDataCloud);

        // pose_inc
        ctrs::Pose<_Scalar> pose_inc(0);
        ctrs::Pose<_Scalar> pose_tmp(0);
        std::vector<int> modelMatchIdx;
        std::vector<float> matchDistance;
        std::vector<int> dataMatchIdx;
        std::vector<E::Vector3f> residual;
        matchDistancefPtr_ = std::make_shared<std::vector<float> >();
        //dataCloud_ = transDataCloud;
        char c;
        //std::cout<<dataCloud_->size()<<"   dataCloud_";

        //translateToCenter(dataCloud_);

        while(true)
        {
            publish(dataCloud_, pub_data_pointcloud_);
            publish(modelCloud_, pub_model_pointcloud_);
            ros::spinOnce();
            std::cin>>c;
            if ('n' == c)
                break;
        }
        transDataCloud = dataCloud_;
        while (true)
        {
           pose_inc.composePoint(dataCloud_, transDataCloud);
             ROS_INFO_STREAM("pose_inc  new "<< pose_inc.t()<<"  "<< pose_inc.q().w()<<"  "<< pose_inc.q().x()<<"  "<< pose_inc.q().y()<<"  "<< pose_inc.q().z()<<"  ");
            //ROS_INFO_STREAM(transDataCloud->size());

            
                publish(transDataCloud, pub_data_pointcloud_);
                publish(modelCloud_, pub_model_pointcloud_);
                ros::spinOnce();
                std::cin>>c;
                if ('n' == c)
                    continue;
                if ('b' == c)
                    break;
           const float residualSum =  searchMatchPoints(transDataCloud 
                                                        , dataMatchIdx 
                                                        , modelMatchIdx 
                                                        , *matchDistancefPtr_);
            ROS_INFO_STREAM("matching ratio  "<< float(dataMatchIdx.size())/transDataCloud->size()<< "   residual "<<residualSum/dataMatchIdx.size());
            if (dataMatchIdx.size() < 10)
            {
                //return ctrs::Pose();
            }
            //ROS_INFO_STREAM("Size   "<<matchDistancefPtr_->size()<<"  "<<dataMatchIdx.size());

            calculateResidual( transDataCloud
                            , dataMatchIdx
                            , modelMatchIdx
                            , residual);
            ctrs::Pose<_Scalar> tmp_inc(0);
            calculateLevenbergMarquardt(transDataCloud
                                        , dataMatchIdx
                                        , modelMatchIdx
                                        , pose_inc
                                        , residual
                                        , tmp_inc);
            
            tmp_inc.t() = pose_inc.t()+tmp_inc.t();
            tmp_inc.q().w() = pose_inc.q().w()+tmp_inc.q().w();
            tmp_inc.q().x() = pose_inc.q().x()+tmp_inc.q().x();
            tmp_inc.q().y() = pose_inc.q().y()+tmp_inc.q().y();
            tmp_inc.q().z() = pose_inc.q().z()+tmp_inc.q().z();
            ROS_INFO_STREAM("Quaterniond = "<<tmp_inc.q().w()<<"  "<<tmp_inc.q().x()<<"  "<<tmp_inc.q().y()<<"  "<<tmp_inc.q().z());
            tmp_inc.normalize();
            
            pose_inc = tmp_inc;

            //tmp_inc.normalize();
        }


    }


    void LM_ICP::calculateLevenbergMarquardt(const PointCloudPtr data
                                    , const std::vector<int>& dataMatchIdx
                                    , const std::vector<int>& modelMatchIdx
                                    , const ctrs::Pose<_Scalar>& pose_k
                                    , const std::vector<E::Vector3f>& residual
                                    , ctrs::Pose<_Scalar>& pose_k1)
    {
        Matrix37f jac37;
        Matrix34f jacQuat;
        MatrixX7f jac(dataMatchIdx.size(), 7);
        //MatrixX7f jacTjac(7, 7);
        Matrix7f jacTjac;
        //Matrix7f jacTjac(7, 7);
        Vector7f result_Pose;
        Vector7f jac_right;

        jac37.setZero();
        jacQuat.setZero();
        jac.setZero();
        jacTjac.setZero();
        result_Pose.setZero();
        jac_right.setZero();
        float lambda(0.5);
        //jac37.topLeftCorner(3,3) = E::MatrixXd::Identity(3, 3);
        //jac37.topLeftCorner(3,3) = E::Matrix3f::Identity();
        for (std::size_t ii = 0; ii < dataMatchIdx.size(); ++ii)
        {
            const PointT& d((*data)[dataMatchIdx[ii]]), m((*modelCloud_)[modelMatchIdx[ii]]);
            E::Matrix<float, 1, 7> rowJ;
            //rowJ <<(2*d.x - 2*m.x), (2*d.y - 2*m.y), (2*d.z - 2*m.z), 0, (4*d.y*(d.z - m.z) - 4*d.z*(d.y - m.y)), (4*d.z*(d.x - m.x) - 4*d.x*(d.z - m.z)), (4*d.x*(d.y - m.y) - 4*d.y*(d.x - m.x));
            //rowJ <<(2*d.x - 2*m.x), (2*d.y - 2*m.y), (2*d.z - 2*m.z), (4*d.y*(d.z - m.z) - 4*d.z*(d.y - m.y)), (4*d.z*(d.x - m.x) - 4*d.x*(d.z - m.z)), (4*d.x*(d.y - m.y) - 4*d.y*(d.x - m.x));
            //ROS_INFO_STREAM_ONCE(rowJ);
            

            Matrix34f jac34(calculateJacobianKernel(pose_k
                                                    , residual[ii]
                                                    , d));
            if (ii == 700)
            {
                //std::cout<<ii<<" = "<<(*data)[ii].x<<"  "<<(*data)[ii].y<<"  "<<(*data)[ii].z<<std::endl;
                //std::cout<<ii<<" = "<<residual[ii](0)<<"  "<<residual[ii](1)<<"  "<<residual[ii](2)<<std::endl;
                //std::cout<<ii<<" = "<<jac34(0)<<"  "<<jac34(1)<<"  "<<jac34(2)<<std::endl;
            }

            E::Matrix<float, 1, 4> jq(residual[ii].transpose()*jac34);
            rowJ<<residual[ii](0), residual[ii](1), residual[ii](2),  jq(0), jq(1), jq(2), jq(3);
            if (ii == 700)
                std::cout<<ii<<" = "<<rowJ<<std::endl;
	    rowJ = rowJ*2;
            jacTjac += rowJ.transpose()*rowJ;
            // JT*e
	    const double sigma = 0.2;
	    const double &&r = std::sqrt((*matchDistancefPtr_)[ii]);
	    //ROS_INFO_STREAM(r);
	    double e = r < sigma ? r*r : 2*sigma*std::abs(r)-sigma*sigma;
            jac_right += rowJ.transpose()*e;
        //std::cout<<ii<<" = "<<rowJ(0)<<"  "<<rowJ(1)<<"  "<<rowJ(2)<<"  "<<rowJ(3)<<"  "<<rowJ(4)<<"  "<<rowJ(5)<<"  "<<rowJ(6)<<std::endl;

        }
        //jacTjac = jac.transpose() * jac;
        //jacTjac
        Matrix7f jac_left;
        Matrix7f j_dia;
        j_dia.setZero();
        jac_left.setZero();
        for (int ii = 0; ii < 7; ++ii)
        {
            j_dia(ii,ii) = jacTjac(ii,ii);

        }
        jac_left = -(jacTjac+lambda*j_dia);
        //ROS_INFO_STREAM(jac_left);
        jac_left = jac_left.inverse();
        //ROS_INFO_STREAM(jac_left);
        result_Pose = jac_left * jac_right;
        
        //ROS_INFO_STREAM(result_Pose);
        //result_Pose = jac_left.inverse()*jac.transpose()*res_tmp;
	//result_Pose = -result_Pose;
        pose_k1.t() = E::Vector3f(-result_Pose(0), -result_Pose(1), -result_Pose(2));
        pose_k1.q() = E::Quaternionf(-result_Pose(3), -result_Pose(4), -result_Pose(5), -result_Pose(6));
        //pose_k1.q() = E::Quaterniond(1, .0, .0, .0);
        //pose_k1.normalize();
        //pose_k1 = lambda
        return;   
    }

inline const Matrix34f LM_ICP::calculateJacobianKernel(const ctrs::Pose<_Scalar>& pose_k
                                                , const E::Vector3f& res
                                                , const PointT& a)
    {

        E::Quaternionf q(pose_k.q().w(), pose_k.q().x(), pose_k.q().y(), pose_k.q().z());
        Matrix34f jacQuat;
        jacQuat.setZero();
	if ( 0.0 == q.x() && 0.0 == q.y() && 0.0 == q.z())
                        {
                            q.x() = 0.01;
                            q.y() = 0.01;
                            q.z() = 0.01;
			    q.normalize();
                        }
        jacQuat << (2*a.z*q.y() - 2*a.y*q.z()) , (2*a.y*q.y() + 2*a.z*q.z())                ,(2*a.z*q.w() - 4*a.x*q.y() + 2*a.y*q.x()) , (2*a.z*q.x() - 4*a.x*q.z() - 2*a.y*q.w())
                , (2*a.x*q.z() - 2*a.z*q.x()) , (2*a.x*q.y() - 2*a.z*q.w() - 4*a.y*q.x()) , (2*a.x*q.x() + 2*a.z*q.z())                 ,( 2*a.x*q.w() - 4*a.y*q.z() + 2*a.z*q.y())
                , (2*a.y*q.x() - 2*a.x*q.y()) , (2*a.y*q.w() + 2*a.x*q.z() - 4*a.z*q.x()) , (2*a.y*q.z() - 2*a.x*q.w() - 4*a.z*q.y()) , (2*a.x*q.x() + 2*a.y*q.y());
        
	E:Matrix44f normalJaco44;
       normalJaco44.setZero();
       normalJaco44 << q.x()*q.x()+q.y()*q.y()+q.z()*q.z(), -q.w()*q.x(), -q.w()*q.y(), -q.w()*q.z(),
		    -q.x()*q.w(), q.w()*q.w()+q.y()*q.y()+q.z()*q.z(), -q.x()*q.y(), -q.x()*q.z(),
		    -q.y()*q.w(), -q.y()*q.x(), q.w()*q.w()+q.x()*q.x()+q.z()*q.z(), -q.y()*q.z(),
		    -q.z()*q.w(), -q.z()*q.x(), -q.z()*q.y(), q.w()*q.w()+q.y()*q.y()+q.x()*q.x();
       normalJaco44 = normalJaco44 / std::pow(q.w()*q.w()+q.y()*q.y()+q.x()*q.x()+q.z()*q.z(), 1.5);

       /*
        [ 2*a.z*q.y() - 2*a.y*q.z(),             2*a.y*q.y() + 2*a.z*q.z(), 2*a.z*q.w() - 4*a.x*q.y() + 2*a.y*q.x(), 2*a.z*q.x() - 4*a.x*q.z() - 2*a.y*q.w();
        [ 2*a.x*q.z() - 2*a.z*q.x(), 2*a.x*q.y() - 2*a.z*q.w() - 4*a.y*q.x(),             2*a.x*q.x() + 2*a.z*q.z(), 2*a.x*q.w() - 4*a.y*q.z() + 2*a.z*q.y()]
        [ 2*a.y*q.x() - 2*a.x*q.y(), 2*a.y*q.w() + 2*a.x*q.z() - 4*a.z*q.x(), 2*a.y*q.z() - 2*a.x*q.w() - 4*a.z*q.y(),             2*a.x*q.x() + 2*a.y*q.y()]
        */
        return jacQuat*normalJaco44;
    }

    void LM_ICP::calculateJacobian()
    {

        return;
    }
/*
ctrs::Pose LM_ICP::lmicpNumerical(const PointCloudPtr data, const PointCloudPtr model, const ctrs::Pose guess)
    {
        if ( ! isNewModel_)
        {
            return ctrs::Pose();
        }
        dataCloud_ = boost::make_shared<PointCloud>(*data);
        ctrs::Pose guessTmp(guess);
        //initGuess(guessTmp);
        guessTmp = ctrs::Pose(0);
        ctrs::Pose& pose_base(guessTmp);
        PointCloudPtr transDataCloud;
        PointCloudPtr transDataCloud2;
        //pose_base.composePoint(data, transDataCloud);

        // pose_inc
        ctrs::Pose pose_inc(0);
        ctrs::Pose pose_tmp(0);
        std::vector<int> modelMatchIdx;
        std::vector<float> matchDistance;
        std::vector<int> dataMatchIdx;
        std::vector<E::Vector3f> residual;
        matchDistancefPtr_ = std::make_shared<std::vector<float> >();
        //dataCloud_ = transDataCloud;
        char c;
        //std::cout<<dataCloud_->size()<<"   dataCloud_";

        //translateToCenter(dataCloud_);

        while(true)
        {
            publish(dataCloud_, pub_data_pointcloud_);
            publish(modelCloud_, pub_model_pointcloud_);
            ros::spinOnce();
            std::cin>>c;
            if ('n' == c)
                break;
        }
        transDataCloud = dataCloud_;
        while (true)
        {
            /*
            if ( 0.0 == pose_inc.q().x() && 0.0 == pose_inc.q().y() && 0.0 == pose_inc.q().z())
                        {
                            pose_inc.q().x() = 0.001;
                            pose_inc.q().y() = 0.001;
                            pose_inc.q().z() = 0.001;
                        }
                        
            //publish(transData, pub_data_pointcloud_);
            //pose_tmp = ctrs::Pose(0);
            //pose_base.composePose(pose_tmp, pose_inc);
            pose_inc.composePoint(dataCloud_, transDataCloud);
            
            /*pose_inc.composePoint(transDataCloud, transDataCloud2);
            transDataCloud = transDataCloud2;

            ROS_INFO_STREAM("pose_inc  new "<< pose_inc.t()<<"  "<< pose_inc.q().w()<<"  "<< pose_inc.q().x()<<"  "<< pose_inc.q().y()<<"  "<< pose_inc.q().z()<<"  ");
            //ROS_INFO_STREAM(transDataCloud->size());

            while(true)
            {
                publish(transDataCloud, pub_data_pointcloud_);
                publish(modelCloud_, pub_model_pointcloud_);
                ros::spinOnce();
                std::cin>>c;
                if ('n' == c)
                    break;
            }
            //pose_tmp.t()(0) = 0.2;
            //pose_tmp.t()(2) = 0.4;
            //pose_tmp.q() = E::AngleAxisd(0.25*M_PI, E::Vector3d::UnitX());
            //pose_tmp.q().x() = 1.0;
            //pose_tmp.q().w() = 0.1;
            //pose_tmp.normalize();
            //continue;
            const float residualSum =  searchMatchPoints(transDataCloud 
                                                        , dataMatchIdx 
                                                        , modelMatchIdx 
                                                        , *matchDistancefPtr_);
            ROS_INFO_STREAM("matching ratio  "<< float(dataMatchIdx.size())/transDataCloud->size()<< "   residual "<<residualSum/dataMatchIdx.size());
            if (dataMatchIdx.size() < 10)
            {
                //return ctrs::Pose();
            }
            //ROS_INFO_STREAM("Size   "<<matchDistancefPtr_->size()<<"  "<<dataMatchIdx.size());

            calculateResidual( transDataCloud
                            , dataMatchIdx
                            , modelMatchIdx
                            , residual);
            ctrs::Pose tmp_inc(0);
            calculateLevenbergMarquardtNumerical(transDataCloud
                                        , dataMatchIdx
                                        , modelMatchIdx
                                        , pose_inc
                                        , residual
                                        , tmp_inc);
            
            tmp_inc.t() = pose_inc.t()+tmp_inc.t();
            tmp_inc.q().w() = pose_inc.q().w()+tmp_inc.q().w();
            tmp_inc.q().x() = pose_inc.q().x()+tmp_inc.q().x();
            tmp_inc.q().y() = pose_inc.q().y()+tmp_inc.q().y();
            tmp_inc.q().z() = pose_inc.q().z()+tmp_inc.q().z();
            tmp_inc.normalize();
            
            pose_inc = tmp_inc;

            //tmp_inc.normalize();
        }


    }

void LM_ICP::calculateLevenbergMarquardtNumerical(const PointCloudPtr data
                                    , const std::vector<int>& dataMatchIdx
                                    , const std::vector<int>& modelMatchIdx
                                    , const ctrs::Pose& pose_k
                                    , const std::vector<E::Vector3f>& residual
                                    , ctrs::Pose& pose_k1)
    {
        Matrix37f jac37;
        Matrix34f jacQuat;
        //MatrixX7f jacTjac(7, 7);
        Matrix7f jacTjac;
        //Matrix7f jacTjac(7, 7);
        Vector7f result_Pose;
        Vector7f jac_right;

        jac37.setZero();
        jacQuat.setZero();
        jacTjac.setZero();
        result_Pose.setZero();
        jac_right.setZero();
        float lambda(0.5);
        //jac37.topLeftCorner(3,3) = E::MatrixXd::Identity(3, 3);
        //jac37.topLeftCorner(3,3) = E::Matrix3f::Identity();
        for (std::size_t ii = 0; ii < dataMatchIdx.size(); ++ii)
        {
            const PointT& d((*data)[dataMatchIdx[ii]]), m((*modelCloud_)[modelMatchIdx[ii]]);
            //E::Matrix<float, 1, 6> rowJ;
            //rowJ <<(2*d.x - 2*m.x), (2*d.y - 2*m.y), (2*d.z - 2*m.z), 0, (4*d.y*(d.z - m.z) - 4*d.z*(d.y - m.y)), (4*d.z*(d.x - m.x) - 4*d.x*(d.z - m.z)), (4*d.x*(d.y - m.y) - 4*d.y*(d.x - m.x));
            //rowJ <<(2*d.x - 2*m.x), (2*d.y - 2*m.y), (2*d.z - 2*m.z), (4*d.y*(d.z - m.z) - 4*d.z*(d.y - m.y)), (4*d.z*(d.x - m.x) - 4*d.x*(d.z - m.z)), (4*d.x*(d.y - m.y) - 4*d.y*(d.x - m.x));
            //ROS_INFO_STREAM_ONCE(rowJ);
            float l2norm(0.0);
            E::Vector3f res;
            ROS_INFO_STREAM("m = "<<m.x<<" "<<m.y<<" "<<m.z);
            ROS_INFO_STREAM("p = "<<d.x<<" "<<d.y<<" "<<d.z);
            E::Matrix<float,1,7> jac17(calculateJacobianKernelNumerical(pose_k
                                                , d
                                                , m
                                                , l2norm
                                                , res));
            //E::Matrix<float, 1, 4> jq(residual[ii].transpose()*jac34);
            //rowJ<<2*res(0), 2*res(1), 2*res(2),  jq(1), jq(2), jq(3);
            //rowJ = 2*rowJ*(*matchDistancefPtr_)[ii];
            //jac17 = -jac17;
            if (std::isnan(l2norm))
            {
                //ROS_INFO("l2norm is nan");
                continue;
            }
            //jac17 = jac17 * (*matchDistancefPtr_)[ii];
            jacTjac += jac17.transpose()*jac17;
            // JT*e
            jac_right += jac17.transpose()*(*matchDistancefPtr_)[ii]*(*matchDistancefPtr_)[ii];
        }
        //jacTjac = jac.transpose() * jac;
        //jacTjac
        Matrix7f jac_left;
        Matrix7f j_dia;
        j_dia.setZero();
        jac_left.setZero();
        for (int ii = 0; ii < 7; ++ii)
        {
            j_dia(ii,ii) = jacTjac(ii,ii);

        }
        //j_dia = Matrix7f::Identity();
        jac_left = (jacTjac+lambda*j_dia);
        ROS_INFO_STREAM(jac_left);
        jac_left = -jac_left.inverse();
        ROS_INFO_STREAM(jac_left);
        ROS_INFO_STREAM(jac_right);
        result_Pose = jac_left * jac_right;
        
        ROS_INFO_STREAM(result_Pose);
        //result_Pose = jac_left.inverse()*jac.transpose()*res_tmp;
        pose_k1.t() = E::Vector3d(result_Pose(0), result_Pose(1), result_Pose(2));
        pose_k1.q() = E::Quaterniond(result_Pose(3), result_Pose(4), result_Pose(5), result_Pose(6));
        //pose_k1.normalize();
        //pose_k1 = lambda
        return;
    }

inline const E::Matrix<float,1,7> LM_ICP::calculateJacobianKernelNumerical(const ctrs::Pose& pose_k
                                                    , const PointT& p
                                                    , const PointT& m
                                                    , float& l2norm
                                                    , E::Vector3f res)
    {

        //const E::Quaternionf q(pose_k.q().w(), pose_k.q().x(), pose_k.q().y(), pose_k.q().z());
        const E::Quaternionf q(pose_k.q().w(), pose_k.q().x(), pose_k.q().y(), pose_k.q().z());
        Matrix34f jacQuat34;
        jacQuat34.setZero();
        const PointT& a(p);
        jacQuat34 << (2*a.z*q.y() - 2*a.y*q.z()) , (2*a.y*q.y() + 2*a.z*q.z())                ,(2*a.z*q.w() - 4*a.x*q.y() + 2*a.y*q.x()) , (2*a.z*q.x() - 4*a.x*q.z() - 2*a.y*q.w())
                    , (2*a.x*q.z() - 2*a.z*q.x()) , (2*a.x*q.y() - 2*a.z*q.w() - 4*a.y*q.x()) , (2*a.x*q.x() + 2*a.z*q.z())                 ,( 2*a.x*q.w() - 4*a.y*q.z() + 2*a.z*q.y())
                    , (2*a.y*q.x() - 2*a.x*q.y()) , (2*a.y*q.w() + 2*a.x*q.z() - 4*a.z*q.x()) , (2*a.y*q.z() - 2*a.x*q.w() - 4*a.z*q.y()) , (2*a.x*q.x() + 2*a.y*q.y());
        
        const float numercialStepT(0.01);
        const float numercialStepQ(0.05);
        float step(numercialStepT);
        Matrix37f jacQuat;
        jacQuat.setZero();
        E::Matrix<float, 7,1> rowJ;
        rowJ.setZero();

        PointT resOld;
        resOld.x = m.x-p.x;
        resOld.y = m.y-p.y;
        resOld.z = m.z-p.z;
        //ROS_INFO_STREAM(jacQuat34);
        for (int ii = 0; ii < 7; ++ii)
        {
            if (ii >= 3)
            {
                step = numercialStepQ;
            }
            ctrs::Pose posetmp(pose_k);
            switch (ii)
            {
                case 0 : posetmp.t()(0) +=  step; 
                    //ROS_INFO_STREAM_ONCE(posetmp.t()(0)); 
                    break;
                case 1 : posetmp.t()(1) +=  step; 
                    //ROS_INFO_STREAM_ONCE(posetmp.t()(1)); 
                    break;
                case 2 : posetmp.t()(2) +=  step; 
                    //ROS_INFO_STREAM_ONCE(posetmp.t()(2)); 
                    break;
                case 3 : 
                        if ( 0.0 == posetmp.q().x() && 0.0 == posetmp.q().y() && 0.0 == posetmp.q().z())
                        {
                            posetmp.q().x() = 0.01;
                            posetmp.q().y() = 0.01;
                            posetmp.q().z() = 0.01;
                        }
                        posetmp.q().w() = posetmp.q().w() + step; 
                    //ROS_INFO_STREAM_ONCE(posetmp.q().w()); 
                    break;
                case 4 : posetmp.q().x() = posetmp.q().x() + step; 
                    //ROS_INFO_STREAM_ONCE(posetmp.q().x()); 
                    break;
                case 5 : posetmp.q().y() = posetmp.q().y() + step; 
                    //ROS_INFO_STREAM_ONCE(posetmp.q().y()); 
                    break;
                case 6 : posetmp.q().z() = posetmp.q().z() + step; 
                    //ROS_INFO_STREAM_ONCE(posetmp.q().z()); 
                    break;
            }
            E::Vector3f orig;
            //searchMatchPoints(p, orig);
            //ROS_INFO_STREAM("p =  "<< p.x<< "  "<< p.y<< "  "<<p.z );

            PointT a_out;
            //posetmp.normalize();
            posetmp.composePoint(p, a_out);
            l2norm = searchMatchPoints(a_out, res);
            //ROS_INFO_STREAM(res );
            PointT dx_DT;
            dx_DT.x = 1;//a_out.x - p.x;
            dx_DT.y = 1;//a_out.y - p.y;
            dx_DT.z = 1;//a_out.z - p.z;
            E::Vector3f dDTdx;
            E::Vector3f resOldVec;
            static const float nearZero(1e-10);
            if (dx_DT.x > nearZero)
            {
                dDTdx(0) = (res(0)-resOld.x);// / dx_DT.x;
                //ROS_INFO_STREAM("d res =  "<< (res(0)-resOld.x));
            }
            else
            {
                dDTdx(0) = 0.0;
            }
            if (dx_DT.y > nearZero)
            {
                dDTdx(1) = (res(1)-resOld.y);// / dx_DT.y;
                //ROS_INFO_STREAM("d res =  "<< (res(1)-resOld.y));
            }
            else
            {
                dDTdx(1) = 0.0;
            }
            if (dx_DT.z > nearZero)
            {
                dDTdx(2) = (res(2)-resOld.z);// / dx_DT.z;
                //ROS_INFO_STREAM("d res =  "<< (res(2)-resOld.z));
            }
            else
            {
                dDTdx(2) = 0.0;
            }
            resOldVec(0) = resOld.x;
            resOldVec(1) = resOld.y;
            resOldVec(2) = resOld.z;
            ROS_INFO_STREAM("a_out =  "<< a_out.x<< "  "<< a_out.y<< "  "<<a_out.z );
            //ROS_INFO_STREAM("dDTdx =  "<< dDTdx(0)<< "  "<< dDTdx(1)<< "  "<<dDTdx(2) );
            //ROS_INFO_STREAM("resOld = "<< resOld.x<< "  "<< resOld.y<< "  "<< resOld.z<< "  " );
            ROS_INFO_STREAM("res =  "<< res(0)<<"- "<<resOld.x<< "  "<< res(1)<<"- "<<resOld.y<< "  "<<res(2)<<"- "<<resOld.z );
            ROS_INFO_STREAM("dDTdx =  "<< dDTdx(0)<< "  "<< dDTdx(1)<< "  "<<dDTdx(2));
            if (std::isnan(l2norm) )
            {
                //rowJ<<std::nanf(""), std::nanf("");
                return rowJ.transpose();
            }
            //jacQuat.block<3,1>(0,ii) << res(0)/step, res(1)/step, res(2)/step;
            //rowJ(ii,0) = (res(0)-resOld.x)/step + (res(1)-resOld.y)/step + (res(2)-resOld.z)/step;
            if (ii < 3)
            {
                rowJ(ii,0) = dDTdx(ii);//*resOldVec(ii);//+dDTdx(1)+dDTdx(2);
            }
            else
            {
                for (int jj = 0; jj < 3; jj++)
                {
                    rowJ(ii,0) += jacQuat34.block<3,1>(0, ii-3)(jj)*dDTdx(jj);//*resOldVec(jj);
                }
                //E::Vector3f tmpv(jacQuat34.block<3,1>(0, ii-3));
                    //ROS_INFO_STREAM("jacQuat34 =  " << tmpv );
            }
            float tmpf(rowJ(ii,0));
            //ROS_INFO_STREAM(" "<<tmpf<<std::abs(tmpf));
            if (std::abs(tmpf) < nearZero)
            {
                ROS_INFO_STREAM("warning Jacobian near Zero "<<ii<<" "<<rowJ(ii,0) << " "<<(rowJ(ii,0) > 0 ? abs(rowJ(ii,0)) : -abs(rowJ(ii,0)))<< " "<<abs(tmpf));

                rowJ(ii,0) = nearZero;
            }
            //rowJ(ii,0) = rowJ(ii,0) * 2 * 
            //ROS_INFO_STREAM(res);
            //ROS_INFO_STREAM("res diff = "<< res(0)-resOld.x<< "  "<< res(1)-resOld.y<< "  "<< res(2)-resOld.z<< "  " );
            ROS_INFO_STREAM("J "<<ii<<" "<<rowJ(ii,0) );
            ROS_INFO_STREAM("  ");
        }
        //rowJ = 2*jacQuat.transpose()*res;
        //rowJ *= 2;

        //rowJ(ii,0) = (res(0)-resOld.x)/step + (res(1)-resOld.y)/step + (res(2)-resOld.z)/step;
        
        
        ROS_INFO_STREAM(rowJ.transpose());
        return rowJ.transpose();
    }
*/
    void LM_ICP::calculateResidual(PointCloudPtr data
                                            , const std::vector<int>& dataMatchIdx
                                            , const std::vector<int>& modelMatchIdx
                                            , std::vector<E::Vector3f>& residual)
    {
        residual.clear();
        residual.reserve(dataMatchIdx.size());
        E::Vector3f tmp;
        for (std::size_t ii = 0; ii < dataMatchIdx.size(); ++ii)
        {
            tmp(0) = (*modelCloud_)[modelMatchIdx[ii]].x - (*data)[dataMatchIdx[ii]].x;
            tmp(1) = (*modelCloud_)[modelMatchIdx[ii]].y - (*data)[dataMatchIdx[ii]].y;
            tmp(2) = (*modelCloud_)[modelMatchIdx[ii]].z - (*data)[dataMatchIdx[ii]].z;
            residual.push_back(tmp);
        }

        return;
    }

    const float LM_ICP::searchMatchPoints(PointCloudPtr data
                                            , std::vector<int>& dataMatchIdx
                                            , std::vector<int>& modelMatchIdx
                                            , std::vector<float>& matchDistance)
    {
        float r(0.0);
        dataMatchIdx.clear();
        dataMatchIdx.reserve(data->size());
        modelMatchIdx.clear();
        modelMatchIdx.reserve(data->size());
        matchDistance.clear();
        matchDistance.reserve(data->size());

        int K(1);
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        /// kdtreeptr_ is updated by setModelCloud
        if (nullptr == kdtreeptr_)
            ROS_INFO("nullptr kdtree");

        for (int ii = 0; ii < data->size(); ++ii)
        {
            if ( 0 == kdtreeptr_->nearestKSearch ((*data)[ii], K, pointIdxNKNSearch, pointNKNSquaredDistance)  )
            {
                continue;
            }
            if (pointNKNSquaredDistance[0] > max_correspondence_dist_square_)
            {
                continue;
            }
            dataMatchIdx.push_back(ii);
            modelMatchIdx.push_back(pointIdxNKNSearch[0]);
            /// Use Euclidean distance
            const PointT& pTmp((*modelCloud_)[pointIdxNKNSearch[0]]), p2Tmp((*data)[ii]);
            int signTmp = 1;//((pTmp.x - p2Tmp.x)+(pTmp.y - p2Tmp.y)+(pTmp.z - p2Tmp.z)) > 0 ? 1 : -1;
            float rtmp(signTmp*sqrt(pointNKNSquaredDistance[0]));
            matchDistance.push_back(pointNKNSquaredDistance[0]);
            //ROS_INFO_STREAM(rtmp);
            r += sqrt(pointNKNSquaredDistance[0]);

        }
        
        return r;
    }

const float LM_ICP::computeResidual(PointCloudPtr data
                                            , ctrs::Pose<_Scalar> &pose
                                            , std::vector<int>& dataMatchIdx
                                            , std::vector<int>& modelMatchIdx
                                            , std::vector<float>& matchDistance)
{
  
  PointCloudPtr transDataCloud;
  pose.composePoint(data, transDataCloud);
  float residualSum =  searchMatchPoints(transDataCloud 
                                        , dataMatchIdx 
                                        , modelMatchIdx 
                                        , *matchDistancefPtr_);

} 

inline  const float LM_ICP::searchMatchPoints(const PointT& data
                                            , E::Vector3f& res)
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
        int signTmp = 1;//(res(0)+res(1)+res(2)) > 0 ? 1 : -1;
        //ROS_INFO_STREAM(rtmp);

        return signTmp*sqrt(pointNKNSquaredDistance[0]);
    } 
    void LM_ICP::initGuess(ctrs::Pose<_Scalar>& guess)
    {
        Eigen::Matrix3f data_cov, model_cov;
        Eigen::Vector4f data_centroid, model_centroid;
        double infiniteDecimal(1e-8);
        // is there is NO valid initial guess
        // use PCA to initial
        if (std::isnan(guess.t()(0)))
        {
            // 16-bytes aligned placeholder for the XYZ centroid of a surface patch

            // Estimate the XYZ centroid
            calculateCenter(dataCloud_, data_centroid);
            //pcl::computeCovarianceMatrix (*dataCloud_, data_centroid, data_cov);
            calculateCenter(modelCloud_, model_centroid);
            //pcl::computeCovarianceMatrix (*modelCloud_, model_centroid, model_cov);
            Eigen::Vector4f tTmp(model_centroid-data_centroid);
            guess.t()(0) = tTmp(0);
            guess.t()(1) = tTmp(1);
            guess.t()(2) = tTmp(2);
            guess.q().w() = 1.0;
            guess.q().x() = 0.0;
            guess.q().y() = 0.0;
            guess.q().z() = 0.0;
        }
        else
        {
            return;
        }
        return;
    }


    void LM_ICP::calculateCenter(const PointCloudConstPtr data
                                , Eigen::Vector4f& center)
    {
        //std::cout<<data->size()<<"  calculateCenter sdataCloud_"<<std::endl;
        pcl::compute3DCentroid (*data, center);
        return ;
    }

    void LM_ICP::translateToCenter(PointCloudPtr& data)
    {
        Eigen::Vector4f data_centroid;
        calculateCenter(data, data_centroid);
        for (int ii = 0; ii < data->size(); ++ii)
        {
            (*data)[ii].x = (*data)[ii].x - data_centroid(0);
            (*data)[ii].y = (*data)[ii].y - data_centroid(1);
            (*data)[ii].z = (*data)[ii].z - data_centroid(2);
            /* code */
        }
        return;
    }

    bool LM_ICP::setModelCloud(PointCloudConstPtr model)
    {
        if (nullptr == model || model->size() < 10)
        {
            //return false;
        }
        isNewModel_ = true;
        PointCloudPtr tmp = boost::make_shared<PointCloud>(*model);
        //translateToCenter(tmp);
        modelCloud_ = tmp;
        kdtreeptr_ = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        kdtreeptr_->setInputCloud(modelCloud_);
        return true;

    }
    
    void LM_ICP::publish(const PointCloudConstPtr& data, const ros::Publisher& pub)
    {
        sensor_msgs::PointCloud2    cloud_msg;
        //ROS_INFO_STREAM(data->size());
        pcl::toROSMsg(*data, cloud_msg);
        cloud_msg.header.frame_id = "icp";
        cloud_msg.header.stamp = ros::Time::now();
      
        //pub_depth = nhTmpe.advertise<sensor_msgs::PointCloud2>("depth_point_cloud_visualservo", 2);
        pub.publish(cloud_msg);
        return;
    }
    void LM_ICP::publishModel(const Eigen::Matrix4d transform)
    {
        /*
        if (robotiq_model_publisher_.getNumSubscribers() < 1)
            return;

        visualization_msgs::MarkerArray model;
        visualization_msgs::Marker      marker;
        Eigen::Matrix3d m;
        m << transform(0,0), transform(0,1), transform(0,2),
         transform(1,0), transform(1,1), transform(1,2),
         transform(2,0), transform(2,1), transform(2,2);

        Eigen::Quaternion<double> q(m);
        for (int i = 0; i < model_filenames_.size(); i++)
        {

            //rotateOrientation(q, RotationAxis::X_axis_rotation, -M_PI_2);

            marker.header.frame_id = estimate_frame_id_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "estimated_" + estimate_frame_id_;
            marker.id = i;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = transform(0,3);
            marker.pose.position.y = transform(1,3);
            marker.pose.position.z = transform(2,3);
            marker.pose.orientation.w = q.w();
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration();

            marker.mesh_resource = model_filenames_.at(i);
            model.markers.push_back(marker);
        }

        robotiq_model_publisher_.publish(model);
        */
    }

   
    void LM_ICP::publishModelCloud(const PointCloudPtr &cloud)
    {
        /*
        sensor_msgs::PointCloud2    model_cloud_msg;

        pcl::toROSMsg(*cloud, model_cloud_msg);
        model_cloud_msg.header.frame_id = camera_frame_id_;
        model_cloud_msg.header.stamp = ros::Time::now();

        model_cloud_publisher_.publish(model_cloud_msg);
        */
    }

    void LM_ICP::poseToMatrix(gm::Pose &pose, E::Matrix4d &mat)
    {
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z);
        Eigen::Matrix3d m = q.toRotationMatrix();

        mat << m(0, 0), m(0, 1), m(0, 2), pose.position.x,
               m(1, 0), m(1, 1), m(1, 2), pose.position.y,
               m(2, 0), m(2, 1), m(2, 2), pose.position.z,
               0,       0,       0,       1;
    }

}
using namespace cxy;
typedef pcl::PointXYZ PointT;
pcl::PointCloud<PointT>::Ptr loadPlyFile(std::string name);
int main(int argc, char *argv[])
{
    ros::init(argc,argv, "lmicp");
    cxy_lmicp::LM_ICP icp;

    pcl::PointCloud<PointT>::Ptr data(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr model(new pcl::PointCloud<PointT>);

/*
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFile("bun045.ply",mesh);
    pcl::fromROSMsg(mesh.cloud, *data);
    pcl::io::loadPolygonFile("bun090.ply",mesh);
    pcl::fromROSMsg(mesh.cloud, *model);
*/
    //std::ifstream fin_tar("bun045.ply");
    //std::ifstream fin_mod("bun090.ply");
  pcl::PointCloud<PointT>::Ptr transPoint(new pcl::PointCloud<PointT>);
    
    data = loadPlyFile("/home/xiongyi/repo/bun000.ply");
    const int t = 2;
    if (0 == t)
    {
	model = loadPlyFile("/home/xiongyi/repo/bun045.ply");
    }
    else if (1 == t)
    {
	for (int i = 0; i < data->points.size(); ++i)
    	{
    		model->push_back(pcl::PointXYZ(data->points[i].x, data->points[i].y+0.01, data->points[i].z+0.01));
    	}
    }
    else if (2 == t)
    {
            cxy_transform::Pose<float> pose;
          pose.rotateByAxis(cxy_transform::Axis::X_axis_rotation, 30.0);

          pose.composePoint(data, transPoint);
          model = transPoint;
    }

    /*	*/
    //model->push_back(PointT(0.00, 0.00, 0.0));
    //model->push_back(PointT(0.20, 0.00, 0.0));
    //data->push_back(PointT(0.02, 0.01, 0.0));
    //data->push_back(PointT(0.22, 0.01, 0.0));
    //ROS_INFO("read pointcloud");

    icp.setModelCloud(model);
    icp.lmicp(data, model);
    //icp.lmicpNumerical(data, model);
    return 0;
}

pcl::PointCloud<PointT>::Ptr loadPlyFile(std::string name)
{


    pcl::PointCloud<PointT>::Ptr pointcloud(new pcl::PointCloud<PointT>);
    std::ifstream fin(name);
    ROS_INFO_STREAM(fin.is_open());
    std::string line;
    long int count(0);
    while (std::getline(fin, line))
    {
        //std::getline(fin, line, '\n');
        std::size_t pos(line.find("element vertex"));
//        ROS_INFO_STREAM(line);
        if ( std::string::npos != pos)
        {
            std::string tmp(line.begin()+pos+14, line.end());
            count = atol(tmp.c_str());
            //ROS_INFO_STREAM("read count  "<< "  "<<tmp << ".");
            ROS_INFO_STREAM(line <<std::endl);
            continue;
        }
        pos = line.find("end_header");
        if ( std::string::npos != pos)
        {
            //ROS_INFO("read end");
            break;
        }

    }
    std::cout<<"PointCloud Num   "<< count<<std::endl;
    pointcloud->reserve(count);
    for (int i = 0; i < count; ++i)
    {
        /* code */
        PointT p;
        fin >> p.x >> p.y >> p.z;
        pointcloud->push_back(p);
        //ROS_INFO_STREAM("points  "<< "  "<<p.x);

    }
    return pointcloud;
}

