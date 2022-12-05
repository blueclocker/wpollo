/*
 * @Author: blueclocker 1456055290@hnu.edu.cn
 * @Date: 2022-11-06 19:33:07
 * @LastEditTime: 2022-11-09 14:02:09
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 
 * @FilePath: /wpollo/src/lanelet/osmmap/src/navagation/navagation_gnss.cpp
 * Copyright (c) 2022 by bit, All Rights Reserved. 
 */
#include "navagation/navagation_gnss.h"


namespace navagation
{
NavagationGnss::NavagationGnss(ros::NodeHandle &n) : NavagationBase(n)
{
    imucount_ = 0;
    imuinit_flag_ = false;
    gps_sub_ = n_.subscribe("/comb", 10, &NavagationGnss::GpsCallback, this);
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> NavagationGnss::GetSkewMatrix(const Eigen::MatrixBase<Derived> &v)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> w;
    w << 0., -v(2), v(1),
        v(2), 0., -v(0),
        -v(1), v(0), 0.;
    return w;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> NavagationGnss::Amatrix(const Eigen::MatrixBase<Derived> &v)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> w = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity();
    typename Derived::Scalar norm = v.norm();
    Eigen::Matrix<typename Derived::Scalar, 3, 3> skew = GetSkewMatrix(v / norm);

    if (norm < 10e-5)
    {
        return w;
    }

    return Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + (1 - cos(norm)) / norm * skew + (1 - sin(norm) / norm) * skew * skew;
}

void NavagationGnss::ImuInit(const Eigen::Vector3d &imuMsg)
{
    if (++imucount_ < 101)
    {
        Eigen::Vector3d acc{imuMsg[0], imuMsg[1], imuMsg[2]};
        sum_acc_ += acc;
    }
    else if (!imuinit_flag_)
    {
        avg_acc_ = sum_acc_ / 100.0;
        std::cout << "avg acc:  " << avg_acc_.transpose().x() << "  " << avg_acc_.transpose().y()
                  << "  " << avg_acc_.transpose().z() << std::endl;
        double norm = avg_acc_.norm();
        avg_acc_ /= norm;
        Eigen::Vector3d gravity_acc(Eigen::Vector3d(0, 0, 1));
        double cos_value = avg_acc_.dot(gravity_acc);
        double angle = acos(cos_value);
        Eigen::Vector3d axis = GetSkewMatrix(avg_acc_) * gravity_acc;
        // rot = Amatrix(angle * axis);
        Eigen::AngleAxisd rot_vec(angle, axis.normalized());
        rot_ = rot_vec.toRotationMatrix();
        std::cout << "rot: " << rot_ << std::endl;
        imuinit_flag_ = true;
    }
    else
    {
        Eigen::Vector3d acc{imuMsg[0], imuMsg[1], imuMsg[2]};
        adjusted_acc_ = rot_ * acc;
        std::cout << "adjust acc:  " << adjusted_acc_.transpose().x() << "  " << adjusted_acc_.transpose().y()
                  << "  " << adjusted_acc_.transpose().z() << std::endl;
    }
}

void NavagationGnss::GpsCallback(const fsd_common_msgs::Comb::ConstPtr &msg)
{
    // GNSS
    atnowpoint_->elevation_ = msg->Altitude;
    atnowpoint_->latitude_ = msg->Lattitude;
    atnowpoint_->longitude_ = msg->Longitude;
    vectormap_->GPS2Localxy(atnowpoint_);

    start_state_[0] = atnowpoint_->local_x_;
    start_state_[1] = atnowpoint_->local_y_;
    start_state_[2] = ConstrainAngle(msg->Heading)/180*M_PI;

    //imu矫正
    Eigen::Vector3d imuinmsg;
    imuinmsg << msg->Ax, -msg->Az, msg->Ay;
    ImuInit(imuinmsg);

    //当前点定位到路段
    map::centerway::CenterPoint3D atnowcenterpoint = map::centerway::CenterPoint3D(*atnowpoint_);
    //当前路段精确定位到某点
    //int atnowcenterway = globalplans_->InWhichCenterway(atnowcenterpoint, nodesptr, waysptr, relationsptr);
    std::vector<int> lanelets_res = globalplans_->LocateLanelets(atnowcenterpoint, nodesptr_, waysptr_, relationsptr_);
    //ROS_INFO("lanelets number is %d!", (int)lanelets_res.size());

    //HDmap state
    osmmap::CarState hdmapstate;
    hdmapstate.header.frame_id = "map";
    hdmapstate.header.stamp = ros::Time::now();
    hdmapstate.inMap = false;
    hdmapstate.isEndExist = false;
    hdmapstate.isFindPath = false;
    hdmapstate.laneletid = 0;
    //plan
    //visualmap->pathmarkerclear();
    path_markerarray_.markers.clear();
    if(!lanelets_res.empty())
    {
        hdmapstate.inMap = true;

        //融合上一帧路径
        if(paths_.empty())
        {
            //第一帧，优先选取第一个定位结果
            start_path_ = lanelets_res[0];
            // for(int i = 0; i < lanelets_res.size(); i++)
            // {
            //     start_path_ = lanelets_res[i];
            //     if(!globalplans_->Run(start_path_, end_path_).empty())
            //     {
            //         break;
            //     }
            // }
            std::cout << "first plan!" << std::endl;
        }else{
            //其他帧，优先选取上一帧路径lanelet范围内的结果
            bool isInLastPaths = false;
            for(int i = 0; i < lanelets_res.size(); i++)
            {
                if(std::find(paths_.begin(), paths_.end(), lanelets_res[i]) != paths_.end())
                {
                    start_path_ = lanelets_res[i];
                    isInLastPaths = true;
                    std::cout << "select last frame lanelet id successed!" << std::endl;
                    break;
                }
            }
            //如果均不在上一帧lanelet范围内, 优先选取第一个
            if(!isInLastPaths)
            {
                // for(int i = 0; i < lanelets_res.size(); i++)
                // {
                //     start_path_ = lanelets_res[i];
                //     if(!globalplans_->Run(start_path_, end_path_).empty())
                //     {
                //         break;
                //     }
                // }
                start_path_ = lanelets_res[0];
            }
        }
        // start_path_ = atnowcenterway;
        
        hdmapstate.laneletid = start_path_;
        start_centerpoint_id_ = globalplans_->AtWhichPoint(atnowcenterpoint, centerwaysptr_->Find(start_path_));
        if(isend_path_exist_)
        {
            hdmapstate.isEndExist = true;
            //visualmap->pathmarkerclear();
            paths_.clear();
            paths_ = globalplans_->Run(start_path_, end_path_);
            if(!paths_.empty())
            {
                hdmapstate.isFindPath = true;
                visualmap_->Path2Marker(centerwaysptr_, paths_, path_markerarray_, currenttime_);
                PushCenterPoint(paths_);
                //smooth
                SmoothPath();
                visualmap_->Smoothpath2Marker(smoothpathnode_, path_markerarray_, currenttime_);
                //发布导航信息
                FullNavigationInfo();
                navigation_pub_.publish(laneletinfo_);
                //发路径
                visualization_msgs::Marker golbalpath = path_markerarray_.markers.back();
                golbalpath_pub_.publish(golbalpath);
                //std::cout << "path_markerarray size is " << path_markerarray.markers.size() << std::endl;
                //发下一个车道
                FullLanesInfo(start_path_);
                lanes_pub_.publish(lanesinfo_);
            }else{
                smoothpathnode_.clear();
                smoothpathnode_.push_back(map::centerway::CenterPoint3D(*atnowpoint_));
                // double atnowpoint_heading = ConstrainAngle(msg->Heading)/180*M_PI;
                double atnowpoint_heading = start_state_[2];
                OutMapPlan(atnowcenterpoint, atnowpoint_heading);
                ROS_WARN("gps point is in reverse direction path!");
            }

            //该点到下一个路口(下一次转向)距离测试
            //double intersectiondis = centerways->length2intersection(start_centerpoint_id_, paths, relations);
            //std::cout << "distance to next intersection is " << intersectiondis << std::endl;
            std::cout << "---------------------------------------------------" << std::endl;

        }else{
            ROS_WARN("goal point has not set!");
        } 
    }else{
        //越界：直接连接当前点与最近的道路中心点！！
        smoothpathnode_.clear();
        smoothpathnode_.push_back(map::centerway::CenterPoint3D(*atnowpoint_));
        // double atnowpoint_heading = ConstrainAngle(msg->Heading)/180*M_PI;
        double atnowpoint_heading = start_state_[2];
        
        // int nearestcenterwayid = centerwaysptr->findNearestCenterwaypointid(atnowpoint);
        // centerway::CenterPoint3D tartgetpoint;
        // centerwaysptr->nextCenterwayPointid(nearestcenterwayid, tartgetpoint);
        // smoothpathnode.push_back(tartgetpoint);

        // visualmap->smoothpath2marker(smoothpathnode, path_markerarray, currenttime);
        // //发路径
        // visualization_msgs::Marker golbalpath_ = path_markerarray.markers.back();
        // golbalpath_pub.publish(golbalpath_);
        OutMapPlan(atnowcenterpoint, atnowpoint_heading);
        
        ROS_WARN("gps point out of HD-map!");
    }

    //tf
    // double headtemp = msg->Heading;
    // headtemp = ConstrainAngle(headtemp);
    // double headtemp = start_state_[2];
    // std::cout << "headtemp: " << headtemp << std::endl;
    tf::Quaternion qenu2base;
    //GNSS
    qenu2base.setRPY((msg->Roll)/180*M_PI, (msg->Pitch)/180*M_PI, start_state_[2]);
    baselink2map_.setRotation(qenu2base);
    baselink2map_.setOrigin(tf::Vector3(atnowpoint_->local_x_, atnowpoint_->local_y_, atnowpoint_->elevation_));
    //map->rslidar->base_link
    broadcaster_.sendTransform(tf::StampedTransform(baselink2map_, ros::Time::now(), "map", "rslidar"));
    std::cout << "atnowpoint update" << std::endl;
    std::cout << "now x: " << atnowpoint_->local_x_ << ", now y: " << atnowpoint_->local_y_ << ", now z: " << atnowpoint_->elevation_ << std::endl;

    //Ve, Vn, Vu 转化为车辆坐标系速度
    Eigen::Matrix3d m3d;
    m3d << 1, 0, 0, 0, 0, -1, 0, -1, 0;
    Eigen::Quaterniond qbase2imu(m3d);
    Eigen::Quaterniond q1 = Eigen::Quaterniond(qenu2base.w(), qenu2base.x(), qenu2base.y(), qenu2base.z()).normalized();
    Eigen::Vector3d Venu = Eigen::Vector3d(msg->Ve, msg->Vn, msg->Vu);
    Eigen::Vector3d Vxyz = (q1*qbase2imu).normalized().inverse() * Venu;

    //debug
    // std::cout << "vx=" << Vxyz(0) << ", vy=" << Vxyz(1) << ", vz=" << Vxyz(2) << std::endl;
    // std::cout << "ax=" << msg->Ax << ", ay=" << msg->Ay << ", az=" << msg->Az << std::endl;
    // double asum = std::sqrt(msg->Ax * msg->Ax + msg->Ay * msg->Ay + msg->Az * msg->Az);
    // std::cout << "asum=" << asum << std::endl;

    //欧拉角转四元数
    // double siny_cosp = +2.0 * (q1.w() * q1.z() + q1.x() * q1.y());
    // double cosy_cosp = +1.0 - 2.0 * (q1.y() * q1.y() + q1.z() * q1.z());
    // double yaw = atan2(siny_cosp, cosy_cosp);
    // std::cout << "head: " << (headtemp)/180*3.14159 << ", yaw: " << yaw << std::endl;

    //gps path
    gpspath_.header.frame_id = "map";
    gpspath_.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = atnowpoint_->local_x_;
    pose_stamped.pose.position.y = atnowpoint_->local_y_;
    pose_stamped.pose.position.z = atnowpoint_->elevation_;
    pose_stamped.pose.orientation.x = qenu2base.x();
    pose_stamped.pose.orientation.y = qenu2base.y();
    pose_stamped.pose.orientation.z = qenu2base.z();
    pose_stamped.pose.orientation.w = qenu2base.w();
    gpspath_.poses.push_back(pose_stamped);
    gpspath_pub_.publish(gpspath_);

    //car state
    hdmapstate.heading = start_state_[2];
    hdmapstate.carPose.position.x = atnowpoint_->local_x_;
    hdmapstate.carPose.position.y = atnowpoint_->local_y_;
    hdmapstate.carPose.position.z = atnowpoint_->elevation_;
    hdmapstate.carPose.orientation.x = qenu2base.x();
    hdmapstate.carPose.orientation.y = qenu2base.y();
    hdmapstate.carPose.orientation.z = qenu2base.z();
    hdmapstate.carPose.orientation.w = qenu2base.w();
    hdmapstate.endPose = endPose_;
    hdmapstate.nextpoint.x = 0;
    hdmapstate.nextpoint.y = 0;
    hdmapstate.nextpoint.z = 0;

    //给hybid A*终点
    // if(smoothpathnode_.size() >= 2) 
    // {
    //     plan::Spline2D csp_obj(smoothpathnode_);
    //     if(csp_obj.s.back() >= 6)
    //     {
    //         std::array<double, 3> point_ = csp_obj.calc_postion(6);
    //         hdmapstate.nextpoint.x = point_[0];
    //         hdmapstate.nextpoint.y = point_[1];
    //         hdmapstate.nextpoint.z = csp_obj.calc_yaw(6);
    //     }
    // }

    //GNSS
    hdmapstate.linear.x = Vxyz(0);
    hdmapstate.linear.y = Vxyz(1);
    hdmapstate.linear.z = Vxyz(2);
    hdmapstate.Accell.x = msg->Ax;
    hdmapstate.Accell.y = msg->Ay;
    hdmapstate.Accell.z = msg->Az;
    if(imuinit_flag_)
    {
        hdmapstate.Accell.x = adjusted_acc_[0];
        hdmapstate.Accell.y = adjusted_acc_[1];
        hdmapstate.Accell.z = adjusted_acc_[2];
    }
    carstate_pub_.publish(hdmapstate);

    //发布速度
    std_msgs::Float32 sp;
    sp.data = hdmapstate.linear.x;
    speed_pub_.publish(std::move(sp));
}

void NavagationGnss::Process()
{
    ROS_INFO("\033[1;32m --> GNSS mode is working ...  \033[0m\n");
    //main loop
    ros::Rate r(10);
    while(n_.ok())
    {
        //visualmap->run();
        currenttime_ = ros::Time::now();
        map_pub_.publish(map_markerarray_);
        path_pub_.publish(path_markerarray_);
        // gridmapmsg.header.stamp = currenttime;
        // gridmap_pub.publish(gridmapmsg);
        // navigation_pub.publish(laneletinfo);
        //到达终点退出程序
        if(start_centerpoint_id_ == end_centerpoint_id_)
        {
            std::cout << "***************************************************" << std::endl;
            std::cout << std::endl;
            std::cout << "               arrvied at goal point               " << std::endl;
            std::cout << std::endl;
            std::cout << "***************************************************" << std::endl;
            break;
        }
        ros::spinOnce();
        r.sleep();
    }
}

}// namespace navagation