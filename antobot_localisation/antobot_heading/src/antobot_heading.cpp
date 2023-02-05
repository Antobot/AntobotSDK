/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

Description: The primary purpose of this code is to calculate yaw offset from the robot motion with single GPS. 
            This script subscribes to the IMU and GPS topics and publishes Imu messages over 
		    imu/data_corrected topic. antobot_heading node performs the same functions as the antobot_heading_node.py but is written in C++.

Contacts: 	soyoung.kim@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/
#include <antobot_heading/antobot_heading.h>

namespace antobot_heading
{
    AmHeading::AmHeading(ros::NodeHandle& nh)
    : nh_(nh)
    {
        ROS_INFO("Initialise");
        initialise();
    }

    AmHeading::~AmHeading(){
        
    }

    void AmHeading::initialise(){
        // configuration parameters
        calib_distance = 1.0; // calibration distance 
        angular_zero_tol = 0.1; // Angular velocity (rad/sec)
        lin_tol = 0.01; // linear velocity x (m/s)
        calib_deg = 3; // deg

        // GPS
        gps_start.resize(2);
        gps_end.resize(2);
        gps_received = false;
        utm_y = 0;
        utm_x = 0;
        utm_zone = "";
        rtk_status = 0;

        // imu
        imu_frame = "imu";
        imu_received = false;
        imu_ang_vel_z = 0;
        imu_offset = 0;
        imu_calibration_status = -1;

        // odometry
        odometry_received = false;

        // wheel odometry
        wheel_odom_v = 0;
        
        // Other parameters
        direction = true; // default value
        gps_yaw = 0;

        // Read parameters to set the topic names
        nh_.getParam("/antobot_heading_node/gps_topic", gpsTopic);
        nh_.getParam("/antobot_heading_node/imu_topic", imuTopic);
        nh_.getParam("/antobot_heading_node/odometry_topic", odometryTopic);
        nh_.getParam("/antobot_heading_node/wheel_odometry_topic", wheelodometryTopic);

        nh_.getParam("/antobot_heading_node/new", new_gps);
        nh_.param<bool>("/simulation", sim, false);

        if (new_gps){
            ROS_INFO("use new am_gps_urcu");
            rtk_target_status = 3; // with the new code, status 2 is 3D fixed mode
            if (sim){
                rtk_target_status = 0; // in simulation, gps status is always 0
            }
        }    
        else{
            ROS_INFO("use old am_gps_urcu");
            rtk_statusTopic = "/am_gps_urcu_status";
            subRtkStatus = nh_.subscribe<std_msgs::UInt8>(rtk_statusTopic, 100, &AmHeading::rtkstatusCallback,this); // queue
            rtk_target_status = 4;
        }

        // Subscribers
        subGps = nh_.subscribe<sensor_msgs::NavSatFix>(gpsTopic, 100, &AmHeading::gpsCallback,this);
        subImu = nh_.subscribe<sensor_msgs::Imu>(imuTopic, 100, &AmHeading::imuCallback,this);
        subOdometry = nh_.subscribe<nav_msgs::Odometry>(odometryTopic, 100, &AmHeading::odometryCallback,this);
        subwheelOdom = nh_.subscribe<nav_msgs::Odometry>(wheelodometryTopic, 100, &AmHeading::wheelOdomCallback,this);


        // Publisher
        pubImu = nh_.advertise<sensor_msgs::Imu>("/imu/data_corrected", 1);
        pubImu_z = nh_.advertise<std_msgs::Float32>("/imu/heading_z", 1);
        pubImuOffset = nh_.advertise<std_msgs::Float32>("/imu/data_offset", 1);
        pubCalib = nh_.advertise<std_msgs::UInt8>("/imu_calibration_status", 1);

        // Timer
        autoCalibrationTimer = nh_.createTimer(ros::Duration(30), &AmHeading::AutoCalibrate, this); // every 10 secs
        imuPubTimer = nh_.createTimer(ros::Duration(1.0/10.0), &AmHeading::PublishNewIMU, this); // 10Hz 
    }   

    // class methods

    std::vector<double> AmHeading::eulerFromQuart(tf2::Quaternion q){
        tf2::Matrix3x3 m(q);
        double r,p,y;
        m.getRPY(r,p,y);
        std::vector<double> v;
        v.push_back(r);
        v.push_back(p);
        v.push_back(y);
        return v; 
    }

    tf2::Quaternion AmHeading::quartFromEuler(double a, double b, double c){
        tf2::Quaternion q;
        q.setRPY(a,b,c);
        q =q.normalize();
        return q;
    }

    bool AmHeading::SaveStartGPS(void){
        double vel = wheel_odom_v; // to use consistent value
        if ((abs(imu_ang_vel_z) < angular_zero_tol) && (abs(vel) > lin_tol)){
            gps_start.clear();
            gps_start.push_back(utm_x);
            gps_start.push_back(utm_y); // utm zone is not used for calculation - check needed
            if (vel > 0){
                direction = true; // forward
            }
            else{
                direction = false; // backward
            }
            return true;
        }   
        else{
            return false;
        }
    }

    int AmHeading::CheckCondition(void){
        // Reset if angular vel is large
        if (abs(imu_ang_vel_z) > angular_zero_tol){
            ROS_INFO("Reset due to angular velocity");
            return 1;
        }

        bool curret_dir; 
        // Reset if direction is changed
        if (wheel_odom_v > lin_tol){
            curret_dir = true; // forward
        }
        else if (abs(wheel_odom_v) <= lin_tol){
            curret_dir = direction; // very small velocity - hasn't changed the direction
        } 
        else{
            curret_dir = false; // backward
        }
            
        if (curret_dir != direction){
            ROS_INFO("Reset due to change in direction");
            return 1;
        }

        // Check the distance
        gps_end.clear();
        gps_end.push_back(utm_x);
        gps_end.push_back(utm_y); 

        double dx, dy, d;
        if (direction){ // forward
            dy = gps_end[1] - gps_start[1];
            dx = gps_end[0] - gps_start[0];
        } 
        else{ // backward
            dy = gps_start[1] - gps_end[1];
            dx = gps_start[0] - gps_end[0];
        }     

        d = sqrt(dx*dx + dy*dy);   // distance between start and end points of calibration
        gps_yaw = atan2(dy, dx);   // calculate yaw from gps start and end points

        if (d >= calib_distance){
            ROS_INFO("calibration distance satisfied");
            return 3;
        }
        else{
            return 2; // not satisfied
        }
    }

    void AmHeading::InitialCalibration(void){
        while ((!imu_received) || (!gps_received)){
            ROS_INFO("Waiting for initial imu and gps data new");
            ros::Duration(0.1).sleep();
            ros::spinOnce(); // Initial Calibration runs before spin() in main loop
        }
            
        ROS_INFO("IMU and gps value received, check for RTK status");

        while (rtk_status != rtk_target_status){
            ROS_INFO("Waiting for RTK status to become %d",rtk_target_status);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
            
        ROS_INFO("RTK status is %d - start calibration",rtk_target_status);

        int state = 1;
        while (true){
            ros::spinOnce();
            if (state == 1){
                // moving forward/backward, no angular : set start gps point
                if (SaveStartGPS()){
                    state = 2;
                }
            }
            else if (state == 2){
                state = CheckCondition(); // returns the state
            }
            else if (state == 3){ // use gps_yaw
                break;
            }
            ros::Duration(0.1).sleep(); 
        }

        ROS_INFO("gps_yaw is %f", gps_yaw); // wrt map x frame
        ros::spinOnce();
        tf2::Quaternion quat_tf;
        // Convert  ROS Quaternion to tf2 Quaternion msg
        tf2::convert(q_imu , quat_tf);
        // Convert from Quaternion to euler
        std::vector<double> result = eulerFromQuart(quat_tf); // result[2] is yaw angle
        // imu_yaw + imu_offset = gps_yaw
        imu_offset = gps_yaw - result[2];  // difference between orientations from imu and gps
        ROS_INFO("calibration successful (offset = %f degs)", imu_offset/M_PI*180.0); // wrt map x frame

        imu_calibration_status = 1;

        // Let heading launcher know the inital calibration is finished 
        std_msgs::UInt8 msg;
        msg.data = imu_calibration_status;
        pubCalib.publish(msg);
        ROS_INFO("Initial calibration finished");
    }


    void AmHeading::AutoCalibrate(const ros::TimerEvent& event){
        ROS_INFO("AutoCalibration called");
        ros::Time started_time =ros::Time::now();

        if (imu_calibration_status == 1 && odometry_received){
            ROS_INFO("auto calibration called but skip one time");
            imu_calibration_status = 2;
        }  
        else if (imu_calibration_status == 2){ // depending on the timer setting, run this periodically 
            ROS_INFO("auto calibration checking started");
            int state = 0;

            while(true){
                ros::spinOnce();
                if (state == 0){
                    ROS_INFO("Check if rtk status and odometry from EKF");
                    if (rtk_status == rtk_target_status && odometry_received){
                        state = 1;
                    }
                }
                else if (state == 1){
                    // moving forward/backward, no angular : set start gps point
                    if (SaveStartGPS()){
                        state = 2;
                    }
                }
                else if (state == 2){
                    state= CheckCondition(); // returns the status
                }
                else if (state == 3){
                    // Calibrate 
                    // gps angle calculated 
                    ROS_INFO("gps angles = %f", gps_yaw);

                    tf2::Quaternion quat_tf_imu;
                    tf2::convert(q_imu , quat_tf_imu);
                    std::vector<double> result_imu = eulerFromQuart(quat_tf_imu); // returns r,p,y

                    tf2::Quaternion quat_tf_odom;
                    tf2::convert(q_odom , quat_tf_odom);
                    std::vector<double> result_odom = eulerFromQuart(quat_tf_odom);

                    // imu_yaw + imu_offset = gps_yaw
                    // Compare two angles in [-pi,pi] and returns signed value in radian
                    double diff = gps_yaw - result_odom[2];
                    if (diff > M_PI){
                        diff -= 2*M_PI;
                    }
                    else if (diff < -M_PI){
                        diff += 2*M_PI;
                    }
                    ROS_INFO("diff = %f", diff);
                    double diff_deg = abs(diff*180.0/M_PI);
                    ROS_INFO("angle diff = %f",diff_deg);

                    if (diff_deg > calib_deg){
                        double imu_offset_tmp = gps_yaw - result_imu[2];  // difference between orientations from imu and gps
                        if (imu_offset_tmp > M_PI){
                            imu_offset_tmp -= 2*M_PI;
                        }
                        else if (imu_offset_tmp < -M_PI){
                            imu_offset_tmp += 2*M_PI;
                        }
                        imu_offset = imu_offset_tmp;
                        ROS_INFO("auto-calibration successful (imu offset = %f degs)", imu_offset / M_PI * 180.0);
                    }
                    else{
                        ROS_INFO("auto-calibration not required %f deg",diff_deg);
                    }
                    break; // Auto calibration finished
                }

                double duration = (ros::Time::now()- started_time).toSec();
                if (duration > 20.0){ // Every time Auto calibration is called, it will check for 20 seconds.
                    ROS_INFO("auto calibration conditions not met - cancelled");
                    break;
                }
                ros::Duration(0.1).sleep();
            }
        }
        else{
            ROS_INFO("auto calibration called but ignored - Do initial calibration first!");
        }

    }

    void AmHeading::PublishNewIMU(const ros::TimerEvent& event){
        if (imu_calibration_status > 0){
            tf2::Quaternion quat_tf;
            geometry_msgs::Quaternion quat_msg;

            // Convert  ROS Quaternion to tf2 Quaternion msg
            tf2::convert(q_imu , quat_tf);
            // Convert from Quaternion to euler
            std::vector<double> result = eulerFromQuart(quat_tf);
            // Add imu offset and convert back to Quaternion  
            tf2::Quaternion result_tf =  quartFromEuler(result[0],result[1],result[2]+imu_offset);

            std_msgs::Float32 heading_z_msg;
            heading_z_msg.data = result[2]+imu_offset;
            pubImu_z.publish(heading_z_msg);

            // Convert tf2 Quaternion to ROS Quaternion
            tf2::convert(result_tf ,quat_msg);
            
            // create imu message
            sensor_msgs::Imu imuMsg;
            imuMsg.header.stamp = ros::Time::now();
            imuMsg.header.frame_id = imu_frame;
            imuMsg.orientation.x = quat_msg.x;
            imuMsg.orientation.y = quat_msg.y;
            imuMsg.orientation.z = quat_msg.z;
            imuMsg.orientation.w = quat_msg.w;
            
            imuMsg.angular_velocity.z = imu_ang_vel_z;  // for EKF inupt

            // publish /imu/data_corrected
            pubImu.publish(imuMsg);

            // Publsih /imu/data_offset
            std_msgs::Float32 imu_offset_msg;
            imu_offset_msg.data = (float)imu_offset;
            pubImuOffset.publish(imu_offset_msg);
        }       
    }


    // Callback functions

    void AmHeading::rtkstatusCallback(const std_msgs::UInt8::ConstPtr& msg){
        //ROS_INFO("rtk callback");
        rtk_status = msg->data;
    }

    void AmHeading::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
        // UTMNorthing, UTMEasting, UTMZone
        LLtoUTM(msg->latitude, msg->longitude, utm_y, utm_x, utm_zone); //convert from gps to utm coordinates  
        if (new_gps){
            rtk_status = msg->status.status;
        } 

        if (!gps_received){
            gps_received = true;
        }
    }

    void AmHeading::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
        q_imu = msg->orientation;
        imu_ang_vel_z = msg->angular_velocity.z; //value used for checking angular vel
        
        if (!imu_received){
            imu_received = true;
        }
    }

    void AmHeading::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
        //EKF odometry orientation is the same as calibrated imu's orientation
        q_odom = msg->pose.pose.orientation; // value to compare with gps yaw degree
        if (!odometry_received){
            odometry_received = true;
        }
    }

    void AmHeading::wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        wheel_odom_v = msg->twist.twist.linear.x; // value used for checking linear vel
        
    }

    


    
}
