/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
# Description: The primary purpose of this code is to calculate yaw offset from the robot motion with single GPS. 
#             This script subscribes to the IMU and GPS topics and publishes Imu messages over 
# 		    imu/data_corrected topic. antobot_heading node performs the same functions as the antobot_heading_node.py but is written in C++.
# Subscribes to: GPS topic (/antobot_gps_urcu)
#                imu topic (/imu/data)
#                EKF odometry topic (/odometry/filtered)
#                wheel odometry topic (/antobot_robot/odom)
# Publishes : Calibrated imu topic (/imu/data_corrected) - which is then used in EKF
# Contacts: 	soyoung.kim@antobot.ai
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/

#include <antobot_heading/antobot_heading.h>

namespace antobot_heading
{
    amHeading::amHeading(ros::NodeHandle& nh)
    : nh_(nh)
    {   
        // Description: Constructor for the amHeading class
        ROS_INFO("Initialise");
        initialise();
    }

    amHeading::~amHeading(){
        // Description: Deconstructor for the amHeading class
    }

    void amHeading::initialise(){
        // Description: Initialises autoCalibration class

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
        rtk_status = -1; // default value

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
        direction = true; // true if the robot is moving forward, false if the robot is moving backward
        gps_yaw = 0;

        // Read parameters to set the topic names
        nh_.getParam("/antobot_heading_node/gps_topic", gps_topic);
        nh_.getParam("/antobot_heading_node/imu_topic", imu_topic);
        nh_.getParam("/antobot_heading_node/odometry_topic", odometry_topic);
        nh_.getParam("/antobot_heading_node/wheel_odometry_topic", wheelodometry_topic);

        nh_.param<bool>("/simulation", sim, false);

        rtk_target_status = 3; // rtk status is 3 in 3D fixed mode
        if (sim){
            rtk_target_status = 0; // in simulation, gps status is always 0
        }

        // Subscribers
        sub_gps = nh_.subscribe<sensor_msgs::NavSatFix>(gps_topic, 100, &amHeading::gpsCallback,this);
        sub_imu = nh_.subscribe<sensor_msgs::Imu>(imu_topic, 100, &amHeading::imuCallback,this);
        sub_odometry = nh_.subscribe<nav_msgs::Odometry>(odometry_topic, 100, &amHeading::odometryCallback,this);
        sub_wheel_odom = nh_.subscribe<nav_msgs::Odometry>(wheelodometry_topic, 100, &amHeading::wheelOdomCallback,this);

        // Publisher
        pub_imu = nh_.advertise<sensor_msgs::Imu>("/imu/data_corrected", 1);
        pub_imu_z = nh_.advertise<std_msgs::Float32>("/imu/heading_z", 1); // For opcua
        pub_imu_offset = nh_.advertise<std_msgs::Float32>("/imu/data_offset", 1); // For debug
        pub_calib = nh_.advertise<std_msgs::UInt8>("/imu_calibration_status", 1); // For heading_launcher. Notify when the initial calibraion is done.

        // Service
        ekf_srv = nh_.serviceClient<std_srvs::Trigger>("launch_ekf"); // launch ekf nodes once the initial calibration is done

        // Timer
        auto_calibration_timer = nh_.createTimer(ros::Duration(30), &amHeading::autoCalibrate, this); // every 10 secs
        imu_pub_timer = nh_.createTimer(ros::Duration(1.0/10.0), &amHeading::publishNewIMU, this); // 10Hz 
    }   

    // class methods

    std::vector<double> amHeading::eulerFromQuat(tf2::Quaternion q){
        // Description: Convert tf2::Quaternion to euler and return vector<double> that is filled with euler values. 
        // Input: tf2::Quaternion q
        // Output: vector with euler value - roll, pitch, yaw 

        tf2::Matrix3x3 m(q);
        double r,p,y;
        m.getRPY(r,p,y);
        std::vector<double> v;
        v.push_back(r);
        v.push_back(p);
        v.push_back(y);
        return v; 
    }

    tf2::Quaternion amHeading::quatFromEuler(double a, double b, double c){
        // Description: Convert euler values to tf2::Quaternion 
        // Input: euler value - roll, pitch, yaw 
        // Returns: tf2::Quaternion

        tf2::Quaternion q;
        q.setRPY(a,b,c);
        q =q.normalize();
        return q;
    }

    bool amHeading::saveStartGPS(void){
        // Description: Save gps point as the starting point if the robot is moving. Set direction depending on the linear x velocity.
        // Robot wheel odometry's linear x should be larger than lin_tol and the imu angular z velocity should be smaller than angular_zero_tol.
        // direction is set to true when the robot is moving forward.
        // Returns: True if the start GPS point is saved, False if the robot velcoity didn't meet the conditons and failed to save the start GPS point.

        double vel = wheel_odom_v; 
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

    int amHeading::checkCondition(void){
        // Description: Check several conditions in order to start calibration and returns the state that will be used in the while loop that called this function.
        // Condition: 1. imu angular z velocity should be smaller than angular_zero_tol.
        //            2. Robot's direction (forward/backward) should be consistent and hasn't change since the saving of the staring GPS point.
        //            3. Robot has moved more than calib_distance (calculated based on gps lat/long data)
        // Returns: state that will be used in the while loop that called this function
        //          state 1: Reset and save another GPS start point
        //          state 2: Distance moved is not enough, continue to call checkCondition
        //          state 3: Distance moved is larger than calib_distance, now exit the while loop and use the gps_yaw in the next step of the calibration.

        // Reset if angular vel is large
        if (abs(imu_ang_vel_z) > angular_zero_tol){
            ROS_INFO("Reset due to angular velocity");
            return 1;
        }

        bool current_dir; 
        // Reset if direction is changed
        if (wheel_odom_v > lin_tol){
            current_dir = true; // forward
        }
        else if (abs(wheel_odom_v) <= lin_tol){
            current_dir = direction; // very small velocity - hasn't changed the direction
        } 
        else{
            current_dir = false; // backward
        }
            
        if (current_dir != direction){
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

    void amHeading::checkInputs(void){
        // Description: Check if the imu, gps data are recieved and check if the RTK status is fixed mode value

        while ((!imu_received) || (!gps_received)){
            ROS_INFO("Waiting for initial imu and gps data");
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
    }

    void amHeading::initialCalibration(void){
        // Description: Function that is called once in the main function. When all the required topics are recieved (self.checkInputs)
        // this function performs the initial calibration. In the while loop, the starting gps postion is saved and when the 
        // Calibration conditions are met, the imu_offset is calculated. 

        // Check if all input toics are being published 
        checkInputs();

        int state = 1;
        while (true){
            ros::spinOnce();
            if (state == 1){
                // moving forward/backward, no angular : set start gps point
                if (saveStartGPS()){
                    state = 2;
                }
            }
            else if (state == 2){
                state = checkCondition(); // returns the state
            }
            else if (state == 3){ // use gps_yaw
                break;
            }
            ros::Duration(0.1).sleep(); 
        }

        ROS_INFO("gps_yaw is %f", gps_yaw); 
        ros::spinOnce();
        tf2::Quaternion quat_tf;
        // Convert  ROS Quaternion to tf2 Quaternion msg
        tf2::convert(q_imu , quat_tf);
        // Convert from Quaternion to euler
        std::vector<double> result = eulerFromQuat(quat_tf); // result[2] is yaw angle
        // imu_yaw + imu_offset = gps_yaw
        imu_offset = calculateDifference(gps_yaw, result[2]); // difference between orientations from imu and gps
        ROS_INFO("calibration successful (offset = %f degs)", imu_offset/M_PI*180.0); // wrt map x frame

        // Let heading launcher know the inital calibration is finished 
        imu_calibration_status = 1;
        std_msgs::UInt8 msg;
        msg.data = imu_calibration_status;
        pub_calib.publish(msg);
        ROS_INFO("Initial calibration finished - now launch ekf nodes");

        std_srvs::Trigger srv;
        ekf_srv.call(srv);

    }

    double amHeading::calculateDifference(double input_a, double input_b){
        // Description: Compare two values in [-pi,pi] and return signed value in the same range
        // Input: Two double values in [-pi,pi]
        // Returns: Difference between two input values in [-pi,pi] 
        double diff = input_a - input_b;
        if (diff > M_PI){
            diff -= 2*M_PI;
        }
        else if (diff < -M_PI){
            diff += 2*M_PI;
        }
        return diff;
    }

    void amHeading::autoCalibrate(const ros::TimerEvent& event){
        // Description: Function that is called every 30 seconds to check if the calibration is needed. 
        // In the while loop, several conditions are checked and if theses conditions are met within 20 seconds, the new imu_offset is calculated (calibration)
        
        ROS_INFO("AutoCalibration called");
        ros::Time started_time =ros::Time::now();

        if (imu_calibration_status == 1 && odometry_received){
            ROS_INFO("auto calibration called but skip one time"); // Since the initial calibration was done recently
            imu_calibration_status = 2;
        }  
        else if (imu_calibration_status == 2){
            ROS_INFO("auto calibration checking started");
            int state = 0;

            while(true){
                ros::spinOnce();
                if (state == 0){ // Check rtk status and ekf odometry
                    if (rtk_status == rtk_target_status && odometry_received){
                        state = 1;
                    }
                }
                else if (state == 1){ // Save start gps position
                    if (saveStartGPS()){ // If the first gps position is saved
                        state = 2;
                    }
                }
                else if (state == 2){ // Check if the auto calibration conditions are met
                    state= checkCondition(); // returns the state
                }
                else if (state == 3){ // Conditions met, start the calibration
                    // gps angle calculated 
                    ROS_INFO("gps angles = %f", gps_yaw);

                    tf2::Quaternion quat_tf_imu;
                    tf2::convert(q_imu , quat_tf_imu);
                    std::vector<double> result_imu = eulerFromQuat(quat_tf_imu); // returns r,p,y

                    tf2::Quaternion quat_tf_odom;
                    tf2::convert(q_odom , quat_tf_odom);
                    std::vector<double> result_odom = eulerFromQuat(quat_tf_odom);

                    // imu_yaw + imu_offset = gps_yaw
                    // Compare two angles in [-pi,pi] and returns signed value in radian
                    double diff = calculateDifference(gps_yaw,result_odom[2]);
                    ROS_INFO("diff = %f", diff);
                    double diff_deg = abs(diff*180.0/M_PI);
                    ROS_INFO("angle diff = %f",diff_deg);

                    if (diff_deg > calib_deg){
                        double imu_offset_tmp = calculateDifference(gps_yaw, result_imu[2]); // difference between orientations from imu and gps
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

    void amHeading::publishNewIMU(const ros::TimerEvent& event){
        // Description: Publish calibrated imu data (/imu/dadta_coreccted) and a topic for debugging purpose (/imu/data_offset)
        // Input: imu raw data and calculated imu_offset. imu_offset is added to the yaw value of the imu data. 

        if (imu_calibration_status > 0){
            tf2::Quaternion quat_tf;
            geometry_msgs::Quaternion quat_msg;

            // Convert  ROS Quaternion to tf2 Quaternion msg
            tf2::convert(q_imu , quat_tf);
            // Convert from Quaternion to euler
            std::vector<double> result = eulerFromQuat(quat_tf);
            // Add imu offset and convert back to Quaternion  
            tf2::Quaternion result_tf =  quatFromEuler(result[0],result[1],result[2]+imu_offset);

            std_msgs::Float32 heading_z_msg;
            heading_z_msg.data = result[2]+imu_offset;
            pub_imu_z.publish(heading_z_msg);

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
            pub_imu.publish(imuMsg);

            // Publsih /imu/data_offset
            std_msgs::Float32 imu_offset_msg;
            imu_offset_msg.data = (float)imu_offset;
            pub_imu_offset.publish(imu_offset_msg);
        }       
    }

    // Callback functions

    void amHeading::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
        // Description: GPS callback function that gets utm coordinates(from lat, long values) and rts status

        // UTMNorthing, UTMEasting, UTMZone
        LLtoUTM(msg->latitude, msg->longitude, utm_y, utm_x, utm_zone); //convert from gps to utm coordinates  
        rtk_status = msg->status.status;
        if (!gps_received){
            gps_received = true;
        }
    }

    void amHeading::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
        // Description: IMU callback function (IMU sensor raw data)

        q_imu = msg->orientation;
        imu_ang_vel_z = msg->angular_velocity.z; //value used for checking angular vel
        if (!imu_received){
            imu_received = true;
        }
    }

    void amHeading::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
        // Description: EKF odometry topic callback function.

        //EKF odometry orientation is the same as calibrated imu's orientation
        q_odom = msg->pose.pose.orientation; // value to compare with gps yaw degree
        if (!odometry_received){
            odometry_received = true;
        }
    }

    void amHeading::wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        // Description: Wheel odometry callback function that gets linear x velocity 

        wheel_odom_v = msg->twist.twist.linear.x; // value used for checking linear vel
    }
    
}
