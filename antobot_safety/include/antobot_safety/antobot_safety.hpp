/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

Description: 	The primary purpose of this code is to define the functions and variables used in antobot_safety.cpp

Contacts: 	daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/

#include <time.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <antobot_msgs/UInt8_Array.h>
#include <antobot_msgs/UInt16_Array.h>

#include <vector>
#include <numeric>

class AntobotSafety
{

    protected:
        
        int uss_win_size = 10;  // 0.4 second of USS data filter size 
        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Publisher output_cmd_vel_pub;
        ros::Publisher output_uss_dist_filt_pub;
        ros::Publisher lights_f_pub;
        ros::Publisher lights_b_pub;
        ros::Publisher force_stop_type_pub;

        std::vector<std::vector<int>> uss_dist_windows;
        std::vector<bool> light_cmd_ar;

        float time_to_collision = 100;
        float lin_vel_thresh = 0.2;
        float ang_vel_thresh = 0.8;
        float time_collision_thresh = 1.0;      // Threshold could depend on operation type?
        int hard_dist_thresh = 75;
        int hard_dist_thresh_diag = 50;
        bool not_safe = false;
        int force_stop_type = 0;
        

        int active_cmd_vel;
        float prev_linear_vel;
        float prev_angular_vel;
        float linear_vel;
        float angular_vel;
        geometry_msgs::Twist cmd_vel_msg;
        clock_t t_lastRcvdCmdVel;

        bool force_stop;
        clock_t t_force_stop;       // Can be used to release the force stop, if desired
        bool force_stop_release;
        float fs_release_thresh = 3.0;
        clock_t t_release;

        int safety_light_pattern;
        float safety_light_freq;
        clock_t t_safety_light;     // Can be used to make the lights blink, if desired

        antobot_msgs::UInt16_Array uss_dist_filt;
    
    public:
    
        AntobotSafety(ros::NodeHandle& nh);
        ~AntobotSafety();
        void update(const ros::TimerEvent& e);
        bool ussDistSafetyCheck();
        bool ussDistSafetyCheck_f();
        bool ussDistSafetyCheck_b();
        void lightsSafetyOut();
        void lightCmdFreq();
        void safetyCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
        void activeCmdVelCallback(const std_msgs::String::ConstPtr& msg);
        void ussDistCallback(const antobot_msgs::UInt16_Array::ConstPtr& msg);
        antobot_msgs::UInt16_Array ussDistFilt(uint16_t uss_dist_ar[8]);
        uint16_t ussDistFilt_i(uint16_t uss_dist, int i);
        std::vector<int> popFront(std::vector<int> vec);
        int getUssVecMean(std::vector<int> vec);
        void autoRelease();
        float scaleCmdVel();
        float calcVelScale();
        bool scaleCornerTurn(float vel_scale);
        void releaseCallback(const std_msgs::Bool::ConstPtr& msg);
    
};
