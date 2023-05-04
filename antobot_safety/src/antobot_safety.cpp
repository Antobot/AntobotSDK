/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

Description: 	The primary purpose of this code is to take command velocities and information gathered by robotic sensors 
in order to scale the output robot command velocity to an appropriate level. More specifically, this code filters data from
ultrasonic sensors and forces the robot to stop if a collision is predicted or if a distance threshold is too close.
If a force stop is commanded, then the lights are made to blink.
If antobot_safety is properly implemented with working sensors, the AntoSafe code in Aurix should never have to be used.

Contacts: 	daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/

#include <time.h>
#include <math.h>

#include <ros/ros.h>
#include "antobot_safety/antobot_safety.hpp"
#include <string>

AntobotSafety::AntobotSafety(ros::NodeHandle& nh) : nh_(nh)
{
    /*  Initialises the AntobotSafety class*/
    double loop_hz_ = 25.0;

    AntobotSafety::output_cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/antobot_robot/cmd_vel", 10);
    AntobotSafety::output_uss_dist_filt_pub = nh_.advertise<antobot_msgs::UInt16_Array>("/anto_safety/uss_dist", 10);
    force_stop_type_pub = nh_.advertise<std_msgs::Int8>("/antobot_safety/force_stop_type", 10);  // 0 - none (or release); 1 - front left; 2 - front; 3 - front right; 4 - right spot turn; 
                                                                                            // 5 - back right; 6- back straight; 7 - back left; 8 - left spot turn
    lights_f_pub = nh_.advertise<std_msgs::Bool>("/antobridge/lights_f", 1);
    lights_b_pub = nh_.advertise<std_msgs::Bool>("/antobridge/lights_b", 1);

    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	non_realtime_loop_ = nh_.createTimer(update_freq, &AntobotSafety::update, this);

    uss_dist_windows = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}};
    uss_dist_filt.data = {204, 204, 204, 204, 204, 204, 204, 204};

    safety_light_pattern = 1;
    safety_light_freq = 5;
    light_cmd_ar = {false, false};

    force_stop = false;
    force_stop_type = 0;

}

AntobotSafety::~AntobotSafety()
{
    /*  Deconstructor for the AntobotSafety class*/
}

void AntobotSafety::update(const ros::TimerEvent& e)
{
    /*  Fixed update rate to check various safety inputs and broadcast the correct outputs
    */

    // Check USS recommendation
    if (ussDistSafetyCheck() && !force_stop && !force_stop_release)
    {
        if (force_stop_type > 0)
        {
            // For corner turns, try to scale the movement   
            float vel_scale = scaleCmdVel();

            // If it isn't safe to scale, force stop the robot
            if (vel_scale == 0)
            {
                force_stop = true;
                t_force_stop = clock();         // Sets when the robot force stopped
                t_safety_light = clock();
            }
            else
            {
                force_stop_type = 0;
            }
        }
        else
        {
            // If force stop is triggered while moving straight, force stop the robot
            force_stop = true;
            t_force_stop = clock();         // Sets when the robot force stopped
            t_safety_light = clock();
        }
    }

    // Check costmap recommendation

    // Check time of last received command - if none received in the last ~1s, the robot should stop
    if ((float)(clock() - t_lastRcvdCmdVel)/CLOCKS_PER_SEC > 0.05)
    {   
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.angular.z = 0;
    }

    // Robot is moving too quickly toward an obstacle
    if (force_stop)
    {
        lightsSafetyOut();	// Broadcast safety lights (if needed)

        // Set command velocity to 0
        cmd_vel_msg.linear.x = 0;
        cmd_vel_msg.angular.z = 0;
    }

    prev_linear_vel = cmd_vel_msg.linear.x;
    prev_angular_vel = cmd_vel_msg.angular.z;

    AntobotSafety::output_cmd_vel_pub.publish(cmd_vel_msg);

    std_msgs::Int8 force_stop_type_msg;
    force_stop_type_msg.data = force_stop_type;
    force_stop_type_pub.publish(force_stop_type_msg);

    autoRelease();
    
    if (10.0*(clock() - t_release)/(float)CLOCKS_PER_SEC > 0.2)
        force_stop_release = false;  
}

bool AntobotSafety::ussDistSafetyCheck()
{
    /*  Checks whether, based on the ultrasonic sensor (USS) data, the robot should 
        slow down or stop */
    //  Inputs: uss_dist_filt <msgs::UInt16_Array> - 8-element array of filtered USS distances
    //          linear_vel <float> - robot commanded linear velocity
    //          angular_vel <float> - robot commanded angular velocity
    //  Returns: not_safe <bool> - true indicates the robot may collide with an object; false means safe movement is possible

    not_safe = false;
    //force_stop_type = 0;

    if (linear_vel > lin_vel_thresh)
    {
        // Robot is moving forward
        not_safe = ussDistSafetyCheck_f();
        
    }
    else if (linear_vel < -lin_vel_thresh)
    {
        // Robot is moving backward
        not_safe = ussDistSafetyCheck_b();
    }
    else
    {
        // Spot turn
        if (angular_vel > ang_vel_thresh)
        {
            // Left turn
            /*if (uss_dist_filt.data[3] < hard_dist_thresh || uss_dist_filt.data[7] < hard_dist_thresh)
            {
                // not_safe = true;
                // force_stop_type = 7;
            }*/
        }
        else if (angular_vel < -ang_vel_thresh)
        {
            // Right turn
            /* if (uss_dist_filt.data[3] < hard_dist_thresh || uss_dist_filt.data[7] < hard_dist_thresh)
            {
                // not_safe = true;
                // force_stop_type = 8;
            }*/
            
        }
        else    // Robot is not moving
        {
            time_to_collision = 100.0;
        }
    }

    return not_safe;
}

bool AntobotSafety::ussDistSafetyCheck_f()
{
    /*  Checks whether, based on the forward-facing ultrasonic sensor (USS) data, the robot should 
        slow down or stop */
    //  Inputs: uss_dist_filt <msgs::UInt16_Array> - 8-element array of filtered USS distances
    //          linear_vel <float> - robot commanded linear velocity
    //          angular_vel <float> - robot commanded angular velocity
    //  Returns: not_safe <bool> - true indicates the robot may collide with an object; false means safe movement is possible

    int fst = 2;    // 1 - left front; 2 - straight front; 3 - right front
    bool not_safe_f = false;

    time_to_collision = (float)(uss_dist_filt.data[1])/(100.0*linear_vel);      // Check time to reach nearest obstacle to the robot's front
    if (time_to_collision < time_collision_thresh || uss_dist_filt.data[1] < hard_dist_thresh)
    {
        not_safe_f = true;
        force_stop_type = 2;
        return not_safe_f;
    } 

    if (angular_vel > ang_vel_thresh * linear_vel)           // Robot is turning left while moving forward
    {
        float time_to_collision_fl = (float)(uss_dist_filt.data[0])/(100.0*linear_vel);      // Check time to reach nearest obstacle to the robot's front left
        if (time_to_collision_fl < time_collision_thresh || uss_dist_filt.data[0] < hard_dist_thresh_diag)
        {
            force_stop_type = 1;
            not_safe_f = true;
            return not_safe_f;
        }
    }
    else if (angular_vel < -ang_vel_thresh * linear_vel)     // Robot is turning right while moving forward
    {
        float time_to_collision_fr = (float)(uss_dist_filt.data[2])/(100.0*linear_vel);      // Check time to reach nearest obstacle to the robot's front right
        if (time_to_collision_fr < time_collision_thresh || uss_dist_filt.data[2] < hard_dist_thresh_diag)
        {
            force_stop_type = 3;
            not_safe_f = true;
            return not_safe_f;
        } 
    }

    return not_safe_f;
}

bool AntobotSafety::ussDistSafetyCheck_b()
{
    /*  Checks whether, based on the backward-facing ultrasonic sensor (USS) data, the robot should 
        slow down or stop */
    //  Inputs: uss_dist_filt <msgs::UInt16_Array> - 8-element array of filtered USS distances
    //          linear_vel <float> - robot commanded linear velocity
    //          angular_vel <float> - robot commanded angular velocity
    //  Returns: not_safe <bool> - true indicates the robot may collide with an object; false means safe movement is possible
    
    int fst = 6;    // 7 - left back; 6 - straight back; 5 - right back (force stop type)
    bool not_safe_b = false;

    time_to_collision = (float)(uss_dist_filt.data[5])/(-100.0*linear_vel);      // Check time to reach nearest obstacle to the robot's back
    if (time_to_collision < time_collision_thresh || uss_dist_filt.data[5] < hard_dist_thresh)
    {
        not_safe_b = true;
        force_stop_type = 6;
        return not_safe_b;
    } 

    if (angular_vel > - ang_vel_thresh * linear_vel)
    {
        // Robot is moving back right
        float time_to_collision_br = (float)(uss_dist_filt.data[4])/(-100.0*linear_vel);      // Check time to reach nearest obstacle to the robot's back right
        if (time_to_collision_br < time_collision_thresh || uss_dist_filt.data[4] < hard_dist_thresh_diag)
        {
            force_stop_type = 5;
            not_safe_b = true;
            return not_safe_b;
        }      
    }
    else if (angular_vel <  ang_vel_thresh * linear_vel)
    {
        // Robot is moving back left
        float time_to_collision_bl = (float)(uss_dist_filt.data[6])/(-100.0*linear_vel);      // Check time to reach nearest obstacle to the robot's back left
        if (time_to_collision_bl < time_collision_thresh || uss_dist_filt.data[6] < hard_dist_thresh_diag)
        {
            force_stop_type = 7;
            not_safe_b = true;
            return not_safe_b;
        }   
    }

    return not_safe_b;
}

void AntobotSafety::lightsSafetyOut()
{
    /* Sends light commands to AntoBridge based on the set pattern
    */

    std_msgs::Bool lights_f_cmd;
    std_msgs::Bool lights_b_cmd;

    if (safety_light_pattern == 0)
    {
        // All lights on

        lights_f_cmd.data = true;
        lights_b_cmd.data = true;
    }
    if (safety_light_pattern == 1)
    {
        // Blinking with constant frequency (light_freq)
        lightCmdFreq();
        lights_f_cmd.data = light_cmd_ar[0];
        lights_b_cmd.data = light_cmd_ar[1];
    }
    
    // Publishes the data to ROS
    lights_f_pub.publish(lights_f_cmd);
    lights_b_pub.publish(lights_b_cmd);
}

void AntobotSafety::lightCmdFreq()
{
    /* Sends light commands at a set frequency, defined in the class initialisation
    */

    float t_light_freq_thresh;
    t_light_freq_thresh = 1.0/safety_light_freq;

    // If past a time threshold, lights will change state
    if (10.0*(clock() - t_safety_light)/(float)CLOCKS_PER_SEC > t_light_freq_thresh)
    {
        light_cmd_ar[0] = !light_cmd_ar[0];
        light_cmd_ar[1] = !light_cmd_ar[1];
        t_safety_light = clock();
    }
}

void AntobotSafety::safetyCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    /* Callback function for /antobot_safety/cmd_vel. Assigns the velocity data received from
    this ROS topic to the robot if appropriate, and ensures reasonable acceleration.
    */

    // Max acceleration
    float max_acc = 3.0;  // m/s^2
    float max_acc_s = max_acc / 25.0;   // Conversion to expected speed increase at 25 hz (loop rate)
    float max_dec = 12.0;   // m/s^2
    float max_dec_s = max_dec / 25.0;   // Conversion to expected speed decrease at 25 hz (loop rate)
    
    // Assigns received value to velocity variables
    linear_vel = msg->linear.x;
    angular_vel = msg->angular.z;

    // Checks acceleration
    if (abs(linear_vel) > abs(prev_linear_vel) + max_acc_s)
       linear_vel = prev_linear_vel + copysign(max_acc_s, linear_vel); // If acceleration is too high, lowers command speed

    // Checks deceleration
    if (abs(linear_vel) < abs(prev_linear_vel) - max_dec_s)
       linear_vel = prev_linear_vel - copysign(max_dec_s, prev_linear_vel); // If deceleration is too fast, keeps speed up

    // Assigns values to final output
    cmd_vel_msg.linear.x = linear_vel;
    cmd_vel_msg.angular.z = angular_vel;

    // Provides time that the command was received
    t_lastRcvdCmdVel = clock();
}

void AntobotSafety::activeCmdVelCallback(const std_msgs::String::ConstPtr& msg)
{
    /*  Collects information from ROS about the robot's current operation state. This is because different operation states may have different
        limits based on the environment (i.e. teleoperation via the app may have significant delay)  */
    //  Inputs: msg <std_msgs::String> - received whenever the operation mode changes; "idle", "Teleop" (joystick or keyboard); "MQTT" (app control); or "Navigation stack"
    //  Outputs: changes the active_cmd_vel class variable for use in other parts of the script.
    
    if (msg->data == "idle")
        active_cmd_vel = 0;
    else if (msg->data == "Teleop")
        active_cmd_vel = 1;
    else if (msg->data == "MQTT")
        active_cmd_vel = 2;
    else if (msg->data == "Navigation stack")
        active_cmd_vel = 3;
}

void AntobotSafety::ussDistCallback(const antobot_msgs::UInt16_Array::ConstPtr& msg)
{
    /*  Reads in the data from the ultrasonic sensors and, based on the current movement of the robot, makes a recommendation 
        for whether the robot should slow down or whether its current speed/movement is acceptable. */
    //  Inputs: msg <antobot_msgs::UInt16_Array> - currently an 8-element array which provides the distances sensed by each ultrasonic sensor. 
    //                                                The order starting from msg->data[0] is: 0 - front left; 1 - front; 2 - front right; 3 - right;
    //                                                4 - back right; 5 - back; 6 - back left; 7 - left
    //  Outputs: publishes filtered USS data to /antobot_safety/uss_dist ROS topic
    antobot_msgs::UInt16_Array uss_dist_filt_all;
    uint16_t uss_dist_ar[8];
    uint16_t uss_dist_filt_i;
    for (int i=0; i<8; i++)
        uss_dist_ar[i] = msg->data[i];

    // Define the filtered USS dist class variable 
    //uss_dist_filt = ussDistFilt(uss_dist_ar);
    for (int i=0; i<8; i++)
        {
            uss_dist_filt_i = uss_dist_ar[i];
            uss_dist_filt_all.data.push_back(uss_dist_filt_i);
        }
    uss_dist_filt=uss_dist_filt_all;
    AntobotSafety::output_uss_dist_filt_pub.publish(uss_dist_filt);
}

antobot_msgs::UInt16_Array AntobotSafety::ussDistFilt(uint16_t uss_dist_ar[8])
{
    /*  Gets filtered ultrasonic sensor data for each individual sensor, creates the structure
        for the data to be sent, and returns this to the main USS callback function. */
    //  Inputs: uss_dist_ar <uint8_t[8]> - the most recent USS data pulled in for each of the 8 sensors
    //  Returns: uss_dist_filt_all <antobot_msgs::UInt16_Array> - the filtered data to publish

    antobot_msgs::UInt16_Array uss_dist_filt_all;
    uint16_t uss_dist_filt_i;

    for (int i=0; i<8; i++)
    {
        uss_dist_filt_i = ussDistFilt_i(uss_dist_ar[i], i);
        uss_dist_filt_all.data.push_back(uss_dist_filt_i);
    }

    return uss_dist_filt_all; 
}

uint16_t AntobotSafety::ussDistFilt_i(uint16_t uss_dist_i, int i)
{
    /*  Calculates filtered data of a single USS sensor by first formatting data to fit into a predetermined window size,
        then carrying out the filter (currently a simple mean) */
    //  Inputs: uss_dist_i <uint16_t> - the newest data received for a particular sensor
    //          i <int> - the element of the corresponding USS sensor
    
    uss_dist_windows[i].push_back(uss_dist_i);
    if (uss_dist_windows[i].size() > AntobotSafety::uss_win_size)
    {
        uss_dist_windows[i] = AntobotSafety::popFront(uss_dist_windows[i]);
    }

    int uss_dist_filt_ii = AntobotSafety::getUssVecMean(uss_dist_windows[i]);

    return uss_dist_filt_ii;
}

std::vector<int> AntobotSafety::popFront(std::vector<int> vec)
{
    /*  Simple function to pop off the first element of a vector. */
    //  Input: vec <std::vector<int>> - a generic int vector of length >1
    //  Returns: vec <std::vector<int>> - a generic int vector of length >0
    
    assert(!vec.empty());
    vec.erase(vec.begin());

    return vec;
}

int AntobotSafety::getUssVecMean(std::vector<int> vec)
{
    /*  Filters and gets the mean of a single US sensor over a set window. */
    //  Input: vec <std::vector<int>> - a vector of USS data
    //  Returns: <int> - the filtered average value

    // If the vector is empty, the assumed initial difference is 2.04 meters
    if (vec.empty()) {
        return 204;
    }

    // If any data is returned as 0, it is assumed that there are no obstacles, so the maximum value (204) is assigned
    std::replace_if(vec.begin(), vec.end(), [](int &i) {
        return i == 0;
    }, 204);
 
    // Calculates and returns the average value of the vector
    return std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
}

void AntobotSafety::autoRelease()
{
    /* Automatically releases the robot from its force stopped state if the previously 
    detected object is no longer being detected */
    
    if (force_stop)
    {
        if (force_stop_type > 0)
        {
            if (uss_dist_filt.data[force_stop_type - 1] > hard_dist_thresh)
            {
                // Checks timer to re-start navigation (if safe)
                if (10.0*(clock() - t_force_stop)/(float)CLOCKS_PER_SEC > fs_release_thresh)
                {
                    // Force stop release if the time threshold has passed
                    force_stop = false;
                    force_stop_release = true;
                    force_stop_type = 0;
                    t_release = clock();
                }
            }
            else
            {
                t_force_stop = clock();     // Resets the timer if the object is still there
            }
        }
    }  
}

float AntobotSafety::scaleCmdVel()
{
    float vel_scale = 0;

    vel_scale = calcVelScale();

    if (scaleCornerTurn(vel_scale))
        return vel_scale;

    // Obstacle in front
    if (force_stop_type == 2)
    {
        if (cmd_vel_msg.angular.z > 0.2 && uss_dist_filt.data[0] > hard_dist_thresh)      // Left turn
        {
            cmd_vel_msg.linear.x = vel_scale * cmd_vel_msg.linear.x;
            return vel_scale;
        }
        else if (cmd_vel_msg.angular.z < -0.2 && uss_dist_filt.data[2] > hard_dist_thresh)      // Right turn
        {
            cmd_vel_msg.linear.x = vel_scale * cmd_vel_msg.linear.x;
            return vel_scale;
        }
    }
    
    // Obstacle behind
    else if (force_stop_type == 6)
    {
        if (cmd_vel_msg.angular.z > 0.2 && uss_dist_filt.data[4] > hard_dist_thresh)      // Left turn
        {
            cmd_vel_msg.linear.x = vel_scale * cmd_vel_msg.linear.x;
            return vel_scale;
        }
        else if (cmd_vel_msg.angular.z < -0.2 && uss_dist_filt.data[6] > hard_dist_thresh)      // Right turn
        {
            cmd_vel_msg.linear.x = vel_scale * cmd_vel_msg.linear.x;
            return vel_scale;
        }
    }

    return vel_scale;  
}

float AntobotSafety::calcVelScale()
{
    /*  Calculates the magnitude by which to scale the velocity of the robot based on which ultrasonic sensor
        has detected an obstacle, and how far away that obstacle is. */
    //  Returns: vel_scale <float> the scale (between 0 and 1) by which the velocity will be scaled
    
    float vel_scale = 0;
    
    // Scale movement based on distance to obstacle
    if (force_stop_type > 0)
    {
        int uss_data = uss_dist_filt.data[force_stop_type - 1];

        if(uss_data > 100)
            vel_scale = 1;
        else
            vel_scale = log10((uss_data-45)/6);
        if (vel_scale < 0)
            vel_scale = 0;

    }
    return vel_scale;
}

bool AntobotSafety::scaleCornerTurn(float vel_scale)
{
    // Forward corner turn (obstacle inside)
    if (force_stop_type == 1 || force_stop_type == 3)
    {
        if (uss_dist_filt.data[1] > hard_dist_thresh)       // If nothing is too close to the front of the robot
        {
            // if robot is turning left and the force stop type is in that direction
            if ((cmd_vel_msg.angular.z > 0.05 && force_stop_type == 1) || (cmd_vel_msg.angular.z < -0.05 && force_stop_type == 3))
            {
                cmd_vel_msg.angular.z = vel_scale * cmd_vel_msg.angular.z;
                return true;
            }
        } 
    }

    // Backward corner turn (obstacle inside)
    else if (force_stop_type == 5 || force_stop_type == 7)
    {
        if (uss_dist_filt.data[5] > hard_dist_thresh)       // If nothing is too close to the back of the robot
        {
            if ((cmd_vel_msg.angular.z > 0.05 && force_stop_type == 5) || (cmd_vel_msg.angular.z < -0.05 && force_stop_type == 7))
            {
                cmd_vel_msg.angular.z = vel_scale * cmd_vel_msg.angular.z;
                return true;
            }
        }
    }

    return false;
}

void AntobotSafety::releaseCallback(const std_msgs::Bool::ConstPtr& msg)
{
   /*   Callback function for /antobridge/force_stop_release. Tells the robot it is okay to move again
        after a force stop command
   */
   
   if (msg->data) 
   {
	  force_stop = false;
	  force_stop_release = true;
      force_stop_type = 0;
      t_release = clock();
   }
}

int main(int argc, char** argv)
{
    // Initialises the ROS node and gets the node handle
    ros::init(argc, argv, "antobot_safety");
    ros::NodeHandle nh;
    ros::Rate rate(25);

    // Defines an AntobotSafety class object using the defined ROS node
    AntobotSafety AntobotSafety1(nh);

    // Defines subscribers for calibration and track width adjustment, and links them to the specific class instance of AntobotSafety
    ros::Subscriber sub_safety_cmd_vel = nh.subscribe("/antobot_safety/cmd_vel", 10, &AntobotSafety::safetyCmdVelCallback, &AntobotSafety1);
    ros::Subscriber sub_active_cmd_vel = nh.subscribe("/yocs_cmd_vel_mux/active", 10, &AntobotSafety::activeCmdVelCallback, &AntobotSafety1);
    ros::Subscriber sub_uss_dist = nh.subscribe("/antobridge/uss_dist", 10, &AntobotSafety::ussDistCallback, &AntobotSafety1);
    ros::Subscriber sub_force_stop_release = nh.subscribe("/antobridge/force_stop_release", 10, &AntobotSafety::releaseCallback, &AntobotSafety1);

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}

