/*
# Copyright (c) 2019, ANTOBOT LTD.
# All rights reserved.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
#include <anto_bridge_msgs/UInt8_Array.h>
#include <anto_bridge_msgs/UInt16_Array.h>

#include <vector>
#include <numeric>

class AmSafety
{

    protected:
        
        int uss_win_size = 10;  // 0.4 second of USS data filter size 
        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Publisher output_cmd_vel_pub;
        ros::Publisher output_uss_dist_filt_pub;
        ros::Publisher lights_f_pub;
        ros::Publisher lights_b_pub;

        std::vector<std::vector<int>> uss_dist_windows;
        std::vector<bool> light_cmd_ar;

        int active_cmd_vel;
        float linear_vel;
        float angular_vel;
        geometry_msgs::Twist cmd_vel_msg;
        clock_t t_lastRcvdCmdVel;

        bool force_stop;
        clock_t t_force_stop;       // Can be used to release the force stop, if desired
        bool force_stop_release;
        clock_t t_release;

        int safety_light_pattern;
        float safety_light_freq;
        clock_t t_safety_light;     // Can be used to make the lights blink, if desired

        anto_bridge_msgs::UInt16_Array uss_dist_filt;
    
    public:
    
        AmSafety(ros::NodeHandle& nh);
        ~AmSafety();
        void update(const ros::TimerEvent& e);
        bool ussDistSafetyCheck();
        void lightsSafetyOut();
        void light_cmd_freq();
        void safetyCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
        void activeCmdVelCallback(const std_msgs::String::ConstPtr& msg);
        void ussDistCallback(const anto_bridge_msgs::UInt16_Array::ConstPtr& msg);
        anto_bridge_msgs::UInt16_Array ussDistFilt(uint16_t uss_dist_ar[8]);
        uint16_t ussDistFilt_i(uint16_t uss_dist, int i);
        std::vector<int> pop_front(std::vector<int> vec);
        int get_uss_vec_mean(std::vector<int> vec);
        void releaseCallback(const std_msgs::Bool::ConstPtr& msg);


    
};
