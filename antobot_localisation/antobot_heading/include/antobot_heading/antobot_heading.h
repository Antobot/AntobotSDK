#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

#include <geonav_transform/navsat_conversions.h>

using namespace GeonavTransform::NavsatConversions;

namespace antobot_heading
{
    class AmHeading{
        public:
            AmHeading(ros::NodeHandle& nh);
            ~AmHeading();
            void InitialCalibration(void);
        protected:
			//antobot_driver::Robot antobot1;
			ros::NodeHandle nh_;
            tf2::Matrix3x3 mat;

            // configuration parameters
            double calib_distance, angular_zero_tol, lin_tol, calib_deg;

            // GPS
            std::vector<double> gps_start, gps_end;
            bool gps_received;
            double utm_y, utm_x, rtk_status;
            std::string utm_zone;

            // imu
            geometry_msgs::Quaternion q_imu;
            std::string imu_frame;
            bool imu_received;
            float imu_ang_vel_z;
            double imu_offset;
            int imu_calibration_status;

            // odometry
            geometry_msgs::Quaternion q_odom;
            bool odometry_received;

            // wheel odometry
            double wheel_odom_v;
            
            // Other parameters
            bool direction;
            double gps_yaw;

            // Read parameters to set the topic names
            std::string gpsTopic, imuTopic, odometryTopic, wheelodometryTopic;
            std::string rtk_statusTopic;
            bool new_gps, sim;
            int rtk_target_status;
            
            // Subscriber
            ros::Subscriber subRtkStatus, subGps, subImu, subOdometry, subwheelOdom;

            // Publisher
            ros::Publisher pubImu, pubImuOffset, pubCalib; 

            // Timer
            ros::Timer autoCalibrationTimer;
            ros::Timer imuPubTimer;

            // Functions
            void initialise(void);
            std::vector<double> eulerFromQuart(tf2::Quaternion);
            tf2::Quaternion quartFromEuler(double a, double b, double c);
            void AutoCalibrate(const ros::TimerEvent& event);
            void PublishNewIMU(const ros::TimerEvent& event);
            bool SaveStartGPS(void);
            int CheckCondition(void);
            



            void rtkstatusCallback(const std_msgs::UInt8::ConstPtr& msg);
            void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
            void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
            void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
            void wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);


    };
}