# AntoBridge

## Package Description
AntoBridge (anto_bridge) is intented to act as a direct channel for communication with the uRCU. Desired wheel velocities, steering motor positions, lighting commands, and more can be sent which will yield a commanded output from the uRCU that can be used to control the robot.
The commands sent to AntoBridge should be sent via ROS topics, as defined below:

## Interface (ROS Topic) Description
### AntoBridge Subscribes To (/topic_name [MsgType]):
<pre>/antobridge/wheel_vel_cmd     [anto_bridge/Float32_Array (length 4)]</pre>
Description: The command sent to AntoBridge to make each wheel turn with the specified angular velocity (rad/s).
<pre>/antobridge/steer_pos_cmd     [anto_bridge/Float32_Array (length 4)]</pre>
Description: The command sent to AntoBridge to make each steering motor (4 available) turn to a target position (rad).
<pre>/antobridge/track_width_cmd   [std_msgs/Float32]</pre>
Description: The command sent to AntoBridge to set a track width for an ATWAS (adjustable track width) robot

### AntoBridge Publishes To (/topic_name [MsgType]:
<pre>/antobridge/wheel_vel        [anto_bridge::Float32_Array (length 4)]</pre>
Description: The true wheel velocities (rad/s) received from encoders of the wheel motors. Can be used to calculate wheel odometry.
<pre>/antobridge/steer_pos        [anto_bridge::Float32_Array (length 4)</pre>
Description: The true position (radians) of the steering motors for each wheel of a steerable system.
<pre>/antobridge/track_width      [std_msgs::Float32] </pre>
Description: The current track width of the ATWAS robot as calculated by wheel turns 
<pre>/antobridge/uss_dist         [anto_bridge::UInt8_Array (length 8)] </pre>
Description: The distances sensed by each ultrasonic sensor (cm)
<pre>/antobridge/force_stop       [std_msgs::Bool]</pre>
Description: A boolean flag to indicate whether some lower-level logic has stopped the robot (for example bump stop or ultrasonic sensors).



