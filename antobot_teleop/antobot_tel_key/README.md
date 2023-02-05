# antobot_tel_key

Keyboard teleoperation code robot. Teleoperation can be keyboard teleop, 'Antobot' app teleop or joystick teleop.

Teleoperation is launched along with the rest of the robotic components on bringup. However, if you wish to launch the teleop separately, run the following command:
**rosrun antobot_tel antobot_teleop.py**

The robot can be teleoperated using either the keyboard, joystick or app.

Keyboard control:
* Press the '0' key to enter keyboard teleoperation mode.

---------------------------
        Moving around:
           u    i    o
           j    k    l
           m    ,    .

        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%
        space key, k : force stop
        anything else : stop smoothly

        CTRL-C to quit

