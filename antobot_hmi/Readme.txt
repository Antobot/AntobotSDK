This is the package for HMI.
structure:
* antobot_manager_hmi
  * Arduino:software to be burnt into Arduino. 
    * HMI_assist
      * HMI_assist.ino:send additional UV battery SoC to screen
    *HMI_uv
      * HMI_uv.ino
  * Screen: folder contains the project file and tft file for UX design of HMI screen.
    * Nextion project
      * font
      * HMI_assist.HMI: assist version project file which can be open and edit use software called Nextion Editor. .
      * HMI_assist.TFT file are auto generated from .HMI project, the TFT file need to be copied into specific SD card and install into screen. /font folder contains the font used in .HMI project.
      * HMI_uv.HMI: uv version project file
      * HMI_uv.TFT: uv version TFT file
  * src： folder contains the python script to communicate between ros and arduino. the same code is used for both UV and assist, as long as the robot platform is configured properly in /AntoMove/am_description/antobot_description/config/robot_config.yaml.
    * hmi_xavier.py
  * CMakeLists.txt
  * pacvkage.xml
  * Readme.Md