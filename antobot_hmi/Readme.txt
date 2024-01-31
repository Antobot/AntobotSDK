This is the package for HMI.
structure:
* antobot_hmi
  * Arduino:software to be burnt into Arduino. 
    * hmi
      * hmi.ino:send additional UV battery SoC to screen
  * Screen: folder contains the project file and tft file for UX design of HMI screen.
    * Nextion project
      * font:font used in .HMI project.
      * material
        * background image used in Nextion project
      * HMI_sdk.HMI: assist version project file which can be open and edit use software called Nextion Editor. .
      * HMI_sdk.TFT file are auto generated from .HMI project, the TFT file need to be copied into specific SD card and install into screen. 
    *hmi_screen_transition_design.svg: diagram of logic transition
  * src： folder contains the python script to communicate between ros and arduino. 
    * hmi_xavier.py
  * CMakeLists.txt
  * pacvkage.xml
  * Readme.Md