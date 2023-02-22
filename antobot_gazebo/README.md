# antobot_gazebo

This package is for simulated testing using Gazebo.

To use the simulation requires:
  - Robot description files for each robot, which should be located in the antobot_description package 
  - **World** files detailing suitable testing scenarios, which are located in this package
  - Additional large model files (e.g. terrain/obstacles) which are usually installed into a hidden gazebo folder, located at "~.gazebo".
  
 For simulations that aren't released publically, any large files should be stored on sharepoint to save space on github. These can be downloaded from [here](https://antorobot.sharepoint.com/:f:/r/sites/SoftwareSystem/Shared%20Documents/03_HardwareInstallation/01_OriginalFiles/08_Simulator?csf=1&web=1&e=HyGbTi).
 

As farms can be very large, each farm may have multiple world files for different areas/testing setups. For example, to launch a simulation of Antv3 in the strawberry section of Lathcoats farm:
**roslaunch antobot_gazebo windsor_ant_v3.launch **

At present, each simulation has a seperate launch and settings files. This is intended to make launching each simulation easy (with no command line inputs) and eliminate the risk of any changes to one simulation adversely affecting another.
