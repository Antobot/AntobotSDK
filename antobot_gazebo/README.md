# am_sim

This package is for synthetic testing using the Gazebo simulator.

To use the simulation requires:
  - Robot description files for each robot, which should be located in the am_description package 
  - **World** files detailing suitable testing scenarios, which are located in this package
  - Additional large model files (e.g. terrain/obstacles) which are located on the [AntoBot sharepoint](https://antorobot.sharepoint.com/sites/SoftwareSystem/Shared%20Documents/Forms/AllItems.aspx?id=%2Fsites%2FSoftwareSystem%2FShared%20Documents%2F03%5FInstallation%26CodeTransfer%2F01%5FOriginalFiles%2F08%5FSimulator&viewid=d9622cc4%2D85f0%2D49e9%2D97b5%2D0377ace98ec2) and downloaded seperately
 
To minimise the ammount of github storage space used, please use sharepoint for any models other than robots.

To launch a simulation (e.g. Ant robot at Farm):

**roslaunch antobot_gazebo ant_sim.launch**


