### antobot_gps

The antobot_gps repository outlook:
* build
* src
  * antobot_gps_node.py: Python script for F9P in uRCU via SPI using the sparkfun library 
  * gps_config.py: Python script to configure the F9P in uRCU via SPI
  * antobot_gps:
    * third party dependencies to run the antobot_gps_node.py (LICENCE provided)
* setup.py: installation file for antobot_gps



## How to use this repository:

* Always configure the F9P chip before it.

* run 'gps_config.py' to configure the F9P. This will give 8Hz GPS frequency. To make changes to the configuration, make sure that the new configurations are written to both flash and ram memory of F9P

* use 'antobot_gps_nRTK.py' to get the nRTK 8Hz F9P. this works only in HPG1.32 or 04B version of F9P. This script is for USB communication

* 'antobot_gps_node.py' is used in uRCU for gps-spi communication


