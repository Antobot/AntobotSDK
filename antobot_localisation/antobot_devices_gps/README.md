## antobot_devices_gps

The package to use and manage GPS devices on Antobot's robot platform

### config

- Different GPS devices and their relative location to the robot should be configured in the antobot_description package (platform_config.yaml)
  - Correction type should also be defined in platform_config.yaml
- Your method for receiving GPS correction messages should be configured in antobot_devices_gps/config/corrections_config.yaml (example provided - true config file gitignored by default)
  - Correction Types:
    - ant_mqtt: if using an Antobot base station, messages will come via an Antobot server via MQTT. An appropriate configuration file should come with your robot.
    - ntrip: using ublox's Thingstream (PointPerfect), a username and password is all you need to receive NRTIP messages for GPS corrections
    - ppp: ublox's Thingstream (PointPerfect) MQTT-based corrections. For this, a key and cert file are required for corrections messages

### launch
- gps_config.launch - used to appropriately configure different GPS nodes depending on configuration (used by gpsManager.py)

### meshes
- dae file of the standard GPS antenna used by the Ant Platform

### src
- gpsManager.py - the main script to launch and manage all GPS code
- gps_f9p.py - pulls data from an F9P (either the robot's internal F9P or a UART-connected one) and publishes the GPS data
- gps_corrections.py - collects corrections from the configured source and sends it to the F9P
- gps_movingbase.py - used for dual-GPS setups, pulls and publishes the relative location between the two GPS antennae
- antobot_devices_gps: supporting scripts
- utils:
  - f9p_config.py - use this to configure the F9P to receive different messages or other similar changes
  - field_survey.cpp - an example script for how to survey many points which can be put into
  - gps_strength_test.py - can be used to print some information about GPS signal strength (number of usable satellites, signal strength, etc.) [likely to be removed in future releases]
  - RenameSerialPort.sh - used to define a specific USB port which can be used by the robot without changing its name









