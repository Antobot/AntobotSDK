# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;pluginlib;nodelet;dynamic_reconfigure;geometry_msgs;antobot_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lantobot_safety_nodelet;-lyaml-cpp".split(';') if "-lantobot_safety_nodelet;-lyaml-cpp" != "" else []
PROJECT_NAME = "antobot_safety"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
