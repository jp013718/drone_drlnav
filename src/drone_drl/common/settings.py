##########################################################################
# 
# Common settings for the drl environment and its agents
#
##########################################################################

# Environment Settings
EPISODE_TIMEOUT_SECONDS = 120
THRESHOLD_COLLISION = 0.05
THRESHOLD_GOAL = 0.05
NUM_OBSTACLES = 5
ARENA_LENGTH = 4.2
ARENA_WIDTH = 4.2

# Multiagent Settings
MULTIAGENT = True
NUM_AGENTS = 3

# Communication Settings
TOPIC_SCAN = '/laserscan'
TOPIC_VELO = '/cmd_vel'
TOPIC_ODOM = '/odom'
TOPIC_GOAL = '/goal_pose'

# Sensor and Actuator Settings
LIDAR_DISTANCE_CAP = 10
SPEED_LINEAR_MAX = 3
SPEED_ANGULAR_MAX = 6.28
