import sys
import copy
import numpy as np
from functools import partial

from geometry_msgs.msg import Pose, Twist
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from drone_msgs.srv import SphereGoal, DrlStep, Goal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from drone_drl.common.settings import *

MAX_GOAL_DIST = np.sqrt(ARENA_LENGTH**2+ARENA_WIDTH**2+ARENA_HEIGHT**2)
NUM_SCAN_SAMPLES = 360
class DRLEnvironment(Node):
  def __init__(self):
    super.__init__('drl_environment')
    with open('/tmp/drlnav_current_stage.txt', 'r') as f:
      self.stage = int(f.read())
    print(f"Running on stage {self.stage}")

    # Initialize settings
    self.episode_timeout = EPISODE_TIMEOUT_SECONDS
    self.num_agents = NUM_AGENTS if MULTIAGENT else 1
    self.scan_topic = TOPIC_SCAN
    self.velo_topic = TOPIC_VELO
    self.odom_topic = TOPIC_ODOM
    self.goal_topic = TOPIC_GOAL

    # Initialize environment variables
    self.goals_pos = [np.zeros(shape=(2))]*self.num_agents
    self.robots_pos = [np.zeros(shape=(2))]*self.num_agents
    self.robots_pos_prev = [np.zeros(shape=(2))]*self.num_agents
    self.robots_heading = [np.zeros(shape=(2))]*self.num_agents
    self.total_distances = [np.zeros(shape=(2))]*self.num_agents
    self.robots_tilt = [np.zeros(shape=(2))]*self.num_agents
    
    self.dones = [False]*self.num_agents
    self.succeeds = [False]*self.num_agents
    self.episode_deadline = np.Infinity
    self.reset_deadlines = [False]*self.num_agents
    self.clock_msgs_skipped = 0

    self.true_obstacle_distances = [[np.Infinity]*NUM_OBSTACLES]*self.num_agents
    
    self.new_goals = [False]*self.num_agents
    self.goal_angles = [0.0]*self.num_agents
    self.goal_dists = [MAX_GOAL_DIST]*self.num_agents
    self.init_dists_to_goals = [MAX_GOAL_DIST]*self.num_agents

    self.scan_ranges = [[LIDAR_DISTANCE_CAP]*NUM_SCAN_SAMPLES]*self.num_agents
    self.obstacle_distances = [LIDAR_DISTANCE_CAP]*self.num_agents

    ###########################################
    # Initialize Publishers and Subscribers
    ###########################################
    qos = QoSProfile(depth=10)
    qos_clock = QoSProfile(depth=1, reliability=0)
    # publishers
    self.cmd_vel_pubs = [self.create_publisher(Twist, f"agent_{i}/{self.velo_topic}", qos) for i in range(self.num_agents)]
    # subscribers
    self.goal_pose_subs = [self.create_subscription(Pose, f"agent_{i}/{self.goal_topic}", partial(self.goal_pose_callback, id=i), qos) for i in range(self.num_agents)]
    self.odom_subs = [self.create_subscription(Odometry, f"agent_{i}/{self.odom_topic}", partial(self.odom_callback, id=i), qos) for i in range(self.num_agents)]
    self.scan_subs = [self.create_subscription(LaserScan, f"agent_{i}/{self.scan_topic}", partial(self.scan_callback, id=i), qos) for i in range(self.num_agents)]
    self.clock_sub = self.create_subscription(Clock, "/clock", self.clock_callback, qos_profile=qos_clock)
    self.obstacle_odom_sub = self.create_subscription(Odometry, 'obstacle/odom', self.obstacle_odom_callback, qos)
    # clients
    self.task_succeed_clients = [self.create_client(SphereGoal)]