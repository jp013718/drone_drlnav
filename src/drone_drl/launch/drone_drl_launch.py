from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ros_gz_sim.actions import GzServer
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import Node

from drone_drl.common.settings import WORLD_SDF


def generate_launch_description():
  return LaunchDescription([
    # Node(
    #   package='drone_drl',
    #   executable='environment',
    #   name='env'
    # ),
    GzServer(
      world_sdf_file=PathJoinSubstitution([FindPackageShare('drone_sim'), 'worlds/world.sdf']),
      use_composition=str(True),
      create_own_container=str(False),
    )
  ])