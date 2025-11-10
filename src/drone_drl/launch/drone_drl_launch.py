from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_sim.actions import GzServer
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import Node

from drone_drl.common.settings import WORLD_SDF


def generate_launch_description():
  ros_gz_sim_path = get_package_share_directory("ros_gz_sim")
  package_path = FindPackageShare('drone_sim')
  gz_launch_path = PathJoinSubstitution([ros_gz_sim_path, 'launch', 'gz_sim.launch.py'])
  
  return LaunchDescription([
    SetEnvironmentVariable(
      'GZ_SIM_RESOURCE_PATH',
      PathJoinSubstitution([package_path, 'models'])
    ),
    # GzServer(
    #   world_sdf_file=PathJoinSubstitution([FindPackageShare('drone_sim'), 'worlds/world.sdf']),
    #   use_composition=str(True),
    #   create_own_container=str(False),
    # ),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(gz_launch_path),
      launch_arguments={
        'gz_args': PathJoinSubstitution([package_path, 'worlds/world.sdf']),
        'on_exit_shutdown': 'True',
      }.items(),
    ),
    Node(
      package='ros_gz_bridge',
      executable='parameter_bridge',
      arguments=[
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/world/drl_world/control@ros_gz_interfaces/srv/ControlWorld',
        '/world/drl_world/create@ros_gz_interfaces/srv/SpawnEntity',
        '/world/drl_world/remove@ros_gz_interfaces/srv/DeleteEntity',
        '/world/drl_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
      ],
      parameters=[{
        'qos_overrides./tf_static.publisher.durability': 'transient_local'
      }],
      output='screen',
    ),
    # Node(
    #   package='drone_drl',
    #   executable='environment',
    #   name='env'
    # )
  ])