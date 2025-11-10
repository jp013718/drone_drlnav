Drone DRL Nav
---
## Description
In-progress project aimed to achieve multi-agent autonomy of UAV swarms through training and simulation using the Gazebo and ROS platforms. This repo is intended to work on MacOS devices using Docker and the VSCode Dev Containers extension.
## Setting Up
Follow Step 1 of the directions for setup using noVNC [here](https://wiki.ros.org/docker/Tutorials/GUI). It may be helpful to create an alias for the third command in that step in your .zshrc file.

Clone this repo, then open in VSCode. Ensure that the Dev Containers extension from Microsoft is installed, as well as Docker. If Docker is not installed when you install the Dev Containers extension, you will be prompted to install Docker from VSCode.

Click the blue button in the bottom-left corner of the VSCode window. A menu will appear on the top bar. Select "Reopen in Container". This will create a new Docker container attached to your VSCode session, including a terminal.

Inside the terminal, enter the command ```colcon build```. 

Then, source the packages built as so: ```source install/setup.bash```.

## Running the Code
If the noVNC container is not running, enter the third command from the noVNC setup in a terminal separate from VSCode.

In any browser, open the page ```127.0.0.1:8080/vnc.html``` (assuming you used port ```8080``` as the noVNC directions suggest).

In the VSCode terminal, enter the command ```ros2 launch drone_drl drone_drl_launch.py```. In the VNC webpage, the Gazebo GUI should appear, loading in a world that will contain a few hoops and a small floating platform. Additionally, you may notice that the output of this command informs you that several ros_gz_bridges were created. This is important for communicating to Gazebo using ROS.

### Spawning a UAV
As of right now, the only way to create a UAV in the world is by manually doing so using the terminal.

Open a second terminal in the VSCode window attached to the Docker container. To create a UAV named "drone_0" at the coordinates (0, 0, 0), run the following command (separated for ease of reading):

```
ros2 service call /world/drl_world/create ros_gz_interfaces/srv/SpawnEntity \
  '{entity_factory: \
    {name: "drone_0", \
    sdf_filename: "src/drone_sim/models/X3_UAV/model.sdf", \
    pose: {position: \
      {x: 0, \
      y: 0, \
      z: 0}\
    } \
  } \
}'
```

You should notice a UAV appear on the small platform.

NOTE: A keyboard-controllable model also exists (at the same location as the other model, but by the name ```model_controllable.sdf```), but the setup does not yet handle the steps necessary to publish keystrokes.

### Deleting a UAV
To delete the UAV created in the previous step, run the following command:

```
ros2 service call /world/drl_world/remove ros_gz_interfaces/srv/DeleteEntity '{entity: {name: "drone_0", type: 2}}'
```
