<!-- ---------------------------------------------------------------------- -->
## **Running The Dev-Container**
 
If you have your [VSCode](https://code.visualstudio.com/) with the right extensions, and if you have Docker and Docker-compose installed in your system, when you open the project's main folder you'll see a window on the bottom right corner, click in "reopen in container" button, if you don't see anything press `Ctrl+Shift+P` and type `Remote-Containers: Rebuild and Reopen in container` or `Docker-images: Build Image` option. When the container is opened, and executed for the first time or when there are changes on it, you can go for a walk because the building image process will start and it'll take a while due to the installation of all packages and dependencies of the dev-environment as [ROS2](https://index.ros.org/doc/ros2/), [OpenCV](https://opencv.org/), [Python](https://www.python.org/), and more stuff related to, while the process is completed here are some videos of [puppies](https://www.youtube.com/watch?v=mRf3-JkwqfU). You can see at any time the logs of the building process clicking in `Starting with Dev Container` on the bottom right corner. When the process is done you'll see some messages of process succeed.
 
<img src="https://user-images.githubusercontent.com/43115782/87437367-d5806200-c5b3-11ea-9bf2-836e45f46ed8.gif" alt="building_dev-container" width="1200">
 
When the process is done you can open a terminal in the dev-container going to the menu bar `Terminal` and then `New Terminal`. Congratulations now you have everything that we use for our deployments.
 

<br />

<!-- ---------------------------------------------------------------------- -->
## **Architecture**
 
Find the distribution of final project in the next list:
 
- **[robotics](../robotics):** Main folder where most of the source code is located
  - **[configs](../robotics/configs):** Path robotics config files
     - [*startRobotics.sh:*](../robotics/configs/startRobotics.sh) bash script to run stack of the project
     - [*env_vars.sh:*](../robotics/configs/env_vars.sh) local environment variables
     - [*nodes_launch.yaml:*](../robotics/configs/nodes_launch.yaml) file describing which nodes launch or not
  - **[ros2/src](../robotics/ros2/src):** Development workspace & ROS 2 packages
  - **[media](../robotics/media):** Media files for project such images, and audios 

Only some files are listed, and explained (most important).
 
<br />
 

<!-- ---------------------------------------------------------------------- -->
## **Running The Project Stack**
 
Find a brief explanation on how to run our stack in your IDE and the explanation of the launch file, and config files as the key to managing which nodes are going to be launched and how they're going to work.
 
In order to launch locally (Inside your IDE), please locate into the `scripts/` folder in the dev-container terminal and execute the following prompt command:
 
     $ bash startRobotics.sh

*Note:* if you've already launch and compile the whole stack and there's no a *hot* modification inside the stack, it's possible to avoid the entire compiling step running:

     $ bash startRobotics.sh  # [args: -b {build}, -l {launch}]

> There is some arguments to choose whether build/launch nodes or not.
>- Build: ```bash startRobotics.sh -b 0``` or ```bash startRobotics.sh --build 0``` 1 to build 0 to not. IMPORTANT: only the nodes with the `launch` field set to 1 in some of the nodes_xxxx_launch.yaml (according to LOCAL_SIMULATOR variable) will be compiled.
>- Launch: ```bash startRobotics.sh -l 0``` or ```bash startRobotics.sh --launch 0``` 1 to launch 0 to not .

This bash performs the following steps to launch the *Robotics* stack:
 
1. Sources the [`env_vars.sh`](../robotics/configs/env_vars.sh) which contains the *Kiwibot* local environment variables.
2. Build the nodes you've selected in the [*nodes_launch.yaml:*](../robotics/configs/nodes_launch.yaml) file
3. Launch ros2 with the specified node in [*nodes_launch.yaml:*](../robotics/configs/nodes_launch.yaml)

For ROS 2 development workspace

1. Sources ROS Galactic and clean the older development workspace (If enabled).
2. Builds the development workspace at [`robotics/ros2/`](robotics/ros2)
3. Sources the resulting setup in the install folder `. install/setup.bash`
4. Executes [`ros2 launch /configs/robotics.launch.py`](../robotics/configs/robotics.launch.py)

You can compile and launch everything by your own if you already have a background and experience with ROS/ROS2, but for those who want everything easy, and fast the bash script will set up and run everything for you. With the [``startRobotics.sh``](../robotics/configs/startRobotics.sh) bash script you can run the stack of the project, this file has all instruction to download third-party packages, other required dependencies if they're missing, and setup, source, and run the ros2 workspace, compile and launch the nodes specified in the file ``nodes_launch.yaml``.


<!-- ---------------------------------------------------------------------- -->
## Launching Specific Nodes
 
The file ``nodes_launch.yaml``, allows to you to select or config the nodes that you want to run. In ``nodes_local_launch.yaml`` set the argument ``launch`` of every element in the yaml list to 1 or 0 to launch a node or not.


<!-- ---------------------------------------------------------------------- -->
## Troubleshooting

*Note (Audio is not reproducing IN THE SOLUTION VERSION):* if you are having troubles getting audio from the virtual environment, please create a issue with the error description. But also check the audio device is not busy, or try connecting and disconnecting headsets in the audio port.

*Note (Launching specific nodes):* Remember, if you want to launch or build just a single node or some of them or the whole stack, go to the file ``nodes_launch.yaml`` and change the key *launch* for 1 to launch and 0 to don't launch. this could be useful when you are testing some nodes or just a single node.

<br />