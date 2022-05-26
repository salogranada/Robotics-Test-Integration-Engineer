# Valgrind usage

* _Responsible [Davidson Daniel Rojas Cediel](https://www.linkedin.com/in/dadaroce/) 
e-mail: davidson@kiwibot.com_

Manual to run valgrind test over cpp files

---

---



## Installation

```bash
sudo apt update
sudo apt install valgrind
```

## Usage

This brief tutorial is oriented to using Valgrind on .cpp files compiled with the use of ROS2 (Colcon build too to be more specific), for that reasons the all-executable paths might be weird.

First, on the root of the development workspace find the install folder (Find Wheel Odometry package as an example). Inside that install folder, you need to search for the specific package name and then the lib sub-folder which contains the executable we want to inspect with Valgrind.

```bash
 cd ros2/install/wheel_odometry/lib/wheel_odometry
```

If the executable requires any shared library (.so), you need to add the path using the environment variable **LD_LIBRARY_PATH**. The following examples provide this linking for the usr_msgs rosidl interfaces. It might be different for other library.

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/workspace/rover/ros2/install/usr_msgs/lib
```

Once all is set, you are able to run Valgrind for the desired executable. Try the following command to get the console report about the memory leak checking.

```bash
valgrind --leak-check=full -v ./wheel_odometry
```

You can use different flags to get a report or perform other memory verification. 

```bash
valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file='output.txt' -v ./wheel_odometry
```

The output from running Valgrind will be a complete report explaining possible memory leaks in our program or even in the shared library.

This output file is spliced into several `blocks`, the most important blocks are:

#### Valgrind headers:

Summary of the passed flags.

<p align="center">
<img src="https://user-images.githubusercontent.com/39452483/144321481-87e79cc1-f224-453f-9a78-20107deb0b9a.png"  width="700"/>
</p>

#### Leak summary:

This block contains the summary of memory leaks found by valgrind, and above it, all the bytes and blocks that could possibly be lost or reachable and the function/library where this was perform.

<p align="center">
<img src="https://user-images.githubusercontent.com/39452483/144320732-91e9d6ba-f1ca-41f0-9ecc-9ddec614b094.png"  width="700"/>
</p>


