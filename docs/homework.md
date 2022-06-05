<!-- ---------------------------------------------------------------------- -->

---
# FINAL PROJECT
NAME: **WRITE YOUR NAME HERE**

---
**INSTRUCTIONS**:
1. Fork this repository, and associate it with your GitHub account.
2. Create a new branch name as `develop` and work all your solutions there.
3. Every time that you complete a basic point create a commit, add the files and push with the next format description: `[FEAT]: description of what you did`.
4. You can do your last commit and push until and before the deadline (date & time) if you do more after, we will make a reverse of your code until the last commit before the dead-line date-time (If the project is not complete there, it'll be rejected).
5. Remember partial solutions will be no accepted, you have to complete at least **BASIC POINTS**, **Questionnaire**, but we consider the real project the **EXTRA-HOMEWORK** session (Without it your chances will be less).
6. Rosbag is located in [rosbag](/robotics/configs/subset/). You should source `. /opt/ros/galactic/setup.bash` and `. /workspace/robotics/ros2/install/setup.bash` in order to be able to check all the topics. 

> Note: the last one is generated once you build `usr_msgs` package

--------
**BASIC POINTS**:

Here are the basic points to accomplish the project, what we are evaluating you is the knowledge in python, C++, ROS2 in a basic level of concepts and implementation, and also a control, logical and algorithms. This part of the project is more for the understanding of itself because we invite you to make the extra-homework, which is the real challenge, where you'll have to be more creative and go further. 

Well, the recommendation in this section is to start with the C++ nodes or points and then go with the Python ones. You won't have to change the input or output attributes in any function or method, just implement functions contents, and operations, import a library and use it inside (this rule doesn't apply in the extra-homework points).

Try first to understand the function of the node in the whole stack, then what is every function for, read the documentation and docs provided, and look for the To-do sections in these.


Running the stack from the root, remember the command is:

```
koda@airobotics:/workspace$ bash robotics/configs/startRobotics.sh
```



## *C++* 

**The input topics will be supplied by a `rosbag`. Such as:**

- `/motion_control/speed_controller/reference_cmd`
- `/imu/data`
- `/uavcan/chassis/motors_rpm_feedback`

### Interfaces

This node is shown as **interfaces**. It is in charge of:

* Perform control of **__Physical Devices__** used by the Project (audio).
* Play an ambient sound when the stack is running
* Play sounds according to some events

First, you need to modify `robotics/configs/nodes_launch.yaml` files to build the `interfaces` node. 

Don't worry almost every C++ will fail when you try to compile, but that is part of the below exercises. Remember, if you won't use a node deactivate setting 0 in the launch parameter.

  1. Identify the `NODE_INTERFACES` object on the `robotics/configs/nodes_launch.yaml` file: 

  ```.yaml
  NODE_INTERFACES:
    launch: 0
    node_executable: interfaces_node
    output: screen
    package: interfaces
  ```

  2. Change launch parameter. In order to indicate that you'll build and launch this file: 
  
  ```.yaml
  NODE_INTERFACES:
    launch: 1
    node_executable: interfaces_node
    output: screen
    package: interfaces
  ```

1.  **Create a subscriber inside the speaker node** - package: [`interfaces`](../robotics/ros2/src/interfaces) file to modify: `interfaces/src/modules/speaker.cpp` -> Implement a subscriber `Amazing Speaker Subscriber` type: `std_msgs::msg::Int8` related to the CallBack Function `speakerCb`, the publisher come from `node_robotics` and publish a Int8 message to indicated which track will play. **Note:** Use the `m_speaker_sub` defined in the `.hpp` file to create your subscriber.

2. **Create a publisher inside the speaker node** - package: [`interfaces`](../robotics/ros2/src/interfaces) file to modify: `interfaces/src/modules/speaker.hpp` -> Define a publisher `Amazing Speaker Publisher` type: `std_msgs::msg::Bool` call it ```m_done_pub```. 

3. **Publish a Bool message using the previous publisher created** - package: [`interfaces`](../robotics/ros2/src/interfaces) file to modify: `interfaces/src/modules/speaker.cpp` -> Use the documentation provided and your logic to discover how to publish a UniquePtr message in a simple publisher, which publish a True value each time a sound is ended and False each time a sound begin. Use the publisher called `m_done_pub`.

4. **Create a condition when a file isn't found** - package: [`interfaces`](../robotics/ros2/src/interfaces) file to modify: `interfaces/src/modules/speaker.cpp` -> When a soundtrack isn't found, play the default sound called `track2` located in the same folder as the another ones. You could test this point using `ros2 topic pub -1 /device/speaker/command std_msgs/Int8 "data: 2"`.

### Wheel Odometry

Wheel Odometry stands for the calculation of the vehicle motion based on the information provided by wheel encoders which represents the wheel velocities (Directly related with the Robot motion).

For simplicity of our system and controllers, our current chassis is represented as a differential mobile robot. This representation is widely used in robotics as it keeps thing simple. 

Our wheel Odometry calculation relies on the data provided by the motors (RPM for extract the velocity and error for handling undesired data); however, we also use the IMU angular velocity to calculate a slip factor and hence improve the overall Odometry. Take care, this node was implemented using [CascadeLifecycleNode](https://github.com/fmrico/cascade_lifecycle) instead of pure `Node`.

First, you need to modify `robotics/configs/nodes_launch.yaml` files to build the `wheel_odometry` node. 

Don't worry almost every C++ will fail when you try to compile, but that is part of the below exercises. Remember, if you won't use a node deactivate setting 0 in the launch parameter.

  1. Identify the `NODE_MOTION_SPEED_CONTROL` object on the `robotics/configs/nodes_launch.yaml` file:

  ```.yaml
    NODE_WHEEL_ODOMETRY:
        launch: 0
        node_executable: wheel_odometry
        package: wheel_odometry
        respawn: true
        respawn_delay: 5.0
  ```

  2. Change launch parameter. In order to indicate that you'll build and launch this file: 
  
  ```.yaml
    NODE_WHEEL_ODOMETRY:
        launch: 1
        node_executable: wheel_odometry
        package: wheel_odometry
        respawn: true
        respawn_delay: 5.0
  ```

1.  **Calculate the linear velocity for each wheel** - package: [`wheel_odometry`](../robotics/ros2/src/wheel_odometry) file to modify: `wheel_odometry/src/wheel_odometry.cpp` -> Calculate the linear velocity for each wheel based on the explication inside the [`README.md`](../robotics/ros2/src/wheel_odometry/README.md). The RPMs signs were previously corrected, so use the rpm values as they are given. The RPMs are provided by the subscriber callback `WheelOdometry::MotorsRPMCb` and the calculation should be done inside the `WheelOdometry::CalculateOdometry` function.

2. **Calculate final X and Y values** - package: [`wheel_odometry`](../robotics/ros2/src/wheel_odometry) file to modify: `wheel_odometry/src/wheel_odometry.cpp` -> Calculate the `X` and `Y` final positions based on the explication inside the [`README.md`](../robotics/ros2/src/wheel_odometry/README.md). The calculation should be done inside the `WheelOdometry::CalculateOdometry` function and in the same way using the `X_dot` and `Y_dot` variables, calculate the global X and Y values using the `X_dot_global` and `Y_dot_global`.

3. **Calculate total distance** - package: [`wheel_odometry`](../robotics/ros2/src/wheel_odometry) file to modify: `wheel_odometry/src/wheel_odometry.cpp` -> Calculate the total distance based on the `X` and `Y` positions [`Reference`](https://answers.ros.org/question/337813/distance-traveled-by-robot-using-odometry/). The calculation should be done inside the `WheelOdometry::CalculateOdometry` function.

### PID Controller and Speed Controller

Speed controller is in charge of driving the robot at the desired linear and angular velocity based on the desired linear and angular velocities. It is done through a PID controller which takes as a reference the velocity commands and compares it with the current velocity measured with wheel Odometry. The output of this node must be the desired linear and angular velocity commands. Take care, this node was implemented using [CascadeLifecycleNode](https://github.com/fmrico/cascade_lifecycle) instead of pure `Node`.

First, you need to modify `robotics/configs/nodes_launch.yaml` files to build the `speed_controller` node.

Don't worry almost every C++ will fail when you try to compile, but that is part of the below exercises. Remember, if you won't use a node deactivate setting 0 in the launch parameter.

  1. Identify the `NODE_MOTION_SPEED_CONTROL` object on the `robotics/configs/nodes_launch.yaml` file:

  ```.yaml
    NODE_MOTION_SPEED_CONTROL:
        launch: 0
        node_executable: speed_controller
        package: motion_control
        respawn: true
        respawn_delay: 5.0
  ```

  2. Change launch parameter. In order to indicate that you'll build and launch this file: 
  
  ```.yaml
    NODE_MOTION_SPEED_CONTROL:
        launch: 1
        node_executable: speed_controller
        package: motion_control
        respawn: true
        respawn_delay: 5.0
  ```

1.  **Implement a Throttle PID controller** - package: [`motion_control`](../robotics/ros2/src/motion_control) file to modify: `motion_control/src/pid_controller.cpp` -> Implement a PID controller using the reference, current velocity and dt, all these values are currently sent by the main node `speed_controller` to the function `PIDController::ThrottlePID`. Requirements:
    - The variable: `m_throttle_ctrl` define if the controller is used or not. This variable is controlled by the env_var `SPEED_CONTROLLER_THROTTLE_CONTROL`. In case the controller was set to not use it, return the same reference value.
    - In case the ref velocity was 0 return immediately 0 and reset the current integral error.
    - Calculate the output using the member variables:  `m_vx_int_error` and `m_vx_prop_ek1` to storage the accumulated integral error and the previous proportional error. 
    - Use the constant variables: `m_kp_thr`, `m_ki_thr`, and `m_kd_thr` to calculate the final output.

2.  **Implement a Steering PID controller** - package: [`motion_control`](../robotics/ros2/src/motion_control) file to modify: `motion_control/src/pid_controller.cpp` -> Implement a PID controller using the reference, current velocity and dt, all these values are currently sent by the main node `speed_controller` to the function `PIDController::SteeringPID`. Requirements:
    - The variable: `m_steering_ctrl` define if the controller is used or not. This variable is controlled by the env_var `SPEED_CONTROLLER_STEERING_CONTROL`. In case the controller was set to not use it, return the same reference value.
    - In case the ref velocity was 0 return immediately 0 and reset the current integral error.
    - Calculate the output using the member variables: `m_wz_int_error` and `m_wz_prop_ek1` to storage the accumulated integral error and the previous proportional error. 
    - Use the constant variables: `m_kp_str`, `m_ki_str`, and `m_kd_str` to calculate the raw output.
    - Calculate the output using a FeedForward controller

    <p align="center">
    <img height="300" src="https://user-images.githubusercontent.com/39452483/170380717-55b76256-47c9-4e5c-aebf-cb6c456a252d.png">
    </p>


3.  **Calculate the error between reference and current velocities and fill a TwistStamped message** - package: [`motion_control`](../robotics/ros2/src/motion_control) file to modify: `motion_control/src/speed_controller.cpp` -> Requirements:
    - You should use the variable `output_error_msg`. This variable is a `std::shared_ptr<geometry_msgs::msg::TwistStamped>`, please Don't CAST this to another type of variable like a simple `geometry_msgs::msg::TwistStamped` message.
    - Only fill the header `stamp` field in the `TwistStamped` message
    - Calculate the difference between the reference linear and angular velocities and the provided by the Odometry.
    - Publish the message using the Publisher: `m_ctrl_error_pub`
## *Python*

### Amazing Plotter

Plotter node is a graph tool to visualize data in real time.

First, you need to modify `robotics/configs/nodes_launch.yaml` files to build the `plotter_node` node. 

  1. Identify the `NODE_PLOTTER` object on the `robotics/configs/nodes_launch.yaml` file:

  ```.yaml
    NODE_PLOTTER:
        launch: 0
        node_executable: plotter_node
        package: plotter
        respawn: true
        respawn_delay: 5.0
  ```

  2. Change launch parameter. In order to indicate that you'll build and launch this file: 
  
  ```.yaml
    NODE_PLOTTER:
        launch: 1
        node_executable: plotter_node
        package: plotter
        respawn: true
        respawn_delay: 5.0
  ```

1.  **Create a Thread for spin the Node** - package: [`plotter`](../robotics/ros2/src/plotter) file to modify: `plotter/plotter_node.py` -> create a Thread that point to the `spin_node` function. Located in the `main` function.

2. **Initialize the second subplot** - package: [`plotter`](../robotics/ros2/src/plotter) file to modify: `plotter/plotter_node.py` -> initialize a second subplot to show the `angular signal` and `angular error`. Located in the `__init__` function.

3. **Create a third subplot to show the RPMs for each motor** - package: [`plotter`](../robotics/ros2/src/plotter) file to modify: `plotter/plotter_node.py` -> Create a third plot to show the RPM for each motor. Located along the code.
  - Legends for each motor
  - A different color for each motor

Reference Image
  <p align="center">
  <img height="400" src="https://user-images.githubusercontent.com/39452483/170585356-1d7a64e2-7920-465f-985d-2b31f383f2d4.png">
  </p>

---
**REMEMBER**: 
1. If you push just one minute after the time given by email we won't review your project solution (at least, that commit).
2. If you find an error in the code, bug, drawback, and there's no already an issue created in the main repository, please create it, and you win extra points (+3%/5: **this is 3% more over 5.0, which is equal to 0.15 in the final project's grade**), we'll try to give a solution ASAP.
3. You have 3 questions that can be done as an issue in the main repository, so please check that what you are asking for is not already answered in the issues section (don't waste your questions).
4. It's forbidden talk with other project participants, if we think that you cheated, or you cheated, your project and the others (if apply) won't be reviewed and your application will be canceled.
5. We'll push the solution when the application job closes, we'll notify all participants of this by email.
6. What we are evaluating is:
    * **[5%/5]** Code style
    * **[5%/5]** Code Documentation
    * **[15%/5]** Questionnaire
    * **[25%/5]** Solution to basic home-work
    * **[50%/5]** Solution review & answers
7. You can have a grade higher than 5.0.
8. We'll share the final grades with every candidate by email, with feedback included of his/her application (things to improve, to keep, to remove). The application process percentages are:
    * **[10%/5]** Experience Fit
    * **[10%/5]** Cultural feed Interview
    * **[25%/5]** Pair programming Interview
    * **[25%/5]** Knowledge Test
    * **[30%/5]** Final Project
9. We'll select the 3 best candidates by their grades, and we'll make the final decision with the Ai&Robotics Team.
10. We are evaluating your concepts and knowledge in ROS2, Python, C++, Some basic control concepts, other Code stuff.
11. If no participant send the project before the deadline time, we will extend the deadline date and time for everyone. However, just the people that complete the questionnaire and the basic-points the rest are doomed. Also, if we considered that the solutions received are not enough or not well explained we also can extend the deadline, but we are pretty sure that this won't happen.

---
<!-- ---------------------------------------------------------------------- -->
## **Questionnaire**

Respond below in the same solution branch every question. In case your answer isn't in this file, it'll not be valid:

1. [C++] What is the mean of the number "15" used in the `pthread_kill` inside the destructor method?

2. [C++] Why are we using `UniquePointer` instead of `SharedPointers` to publish a ROS2 message?

Using `UniquePointer` allow us to handle allocation in memory in a more organized way. That way, our program won't have unnecessary copies of the published message in memory. The reference will be unique and can go through the system safely. This will result in better performance for programs as Kiwibot's.

Reference: https://docs.ros.org/en/humble/Tutorials/Intra-Process-Communication.html 
          https://herbsutter.com/2013/06/05/gotw-91-solution-smart-pointer-parameters/ 

3. [Logic] Why are we using an m_multi_sound variable? Explain ...

We need the m_multi_sound variable because, as AmbientSound is running through a Thread, it will confirm if something is already playing. m_multi_sound changes to 1 when the sound has stopped, therefore it will allow us (in the IF that it is used) to write into the pcm handle as nothing else is playing. 

4. [C++] Why are we freeing the memory allocated by raw pointer "buff" variable and not freeing the memory allocated by the Shared and Unique Pointers? (HARD)

Because the Shared and Unique Pointers are Smart Pointers were their lifetime is managed. They will actually be free/killed as soon as the program is out of the scope. Meanwhile the "buff" as it is a raw pointer we have to explicitly destroy the object.

5. [C++] Why should we use a "member variable" (persistent over the class) to storage the integral error? `m_vx_int_error`

6. [Control] What is the function of the FeedForward controller?

A FeedFoward controller will allow us to handle Setpoint changes and disturbances before they enter into the system. 
Setpoint changes are somehow predictable as we with some anteriority know the change is going to happen and disturbances are measurable. Instead of allowing error and afterwards controlling it with the Feedback Controller we can predict/measure the change and handle it with the FeedFoward controller beforehand and add it to the output of the Feedback control. That way we can decrease hard changes in the output of the system and run smoothly into our setpoint. 

Reference: https://www.youtube.com/watch?v=FW_ay7K4jPE 

7. [ROS2] What is the purpose of `CascadeLifecycleNode` type nodes?

8. [Robotics] Why is a global and a local `Odometry` calculated?

Global and local Odometry is needed because we manage two frames of reference, world(global) and robot(local). With the local we will be able to know how much the robot has moved with respect to its initial position (0,0) in its own reference, but we also need the global to know the robots position in the world.
Therefore is needed to make Transforms to get to know one's frame position with respect to the other. Libraries such as tf2 really help with this process, that I can see is been used.


9. [Robotics] If the robot has 4 differential wheels, what type of chassis is it?

10. [Docker] Explain with your own words what is the instructions `apt-get autoremove && apt-get clean -y` for?

Autoremove will allow us to remove files that were needed for installation but that we no longer need them, while clean clears out cache from retrieved packages. The -y option is just for auto-answering any prompted questions that may appear when we run the commands.

Reference: http://manpages.ubuntu.com/manpages/kinetic/en/man8/apt-get.8.html 

11. [Docker] If you modify a layer what happens with the previous and the next ones?

12. [Docker] Can we change the basic image (`FROM ubuntu:20.04`) from the docker file to another?

We could but, we would have to build again the docker image, and will result in creating a new container from that new image. Also, depending on the layers that follow, we will have to check if every instruction is still persistent with that change. 

13. [C++] What is the [libsoft_speed.a](../robotics/ros2/src/motion_control/lib/) file and what is it for?

14. [Python] Why should we use a thread to spin the node?

15. [Python] Why is the limit on the Y-RPM graph 170?

Next questions are after you finish the project, it doesn't give points, but we really appreciate you feedback:
* What do you think about this project? Is it hard or enough? Is it to complicated, is it well structure, explanations and instructions are clear?

---
<!-- ---------------------------------------------------------------------- -->
## **EXTRA-HOMEWORK**

For extra homework you should create a new branch from the developed one when you finish the basic points of the homework, you can name this new branch as *feature/extra_homework*, don't forget to push it before the time that it was given.

1. **[+5%/5.0]**: Modify the docker file to source ROS2 and have autocompleted commands like ```ros2 topic list```.
2. **[+10%/5.0]:** Make that Kiwibot track2.wav don't get distorted. Use: `ros2 topic pub -1 /device/speaker/command std_msgs/Int8 "data: 2"`
3. **[+20%/5.0]:** Transform the `rpm_converter` `Node` to a [CascadeLifecycleNode](https://github.com/fmrico/cascade_lifecycle)
4. **[+5%/5.0]:** Create a Dockerfile to build OpenCV from scratch based on the `ubuntu:20.04` public image
5. **[+10%/5.0]:** Integrate an Anti-Windup based on the max. linear speed 
6. **[+10%/5.0]:** Play a sound if the RPM feedback increase X value and another when the RPM decrease X value. Define by yourself the X value
7. **[+20%/5.0]:** Create a Node to fill a `LocationMsg.msg` [message](../robotics/ros2/src/usr_msgs/msg/location/LocationMsg.msg) and publish to a topic `/custom_gps` of the same type using the incoming GPS signals from two topics `/wifi_geo/fix -> from the router gps` and `/fix -> from the main gps` and `/imu/data -> from the IMU`. The `/wifi_geo/fix` has priority over the another one, but this signal is only published sometimes and after 20 seconds without receiving a new message from the wifi side the `/fix` signal take again the place. Note: the `/fix` signal is constantly publishing data.
8. **[+5%/5.0]**: Add the `DELETE_BUILD` argument to the `startRobotics.sh` file (check line 61). Options : `--delete-build / -d`. Example: `startRobotics.sh -d 1` will delete the previous generated install, build, and log folders. Use 0 as default value.
9. **[+5%/5.0]**: use `Valgrind` to analyses the `speed_controller` executable. [How to use it](./valgrind_usage.md). Why use `Valgrind`?
10. **[+15%/5.0]**: The plotter node graph just until 10000 in the X axis, look for a way to dynamically change this value whilst the data is coming. Also, the lower limit (0) should change and keep a distance of 500 for the upper limit.
11. **[Max. +10%/5.0]**: Beauty the plotter. The best will get 10%, second one 5%. Reference Image:

  <p align="center">
  <img height="400" src="https://user-images.githubusercontent.com/39452483/170590011-0220a4be-60fb-4754-bfb1-0b76c1e4b33a.png">
  </p>

Total possible Extra points: 105% -> 5.0. Maximum total grade: 10.25/5.0. Complete the point it doesn't mean you have 5.0, you have at least 3.0 but for the rest of the grade will evaluate the performance and the beauty of your solution. To complete these points, probably you will have to modify messages, services, or even create new ones, also topics subscribers and publishers, maybe services, who knows :smile:

---

<p align="center">
  <img height="300" src="https://user-images.githubusercontent.com/39452483/170424558-2efbe421-727f-4825-9d29-16118c1cacf6.gif">
</p>


*Rafael DO NOT forget erase the [solution branch](https://www.youtube.com/watch?v=dQw4w9WgXcQ)*