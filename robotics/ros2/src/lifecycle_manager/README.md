Lifecycle Manager Package
---
### Code Information
**Maintainer:** _[Eng. Davidson Daniel Rojas Cediel](https://www.linkedin.com/in/dadaroce/)_ 

**Mail:** _**davidson@kiwibot.com**_ 

**Kiwi Campus / AI & Robotics Team**

---
<p align="center">
    <img src="https://user-images.githubusercontent.com/39452483/127695635-054f97fe-f4d0-434d-93c9-4233e5fcc17e.png"/>
</p>

<p align="center">
    Lifecycle Manager Package

---
---

```
📦lifecycle_manager
 ┣ 📂include
 ┃ ┗ 📂lifecycle_manager
 ┃ ┃ ┗ 📜lifecycle_manager.hpp
 ┣ 📂src
 ┃ ┗ 📜lifecycle_manager.cpp
 ┣ 📜CMakeLists.txt
 ┣ 📜README.md
 ┗ 📜package.xml
```

---
---

## Node Responsibilities

This node is shown as **lifecycle_manager**. It is in charge of:

- Handle the incoming signal to sleep down or wake up nodes in Save Battery Mode

## Node functionality description

Lifecycle Manager package is a handler to control the `cascade` function of lifecycle to sleep down or wake up nodes.

This Node implements the `rclcpp_cascade_lifecycle::CascadeLifecycleNode` package to interconnect several nodes and perform actions over all of them at the same time. This kind of signal come from [sidis_fsm](../sidis_fsm/) and sent by the `stand` state.

## Nodes

### Main node
 
```
lifecycle_manager.cpp
```

Create a Empty  ```lifecycle_manager``` whose links the selected nodes.

```.cpp
#include "lifecycle_manager/lifecycle_manager.hpp"
```


### Node composition


1. Header of each c++ nodes.
```.cpp
/*! @package lifecycle_manager
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/
```

2. Node init and object creation

```.cpp 
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto lifecycle_node = std::make_shared<LifeCycleManager>(options);
```

3. Executor definition:

```.cpp
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(lifecycle_node->get_node_base_interface());
```

4. `cascade lifecycle` functionality to trigger transitions by code
```.cpp
    lifecycle_node->trigger_transition(TRANSITION_CONFIGURE);
    lifecycle_node->trigger_transition(TRANSITION_ACTIVATE);
```

_Available transitions:_
```.cpp
    #define TRANSITION_CONFIGURE 1
    #define TRANSITION_CLEANUP 2
    #define TRANSITION_ACTIVATE 3
    #define TRANSITION_DEACTIVATE 4
```
> These transitions don't fit when the State ID. More information: [lifecycle](https://github.com/ros2/demos/tree/master/lifecycle)



In order to trigger a transition, we call the change_state service:

```.bash
$ ros2 service call /lifecycle_manager/change_state lifecycle_msgs/ChangeState "{transition: {id: 4}}"

requester: making request: lifecycle_msgs.srv.ChangeState_Request(transition=lifecycle_msgs.msg.Transition(id=4, label=''))

response:
lifecycle_msgs.srv.ChangeState_Response(success=True)
```

Check the entire documentation:

```.bash
$ ros2 interface show lifecycle_msgs/msg/Transition
```


5. Link the nodes to our `lifecycle_manager` node

```.cpp
    lifecycle_node->add_activation("speed_controller");
    lifecycle_node->add_activation("wheel_odometry");
```

6. Spin the executor and shutdown when the program finish
```.cpp
    executor->spin();

    rclcpp::shutdown();
```

---
---

## Node dependencies

This node uses the following dependencies: 
- [rclcpp_cascade_lifecycle::CascadeLifecycleNode](https://github.com/fmrico/cascade_lifecycle/tree/galactic-devel) 
- [sidis_fsm](../sidis_fsm/) 
- [lifecycle](https://github.com/ros2/demos/tree/master/lifecycle)

---
