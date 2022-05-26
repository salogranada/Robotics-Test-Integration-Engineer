/*! @package lifecycle_manager
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#ifndef LIFECYCLE_MANAGER_H_INCLUDED
#define LIFECYCLE_MANAGER_H_INCLUDED

// ROS2 Default
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

// STD Libraries
#include <memory>

#define TRANSITION_CONFIGURE 1
#define TRANSITION_CLEANUP 2
#define TRANSITION_ACTIVATE 3
#define TRANSITION_DEACTIVATE 4

class LifeCycleManager : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
    /*!
        LifeCycleManager Class constructor. Inherits from rclcpp_cascade_lifecycle::CascadeLifecycleNode
        @param options: rclcpp::NodeOptions.
    */
    explicit LifeCycleManager(rclcpp::NodeOptions &options) : CascadeLifecycleNode("lifecycle_manager", options) {}

    /*!
        LifeCycleManager Class destructor
    */
    ~LifeCycleManager(){};
};
#endif