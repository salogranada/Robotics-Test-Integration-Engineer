#!/usr/bin/env python3
# =============================================================================
"""! @package plotter_node.py

Code Information:
    Maintainer: Eng. Davidson Daniel Rojas Cediel
	Mail: davidson@kiwibot.com
	Kiwi Campus / AI & Robotics Team
"""

# =============================================================================

# Basics
import threading

# ROS2 dependencies
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

# ROS2 messages
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

# ROS2 Custom Messages
from usr_msgs.msg import MotorsRPM


# Plotter
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class Plotter(Node):
    def __init__(self) -> None:
        """!
        Object class constructor
        """

        super().__init__("plotter_node")

        # =============================================================================
        # Define Plotter Variables
        # =============================================================================
        self.fig, self.ax = plt.subplots(1, 3)
        self.fig.suptitle("Kiwibot's Amazing Plotter", fontsize=18)
        self.fig.set_size_inches(18.5, 10.5)
        # =============================================================================
        # Controller Lines
        # =============================================================================

        # Linear
        (self.control_lin_ln,) = self.ax[0].plot(
            [], [], "r-", linewidth=1, label="Control Linear Signal"
        )
        (self.error_linear_ln,) = self.ax[0].plot(
            [], [], "-.", linewidth=0.5, label="Linear Error", color="#0082F5"
        )
        self.controller_lin_lns = [self.control_lin_ln, self.error_linear_ln]
        self.ax[0].legend()

        self.x_linear_data, self.y_linear_data = [[], []], [[], []]

        # ---------------------------------------------------------------------
        # TODO: Initialize the second subplot
        # Take care about the variables.
        # NOT define new variables use the variables defined along the code
        #
        # Self-contained reference :smile:
        #
        # End Code
        # ---------------------------------------------------------------------

        # Angular Signal sub-plot
        (self.control_ang_ln,) = self.ax[1].plot(
            [], [], "g-", linewidth=1, label="Control Angular Signal"
        )
        (self.error_angular_ln,) = self.ax[1].plot(
            [], [], "-.", linewidth=0.5, label="Angular Error", color="#0082F5"
        )
        self.controller_ang_lns = [self.control_ang_ln, self.error_angular_ln]
        self.ax[1].legend()
        self.x_ang_data, self.y_ang_data = [[], []], [[], []]

        # Motors RPM sub-plot
        (self.rpm_fr,) = self.ax[2].plot([], [], "r", label="FR")
        (self.rpm_rr,) = self.ax[2].plot([], [], ":", label="RR")
        (self.rpm_rl,) = self.ax[2].plot([], [], ":", label="RL")
        (self.rpm_fl,) = self.ax[2].plot([], [], label="FL", color="orange")
        self.controller_rpm = [self.rpm_fr, self.rpm_rr, self.rpm_rl, self.rpm_fl]
        self.ax[2].legend(title="Motors")
        # Initializes list for storing complete rpm data.
        (self.x_rpm_data, self.y_rpm_data) = ([[], [], [], []], [[], [], [], []])
        # =============================================================================
        # ROS2 Stuffs
        # =============================================================================
        # Allow callbacks to be executed in parallel without restriction.
        self.callback_group = ReentrantCallbackGroup()

        # ---------------------------------------------------------------------
        # Subscribers

        self.sub_velocity_cmd = self.create_subscription(
            msg_type=Twist,
            topic="/motion_control/speed_controller/output_cmd",
            callback=self.cb_cmd_vel,
            qos_profile=5,
            callback_group=self.callback_group,
        )

        self.sub_velocity_cmd = self.create_subscription(
            msg_type=TwistStamped,
            topic="/motion_control/speed_controller/error",
            callback=self.cb_error_vel,
            qos_profile=5,
            callback_group=self.callback_group,
        )

        self.sub_rpm = self.create_subscription(
            msg_type=MotorsRPM,
            topic="/uavcan/chassis/motors_rpm_feedback",
            callback=self.cb_rpm_feedback,
            qos_profile=5,
            callback_group=self.callback_group,
        )

    # Hack Function to not block the thread
    def spin_node(self) -> None:
        """!
        Function to spin the node
        """
        rclpy.spin(self)

    # Plotter Functions
    def plot_init(self) -> None:
        """!
        Function to set the initial plot status.
        """
        self.ax[0].set_xlim(0, 10000)
        self.ax[0].set_ylim(-3, 3)
        self.ax[0].set_title("Linear Signal / Linear Error")

        self.ax[1].set_xlim(0, 10000)
        self.ax[1].set_ylim(-3, 3)
        self.ax[1].set_title("Angular Signal / Angular Error")

        self.ax[2].set_xlim(0, 10000)
        self.ax[2].set_ylim(-170, 170)
        self.ax[2].set_title("RPMs")

        return [self.controller_lin_lns, self.controller_ang_lns, self.controller_rpm]

    def update_plot(self, frame=None) -> None:
        """!
        Function to update the current figure
        """
        self.controller_lin_lns[0].set_data(
            self.x_linear_data[0], self.y_linear_data[0]
        )
        self.controller_lin_lns[1].set_data(
            self.x_linear_data[1], self.y_linear_data[1]
        )

        self.controller_ang_lns[0].set_data(self.x_ang_data[0], self.y_ang_data[0])
        self.controller_ang_lns[1].set_data(self.x_ang_data[1], self.y_ang_data[1])

        # Update rpms data as well.
        self.controller_rpm[0].set_data(self.x_rpm_data[0], self.y_rpm_data[0])
        self.controller_rpm[1].set_data(self.x_rpm_data[1], self.y_rpm_data[1])
        self.controller_rpm[2].set_data(self.x_rpm_data[2], self.y_rpm_data[2])
        self.controller_rpm[3].set_data(self.x_rpm_data[3], self.y_rpm_data[3])

        return [self.controller_lin_lns, self.controller_ang_lns, self.controller_rpm]

    # Callback functions
    def cb_cmd_vel(self, msg: Twist) -> None:
        """!
        Callback function to get the velocity control signal.
        @param msg 'Twist' message containing the velocities of the robot
        """
        self.y_linear_data[0].append(msg.linear.x)
        x_index = len(self.x_linear_data[0])
        self.x_linear_data[0].append(x_index + 1)

        self.y_ang_data[0].append(msg.angular.z)
        x_index2 = len(self.x_ang_data[0])
        self.x_ang_data[0].append(x_index2 + 1)

    def cb_error_vel(self, msg: TwistStamped) -> None:
        """!
        Callback function to get the error between the reference and the current velocity
        @param msg 'TwistStamped' message containing the velocities of the robot
        """

        self.y_linear_data[1].append(msg.twist.linear.x)
        x_index = len(self.x_linear_data[1])
        self.x_linear_data[1].append(x_index + 1)

        self.y_ang_data[1].append(msg.twist.angular.z)
        x_index2 = len(self.x_ang_data[1])
        self.x_ang_data[1].append(x_index2 + 1)

    def cb_rpm_feedback(self, msg: MotorsRPM) -> None:
        """!
        Callback function to get motors RPMS feedback
        @param msg 'MotorsRPM' message containing the velocities of the robot
        """
        # Retrieving the information from "usr_msgs/msg/rpm_converter/MotorsRPM.msg" we can access RPM data.
        # Append them to lists so that we can visualize the whole data.

        # Front right motor
        self.y_rpm_data[0].append(msg.rpms_fr)
        x_index = len(self.x_rpm_data[0])
        self.x_rpm_data[0].append(x_index + 1)

        # Rear right motor
        self.y_rpm_data[1].append(msg.rpms_rr)
        x_index2 = len(self.x_rpm_data[1])
        self.x_rpm_data[1].append(x_index2 + 1)

        # Rear left motor
        self.y_rpm_data[2].append(msg.rpms_rl)
        x_index3 = len(self.x_rpm_data[2])
        self.x_rpm_data[2].append(x_index3 + 1)

        # Front left motor
        self.y_rpm_data[3].append(msg.rpms_fl)
        x_index4 = len(self.x_rpm_data[3])
        self.x_rpm_data[3].append(x_index4 + 1)


# =============================================================================
def main(args=None) -> None:
    """!
    Plotter Node's Main
    """

    # Initialize ROS communications for a given context.
    rclpy.init(args=args)

    # Execute work and block until the context associated with the
    # executor is shutdown.
    plotter_node = Plotter()

    # ---------------------------------------------------------------------
    # TODO: Create a Thread for spin the node
    # Use the function spin_node
    # https://realpython.com/intro-to-python-threading/
    spin_thread = threading.Thread(target=plotter_node.spin_node)
    spin_thread.start()
    #
    # End Code
    # ---------------------------------------------------------------------

    ani = FuncAnimation(
        plotter_node.fig, plotter_node.update_plot, init_func=plotter_node.plot_init
    )

    plt.show(block=True)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plotter_node.destroy_node()
    rclpy.shutdown()


# =============================================================================
if __name__ == "__main__":
    main()
# =============================================================================
