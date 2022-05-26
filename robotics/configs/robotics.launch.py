#!/usr/bin/env python3

# =============================================================================
"""
Code Information:
    Maintainer: Eng. Davidson Rojas Cediel
	Mail: davidson@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

    Examples:
    https://github.com/ros2/launch/blob/a89671962220c8691ea4f128717bca599c711cda/launch/examples/launch_counters.py

"""

# =============================================================================
import inspect
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# ============================================================================

class bcolors:
    LOG = {
        "WARN": ["\033[33m", "WARN"],
        "ERROR": ["\033[91m", "ERROR"],
        "OKGREEN": ["\033[32m", "INFO"],
        "INFO": ["\033[0m", "INFO"],  # ['\033[94m', "INFO"],
        "BOLD": ["\033[1m", "INFO"],
        "GRAY": ["\033[90m", "INFO"],
    }
    BOLD = "\033[1m"
    ENDC = "\033[0m"
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    GRAY = "\033[90m"
    UNDERLINE = "\033[4m"


def printlog(msg, msg_type="INFO", flush=True):
    org = os.path.splitext(os.path.basename(inspect.stack()[1][1]))[0].upper()
    caller = inspect.stack()[1][3].upper()
    _str = "[{}][{}][{}]: {}".format(bcolors.LOG[msg_type][1], org, caller, msg)
    print(bcolors.LOG[msg_type][0] + _str + bcolors.ENDC, flush=flush)


def read_node_launch(default_nodes, default_yml_file="nodes_launch.yaml"):

    CONF_PATH = os.path.dirname(os.path.abspath(__file__))
    FILE_PATH = os.path.join(CONF_PATH, default_yml_file)

    if not os.path.exists(FILE_PATH):

        try:
            with open(FILE_PATH, "w") as outfile:
                yaml.dump(default_nodes, outfile, default_flow_style=False)
            printlog(msg="Nodes launch file created", msg_type="WARN")

        except Exception as e:
            printlog(
                msg="Error creating nodes launch file: {}".format(e), msg_type="ERROR"
            )

        return default_nodes

    else:

        try:
            with open(FILE_PATH, "r") as stream:
                default_nodes = yaml.safe_load(stream)
            printlog(
                msg="Nodes local launch file {}".format(default_yml_file),
                msg_type="OKGREEN",
            )

        except Exception as e:
            printlog(
                msg="Error reading nodes launch file: {}".format(e), msg_type="ERROR"
            )

        return default_nodes


def generate_launch_description():

    LOCAL_SIMULATOR = int(os.getenv(key="LOCAL_SIMULATOR", default=0))
    LOCAL_LAUNCH = int(os.getenv(key="LOCAL_LAUNCH", default=0))

    ld = LaunchDescription(
        [
            LogInfo(msg="Launching Kiwibot ROS2 ..."),
        ]
    )

    # -------------------------------------------------------------------------
    # Default nodes to launch
    respawn_nodes = bool(int(os.getenv(key="RESPAWN_NODES", default=1)))
    respawn_delay = float(os.getenv(key="RESPAWN_DELAY", default=5))
    nodes = {
        # ---------------------------------------------------------------------
        # Data Capture
        "NODE_DATA_CAPTURE": {
            "node_executable": "data_capture",
            "package": "data_capture",
        },
        # ---------------------------------------------------------------------
        # Control nodes
        "NODE_MOTION_SPEED_CONTROL": {
            "node_executable": "speed_controller",
            "package": "motion_control",
        },
        # ---------------------------------------------------------------------
        # Wheel Odometry Package
        "NODE_WHEEL_ODOMETRY": {
            "node_executable": "wheel_odometry",
            "package": "wheel_odometry",
        },
        # ---------------------------------------------------------------------
        # Location package
        "NODE_LOCATION": {
            "node_executable": "location",
            "package": "location",
        },
        # ---------------------------------------------------------------------
        # Interfaces
        "NODE_INTERFACES": {
            "node_executable": "interfaces_node",
            "package": "interfaces",
        },
        # ---------------------------------------------------------------------
        # Lifecycle Manager
        "NODE_LIFECYCLE_MANAGER": {
            "node_executable": "lifecycle_manager",
            "node_name": "lifecycle_manager",
            "package": "lifecycle_manager",
        },
        # ---------------------------------------------------------------------
        # Lifecycle Manager
        "NODE_RPM_CONVERTER": {
            "node_executable": "rpm_converter_node",
            "node_name": "rpm_converter_node",
            "package": "rpm_converter",
        },
        # ---------------------------------------------------------------------
    }

    # Add 'launch' and 'respawn' arguments to nodes kwargs
    for node_name, node_kwargs in nodes.items():
        if "launch" not in node_kwargs:
            # Add launch parameter using env variable based on its name
            node_kwargs["launch"] = int(os.getenv(key=node_name, default=0))

        # Only add respawn to pure nodes, not from launchfiles
        if "from_launch" not in node_kwargs:
            node_kwargs["respawn"] = respawn_nodes
            node_kwargs["respawn_delay"] = respawn_delay

    nodes = read_node_launch(default_nodes=nodes)

    # -------------------------------------------------------------------------
    # Get launch description

    # Print nodes to launch
    srt_launched = "\n\nLAUNCHED:"
    srt_launched_exe = ""
    srt_no_launched = "\n\nNO LAUNCHED:" + "\033[93m"
    srt_no_launched_exe = ""
    ljust = 18

    for key, node_args in nodes.items():
        if node_args["launch"]:
            if "node_name" in node_args.keys():
                srt_launched = srt_launched + "\n\t[package]: {} \t[Node]: {}".format(
                    node_args["package"][:10].ljust(ljust),
                    node_args["node_name"],
                )
            elif "node_executable" in node_args.keys():
                srt_launched_exe = (
                    srt_launched_exe
                    + "\n\t[Package]: {} \t[Executable]: {}".format(
                        node_args["package"].ljust(ljust),
                        node_args["node_executable"],
                    )
                )
            else:
                srt_launched_exe = srt_launched_exe + "\n\t[Package]: {}".format(
                    node_args["package"],
                )
        else:
            if "node_name" in node_args.keys():
                srt_no_launched = (
                    srt_no_launched
                    + "\n\t[package]: {} \t[Node]: {}".format(
                        node_args["package"][:10].ljust(ljust),
                        node_args["node_name"],
                    )
                )
            elif "node_executable" in node_args.keys():
                srt_no_launched_exe = (
                    srt_no_launched_exe
                    + "\n\t[Package]: {} \t[Executable]: {}".format(
                        node_args["package"].ljust(ljust),
                        node_args["node_executable"],
                    )
                )
            else:
                srt_no_launched_exe = srt_no_launched_exe + "\n\t[Package]: {}".format(
                    node_args["package"],
                )

    ld = LaunchDescription(
        [
            LogInfo(
                msg=srt_launched
                + srt_launched_exe
                + srt_no_launched
                + srt_no_launched_exe
                + "\033[0m"
                + "\n"
            ),
        ]
    )

    # -------------------------------------------------------------------------
    for key, node_args in nodes.items():
        # This is for C++ packages
        if "from_launch" in node_args.keys():
            if node_args["launch"] and node_args["from_launch"]:
                if "package" in node_args.keys():
                    launch_dir = get_package_share_directory(
                        "{}".format(node_args["package"])
                    )
                    launch_arguments = node_args.get("launch_arguments", None)

                    included_launch = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            launch_dir + "/launch/{}.py".format(node_args["file"])
                        ),
                        launch_arguments=launch_arguments.items()
                        if launch_arguments
                        else launch_arguments,
                    )
                    ld.add_action(included_launch)
        # For python packages
        elif "from_file" in node_args.keys():
            if node_args["launch"]:
                included_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.dirname(os.path.abspath(__file__))
                        + "/{}.py".format(node_args["file"])
                    ),
                )
                ld.add_action(included_launch)

        else:
            if node_args["launch"]:
                # Add arguments for log level based on its key, node name identifier
                # Ex: NODE_DATA_CAPTURE -> NODE_DATA_CAPTURE_LOG_LEVEL
                log_level_var = f"{key}_LOG_LEVEL"
                log_level_arguments = [
                    "--ros-args",
                    "--log-level",
                    os.getenv(
                        log_level_var, os.getenv("RCUTILS_LOGGING_LEVEL", "INFO")
                    ),
                ]
                # If node already has arguments, just add them
                if "arguments" in node_args:
                    node_args["arguments"] += log_level_arguments
                else:
                    node_args["arguments"] = log_level_arguments

                prefix_var = f"{key}_PREFIX"
                extra_prefix = os.getenv(prefix_var, "")
                nodes_prefix = os.getenv("NODES_PREFIX", "")
                if not extra_prefix:
                    extra_prefix = nodes_prefix

                if extra_prefix:
                    prefix = f"bash -c '{extra_prefix}; stdbuf -o L $0 $@'"
                else:
                    prefix = "stdbuf -o L"

                ld.add_action(
                    Node(
                        executable=node_args["node_executable"],
                        name=node_args.get("node_name", None),
                        package=node_args["package"],
                        output="screen",
                        respawn=node_args.get("respawn", False),
                        respawn_delay=node_args.get("respawn_delay", None),
                        arguments=node_args.get("arguments", None),
                        prefix=prefix,
                        emulate_tty=True,
                        parameters=node_args.get("parameters", None),
                    )
                )

    # -------------------------------------------------------------------------
    return ld

    # ============================================================================
