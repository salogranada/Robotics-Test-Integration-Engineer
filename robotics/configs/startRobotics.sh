#!/bin/bash
# /etc/init.d/startBot

# -----------------------------------------------------------------------------
# clear
# exit script on any error

set -e

function parse_yaml {
   local s='[[:space:]]*' w='[a-zA-Z0-9_]*' fs=$(echo @|tr @ '\034')
   sed -ne "s|^\($s\):|\1|" \
        -e "s|^\($s\)\($w\)$s:$s[\"']\(.*\)[\"']$s\$|\1$fs\2$fs\3|p" \
        -e "s|^\($s\)\($w\)$s:$s\(.*\)$s\$|\1$fs\2$fs\3|p"  $1 |
   awk -F$fs '{
    indent = length($1)/2;
    vname[indent] = $2;
    for (i in vname) {if (i > indent) {delete vname[i]}}
    
    if (length($3) > 0) {
        vn=""; for (i=0; i<indent; i++) {vn=(vn)(vname[i])("_");}
        printf("%s%s=%s\n", vn, $2, $3);
    }
}'
}

BUILD=1
LAUNCH=1
for i in "$@"
do
    case $i in
        -b|--build) BUILD="${2:-1}"
        ;;
        -l|--launch) LAUNCH="${2:-1}"
        ;;
        *)
        ;;
    esac
    shift
done

# -----------------------------------------------------------------------------
# remove empty log files, which are created for no reason (We dont know yet)

find . -name "*.log" -type f -delete

#  ----------------------------------------------------------------------
# Starting

echo  "Starting Robot"
cd /workspace/robotics

#  ----------------------------------------------------------------------
# Source enviroment variables

source "${PWD%}/configs/env_vars.sh"

#  ----------------------------------------------------------------------
# Delete previous workspaces

if [ ! "$DELETE_BUILD" == "0" ] 
then
  echo  [WARN]: "ROS2 Removing old shit ... "
  rm -r ${PWD%}/ros2/install || true
  rm -r ${PWD%}/ros2/build || true
  rm -r ${PWD%}/ros2/log || true
fi

#  ----------------------------------------------------------------------
#  Build ROS2 packages

cd ${PWD%}/ros2 
. /opt/ros/${ROS_DISTRO}/setup.sh

if [ ! "$BUILD" == "0" ]; then
  echo  "[INFO]: ROS2 Building new stuff ... "

  # Extract nodes to build
  YAML_FILE="/workspace/robotics/configs/nodes_launch.yaml"
  if [ -f "$YAML_FILE" ]; then
    declare -a yaml2launch
    declare -a yaml_packages
    # Extract packages"
    package=""
    yaml2launch=($(parse_yaml $YAML_FILE | sed '/from_launch/d' | grep "_launch=1")) || yaml2launch=()
    if [ ! "${#yaml2launch[@]}" == "0" ] ;then
      for i in "${yaml2launch[@]}"
      do
          NODE=${i%_launch*}_package
          package=$(parse_yaml $YAML_FILE | grep $NODE)
          # Extract package name
          package=${package#*_package=}
          # Add packages if not in the array
          if [[ ! " ${yaml_packages[*]} " =~ " ${package} " ]]; then
              yaml_packages[${#yaml_packages[@]}]=$package
          fi
      done
      echo "[INFO]:Packages to build: ${yaml_packages[@]}"
      colcon build --symlink-install --packages-up-to ${yaml_packages[@]}
    else
      echo "[ERROR]: No Launch field set to any node" 
    fi
  else
    colcon build --symlink-install
  fi
  echo  "[INFO]: ROS2 Build successful ... "
fi
echo  "[INFO]: ROS2 sourcing ... "
source /workspace/robotics/ros2/install/setup.bash || true

if [ ! "$LAUNCH" == "0" ]; then
  #  ---------------------------------------------------------------------
  #  ROS2 Launching
  echo  "[INFO]: ROS2 launching ... "
  ros2 launch "/workspace/robotics/configs/robotics.launch.py"
fi

#  ----------------------------------------------------------------------

exit 0
