#!/bin/bash
set -e -x

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
WORKSPACE="$(dirname $(dirname "$SCRIPT_DIR"))"
OTHER_POSSIBLE_WORKSPACE="$(dirname "$WORKSPACE")"

EXPECTED_RELATIVE_INSTALL_PATH="install/setup.bash"

if [ -f "$WORKSPACE/$EXPECTED_RELATIVE_INSTALL_PATH" ]; then
    echo "Found workspace at $WORKSPACE"
elif [ -f "$OTHER_POSSIBLE_WORKSPACE/$EXPECTED_RELATIVE_INSTALL_PATH" ]; then
    WORKSPACE="$OTHER_POSSIBLE_WORKSPACE"
    echo "Found workspace at $WORKSPACE"
else
    echo "Could not find workspace. Expected to find $EXPECTED_RELATIVE_INSTALL_PATH in either $WORKSPACE or $OTHER_POSSIBLE_WORKSPACE"
    exit 1
fi

echo "SCRIPT_DIR: $SCRIPT_DIR"
echo "ZAAPP_DIR: $ZAAPP_DIR"

CONFIG_FILE="$SCRIPT_DIR/launch_config.tmux"
# You can change this to something like "test" to avoid messing up the main tmux server.
SERVER_NAME="zaapp"

set +e
tmux -L $SERVER_NAME kill-session -t zaapp-nodes
set -e

# -L : separate tmux server
# -f : specify config file
# -s : session name
# -d : don't attach yet.
tmux -L $SERVER_NAME -f "$CONFIG_FILE"  new-session -s zaapp-nodes -n husky -d

# When you hit Ctrl-b + q to show pane numbers, they will stay on screen for 30 seconds.
tmux -L $SERVER_NAME set -g display-panes-time 30000

# Base window layout is 4x1
tmux -L $SERVER_NAME split-window -t zaapp-nodes:husky -v
tmux -L $SERVER_NAME split-window -t zaapp-nodes:husky -v
tmux -L $SERVER_NAME split-window -t zaapp-nodes:husky -v

# Core window layout is 4x1
tmux -L $SERVER_NAME new-window -t zaapp-nodes:husky -a -n core
tmux -L $SERVER_NAME split-window -t zaapp-nodes:core -v
tmux -L $SERVER_NAME split-window -t zaapp-nodes:core -v
tmux -L $SERVER_NAME split-window -t zaapp-nodes:core -v

# Control window layout is 2x1
tmux -L $SERVER_NAME new-window -t zaapp-nodes:core -a -n control
tmux -L $SERVER_NAME split-window -t zaapp-nodes:control -v

# Behavior window layout is 2x1
tmux -L $SERVER_NAME new-window -t zaapp-nodes:control -a -n behavior
tmux -L $SERVER_NAME split-window -t zaapp-nodes:behavior -v

# Debug window is 3x1.
tmux -L $SERVER_NAME new-window -t zaapp-nodes:behavior -a -n debug
tmux -L $SERVER_NAME split-window -t zaapp-nodes:debug -v
tmux -L $SERVER_NAME split-window -t zaapp-nodes:debug -v

tmux -L $SERVER_NAME select-layout -t zaapp-nodes:husky even-vertical
tmux -L $SERVER_NAME select-layout -t zaapp-nodes:core even-vertical
tmux -L $SERVER_NAME select-layout -t zaapp-nodes:control even-vertical
tmux -L $SERVER_NAME select-layout -t zaapp-nodes:behavior even-vertical
tmux -L $SERVER_NAME select-layout -t zaapp-nodes:debug even-vertical

# Note: we do not source a venv as that is in the bashrc. If we remove it from the bashrc, we should add a source
# stanza to the nodes which require the venv.
PREAMBLE="cd $WORKSPACE && source $EXPECTED_RELATIVE_INSTALL_PATH"

# C-j should be newline. If you don't want a node to start automatically, don't add $RUN
RUN="C-j"

tmux -L $SERVER_NAME send-keys -t zaapp-nodes:husky.0 "$PREAMBLE && ros2 launch husky_base base_launch.py" $RUN
tmux -L $SERVER_NAME send-keys -t zaapp-nodes:husky.1 "$PREAMBLE && ros2 launch husky_control teleop_launch.py" $RUN
tmux -L $SERVER_NAME send-keys -t zaapp-nodes:husky.2 "$PREAMBLE && ros2 launch robot_bringup drivers_hack.launch.py" $RUN
tmux -L $SERVER_NAME send-keys -t zaapp-nodes:husky.3 "$PREAMBLE && ros2 launch robot_bringup localization.launch.py" $RUN

tmux -L $SERVER_NAME send-keys -t zaapp-nodes:core.0 "$PREAMBLE && ros2 run downward_facing_camera weed_detection_node" $RUN
tmux -L $SERVER_NAME send-keys -t zaapp-nodes:core.1 "$PREAMBLE && ros2 run forward_facing_camera weed_map_node" $RUN
tmux -L $SERVER_NAME send-keys -t zaapp-nodes:core.2 "$PREAMBLE && ros2 run manipulator_ros manipulator_node" $RUN
tmux -L $SERVER_NAME send-keys -t zaapp-nodes:core.3 "$PREAMBLE && ros2 run odom2world odom2world" $RUN

tmux -L $SERVER_NAME send-keys -t zaapp-nodes:control.0 "$PREAMBLE && ros2 run behavior trajectory_generator_node" $RUN
tmux -L $SERVER_NAME send-keys -t zaapp-nodes:control.1 "$PREAMBLE && ros2 run behavior controller_node" $RUN

tmux -L $SERVER_NAME send-keys -t zaapp-nodes:behavior.0 "$PREAMBLE && ros2 run visual_servoing visual_servoing_server" $RUN
tmux -L $SERVER_NAME send-keys -t zaapp-nodes:behavior.1 "$PREAMBLE && ros2 run behavior aladeen_node --ros-args -p sentience:=0"

tmux -L $SERVER_NAME send-keys -t zaapp-nodes:debug.0 "$PREAMBLE" $RUN
tmux -L $SERVER_NAME send-keys -t zaapp-nodes:debug.1 "$PREAMBLE" $RUN
tmux -L $SERVER_NAME send-keys -t zaapp-nodes:debug.2 "$PREAMBLE" $RUN

# Attach to session.
tmux -L $SERVER_NAME attach -t zaapp-nodes
