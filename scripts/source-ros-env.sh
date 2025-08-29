#
# setup_env.sh - portable setup for ROS, Gazebo, and Ardupilot
#

# Detect current shell (bash or zsh)
if [ -n "$ZSH_VERSION" ]; then
    SHELL_TYPE="zsh"
elif [ -n "$BASH_VERSION" ]; then
    SHELL_TYPE="bash"
else
    SHELL_TYPE="sh"
fi

# Resolve script directory (independent of where it’s called from)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]:-${(%):-%N}}" )" >/dev/null 2>&1 && pwd )"

# Project root (assuming this script lives inside your project)
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "[INFO] Using shell: $SHELL_TYPE"
echo "[INFO] Project root: $PROJECT_ROOT"

# --- Source environments ---
if [ -f "/opt/ros/noetic/setup.$SHELL_TYPE" ]; then
    source "/opt/ros/noetic/setup.$SHELL_TYPE"
fi

if [ -f "$PROJECT_ROOT/devel/setup.$SHELL_TYPE" ]; then
    source "$PROJECT_ROOT/devel/setup.$SHELL_TYPE"
fi

if [ -f "/usr/share/gazebo/setup.sh" ]; then
    source "/usr/share/gazebo/setup.sh"
fi

if [ -f "$PROJECT_ROOT/../ardupilot/Tools/completion/completion.$SHELL_TYPE" ]; then
    source "$PROJECT_ROOT/../ardupilot/Tools/completion/completion.$SHELL_TYPE"
fi

# --- Gazebo paths ---
export GAZEBO_MODEL_PATH="$PROJECT_ROOT/src/iq_sim/models:$PROJECT_ROOT/3rdparty/ardupilot_gazebo/models"
export GAZEBO_PLUGIN_PATH="$PROJECT_ROOT/3rdparty/ardupilot_gazebo/build"

echo "[INFO] GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH"
echo "[INFO] GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH"
