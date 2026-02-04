#!/usr/bin/env bash
set -e

# 1️⃣ Source ROS distro
echo "sourcing /opt/ros/install/setup.bash"
source /opt/ros/install/setup.bash
echo "sourcing /opt/ros/humble/setup.bash"
source /opt/ros/humble/setup.bash
echo "ROS_DISTRO $ROS_DISTRO"
echo "ROS_ROOT   $ROS_ROOT"

# 2️⃣ Source workspace overlay if it exists
if [ -f "$ROS_WS/install/setup.bash" ]; then
    echo "sourcing $ROS_WS/install/setup.bash"
    source "$ROS_WS/install/setup.bash"
fi

# 3️⃣ Build workspace if package not found
cd "$ROS_WS"
if ! ros2 pkg prefix yolo_ros >/dev/null 2>&1; then
    echo "Building workspace..."
    colcon build --cmake-args -DCMAKE_EXE_LINKER_FLAGS="-lcurl"
    source "$ROS_WS/install/setup.bash"
fi

# 4️⃣ Execute the command passed to container
exec "$@"
