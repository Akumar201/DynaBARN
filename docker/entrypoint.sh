#!/usr/bin/env bash
# Source ROS, install the mounted dynabarn-core in editable mode (once per
# container), and build the ROS packages in /ws if there are any.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if ! python -c "import dynabarn_core" 2>/dev/null; then
    pip install --no-deps --quiet -e /work 2>&1 | grep -v "^$" || true
fi

if [ -d /work/ros2 ] && [ -n "$(ls -A /work/ros2 2>/dev/null)" ] && [ "${DYNABARN_SKIP_BUILD:-0}" != "1" ]; then
    (cd /ws && python -m colcon build --base-paths /work/ros2 --symlink-install \
        --event-handlers console_cohesion- summary- >/ws/colcon.log 2>&1) \
        || { echo "colcon build failed, see /ws/colcon.log"; tail -n 30 /ws/colcon.log; }
    [ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
fi

exec "$@"
