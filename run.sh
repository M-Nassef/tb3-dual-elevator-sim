#!/usr/bin/env bash
# Helper for the Jazzy + Gazebo Harmonic Docker environment.
#
#   ./run.sh image                       build the Docker image (once)
#   ./run.sh build                       colcon build the workspace
#   ./run.sh test                        run unit tests (colcon test)
#   ./run.sh sim [launch args]           start Gazebo + robots + elevators + Nav2 + RViz
#   ./run.sh mission --robot tb3_1 --floor 3 --goal room_b
#   ./run.sh missions [launch args]      both robots, e.g. mode:=sequential tb3_1_floor:=2
#   ./run.sh elevator e1 block|clear|open|close|emergency|reset
#   ./run.sh shell                       interactive shell (inside the running sim if any)
#   ./run.sh stop                        stop the simulation container
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE=elevator-sim:jazzy
NAME=elevator_sim
SETUP='source /opt/ros/jazzy/setup.bash && { [ -f /ws/install/setup.bash ] && source /ws/install/setup.bash || true; }'

docker_run() {   # docker_run <interactive:-it|-i> <command...>
  local tty=$1; shift
  xhost +SI:localuser:"$(id -un)" >/dev/null 2>&1 || true
  mkdir -p "$WS/.home" "$WS/metrics"
  docker run --rm $tty --name "${RUN_NAME:-${NAME}_$$}" \
    --network host --ipc host \
    --user "$(id -u):$(id -g)" -e HOME=/ws/.home \
    -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device /dev/dri --group-add video --group-add "$(getent group render | cut -d: -f3 || echo video)" \
    -e ROS_DOMAIN_ID=42 -e ELEVATOR_METRICS_DIR=/ws/metrics \
    -v "$WS":/ws -w /ws \
    "$IMAGE" bash -c "$SETUP && $*"
}

in_sim() {       # run inside the running sim container if present, else a fresh one
  if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
    docker exec -it -u "$(id -u):$(id -g)" -e HOME=/ws/.home -e ELEVATOR_METRICS_DIR=/ws/metrics "$NAME" \
      bash -c "$SETUP && $*"
  else
    docker_run -it "$*"
  fi
}

cmd=${1:-help}; shift || true
case "$cmd" in
  image)    docker build -t "$IMAGE" "$WS/docker" ;;
  build)    docker_run -i "colcon build --symlink-install --event-handlers console_direct-" ;;
  test)     docker_run -i "colcon test --packages-select elevator_sim --event-handlers console_direct- && colcon test-result --verbose" ;;
  sim)      RUN_NAME=$NAME docker_run -it "ros2 launch elevator_sim sim.launch.py $*" ;;
  mission)  in_sim "ros2 run elevator_sim mission $*" ;;
  missions) in_sim "ros2 launch elevator_sim missions.launch.py $*" ;;
  elevator) in_sim "ros2 run elevator_sim elevator_cmd $*" ;;
  shell)    in_sim "bash" ;;
  stop)     docker stop "$NAME" ;;
  *)        sed -n '2,13p' "$0" ;;
esac
