#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./start.sh            # build (if needed) and start container
#   ./start.sh --rebuild  # rebuild image from scratch (no cache) and start
#   ./start.sh --shell    # open an interactive shell in the running container

SERVICE=ros2
COMPOSE_FILE="docker-compose.yml"

# Allow local X11 connections
# (You only need to do this once per host session, but it's safe to run every time)
xhost +local:root
xhost +local:docker

# ensure_ws() {
#   mkdir -p ws/src
# }

rebuild() {
  echo ">>> Rebuilding image from scratch..."
  docker compose -f "${COMPOSE_FILE}" build --no-cache --pull
}

up() {
  echo ">>> Starting container..."
  docker compose -f "${COMPOSE_FILE}" up -d
}

shell() {
  echo ">>> Opening shell in running container (with entrypoint)..."
  docker compose -f "${COMPOSE_FILE}" exec ${SERVICE} /entrypoint.sh /bin/bash || {
    echo "Container not running; starting it now..."
    up
    docker compose -f "${COMPOSE_FILE}" exec ${SERVICE} /entrypoint.sh /bin/bash
  }
}

getin() {
  echo ">>> Starting container (if needed) and getting inside (with entrypoint)..."
  up
  docker compose -f "${COMPOSE_FILE}" exec ${SERVICE} /entrypoint.sh /bin/bash
}


restart() {
  echo ">>> Restarting container..."
  docker compose -f "${COMPOSE_FILE}" restart ${SERVICE}
}


case "${1:-}" in
 --restart)
    restart
    ;;
  --getin)
    getin
    ;;
  --rebuild)
    rebuild
    up
    ;;
  --shell)
    shell
    ;;
  "" )
    # ensure_ws
    up
    ;;
  *)
    echo "Unknown option: $1"
    echo "Usage: $0 [--rebuild|--shell]"
    exit 1
    ;;
esac

echo "Done."
