#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0
#
# Manages the UFACTORY firmware simulator Docker container for integration
# testing. The simulator runs the real controller firmware, so it speaks the
# same command (502) and report (30001/30002/30003) protocol as a physical
# arm — plus the UFACTORY Studio web UI on http://127.0.0.1:18333 for visual
# verification.
#
# Usage:
#   scripts/sim.sh start [axes]   # start container + firmware (default: 6)
#   scripts/sim.sh stop           # stop and remove the container
#   scripts/sim.sh status         # is it running / is port 502 answering?
#   scripts/sim.sh logs           # tail firmware logs
#
# Then run the simulator test suite:
#   mix test --include simulator

set -euo pipefail

IMAGE="danielwang123321/uf-ubuntu-docker"
NAME="bb_ufactory_sim"
AXES="${2:-6}"

wait_for_port() {
  local port="$1" tries="${2:-30}"
  for _ in $(seq 1 "$tries"); do
    if nc -z 127.0.0.1 "$port" 2>/dev/null; then
      return 0
    fi
    sleep 1
  done
  return 1
}

case "${1:-}" in
  start)
    if [ "$(docker ps -q -f name="^${NAME}$")" ]; then
      echo "Container ${NAME} already running."
    else
      docker rm -f "$NAME" >/dev/null 2>&1 || true
      # -it: the image's entrypoint is a bare bash shell, which exits
      # immediately without a TTY. --platform: the image is amd64-only;
      # Apple Silicon hosts run it under emulation.
      docker run -dit --platform linux/amd64 --name "$NAME" \
        -p 18333:18333 \
        -p 502:502 \
        -p 503:503 \
        -p 504:504 \
        -p 30000:30000 \
        -p 30001:30001 \
        -p 30002:30002 \
        -p 30003:30003 \
        "$IMAGE" >/dev/null
      echo "Container ${NAME} started."
    fi

    echo "Starting xArm firmware simulation (${AXES} axes)..."
    docker exec -d "$NAME" /xarm_scripts/xarm_start.sh "$AXES" "$AXES"

    echo -n "Waiting for command port 502..."
    if wait_for_port 502 60; then
      echo " up."
    else
      echo " TIMED OUT. Check: scripts/sim.sh logs"
      exit 1
    fi

    echo -n "Waiting for report port 30003..."
    if wait_for_port 30003 30; then
      echo " up."
    else
      echo " TIMED OUT. Check: scripts/sim.sh logs"
      exit 1
    fi

    echo "Simulator ready. Studio UI: http://127.0.0.1:18333"
    echo "Run tests with: mix test --include simulator"
    ;;

  stop)
    docker rm -f "$NAME" >/dev/null 2>&1 && echo "Container ${NAME} removed." ||
      echo "Container ${NAME} was not running."
    ;;

  status)
    if [ "$(docker ps -q -f name="^${NAME}$")" ]; then
      echo "Container: running"
      if nc -z 127.0.0.1 502 2>/dev/null; then
        echo "Command port 502: answering"
      else
        echo "Command port 502: NOT answering (firmware not started?)"
      fi
    else
      echo "Container: not running"
    fi
    ;;

  logs)
    docker logs --tail 50 -f "$NAME"
    ;;

  *)
    echo "Usage: $0 {start [axes]|stop|status|logs}"
    exit 1
    ;;
esac
