#!/bin/bash

# --- Configuration ---
COMPOSE_FILE="docker-compose.yml"
SERVICE_NAME="sharp-sensor-rig"

# Check if on Linux (most straightforward X11 forwarding)
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "Host OS: Linux. Setting up X11 permissions for Docker..."
    # Ensure DISPLAY is set (e.g., :0 or :1)
    if [ -z "$DISPLAY" ]; then
        echo "Error: DISPLAY environment variable is not set. Please ensure an X server is running and DISPLAY is configured."
        echo "Example: export DISPLAY=:0"
        exit 1
    fi
    # Allow local connections to the X server from Docker containers
    xhost +local:docker > /dev/null 2>&1 || {
        echo "Warning: xhost command failed. X11 forwarding might not work."
        echo "Ensure 'xauth' and 'x11-xserver-utils' are installed (e.g., 'sudo apt-get install xauth x11-xserver-utils')."
    }
elif [[ "$OSTYPE" == "darwin"* ]]; then
    echo "Host OS: macOS. X11 forwarding requires XQuartz."
    echo "Ensure XQuartz is installed and running."
    echo "You might need to enable 'Allow connections from network clients' in XQuartz preferences."
    echo "Consider passing '-e DISPLAY=host.docker.internal:0' in compose environment if issues persist."
elif [[ "$OSTYPE" == "msys"* || "$OSTYPE" == "win32"* ]]; then
    echo "Host OS: Windows. X11 forwarding requires WSL2 and an X server like VcXsrv."
    echo "Ensure VcXsrv is running in 'Disable access control' mode."
    if [ -z "$DISPLAY" ]; then
        echo "Warning: DISPLAY environment variable is not set in WSL2. GUI apps may not work."
        echo "You might need to set 'export DISPLAY=$(awk '/nameserver / {print $2; exit}' /etc/resolv.conf):0.0' in your WSL2 .bashrc."
    fi
else
    echo "Host OS: Unknown. X11 forwarding might not work as expected."
fi

# --- Build the Docker Compose services if necessary ---
echo "Ensuring Docker image is up to date..."
# The 'run' command will use the cached image if available and valid.
# A build is only triggered if the image doesn't exist.
# No explicit 'build' command is needed for this workflow.

# --- Check and Stop/Remove existing container ---
# 2026-10-01: this used to run `docker compose down --remove-orphans`
# unconditionally on every start. That kills a live recording — and with it the
# MCAP footer, metadata.yaml and session_report.yaml — so now it only cleans up
# when nothing is running, or when the operator explicitly asks for it.
RUNNING_IDS="$(docker ps -q --filter 'ancestor=sharp-sensor-rig' 2>/dev/null || true)"
if [ -z "${RUNNING_IDS}" ]; then
    RUNNING_IDS="$(docker compose -f "${COMPOSE_FILE}" ps -q 2>/dev/null || true)"
fi
if [ -n "${RUNNING_IDS}" ]; then
    if [ "${SHARP_FORCE_CLEAN:-0}" = "1" ]; then
        echo "SHARP_FORCE_CLEAN=1: removing the running rig container(s): ${RUNNING_IDS}"
        echo "Any active recording is being destroyed."
        docker compose -f "${COMPOSE_FILE}" down --remove-orphans > /dev/null 2>&1 || true
    else
        echo "Refusing to start: the rig container is already running."
        echo "  container id(s): ${RUNNING_IDS}"
        echo "'docker compose down --remove-orphans' would kill the running rig and any live recording."
        echo "If a session is recording, stop it first (in the rig's 'record' pane: stop), wait for the"
        echo "'session_check:' line, then re-run this script."
        echo "To discard the running rig anyway: SHARP_FORCE_CLEAN=1 $0"
        exit 1
    fi
else
    echo "No rig container running; cleaning up stale resources..."
    docker compose -f "${COMPOSE_FILE}" down --remove-orphans > /dev/null 2>&1 || true
fi
echo "Docker Compose cleanup completed."

# --- Run the Docker Compose service interactively ---
echo "Starting Docker Compose service '${SERVICE_NAME}' in interactive mode..."

docker compose -f "${COMPOSE_FILE}" run --rm "${SERVICE_NAME}" bash

echo "Docker container session ended."
