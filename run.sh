#!/bin/bash

command -v tmux >/dev/null 2>&1 || { echo -e "\033[0;31m Error: tmux is not installed, please install tmux first (e.g., sudo apt install tmux)\033[0m"; exit 1; }

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ENV_FILE="${SCRIPT_DIR}/.env"

if [ -f "$ENV_FILE" ]; then
    echo -e "${BLUE}Load environment configuration from: $ENV_FILE${NC}"
    set -a; source "$ENV_FILE"; set +a
else
    echo -e "${YELLOW}No .env file found, using default values ​​from script.${NC}"
fi

MODEL_NAME="${MODEL_NAME:-drone_sac_model.pt}"
BUFFER_NAME="${BUFFER_NAME:-experience_buffer.pkl}"

TCP_TIMEOUT="${TCP_TIMEOUT:-5}"
PORT_RETRY_COUNT="${PORT_RETRY_COUNT:-3}"

PX4_INIT_LOG="${PX4_INIT_LOG:-"Ready to fly|Simulation started|Startup script returned successfully"}"
XRCE_OK_LOG="${XRCE_OK_LOG:-"Session established|Creation of session|uxrce_dds_client"}"
SESSION="${SESSION_NAME:-drone_sim_live}"
SERVICE_TIMEOUT="${SERVICE_TIMEOUT:-90}"
MAVROS_WAIT_TIME="${MAVROS_WAIT_TIME:-15}"
SIMULATOR_STABILITY_TIME="${SIMULATOR_STABILITY_TIME:-10}"

PX4_HOME="${PX4_HOME:-~/PX4-Autopilot}"
LOG_DIR="${LOG_DIR:-${SCRIPT_DIR}/logs}"
MODEL_DIR="${MODEL_DIR:-${SCRIPT_DIR}/models}"
ROS_LOG_DIR="${ROS_LOG_DIR:-${SCRIPT_DIR}/logs/ros}"
FLIGHT_LOG_DIR="${FLIGHT_LOG_DIR:-${SCRIPT_DIR}/logs/flight}"

mkdir -p "$LOG_DIR"
MODEL_DIR=$(realpath "${MODEL_DIR}")
ROS_LOG_DIR=$(realpath "${ROS_LOG_DIR}")
FLIGHT_LOG_DIR=$(realpath "${FLIGHT_LOG_DIR}")
mkdir -p "$MODEL_DIR" "$ROS_LOG_DIR" "$FLIGHT_LOG_DIR" || { echo -e "${RED} Unable to create log/model directory ！${NC}"; exit 1; }
MODEL_PATH="${MODEL_DIR}/${MODEL_NAME}"
BUFFER_PATH="${MODEL_DIR}/${BUFFER_NAME}"

PIDS=()
ALL_PIDS=()

find_available_port() { local base_port=$1; local max_tries=${2:-5}; for (( i=0; i<max_tries; i++ )); do local port=$((base_port + i)); if ! nc -z -w 2 localhost "$port" 2>/dev/null; then echo "$port"; return 0; fi; done; echo "$base_port"; return 1; }
check_network() { local url_var_name="$1"; local url="${!url_var_name}"; local proto=$(echo "$url" | cut -d: -f1); local port=$(echo "$url" | cut -d: -f3); local check_host="localhost"; if [[ "$proto" = "udp" || "$proto" = "tcp" ]]; then if ! nc -z -w 2 "$check_host" "$port" 2>/dev/null; then echo -e "${GREEN}Port $port ($url_var_name) OK${NC}"; return 0; else echo -e "${YELLOW}Port $port ($url_var_name) used, trying alt...${NC}"; local new_port=$(find_available_port "$port" 5); if [ "$new_port" != "$port" ]; then echo -e "${GREEN}Using alt port: $new_port${NC}"; local new_url="${proto}://:${new_port}"; export "$url_var_name"="$new_url"; echo -e "${GREEN}Updated $url_var_name to ${!url_var_name}${NC}"; return 0; else echo -e "${YELLOW}No alt port found, using $port${NC}"; return 1; fi; fi; fi; return 0; }
check_tcp_backup() { local tcp_port=5760; local retry=0; while [ $retry -lt "$PORT_RETRY_COUNT" ]; do if ! nc -z -w "$TCP_TIMEOUT" localhost $tcp_port 2>/dev/null; then echo -e "${GREEN}TCP backup $tcp_port OK${NC}"; TCP_BACKUP_URL="tcp://127.0.0.1:$tcp_port"; export TCP_BACKUP_URL; return 0; else echo -n -e "${YELLOW}⏳ TCP backup $tcp_port unavailable...${NC}"; tcp_port=$((tcp_port + 1)); retry=$((retry + 1)); sleep 1; fi; done; echo ""; echo -e "${RED}❌ No TCP backup port found${NC}"; return 1; }
get_all_process_pids() { local pids; if tmux has-session -t "$SESSION" 2>/dev/null; then pids=$(tmux list-windows -t "$SESSION" -F '#{window_pid}' 2>/dev/null); pids+=" $(tmux list-panes -s -t "$SESSION" -F '#{pane_pid}' 2>/dev/null)"; fi; pids+=" $(pgrep -f "^${PX4_HOME}/build/px4_sitl.*/bin/px4" || true)"; pids+=" $(pgrep -f "gz sim" || true)"; pids+=" $(pgrep -f "MicroXRCEAgent udp4 -p ${XRCE_PORT}" || true)"; pids+=" $(pgrep -f "ros2 launch mavros px4.launch.py" || true)"; pids+=" $(pgrep -f "ros2 run ros_gz_bridge parameter_bridge" || true)"; pids+=" $(pgrep -f "ros2 launch rosbridge_server rosbridge_websocket_launch.xml" || true)"; pids+=" $(pgrep -f "python3.*src/main.py" || true)"; pids+=" $(pgrep -f "image_tools showimage" || true)"; echo "$pids" | tr ' ' '\n' | grep -v '^$' | sort -nu; }

cleanup() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"
    if [ -n "$CLEANING_UP" ]; then return; fi
    CLEANING_UP=true

    local session_exists=false
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        session_exists=true
    fi

    echo -e "${YELLOW}Terminating processes...${NC}"
    ALL_PIDS_CLEANUP=$(get_all_process_pids)

    local pids_for_commands=$(echo "$ALL_PIDS_CLEANUP" | tr '\n' ' ' | sed 's/ *$//')

    if [ -n "$pids_for_commands" ]; then
        echo -e "${BLUE}Attempting to terminate PIDs: $pids_for_commands${NC}"

        kill -TERM $pids_for_commands 2>/dev/null
        sleep 2

        local remaining_pids_check=""
        for pid in $pids_for_commands; do
            if ps -p "$pid" > /dev/null 2>&1; then
                 remaining_pids_check+="$pid "
            fi
        done
        remaining_pids_check=$(echo "$remaining_pids_check" | sed 's/ *$//')

        if [ -n "$remaining_pids_check" ]; then
            echo -e "${YELLOW}Force killing remaining PIDs: $remaining_pids_check${NC}"
            kill -KILL $remaining_pids_check 2>/dev/null || true
        else
            echo -e "${GREEN}Processes terminated gracefully or were already gone.${NC}"
        fi
    else
        echo -e "${BLUE}No relevant processes found to terminate.${NC}"
    fi

    if $session_exists; then
        echo -e "${YELLOW}Killing tmux session [$SESSION]...${NC}"
        tmux kill-session -t "$SESSION" &> /dev/null
        sleep 1
        echo -e "${GREEN}Tmux session killed${NC}"
    fi

    echo -e "${YELLOW}Cleaning temp files...${NC}"
    find /tmp -maxdepth 1 -name 'fastrtps_*' -exec rm -rf {} + 2>/dev/null
    find /dev/shm -maxdepth 1 -name 'fastdds_*' -exec rm -rf {} + 2>/dev/null
    find /tmp -maxdepth 1 -name 'fastdds*' -exec rm -rf {} + 2>/dev/null
    find /tmp -maxdepth 1 -name 'ros_gz_bridge_*' -exec rm -rf {} + 2>/dev/null
    echo -e "${GREEN}Cleanup finished${NC}"

    unset CLEANING_UP
    exit 0
}

trap cleanup SIGINT SIGTERM SIGTSTP

show_help() {
echo -e "${CYAN} usage:${NC} $0 [option]"
echo -e "\n${CYAN} option:${NC}"
echo -e " ${GREEN}--train-live${NC} ${BOLD}Learn while flying${NC}"
echo -e " ${GREEN}--collect${NC} Collect data only"
echo -e " ${GREEN}--train${NC} Training only (buffer required)"
echo -e " ${GREEN}--test${NC} Test only (model required)"
echo -e " ${GREEN}--monitor${NC} Startup environment only (multi-window view)"
echo -e " ${GREEN}--help${NC} Help"
echo -e "\n${CYAN} Configuration: ${NC} is mainly through ${YELLOW}.env${NC} file."
echo -e "\n${CYAN} Example:${NC} ./run.sh --train-live"
echo -e "\n${CYAN} NOTE:${NC} Make sure to install tmux, htop, nc, image_tools. Press Ctrl+C to stop."
}


setup_environment() {
    echo -e "${BLUE}=== Initialize the simulation environment ===${NC}"

    if tmux has-session -t "$SESSION" 2>/dev/null; then
        echo -e "${YELLOW}An existing tmux session [$SESSION] will be closed...${NC}"
        tmux kill-session -t "$SESSION"
        echo -e "${GREEN}The old session has been closed${NC}"
    fi

    echo -e "${BLUE}Clean up residual programs and cache...${NC}"
    ALL_PIDS=($(get_all_process_pids))
    for pid in "${ALL_PIDS[@]}"; do
        kill -9 "$pid" 2>/dev/null || true
    done

    pkill -f "px4|gz|MicroXRCEAgent|ros2|Fast" || true
    rm -rf /tmp/fastrtps_* /dev/shm/fastdds_* /tmp/fastdds*



    tmux new-session -d -s "$SESSION"
    echo -e "${BLUE}Create a tmux session [$SESSION]${NC}"

    tmux new-window -t "$SESSION" -n "XRCE" "bash -c 'source /opt/ros/humble/setup.bash; MicroXRCEAgent udp4 -p ${XRCE_PORT:-8888}; exec bash'" &
    PIDS+=($!)
    sleep 1
    XRCE_PID=$(pgrep -f "MicroXRCEAgent.*${XRCE_PORT:-8888}" || echo "")
    [ -n "$XRCE_PID" ] && ALL_PIDS+=($XRCE_PID)

    tmux new-window -t "$SESSION" -n "QGC" "bash -c '~/QGroundControl.AppImage; exec bash'" &
    PIDS+=($!)
    sleep 2
    QGC_PID=$(pgrep -f "QGC" || echo "")
     [ -n "$QGC_PID" ] && ALL_PIDS+=($QGC_PID)

    tmux new-window -t "$SESSION" -n "PX4" "bash -c '\

    source /opt/ros/humble/setup.bash \

    export PX4_GZ_HEADLESS="${PX4_GZ_HEADLESS:-1}" \
    export GAZEBO_GPU_RENDERING="${GAZEBO_GPU_RENDERING:-true}" \
    export MAVROS_URL="${MAVROS_URL:-udp://@14550}" \
    export MAVSDK_URL="${MAVSDK_URL:-udp://@14540}" \
    export XRCE_PORT="${XRCE_PORT:-8888}" \
    export ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}" \
    export CAMERA_LINK="${CAMERA_LINK:-camera_link}" \
    export CAMERA_TOPIC="${CAMERA_TOPIC:-camera}" \
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
    export RMW_FASTRTPS_USE_SHM="${RMW_FASTRTPS_USE_SHM:-1}" \

    cd ${PX4_HOME:-~/PX4-Autopilot}; \
    PX4_GZ_WORLD=${PX4_GZ_WORLD:-baylands} \
    PX4_SIM_SPEED_FACTOR=${PX4_SIM_SPEED_FACTOR:-1} \
    GAZEBO_GPU_RENDERING=${GAZEBO_GPU_RENDERING:-true} \
    IGN_TRANSPORT_THREAD_POOL_SIZE=${IGN_TRANSPORT_THREAD_POOL_SIZE:-16} \
    make px4_sitl gz_${PX4_GZ_MODEL:-x500_gimbal}; exec bash'" &
    PIDS+=($!)
    sleep 2
    PX4_PIDS=$(pgrep -f "px4|gazebo" || echo "")
    for pid in $PX4_PIDS; do
        ALL_PIDS+=($pid)
    done

    # echo -e "${BLUE}Send mavlink start command...${NC}"
    # tmux send-keys -t "$SESSION:PX4" "mavlink stop-all" C-m
    # sleep 1
    # tmux send-keys -t "$SESSION:PX4" "mavlink start -p -o 14550" C-m

    echo -e "${BLUE}Waiting for PX4 Gazebo to start...${NC}"
    TIMEOUT=${SERVICE_TIMEOUT:-60}
    START_TIME=$(date +%s)
    until tmux capture-pane -pt "$SESSION:PX4" | grep -Eq "${PX4_INIT_LOG:-ready to fly}|${XRCE_OK_LOG:-started}"; do
        if [ $(( $(date +%s) - START_TIME )) -gt $TIMEOUT ]; then
            echo -e "${YELLOW}PX4 initialization timed out, continue? (y/n)${NC}"
            read -r continue_without_px4
            if [[ "$continue_without_px4" =~ ^[Yy]$ ]]; then
                echo -e "${YELLOW}Execution continues, but PX4 may not be initialized correctly${NC}"
                break
            else
                echo -e "${RED}User Opt-Out${NC}"
                cleanup
                exit 1
            fi
        fi
        sleep 1
    done

    echo -e "${GREEN}PX4 Gazebo startup completed${NC}"
    sleep 5

    MAVROS_CONFIG_YAML=$(ros2 pkg prefix mavros 2>/dev/null || echo "/opt/ros/humble")/share/mavros/config/px4_config.yaml
    tmux new-window -t "$SESSION" -n "MAVROS" "bash -c 'source /opt/ros/humble/setup.bash; ros2 launch mavros px4.launch fcu_url:=\"$MAVROS_URL\" config_yaml:=\"$MAVROS_CONFIG_YAML\"; exec bash'" &
    PIDS+=($!)
    sleep 2
    MAVROS_PID=$(pgrep -f "mavros.*$MAVROS_URL" || echo "")
    [ -n "$MAVROS_PID" ] && ALL_PIDS+=($MAVROS_PID)

    echo -e "${BLUE}Waiting for MAVROS to connect...${NC}"
    sleep ${MAVROS_WAIT_TIME:-15}

    tmux new-window -t "$SESSION" -n "ROSBRIDGE" "bash -c 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=${ROSBRIDGE_PORT:-9090}; exec bash'" &
    PIDS+=($!)
    sleep 2
    ROSBRIDGE_PID=$(pgrep -f "rosbridge.*${ROSBRIDGE_PORT:-9090}" || echo "")
    [ -n "$ROSBRIDGE_PID" ] && ALL_PIDS+=($ROSBRIDGE_PID)

    BRIDGE_TOPIC="/world/${PX4_GZ_WORLD:-baylands}/model/${PX4_GZ_MODEL:-x500_gimbal}_${PX4_MODEL_INSTANCE:0}/link/${CAMERA_LINK:-camera_link}/sensor/${CAMERA_TOPIC:-camera}/image"
    tmux new-window -t "$SESSION" -n "Bridge" "bash -c 'source /opt/ros/humble/setup.bash; ros2 run ros_gz_bridge parameter_bridge $BRIDGE_TOPIC@sensor_msgs/msg/Image@gz.msgs.Image; exec bash'" &
    PIDS+=($!)
    sleep 2
    BRIDGE_PID=$(pgrep -f "ros_gz_bridge.*parameter_bridge" || echo "")
    [ -n "$BRIDGE_PID" ] && ALL_PIDS+=($BRIDGE_PID)

    tmux new-window -t "$SESSION" -n "Image" "bash -c 'source /opt/ros/humble/setup.bash; ros2 run image_tools showimage --ros-args -r image:=$BRIDGE_TOPIC; exec bash'" &
    PIDS+=($!)
    sleep 2
    IMAGE_PID=$(pgrep -f "image_tools.*showimage" || echo "")
    [ -n "$IMAGE_PID" ] && ALL_PIDS+=($IMAGE_PID)

    echo -e "${GREEN}Environment setup is complete and the simulator is ready${NC}"
}

find_latest_model() {
    local model_path_var="$1"; local current_model_path="${!model_path_var}"
    if [[ -z "$current_model_path" || ! -f "$current_model_path" ]] && [ -d "$MODEL_DIR" ]; then
        LATEST_MODEL=$(find "$MODEL_DIR" -maxdepth 1 -name "*.pt" -type f -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-)
        if [ -n "$LATEST_MODEL" ]; then echo -e "${YELLOW}Using latest model: $LATEST_MODEL${NC}"; eval "$model_path_var=\"$LATEST_MODEL\"";
        else echo -e "${YELLOW}No model (.pt) found in $MODEL_DIR${NC}"; fi
    elif [ -f "$current_model_path" ]; then echo -e "${GREEN}Using specified model: $current_model_path${NC}"; fi
}


collect_data() {
    echo -e "${BLUE}=== Collect data (output in current terminal) ===${NC}"; echo -e "${BLUE}MAVSDK: $MAVSDK_URL ${NC}"
    python3 "${SCRIPT_DIR}/src/main.py" \
        --collect-only \
        --sim-address "$MAVSDK_URL" \
        --model-dir "$MODEL_DIR" \
        --model-path "$MODEL_PATH" \
        --buffer-path "$BUFFER_PATH" \
        --flight-log-dir "$FLIGHT_LOG_DIR" \
        --target-altitude 2.0 \
        --min-target-distance 10.0 \
        --max-target-distance 60.0 \
        --random-action-prob 0.5 \
        --collect-episodes 20


    local result=$?
    if [ $result -ne 0 ]; then echo -e "${RED}Data collection script execution failed (exit code: $result)${NC}"; return 1; fi
    echo -e "${GREEN}Data collection completed${NC}"
}

train_model() {
    echo -e "${BLUE}=== Train model ===${NC}"; find_latest_model MODEL_PATH
    if [ ! -f "$BUFFER_PATH" ]; then echo -e "${RED}Buffer not found: $BUFFER_PATH${NC}"; return 1; fi
    echo -e "${BLUE}Buffer: $BUFFER_PATH${NC}"; echo -e "${BLUE}Model: $MODEL_PATH${NC}"
    DEVICE=$(command -v nvidia-smi >/dev/null 2>&1 && echo "cuda" || echo "cpu"); echo -e "${BLUE}Device: $DEVICE${NC}"

    python3 "${SCRIPT_DIR}/src/main.py" \
        --train-only \
        --device "$DEVICE" \
        --model-dir "$MODEL_DIR" \
        --model-path "$MODEL_PATH" \
        --buffer-path "$BUFFER_PATH" \
        --epochs 1000 \
        --batch-size 256 \


    local result=$?
    if [ $result -ne 0 ]; then echo -e "${RED}Training script execution failed (exit code: $result)${NC}"; return 1; fi
    echo -e "${GREEN}Training completed${NC}"
}

test_model() {
    echo -e "${BLUE}=== Test model ===${NC}"; find_latest_model MODEL_PATH
    if [[ -z "$MODEL_PATH" || ! -f "$MODEL_PATH" ]]; then echo -e "${RED}Model not found ($MODEL_PATH)${NC}"; return 1; fi
    echo -e "${BLUE}Model: $MODEL_PATH${NC}"; echo -e "${BLUE}MAVSDK: $MAVSDK_URL ${NC}"

    echo -e "${BLUE}Waiting for stability (${SIMULATOR_STABILITY_TIME}s)...${NC}"; sleep ${SIMULATOR_STABILITY_TIME}

    python3 "${SCRIPT_DIR}/src/main.py" \
        --test-flight \
        --sim-address "$MAVSDK_URL" \
        --model-path "$MODEL_PATH" \
        --flight-log-dir "$FLIGHT_LOG_DIR" \
        --target-altitude 2.0 \
        --min-target-distance 15.0 \
        --max-target-distance 70.0

    local result=$?
    if [ $result -ne 0 ]; then echo -e "${RED}Test flight script execution failed (exit code: $result)${NC}"; return 1; fi
    echo -e "${GREEN}Test flight completed${NC}"
}

train_live() {
    echo -e "${BLUE}=== Learn on the fly ===${NC}"; find_latest_model MODEL_PATH
    DEVICE=$(command -v nvidia-smi >/dev/null 2>&1 && echo "cuda" || echo "cpu"); echo -e "${BLUE}Device: $DEVICE${NC}"
    echo -e "${BLUE}MAVSDK: $MAVSDK_URL ${NC}"
    if [ -f "$BUFFER_PATH" ]; then echo -e "${BLUE}Loading buffer: $BUFFER_PATH${NC}"; else echo -e "${YELLOW}Creating buffer: $BUFFER_PATH${NC}"; fi
    if [ -f "$MODEL_PATH" ]; then echo -e "${BLUE}Loading model: $MODEL_PATH${NC}"; else echo -e "${YELLOW}Creating model: $MODEL_PATH${NC}"; fi

    python3 "${SCRIPT_DIR}/src/main.py" \
        --concurrent \
        --sim-address "$MAVSDK_URL" \
        --model-dir "$MODEL_DIR" \
        --model-path "$MODEL_PATH" \
        --buffer-path "$BUFFER_PATH" \
        --flight-log-dir "$FLIGHT_LOG_DIR" \
        --nav-mode "$NAV_MODE" \
        --device "$DEVICE" \
        --line-navigation \
        --nav-mode="$CAMERA_TOPIC" \
        --no-gps


    local result=$?
    if [ $result -ne 0 ]; then echo -e "${RED} 'Learn on the fly' script failed to execute (exit code: $result)${NC}"; return 1; fi
    echo -e "${GREEN}'Learn on the fly' completed ${NC}"
}

monitor_only() {
    echo -e "${BLUE}=== Monitor only ===${NC}"; echo -e "${BLUE}Background environment has been started (multiple windows).${NC}"
    echo -e "${BLUE}Use 'tmux attach -t $SESSION' to view the background window. ${NC}"
    echo -e "${YELLOW}Press Ctrl+C to stop monitoring and clean up the environment.${NC}"; while true; do sleep 300; done
}


main() {
    if [[ "$1" == "--help" ]]; then show_help; exit 0; fi
    SHOULD_SETUP_ENV=true
    if [[ "$1" == "--train" ]]; then SHOULD_SETUP_ENV=false; fi

    if $SHOULD_SETUP_ENV; then
        setup_environment || { echo -e "${RED}Environment setup failed, exiting.${NC}"; exit 1; }
    fi

    case "$1" in
        --train-live) train_live ;;
        --collect) collect_data ;;
        --train) train_model ;;
        --test) test_model ;;
        --monitor) monitor_only ;;
        *) show_help
           if $SHOULD_SETUP_ENV; then echo -e "${YELLOW}Unknown option '$1'. Entering monitoring mode."; monitor_only;
           else echo -e "${RED}Unknown option '$1'.${NC}"; exit 1; fi ;;
    esac

    local exit_code=$?
    if [ $exit_code -ne 0 ] && [[ "$1" != "--monitor" ]]; then
        echo -e "${RED}The task execution failed (exit code: $exit_code). Cleaning up...${NC}"
    fi
    cleanup
    exit $exit_code
}

if [ -n "$PX4_SIM_SPEED_FACTOR" ] && [ "$(echo "$PX4_SIM_SPEED_FACTOR < 1.0" | bc -l)" -eq 1 ]; then echo -e "${YELLOW}Sim speed $PX4_SIM_SPEED_FACTOR < 1.0 may affect stability.${NC}"; fi
main "$@"
