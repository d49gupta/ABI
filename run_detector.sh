#!/bin/bash
cd "$(dirname "$0")" || exit 1

PIPELINE_PID=""

build_and_run() {
    local mode="$1"
    if [ ! -d "build" ]; then
        echo "Creating build directory..."
        mkdir build
    fi
    cd build || exit 1

    if [ "$mode" == "three-point" ]; then
        echo "Rebuilding with THREE POINT enabled..."
        cmake -DCMAKE_CXX_FLAGS="-DTHREE_POINT" ..
    else
        echo "Rebuilding with FOUR POINT enabled..."
        cmake -DCMAKE_CXX_FLAGS="" ..
    fi
    make -j"$(nproc)"

    if [ ! -f "tag_detector" ]; then
        echo "Build failed. Check errors above."
        return 1
    fi

    echo "Starting camera stream..."
    # We use -o - to send video to stdout, which | sends to your app's stdin
    rpicam-vid -t 0 --inline --framerate 20 --width 640 --height 480 --codec yuv420 -o - 2> camera_log.txt | ./tag_detector
}

start_pipeline() {
    local mode="${1:-four-point}"
    if [ -n "$PIPELINE_PID" ] && kill -0 "$PIPELINE_PID" 2>/dev/null; then
        echo "Pipeline already running (pid $PIPELINE_PID) -- ignoring START"
        return
    fi
    build_and_run "$mode" &
    PIPELINE_PID=$!
    echo "Pipeline started (pid $PIPELINE_PID, mode=$mode)"
}

stop_pipeline() {
    if [ -n "$PIPELINE_PID" ] && kill -0 "$PIPELINE_PID" 2>/dev/null; then
        echo "!!! STOP command received via MQTT !!!"
        # Kill the build_and_run subshell's children (rpicam-vid + tag_detector)
        pkill -P "$PIPELINE_PID" 2>/dev/null
        wait "$PIPELINE_PID" 2>/dev/null
    fi
    PIPELINE_PID=""
}

cleanup() {
    stop_pipeline
    exit 0
}
trap cleanup SIGINT SIGTERM

# Preserve the old manual-launch workflow: `./run_detector.sh [three-point]`
# still starts scanning immediately. The difference now is the script keeps
# running afterward, listening for pi/start and pi/stop over MQTT, instead
# of exiting once the pipeline stops. That also matters for the systemd
# service: STOP only kills the inner pipeline, never this listener process
# itself, so `Restart=on-failure` never sees a STOP as a crash -- it only
# fires if this script's own process dies unexpectedly.
start_pipeline "$1"

echo "Listening for pi/start and pi/stop ..."
mosquitto_sub -v -h localhost -t "pi/start" -t "pi/stop" | while read -r topic payload; do
    case "$topic" in
        pi/start)
            start_pipeline "$payload"
            ;;
        pi/stop)
            if [ "$payload" == "STOP" ]; then
                stop_pipeline
            fi
            ;;
    esac
done
