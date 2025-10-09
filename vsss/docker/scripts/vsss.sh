#!/bin/bash

# Check if NVIDIA GPUs are available
ENV_TYPE="cpu"
if command -v nvidia-smi > /dev/null 2>&1; then
    if nvidia-smi > /dev/null 2>&1; then
        ENV_TYPE="cuda"
    fi
fi
echo "Detected environment: $ENV_TYPE"

function run_container() {
    echo "Running vsss container"
    mkdir ./compose/log ./compose/install ./compose/build
    docker compose -f ./compose/docker-compose-$ENV_TYPE-base.yml build
    docker compose -f ./compose/docker-compose-$ENV_TYPE.yml up -d
}

function dev_mode() {
    run_container "$@"
    docker exec -it vsss_$ENV_TYPE bash
}

function stop_container() {
    docker compose -f ./compose/docker-compose-$ENV_TYPE.yml stop
}

function remove_container() {
    docker compose -f ./compose/docker-compose-$ENV_TYPE.yml down
}

function help_message() {
    echo "Usage: ./vsss.sh [COMMAND]"
    echo
    echo "Commands:"
    echo "  --run                     Start container"
    echo "  --dev-mode                Run + attach to shell"
    echo "  --stop                    Stop container"
    echo "  --remove                  Remove container"
    echo "  --help                    Show this help message"
}

# Main dispatcher
case "$1" in
    --run)
        run_container "$@"
        ;;
    --dev-mode)
        dev_mode "$@"
        ;;
    --stop)
        stop_container "$@"
        ;;
    --remove)
        remove_container "$@"
        ;;
    --help)
        help_message
        ;;
    *)
        echo "Unknown command: $1"
        help_message
        exit 1
        ;;
esac
