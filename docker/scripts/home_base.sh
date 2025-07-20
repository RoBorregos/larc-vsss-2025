#!/bin/bash

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE_PATH="$PROJECT_ROOT/compose/docker-compose.yml"


DEFAULT_BASE_TAG="cpu_base"
CUDA_BASE_TAG="cuda_base"
GPU_TAG="nav2_gpu"
CUDA_TAG="nav2_cuda"
BASE_REPO="roborregos/vsss_sim"
NAV2_REPO="roborregos/nav2"

# Util
function parse_gpu_flag() {
    local use_gpu=false
    for arg in "$@"; do
        if [[ "$arg" == "--gpu" ]]; then
            use_gpu=true
        fi
    done
    echo "$use_gpu"
}

# Utility to determine CUDA support
function has_cuda_support() {
    local use_CUDA=false
    for arg in "$@"; do
        if [[ "$arg" == "--cuda" ]]; then
            use_CUDA=true
        fi
    done
    echo "$use_CUDA"
}

function get_base_tag() {
    local gpu_flag
    gpu_flag=$(parse_gpu_flag "$@")
    cuda_flag=$(has_cuda_support "$@")
    if [[ "$gpu_flag" == "true" ]]; then
        echo "$DEFAULT_BASE_TAG"
    elif [[ "$cuda_flag" == "true" ]]; then
        echo "$CUDA_BASE_TAG"
    else
        echo "$DEFAULT_BASE_TAG"

    fi
}

function get_nav2_service() {
    local gpu_flag
    gpu_flag=$(parse_gpu_flag "$@")
    if [[ "$gpu_flag" == "true" ]]; then
        echo "nav2_gpu"
    elif [[ "$(has_cuda_support "$@")" == "true" ]]; then
        echo "nav2_cuda"
    else
        echo "nav2_cpu"
    fi
}

# Step-by-step helpers
function build_base_image() {
    local base_tag
    base_tag=$(get_base_tag "$@")
    echo "🚧 Building base image: $base_tag"
    docker compose -f "$COMPOSE_FILE_PATH"  build "$base_tag"
}

function build_nav2_image() {
    local base_tag
    base_tag=$(get_base_tag "$@")
    local service
    service=$(get_nav2_service "$@")

    echo "🔧 Building Nav2 image: $service (based on $base_tag)"
    NAV2_BASE_IMAGE="$BASE_REPO:$base_tag" \
    NAV2_BASE_IMAGE_TAG="$base_tag" \
    docker compose -f "$COMPOSE_FILE_PATH"  build "$service"
}

function run_container() {
    local base_tag
    base_tag=$(get_base_tag "$@")
    local base_image="$BASE_REPO:$base_tag"
    local service
    service=$(get_nav2_service "$@")

    echo "🚀 Starting container: $service"
    xhost +local:docker
    NAV2_BASE_IMAGE="$base_image" \
    NAV2_BASE_IMAGE_TAG="$base_tag" \
    docker compose -f "$COMPOSE_FILE_PATH"  up -d "$service"
}

function attach_shell() {
    local service
    service=$(get_nav2_service "$@")

    echo "🧑‍💻 Attaching to $service shell..."
    docker exec -it "$service" bash
}

# Top-level operations
function deploy() {
    # Require at least one argument (e.g., --gpu, --cuda, or default)
    if [[ $# -lt 2 ]]; then
        echo "❌ Error: Please specify a target to deploy (e.g., --gpu, --cuda)"
        exit 1
    fi

    build_base_image "$@"
    build_nav2_image "$@"
    run_container "$@"
    attach_shell "$@"
}


function dev_mode() {
    run_container "$@"
    attach_shell "$@"
}

function stop_container() {
    # At least one argument is required after -stop
    if [[ $# -lt 2 ]]; then
        echo "❌ Error: Please specify a container to stop (e.g., --gpu, --cuda, or 'all')"
        exit 1
    fi

    # Check for "all"
    for arg in "$@"; do
        if [[ "$arg" == "all" ]]; then
            echo "🛑 Stopping all containers from docker-compose.yml..."
            docker compose -f "$COMPOSE_FILE_PATH"  stop
            return
        fi
    done

    # Otherwise stop the specific one
    local service
    service=$(get_nav2_service "$@")

    echo "🛑 Stopping container: $service"
    docker compose -f "$COMPOSE_FILE_PATH"  stop "$service"
}

function remove_container() {
    # At least one argument is required after -remove
    if [[ $# -lt 2 ]]; then
        echo "❌ Error: Please specify a container to remove (e.g., --gpu, --cuda, or 'all')"
        exit 1
    fi

    # Check for "all"
    for arg in "$@"; do
        if [[ "$arg" == "all" ]]; then
            echo "🛑 Removing all containers from docker-compose.yml..."
            docker compose -f "$COMPOSE_FILE_PATH" down
            return
        fi
    done

    # Otherwise remove the specific service
    local service
    service=$(get_nav2_service "$@")

    echo "🛑 Removing container: $service"
    docker compose -f "$COMPOSE_FILE_PATH" down "$service"
}



# function remove_all() {
#     echo "🧹 Removing all containers and resources..."
#     docker compose -f "$COMPOSE_FILE_PATH"  down
# }

function help_message() {
    echo "Usage: ./vsss_sim.sh [COMMAND] [--gpu]"
    echo
    echo "Commands:"
    echo "  -build-base [--gpu]      Build only the base image"
    echo "  -build-vsss [--gpu]      Build only the VSSS image"
    echo "  -run [--gpu]             Start only the container"
    echo "  -dev-mode [--gpu]        Run + attach to shell"
    echo "  -deploy [--gpu|--cuda]       Build everything, run, and attach"
    echo "  -remove                  Remove all containers"
    echo "  -help                    Show this help message"
    echo
    echo "Examples:"
    echo "  ./vsss_sim.sh -deploy"
    echo "  ./vsss_sim.sh -dev-mode --gpu"
    echo "  ./vsss_sim.sh -build-base"
}

# Main dispatcher
case "$1" in
    -build-base)
        build_base_image "$@"
        ;;
    -build-nav2)
        build_nav2_image "$@"
        ;;
    -run)
        run_container "$@"
        ;;
    -dev-mode)
        dev_mode "$@"
        ;;
    -deploy)
        deploy "$@"
        ;;
    -remove)
        remove_container "$@"
        ;;
    -help|--help)
        help_message
        ;;
    -stop)
        stop_container "$@"
        ;;
    *)
        echo "Unknown command: $1"
        help_message
        exit 1
        ;;
esac
