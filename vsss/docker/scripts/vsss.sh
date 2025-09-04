#!/bin/bash

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE_PATH="$PROJECT_ROOT/compose/docker-compose.yml"

# Load environment variables from .env file
if [ -f "$PROJECT_ROOT/compose/.env" ]; then
    export $(grep -v '^#' "$PROJECT_ROOT/compose/.env" | xargs)
else
    echo "❌ Error: .env file not found in $PROJECT_ROOT/compose/"
    exit 1
fi

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

function parse_cuda_flag() {
    local use_cuda=false
    for arg in "$@"; do
        if [[ "$arg" == "--cuda" ]]; then
            use_cuda=true
        fi
    done
    echo "$use_cuda"
}

function parse_cpu_flag() {
    local use_cpu=false
    for arg in "$@"; do
        if [[ "$arg" == "--cpu" ]]; then
            use_cpu=true
        fi
    done
    echo "$use_cpu"
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

function get_torch_index_url() {
    local gpu_flag
    gpu_flag=$(parse_gpu_flag "$@")
    cuda_flag=$(has_cuda_support "$@")
    
    # Por ahora usar CPU para todos los casos hasta resolver las librerías CUDA
    echo "https://download.pytorch.org/whl/cpu"
}

function get_base_tag() {
    local gpu_flag
    gpu_flag=$(parse_gpu_flag "$@")
    cuda_flag=$(has_cuda_support "$@")
    if [[ "$gpu_flag" == "true" ]]; then
        echo "$CPU_BASE_IMAGE_TAG"
    elif [[ "$cuda_flag" == "true" ]]; then
        echo "$CUDA_BASE_IMAGE_TAG"
    else
        echo "$CPU_BASE_IMAGE_TAG"
    fi
}

function get_vsss_service() {
    local gpu_flag
    gpu_flag=$(parse_gpu_flag "$@")
    if [[ "$gpu_flag" == "true" ]]; then
        echo "application_gpu"
    elif [[ "$(has_cuda_support "$@")" == "true" ]]; then
        echo "application_cuda"
    else
        echo "application_cpu"
    fi
}

function get_vsss_container_name() {
    local gpu_flag
    gpu_flag=$(parse_gpu_flag "$@")
    if [[ "$gpu_flag" == "true" ]]; then
        echo "vsss_gpu"
    elif [[ "$(has_cuda_support "$@")" == "true" ]]; then
        echo "vsss_cuda"
    else
        echo "vsss_cpu"
    fi
}

# Step-by-step helpers
function build_base_image() {
    local base_tag
    base_tag=$(get_base_tag "$@")
    echo "🚧 Building base image: $base_tag"
    docker compose -f "$COMPOSE_FILE_PATH" build "$base_tag"
}

function build_vsss_image() {
    local base_tag
    base_tag=$(get_base_tag "$@")
    local service
    service=$(get_vsss_service "$@")
    local torch_url
    torch_url=$(get_torch_index_url "$@")

    echo "🔧 Building vsss image: $service (based on $base_tag)"
    vsss_BASE_IMAGE="$DOCKER_REGISTRY/$PROJECT_NAME:$base_tag" \
    vsss_BASE_IMAGE_TAG="$base_tag" \
    TORCH_INDEX_URL="$torch_url" \
    docker compose -f "$COMPOSE_FILE_PATH" build "$service"
}

function run_container() {
    local base_tag
    base_tag=$(get_base_tag "$@")
    local base_image="$DOCKER_REGISTRY/$PROJECT_NAME:$base_tag"
    local service
    service=$(get_vsss_service "$@")
    local container_name
    container_name=$(get_vsss_container_name "$@")
    local torch_url
    torch_url=$(get_torch_index_url "$@")

    echo "🚀 Starting container: $service"
    xhost +local:docker
    vsss_BASE_IMAGE="$base_image" \
    vsss_BASE_IMAGE_TAG="$base_tag" \
    TORCH_INDEX_URL="$torch_url" \
    docker compose -f "$COMPOSE_FILE_PATH" up -d "$service"
    # until docker exec -it "$container_name" bash -c "ls /tmp/build_done" &>/dev/null; do
    #     echo "⏳ Waiting for build to complete..."
    #     sleep 2
    # done
    echo "✅ Done"
}

function attach_shell() {
    local container_name
    container_name=$(get_vsss_container_name "$@")

    echo "🧑‍💻 Attaching to $container_name shell..."
    docker exec -it "$container_name" bash
}

# Top-level operations
function deploy() {
    # Require at least one argument (e.g., --gpu, --cuda, or default)
    if [[ $# -lt 2 ]]; then
        echo "❌ Error: Please specify a target to deploy (e.g., --gpu, --cuda)"
        exit 1
    fi

    if [[ "$(parse_gpu_flag "$@")" == "true" ]]; then
        echo "🚀 Deploying with GPU support..."
    elif [[ "$(parse_cuda_flag "$@")" == "true" ]]; then
        echo "🚀 Deploying with CUDA support..."
    elif [[ "$(parse_cpu_flag "$@")" == "true" ]]; then
        echo "🚀 Deploying with CPU support..."
    else
        echo "❌ Error: No valid target specified. Please use --gpu, --cuda, or --cpu."
        exit 1
    fi

    build_base_image "$@"
    build_vsss_image "$@"
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
    service=$(get_vsss_service "$@")

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
    service=$(get_vsss_service "$@")

    echo "🛑 Removing container: $service"
    docker compose -f "$COMPOSE_FILE_PATH" down "$service"
}



# function remove_all() {
#     echo "🧹 Removing all containers and resources..."
#     docker compose -f "$COMPOSE_FILE_PATH"  down
# }

function help_message() {
    echo "Usage: ./vsss.sh [COMMAND] [--gpu]"
    echo
    echo "Commands:"
    echo "  -build-base [--gpu]      Build only the base image"
    echo "  -build-development-image [--gpu]      Build only the APPLICATION image"
    echo "  -run [--gpu]             Start only the container"
    echo "  -dev-mode [--gpu]        Run + attach to shell"
    echo "  -deploy [--gpu|--cuda]       Build everything, run, and attach"
    echo "  -remove                  Remove all containers"
    echo "  -help                    Show this help message"
    echo
    echo "Examples:"
    echo "  ./vsss.sh -deploy"
    echo "  ./vsss.sh -dev-mode --gpu"
    echo "  ./vsss.sh -build-base"
}

# Main dispatcher
case "$1" in
    -build-base)
        build_base_image "$@"
        ;;
    -build-development-image)
        build_vsss_image "$@"
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
