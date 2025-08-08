#!/bin/bash

# Test script for running trimmed Julia binary in a minimal Podman container
# This script creates a minimal glibc container and tests the juliac-compiled binary

set -euo pipefail

# Configuration
CONTAINER_NAME="julia-test-$(date +%s)"
IMAGE_NAME="debian:bookworm-slim"
WORK_DIR="/juliatest"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if podman is available
if ! command -v podman &> /dev/null; then
    echo -e "${RED}[ERROR]${NC} Podman not found. Please install podman first."
    exit 1
fi

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

cleanup() {
    log_info "Cleaning up container..."
    podman rm -f "$CONTAINER_NAME" 2>/dev/null || true
}

# Trap to ensure cleanup on exit
trap cleanup EXIT

main() {
    log_info "Testing trimmed Julia binary in minimal container..."
    
    # Check if we're in the right directory
    if [[ ! -f "Makefile" ]] || [[ ! -f "main.c" ]]; then
        log_error "Script must be run from the juliac directory (containing Makefile and main.c)"
        exit 1
    fi
    
    # Check if required files exist
    if [[ ! -f "mainc" ]]; then
        log_error "mainc binary not found. Please run 'make run' or 'make mainc' first."
        exit 1
    fi
    
    if [[ ! -f "RunwayLibCompiled/lib/libposeest.so" ]]; then
        log_error "RunwayLibCompiled/lib/libposeest.so not found. Please ensure libposeest.so is in RunwayLibCompiled/lib/"
        exit 1
    fi
    
    if [[ ! -d "RunwayLibCompiled" ]]; then
        log_error "RunwayLibCompiled/ directory not found. Please ensure the bundle is properly created."
        exit 1
    fi
    
    log_info "Creating minimal container with glibc..."
    
    # Create container with minimal Debian image (has glibc) - NO NETWORK ACCESS
    podman run -d --name "$CONTAINER_NAME" \
        --rm \
        --network=none \
        -v "$(pwd):$WORK_DIR:ro" \
        -w "$WORK_DIR" \
        "$IMAGE_NAME" \
        sleep infinity
    
    log_info "Container created: $CONTAINER_NAME"
    
    # Check what's available in the container
    log_info "Checking container environment..."
    podman exec "$CONTAINER_NAME" uname -a
    podman exec "$CONTAINER_NAME" ls -la /lib64/
    
    # All libraries (including SSL/crypto) are now bundled - no system packages needed!
    log_info "Using fully self-contained bundle - no system dependencies required"
    
    # Copy files to a writable location in container
    log_info "Setting up test environment in container..."
    podman exec "$CONTAINER_NAME" mkdir -p /tmp/juliatest
    podman exec "$CONTAINER_NAME" bash -c "cp -r $WORK_DIR/* /tmp/juliatest/"
    podman exec "$CONTAINER_NAME" chmod +x /tmp/juliatest/mainc
    
    # Test library dependencies
    log_info "Checking library dependencies..."
    podman exec "$CONTAINER_NAME" ldd /tmp/juliatest/mainc
    
    log_info "Checking Julia runtime libraries..."
    podman exec "$CONTAINER_NAME" ldd /tmp/juliatest/RunwayLibCompiled/lib/libposeest.so
    
    # Test if libraries can be loaded
    log_info "Testing library loading..."
    if ! podman exec "$CONTAINER_NAME" bash -c "cd /tmp/juliatest && JULIA_DEPOT_PATH=RunwayLibCompiled/share/julia LD_LIBRARY_PATH=RunwayLibCompiled/lib:RunwayLibCompiled/lib/julia ./mainc"; then
        log_error "Binary execution failed"
        
        # Debug information
        log_warn "Debug: Checking file permissions..."
        podman exec "$CONTAINER_NAME" ls -la /tmp/juliatest/mainc
        
        log_warn "Debug: Checking library structure..."
        podman exec "$CONTAINER_NAME" bash -c "cd /tmp/juliatest && ls -la RunwayLibCompiled/lib/ | head -5"
        
        log_warn "Debug: Checking missing system libraries..."
        podman exec "$CONTAINER_NAME" bash -c "cd /tmp/juliatest && ldd mainc | grep 'not found'" || true
        
        log_warn "Debug: Checking RPATH..."
        podman exec "$CONTAINER_NAME" bash -c "cd /tmp/juliatest && objdump -x mainc | grep RPATH" || true
        
        return 1
    fi
    
    log_info "✅ Test passed! The trimmed Julia binary runs successfully in minimal container."
    
    # Show container size and resource usage
    log_info "Container stats:"
    podman exec "$CONTAINER_NAME" du -sh /tmp/juliatest/
    
    # Test performance (basic)
    log_info "Running performance test..."
    time_output=$(podman exec "$CONTAINER_NAME" bash -c "cd /tmp/juliatest && time -p JULIA_DEPOT_PATH=RunwayLibCompiled/share/julia LD_LIBRARY_PATH=RunwayLibCompiled/lib:RunwayLibCompiled/lib/julia ./mainc 2>&1" | grep -E '^(real|user|sys)')
    echo "$time_output"
}

log_info "Using container runtime: podman"

# Run main function
main "$@"
