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

# Check if docker is available (fallback to podman)

if command -v podman &> /dev/null; then
    CONTAINER_CMD="podman"
elif command -v docker &> /dev/null; then
    CONTAINER_CMD="docker"
else
    echo -e "${RED}[ERROR]${NC} Neither Docker nor Podman found. Please install one of them first."
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
    $CONTAINER_CMD rm -f "$CONTAINER_NAME" 2>/dev/null || true
}

# Trap to ensure cleanup on exit
trap cleanup EXIT

main() {
    log_info "Testing trimmed Julia binary in minimal container..."
    
    # Check if required files exist
    if [[ ! -f "mainc" ]]; then
        log_error "mainc binary not found. Please run 'make mainc' first."
        exit 1
    fi
    
    if [[ ! -f "libposeest.so" ]]; then
        log_error "libposeest.so not found. Please run 'make libposeest.so' first."
        exit 1
    fi
    
    if [[ ! -d "julia" ]]; then
        log_error "julia/ directory not found. Please run 'make julia-clean' first."
        exit 1
    fi
    
    log_info "Creating minimal container with glibc..."
    
    # Create container with minimal Debian image (has glibc)
    if [[ "$CONTAINER_CMD" == "podman" ]]; then
        $CONTAINER_CMD run -d --name "$CONTAINER_NAME" \
            --rm \
            -v "$(pwd):$WORK_DIR:ro" \
            -w "$WORK_DIR" \
            "$IMAGE_NAME" \
            sleep infinity
    else
        $CONTAINER_CMD run -d --name "$CONTAINER_NAME" \
            --rm \
            -v "$(pwd):$WORK_DIR:ro" \
            -w "$WORK_DIR" \
            "$IMAGE_NAME" \
            sleep infinity
    fi
    
    log_info "Container created: $CONTAINER_NAME"
    
    # Function to run exec command
    container_exec() {
        $CONTAINER_CMD exec "$CONTAINER_NAME" "$@"
    }
    
    # Check what's available in the container
    log_info "Checking container environment..."
    container_exec uname -a
    container_exec ls -la /lib64/
    
    # Note: Using self-contained Julia bundle - no package installation needed
    
    # Copy files to a writable location in container
    log_info "Setting up test environment in container..."
    container_exec mkdir -p /tmp/juliatest
    container_exec bash -c "cp -r $WORK_DIR/* /tmp/juliatest/"
    container_exec chmod +x /tmp/juliatest/mainc
    
    # Test library dependencies
    log_info "Checking library dependencies..."
    container_exec ldd /tmp/juliatest/mainc
    
    log_info "Checking Julia runtime libraries..."
    container_exec ldd /tmp/juliatest/libposeest.so
    
    # Test if libraries can be loaded
    log_info "Testing library loading..."
    container_exec bash -c "cd /tmp/juliatest && LD_LIBRARY_PATH=.:julia ./mainc" || {
        log_error "Binary execution failed"
        
        # Debug information
        log_warn "Debug: Checking file permissions..."
        container_exec ls -la /tmp/juliatest/
        
        log_warn "Debug: Checking library path..."
        container_exec bash -c "cd /tmp/juliatest && find . -name '*.so*' | head -10"
        
        log_warn "Debug: Checking missing libraries..."
        container_exec bash -c "cd /tmp/juliatest && ldd mainc | grep 'not found'" || true
        
        return 1
    }
    
    log_info "✅ Test passed! The trimmed Julia binary runs successfully in minimal container."
    
    # Show container size and resource usage
    log_info "Container stats:"
    container_exec du -sh /tmp/juliatest/
    
    # Test performance (basic)
    log_info "Running performance test..."
    time_output=$(container_exec bash -c "cd /tmp/juliatest && time -p LD_LIBRARY_PATH=.:julia ./mainc 2>&1" | grep -E '^(real|user|sys)')
    echo "$time_output"
}

log_info "Using container runtime: $CONTAINER_CMD"

# Run main function
main "$@"
