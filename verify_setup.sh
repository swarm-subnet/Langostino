#!/bin/bash
################################################################################
# Swarm ROS2 Environment Verification Script
#
# This script verifies that all components of the Swarm ROS2 system are
# properly installed and configured.
#
# Usage:
#   ./verify_setup.sh
#
# Author: Swarm Team
# Date: 2024-10-27
################################################################################

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

CHECKS_PASSED=0
CHECKS_FAILED=0
CHECKS_WARNING=0

################################################################################
# Helper Functions
################################################################################

print_header() {
    echo -e "\n${BLUE}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}\n"
}

check_pass() {
    echo -e "${GREEN}✓ $1${NC}"
    ((CHECKS_PASSED++))
}

check_fail() {
    echo -e "${RED}✗ $1${NC}"
    ((CHECKS_FAILED++))
}

check_warn() {
    echo -e "${YELLOW}⚠ $1${NC}"
    ((CHECKS_WARNING++))
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

################################################################################
# Check Functions
################################################################################

check_ubuntu_version() {
    print_header "System Information"

    if [[ -f /etc/os-release ]]; then
        source /etc/os-release
        echo "OS: $NAME $VERSION"
        echo "Kernel: $(uname -r)"
        echo "Architecture: $(uname -m)"

        if [[ "$ID" == "ubuntu" ]] && [[ "$VERSION_ID" == "22.04" ]]; then
            check_pass "Ubuntu 22.04 detected"
        else
            check_warn "Not running Ubuntu 22.04 (found: $ID $VERSION_ID)"
        fi
    else
        check_fail "Cannot determine OS version"
    fi
}

check_ros2() {
    print_header "ROS2 Humble Installation"

    if [[ -f /opt/ros/humble/setup.bash ]]; then
        check_pass "ROS2 Humble installed"

        # Source ROS2 and check version
        source /opt/ros/humble/setup.bash
        if command -v ros2 &> /dev/null; then
            check_pass "ros2 command available"
            echo "  Version: $(ros2 --version 2>&1 | head -n1)"
        else
            check_fail "ros2 command not found"
        fi
    else
        check_fail "ROS2 Humble not installed"
        return
    fi

    # Check for common ROS2 packages
    local packages=("rclpy" "colcon" "rosdep")
    for pkg in "${packages[@]}"; do
        if command -v "$pkg" &> /dev/null || python3 -c "import $pkg" 2>/dev/null; then
            check_pass "$pkg available"
        else
            check_warn "$pkg not found"
        fi
    done
}

check_python_packages() {
    print_header "Python Dependencies"

    local packages=(
        "numpy"
        "scipy"
        "serial:pyserial"
        "smbus2"
        "torch"
        "stable_baselines3"
        "gym"
    )

    for pkg_spec in "${packages[@]}"; do
        # Split on : to get import name and display name
        IFS=':' read -r import_name display_name <<< "$pkg_spec"
        display_name=${display_name:-$import_name}

        if python3 -c "import $import_name" 2>/dev/null; then
            version=$(python3 -c "import $import_name; print(getattr($import_name, '__version__', 'unknown'))" 2>/dev/null)
            check_pass "$display_name ($version)"
        else
            check_fail "$display_name not installed"
        fi
    done
}

check_i2c() {
    print_header "I2C Configuration (LiDAR)"

    # Check if I2C module is loaded
    if lsmod | grep -q i2c_dev; then
        check_pass "i2c-dev module loaded"
    else
        check_fail "i2c-dev module not loaded"
    fi

    # Check if I2C device exists
    if [[ -e /dev/i2c-1 ]]; then
        check_pass "I2C bus /dev/i2c-1 exists"

        # Check permissions
        if [[ -r /dev/i2c-1 ]] && [[ -w /dev/i2c-1 ]]; then
            check_pass "I2C device is accessible"
        else
            check_warn "I2C device exists but may not be accessible"
            print_info "Current user may need to be in 'i2c' group"
        fi

        # Check for devices on I2C bus
        if command -v i2cdetect &> /dev/null; then
            print_info "Scanning I2C bus 1..."
            if i2cdetect -y 1 2>/dev/null | grep -q "08"; then
                check_pass "LiDAR detected at address 0x08"
            else
                check_warn "No device detected at 0x08 (LiDAR address)"
                echo -e "\n  I2C bus scan:"
                i2cdetect -y 1 2>/dev/null | sed 's/^/    /'
            fi
        else
            check_warn "i2cdetect not available (install i2c-tools)"
        fi
    else
        check_warn "I2C bus /dev/i2c-1 not found (may not be on Raspberry Pi)"
    fi

    # Check user groups
    if groups | grep -q i2c; then
        check_pass "User is in 'i2c' group"
    else
        check_warn "User is not in 'i2c' group"
        print_info "Run: sudo usermod -a -G i2c $USER (requires re-login)"
    fi
}

check_uart() {
    print_header "UART Configuration (Flight Controller)"

    # Check if UART device exists
    local uart_devices=("/dev/ttyAMA0" "/dev/ttyUSB0" "/dev/ttyACM0")
    local found=false

    for device in "${uart_devices[@]}"; do
        if [[ -e "$device" ]]; then
            check_pass "Serial device $device exists"
            found=true

            # Check permissions
            if [[ -r "$device" ]] && [[ -w "$device" ]]; then
                check_pass "$device is accessible"
            else
                check_warn "$device exists but may not be accessible"
                print_info "Current user may need to be in 'dialout' group"
            fi
        fi
    done

    if [[ "$found" = false ]]; then
        check_warn "No serial devices found (flight controller may not be connected)"
        print_info "Expected: /dev/ttyAMA0 (or ttyUSB0/ttyACM0)"
    fi

    # Check user groups
    if groups | grep -q dialout; then
        check_pass "User is in 'dialout' group"
    else
        check_warn "User is not in 'dialout' group"
        print_info "Run: sudo usermod -a -G dialout $USER (requires re-login)"
    fi
}

check_workspace() {
    print_header "ROS2 Workspace"

    local workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

    # Check if workspace is built
    if [[ -d "$workspace_dir/install" ]]; then
        check_pass "Workspace built (install directory exists)"
    else
        check_fail "Workspace not built (no install directory)"
        print_info "Run: colcon build --symlink-install"
    fi

    # Check if workspace can be sourced
    if [[ -f "$workspace_dir/install/setup.bash" ]]; then
        check_pass "Workspace setup.bash exists"

        # Try sourcing it
        if source "$workspace_dir/install/setup.bash" 2>/dev/null; then
            check_pass "Workspace can be sourced"
        else
            check_warn "Error sourcing workspace setup.bash"
        fi
    else
        check_fail "Workspace setup.bash not found"
    fi

    # Check if package is available
    source /opt/ros/humble/setup.bash 2>/dev/null
    if [[ -f "$workspace_dir/install/setup.bash" ]]; then
        source "$workspace_dir/install/setup.bash" 2>/dev/null
        if ros2 pkg list 2>/dev/null | grep -q swarm_ai_integration; then
            check_pass "swarm_ai_integration package found"
        else
            check_warn "swarm_ai_integration package not found in workspace"
        fi
    fi
}

check_launch_files() {
    print_header "Launch Configuration"

    local workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

    # Check launch file
    if [[ -f "$workspace_dir/src/swarm_ai_integration/launch/swarm_ai_launch.py" ]]; then
        check_pass "Launch file exists"
    else
        check_fail "Launch file not found"
    fi

    # Check config file
    if [[ -f "$workspace_dir/src/swarm_ai_integration/config/swarm_params.yaml" ]]; then
        check_pass "Configuration file exists"
    else
        check_fail "Configuration file not found"
    fi

    # Check launch script
    if [[ -f "$workspace_dir/launch.sh" ]]; then
        check_pass "launch.sh script exists"
    else
        check_warn "launch.sh script not found"
    fi
}

check_pm2() {
    print_header "PM2 Process Manager (Optional)"

    if command -v pm2 &> /dev/null; then
        check_pass "PM2 installed"
        echo "  Version: $(pm2 --version)"
    else
        check_warn "PM2 not installed (optional)"
        print_info "Install with: sudo npm install -g pm2"
    fi

    if command -v node &> /dev/null; then
        check_pass "Node.js installed"
        echo "  Version: $(node --version)"
    else
        check_warn "Node.js not installed (needed for PM2)"
    fi
}

################################################################################
# Main Execution
################################################################################

main() {
    echo -e "${BLUE}"
    echo "╔═══════════════════════════════════════════════════════════════╗"
    echo "║     Swarm ROS2 Environment Verification                       ║"
    echo "╚═══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"

    check_ubuntu_version
    check_ros2
    check_python_packages
    check_i2c
    check_uart
    check_workspace
    check_launch_files
    check_pm2

    # Summary
    print_header "Verification Summary"

    echo -e "Results:"
    echo -e "  ${GREEN}Passed:  $CHECKS_PASSED${NC}"
    echo -e "  ${YELLOW}Warnings: $CHECKS_WARNING${NC}"
    echo -e "  ${RED}Failed:  $CHECKS_FAILED${NC}"
    echo ""

    if [[ $CHECKS_FAILED -eq 0 ]]; then
        if [[ $CHECKS_WARNING -eq 0 ]]; then
            echo -e "${GREEN}✓ All checks passed! System is ready.${NC}"
            exit 0
        else
            echo -e "${YELLOW}⚠ System is mostly ready but has some warnings.${NC}"
            echo -e "  Review warnings above and address as needed."
            exit 0
        fi
    else
        echo -e "${RED}✗ Some checks failed. Please address the issues above.${NC}"
        exit 1
    fi
}

# Run main function
main "$@"
