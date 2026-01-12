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
ROS_DISTRO=""
ROS_DISTRO_NAME=""
ROS_SETUP_PATH=""

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

        if [[ "$ID" == "ubuntu" ]] && [[ "$VERSION_ID" == "22.04" || "$VERSION_ID" == "24.04" ]]; then
            check_pass "Ubuntu $VERSION_ID detected"
        else
            check_warn "Not running Ubuntu 22.04/24.04 (found: $ID $VERSION_ID)"
        fi
    else
        check_fail "Cannot determine OS version"
    fi
}

check_ros2() {
    print_header "ROS2 Installation"

    local distros=("jazzy" "humble")
    local found=()

    for distro in "${distros[@]}"; do
        if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
            found+=("$distro")
        fi
    done

    if [[ ${#found[@]} -eq 0 ]]; then
        check_fail "ROS2 not installed (Humble or Jazzy)"
        return
    fi

    ROS_DISTRO="${found[0]}"
    case "$ROS_DISTRO" in
        jazzy) ROS_DISTRO_NAME="Jazzy" ;;
        humble) ROS_DISTRO_NAME="Humble" ;;
        *) ROS_DISTRO_NAME="$ROS_DISTRO" ;;
    esac
    ROS_SETUP_PATH="/opt/ros/${ROS_DISTRO}/setup.bash"

    if [[ ${#found[@]} -gt 1 ]]; then
        check_warn "Multiple ROS2 distros found (${found[*]}). Using ${ROS_DISTRO_NAME}"
    fi

    check_pass "ROS2 ${ROS_DISTRO_NAME} installed"

    # Source ROS2 and check version
    source "$ROS_SETUP_PATH"
    if command -v ros2 &> /dev/null; then
        check_pass "ros2 command available"
        echo "  Version: $(ros2 --version 2>&1 | head -n1)"
    else
        check_fail "ros2 command not found"
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

    echo "System Python Packages (for ROS nodes):"
    echo ""

    local system_packages=(
        "numpy"
        "scipy"
        "serial:pyserial"
    )

    for pkg_spec in "${system_packages[@]}"; do
        # Split on : to get import name and display name
        IFS=':' read -r import_name display_name <<< "$pkg_spec"
        display_name=${display_name:-$import_name}

        if python3 -c "import $import_name" 2>/dev/null; then
            version=$(python3 -c "import $import_name; print(getattr($import_name, '__version__', 'unknown'))" 2>/dev/null)
            check_pass "$display_name ($version) [system]"
        else
            check_fail "$display_name not installed [system]"
        fi
    done

    echo ""
    echo "AI Flight Node Virtual Environment:"
    echo ""

    local VENV_PATH="${HOME}/ai_flight_node_env"

    if [[ ! -d "$VENV_PATH" ]]; then
        check_fail "Virtual environment not found at $VENV_PATH"
        print_info "Run setup.sh to create the virtual environment"
        return
    fi

    check_pass "Virtual environment exists at $VENV_PATH"

    # Check if venv has Python
    if [[ ! -f "$VENV_PATH/bin/python3" ]]; then
        check_fail "Python not found in virtual environment"
        return
    fi

    local ai_packages=(
        "numpy"
        "typing_extensions:typing-extensions"
        "gymnasium"
        "stable_baselines3"
        "torch"
    )

    for pkg_spec in "${ai_packages[@]}"; do
        # Split on : to get import name and display name
        IFS=':' read -r import_name display_name <<< "$pkg_spec"
        display_name=${display_name:-$import_name}

        if "$VENV_PATH/bin/python3" -c "import $import_name" 2>/dev/null; then
            version=$("$VENV_PATH/bin/python3" -c "import $import_name; print(getattr($import_name, '__version__', 'unknown'))" 2>/dev/null)
            check_pass "$display_name ($version) [venv]"
        else
            check_fail "$display_name not installed [venv]"
        fi
    done

    if python3 -c "import smbus2" 2>/dev/null; then
        version=$(python3 -c "import smbus2; print(getattr(smbus2, '__version__', 'unknown'))" 2>/dev/null)
        check_pass "smbus2 ($version) [system]"
    elif python3 -c "import smbus" 2>/dev/null; then
        check_pass "smbus [system]"
        check_warn "smbus2 not installed (using smbus fallback)"
    else
        check_fail "smbus2/smbus not installed [system]"
    fi
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

            # CRITICAL: Check if ttyAMA0 is being used by serial console or getty
            if [[ "$device" == "/dev/ttyAMA0" ]]; then
                # Check cmdline.txt for serial console
                if [[ -f /boot/firmware/cmdline.txt ]]; then
                    if grep -q "console=serial0" /boot/firmware/cmdline.txt || grep -q "console=ttyAMA0" /boot/firmware/cmdline.txt; then
                        check_fail "Serial console still enabled in cmdline.txt"
                        print_info "This will interfere with flight controller communication!"
                        print_info "Remove console=serial0 or console=ttyAMA0 from /boot/firmware/cmdline.txt"
                    else
                        check_pass "Serial console removed from cmdline.txt"
                    fi
                fi

                # Check if serial-getty service is disabled
                if systemctl is-enabled serial-getty@ttyAMA0.service &>/dev/null; then
                    check_fail "serial-getty@ttyAMA0.service is enabled"
                    print_info "Run: sudo systemctl disable serial-getty@ttyAMA0.service"
                else
                    check_pass "serial-getty@ttyAMA0.service is disabled"
                fi

                # Check if serial-getty service is running
                if systemctl is-active serial-getty@ttyAMA0.service &>/dev/null; then
                    check_fail "serial-getty@ttyAMA0.service is running"
                    print_info "Run: sudo systemctl stop serial-getty@ttyAMA0.service"
                else
                    check_pass "serial-getty@ttyAMA0.service is not running"
                fi

                # Check if anything is using the UART (requires lsof)
                if command -v lsof &> /dev/null; then
                    local lsof_output=$(sudo lsof /dev/ttyAMA0 2>/dev/null)
                    if [[ -n "$lsof_output" ]]; then
                        check_warn "Something is using /dev/ttyAMA0"
                        echo -e "\n  Processes using ttyAMA0:"
                        echo "$lsof_output" | sed 's/^/    /'
                        print_info "This may interfere with flight controller communication"
                    else
                        check_pass "/dev/ttyAMA0 is not in use (ready for FC)"
                    fi
                else
                    check_warn "lsof not available (cannot check if ttyAMA0 is in use)"
                    print_info "Install with: sudo apt install lsof"
                fi

                # Check dmesg for PL011 confirmation
                if dmesg | grep -q "ttyAMA0.*PL011"; then
                    check_pass "ttyAMA0 is using PL011 hardware UART"
                else
                    check_warn "Cannot confirm PL011 UART (check dmesg | grep tty)"
                fi
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

    # Raspberry Pi specific: Check Bluetooth is disabled
    if [[ -f /boot/firmware/config.txt ]]; then
        if grep -q "^dtoverlay=disable-bt" /boot/firmware/config.txt; then
            check_pass "Bluetooth disabled (dtoverlay=disable-bt)"
        else
            check_warn "Bluetooth not disabled in config.txt"
            print_info "Add 'dtoverlay=disable-bt' to /boot/firmware/config.txt"
        fi
    fi
}

check_workspace() {
    print_header "ROS2 Workspace"

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local workspace_dir="$(cd "${script_dir}/.." && pwd)"

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
    if [[ -n "$ROS_SETUP_PATH" && -f "$ROS_SETUP_PATH" ]]; then
        source "$ROS_SETUP_PATH" 2>/dev/null
        if [[ -f "$workspace_dir/install/setup.bash" ]]; then
            source "$workspace_dir/install/setup.bash" 2>/dev/null
            if ros2 pkg list 2>/dev/null | grep -q swarm_ai_integration; then
                check_pass "swarm_ai_integration package found"
            else
                check_warn "swarm_ai_integration package not found in workspace"
            fi
        fi
    else
        check_warn "ROS2 setup.bash not found; skipping package list check"
    fi
}

check_launch_files() {
    print_header "Launch Configuration"

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local workspace_dir="$(cd "${script_dir}/.." && pwd)"

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

    # Check launch script (repo root or scripts/)
    if [[ -f "$workspace_dir/launch.sh" ]]; then
        check_pass "launch.sh script exists (root)"
    elif [[ -f "$workspace_dir/scripts/launch.sh" ]]; then
        check_pass "launch.sh script exists (scripts/)"
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
