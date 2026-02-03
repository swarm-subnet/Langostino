#!/bin/bash
################################################################################
# Swarm ROS2 Environment Setup Script
#
# This script sets up a complete ROS2 Humble environment on Ubuntu 22.04 for
# the Swarm AI Integration drone control system.
#
# Features:
# - ROS2 Humble installation
# - Python dependencies (PyTorch, stable-baselines3, etc.)
# - Hardware configuration (I2C for LiDAR, UART for Flight Controller)
# - Workspace build and configuration
# - PM2 process manager setup (optional)
# - Hardware connectivity checks
#
# Usage:
#   sudo ./setup.sh [--skip-ros] [--skip-hardware-check] [--skip-pm2]
#
# Options:
#   --skip-ros             Skip ROS2 installation (if already installed)
#   --skip-hardware-check  Skip hardware connectivity checks
#   --skip-pm2            Skip PM2 installation (installed by default)
#
# Author: Swarm Team
# Date: 2024-10-27
################################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SKIP_ROS=false
SKIP_HARDWARE_CHECK=false
INSTALL_PM2=true  # Install PM2 by default for process management
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-ros)
            SKIP_ROS=true
            shift
            ;;
        --skip-hardware-check)
            SKIP_HARDWARE_CHECK=true
            shift
            ;;
        --install-pm2)
            INSTALL_PM2=true
            shift
            ;;
        --skip-pm2)
            INSTALL_PM2=false
            shift
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            echo "Usage: sudo ./setup.sh [--skip-ros] [--skip-hardware-check] [--install-pm2] [--skip-pm2]"
            exit 1
            ;;
    esac
done

################################################################################
# Helper Functions
################################################################################

print_header() {
    echo -e "\n${BLUE}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

check_root() {
    if [[ $EUID -ne 0 ]]; then
        print_error "This script must be run as root (use sudo)"
        exit 1
    fi
}

check_ubuntu_version() {
    print_header "Checking Ubuntu Version"

    if [[ ! -f /etc/os-release ]]; then
        print_error "Cannot determine OS version"
        exit 1
    fi

    source /etc/os-release

    if [[ "$ID" != "ubuntu" ]]; then
        print_error "This script is designed for Ubuntu. Detected: $ID"
        exit 1
    fi

    if [[ "$VERSION_ID" != "22.04" ]]; then
        print_warning "This script is tested on Ubuntu 22.04. You are running $VERSION_ID"
        read -p "Continue anyway? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    else
        print_success "Ubuntu 22.04 detected"
    fi
}

################################################################################
# ROS2 Humble Installation
################################################################################

install_ros2_humble() {
    if [[ "$SKIP_ROS" = true ]]; then
        print_info "Skipping ROS2 installation (--skip-ros flag)"
        return
    fi

    print_header "Installing ROS2 Humble"

    # Check if ROS2 is already installed
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        print_info "ROS2 Humble already installed"
        read -p "Reinstall ROS2? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            print_success "Skipping ROS2 installation"
            return
        fi
    fi

    print_info "Setting up ROS2 apt repository..."

    # Set up locale
    apt update && apt install -y locales
    locale-gen en_US en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # Add ROS2 apt repository
    apt install -y software-properties-common
    add-apt-repository -y universe

    apt update && apt install -y curl gnupg lsb-release
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

    print_info "Installing ROS2 Humble base packages..."
    apt update
    apt install -y ros-humble-ros-base

    print_info "Installing ROS2 development tools..."
    apt install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-dev-tools

    # Initialize rosdep
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        print_info "Initializing rosdep..."
        rosdep init
    fi

    print_info "Updating rosdep..."
    rosdep update

    print_success "ROS2 Humble installed successfully"
}

################################################################################
# System Dependencies
################################################################################

install_system_dependencies() {
    print_header "Installing System Dependencies"

    print_info "Updating package lists..."
    apt update

    print_info "Installing build tools and libraries..."
    apt install -y \
        build-essential \
        cmake \
        git \
        python3-pip \
        python3-dev \
        python3-venv \
        libi2c-dev \
        i2c-tools \
        python3-smbus \
        python3-serial \
        python3-numpy \
        python3-scipy \
        lsof \
        udev

    print_success "System dependencies installed"
}

################################################################################
# Python Dependencies
################################################################################

install_python_dependencies() {
    print_header "Installing Python Dependencies"

    print_info "Upgrading pip..."
    python3 -m pip install --upgrade pip

    # Install system-wide packages for ROS nodes (non-AI)
    print_info "Installing system Python packages (ROS nodes)..."
    python3 -m pip install \
        numpy \
        scipy \
        pyserial \
        smbus2

    print_success "System Python dependencies installed"

    # Create virtual environment for AI Flight Node
    print_info "Creating virtual environment for AI Flight Node..."

    # Determine correct user's home directory
    if [[ -n "$SUDO_USER" ]]; then
        local USER_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
        local ACTUAL_USER="$SUDO_USER"
    else
        local USER_HOME="${HOME}"
        local ACTUAL_USER="${USER}"
    fi

    local VENV_PATH="${USER_HOME}/ai_flight_node_env"
    local REQUIREMENTS_FILE="$WORKSPACE_DIR/requirements.txt"

    # Check if requirements file exists
    if [[ ! -f "$REQUIREMENTS_FILE" ]]; then
        print_error "Requirements file not found: $REQUIREMENTS_FILE"
        exit 1
    fi

    if [[ -d "$VENV_PATH" ]]; then
        print_warning "Virtual environment already exists at $VENV_PATH"
        read -p "Recreate virtual environment? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$VENV_PATH"
            print_info "Removed existing virtual environment"
        else
            print_info "Keeping existing virtual environment"
            print_success "Python dependencies configured"
            return
        fi
    fi

    # Create venv
    python3 -m venv "$VENV_PATH"
    print_success "Virtual environment created at $VENV_PATH"

    # Ensure venv has correct ownership
    if [[ -n "$SUDO_USER" ]] && [[ "$EUID" -eq 0 ]]; then
        chown -R "$ACTUAL_USER:$ACTUAL_USER" "$VENV_PATH"
        print_info "Set venv ownership to $ACTUAL_USER"
    fi

    # Activate venv and install AI packages from requirements file
    print_info "Installing AI packages from requirements file..."
    print_info "Using: $REQUIREMENTS_FILE"

    # Detect architecture for PyTorch installation
    local ARCH=$(dpkg --print-architecture)
    local PYTORCH_INDEX=""

    if [[ "$ARCH" == "arm64" || "$ARCH" == "aarch64" ]]; then
        print_info "ARM64 architecture detected - using PyTorch CPU index for ARM"
        PYTORCH_INDEX="--index-url https://download.pytorch.org/whl/cpu"
    fi

    # Use venv's pip directly (run as actual user if using sudo)
    if [[ -n "$SUDO_USER" ]] && [[ "$EUID" -eq 0 ]]; then
        sudo -u "$ACTUAL_USER" "$VENV_PATH/bin/pip" install --upgrade pip
        # Install PyTorch first with the correct index for ARM if needed
        if [[ -n "$PYTORCH_INDEX" ]]; then
            print_info "Installing PyTorch with ARM-compatible wheels..."
            sudo -u "$ACTUAL_USER" "$VENV_PATH/bin/pip" install torch $PYTORCH_INDEX
        fi
        sudo -u "$ACTUAL_USER" "$VENV_PATH/bin/pip" install -r "$REQUIREMENTS_FILE"
    else
        "$VENV_PATH/bin/pip" install --upgrade pip
        # Install PyTorch first with the correct index for ARM if needed
        if [[ -n "$PYTORCH_INDEX" ]]; then
            print_info "Installing PyTorch with ARM-compatible wheels..."
            "$VENV_PATH/bin/pip" install torch $PYTORCH_INDEX
        fi
        "$VENV_PATH/bin/pip" install -r "$REQUIREMENTS_FILE"
    fi

    print_success "AI packages installed in virtual environment"

    # Verify PyTorch installation
    print_info "Verifying PyTorch installation..."
    if "$VENV_PATH/bin/python3" -c "import torch; print(f'PyTorch {torch.__version__} installed successfully')" 2>/dev/null; then
        print_success "PyTorch verified"
    else
        print_error "PyTorch verification failed!"
        print_warning "You may need to manually install PyTorch in the venv:"
        print_info "  source $VENV_PATH/bin/activate"
        print_info "  pip install torch --index-url https://download.pytorch.org/whl/cpu"
    fi

    # Verify stable-baselines3 installation
    print_info "Verifying stable-baselines3 installation..."
    if "$VENV_PATH/bin/python3" -c "from stable_baselines3 import PPO; print('stable-baselines3 installed successfully')" 2>/dev/null; then
        print_success "stable-baselines3 verified"
    else
        print_error "stable-baselines3 verification failed!"
        print_warning "You may need to manually install stable-baselines3 in the venv:"
        print_info "  source $VENV_PATH/bin/activate"
        print_info "  pip install stable-baselines3"
    fi

    print_info "Virtual environment location: $VENV_PATH"
    print_info "Requirements file: $REQUIREMENTS_FILE"
    print_info "Used by: scripts/ai_flight_node_wrapper.sh"
}

################################################################################
# Hardware Configuration
################################################################################

configure_i2c() {
    print_header "Configuring I2C for LiDAR"

    print_info "Enabling I2C kernel module..."

    # Enable I2C in /boot/firmware/config.txt (Raspberry Pi) or equivalent
    if [[ -f /boot/firmware/config.txt ]]; then
        if ! grep -q "^dtparam=i2c_arm=on" /boot/firmware/config.txt; then
            echo "dtparam=i2c_arm=on" >> /boot/firmware/config.txt
            print_success "I2C enabled in config.txt"
        else
            print_info "I2C already enabled in config.txt"
        fi
    fi

    # Load I2C kernel modules
    if ! lsmod | grep -q i2c_dev; then
        modprobe i2c-dev
        print_success "i2c-dev module loaded"
    fi

    # Ensure I2C loads on boot
    if ! grep -q "i2c-dev" /etc/modules; then
        echo "i2c-dev" >> /etc/modules
        print_success "i2c-dev added to /etc/modules"
    fi

    # Add user to i2c group
    if [[ -n "$SUDO_USER" ]]; then
        usermod -a -G i2c "$SUDO_USER"
        print_success "User $SUDO_USER added to i2c group"
    fi

    # Set I2C permissions
    cat > /etc/udev/rules.d/99-i2c.rules << EOF
KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0660"
EOF
    udevadm control --reload-rules
    udevadm trigger

    print_success "I2C configured successfully"
}

configure_uart() {
    print_header "Configuring UART for Flight Controller"

    print_info "Setting up serial port access..."

    # Add user to dialout group for serial access
    if [[ -n "$SUDO_USER" ]]; then
        usermod -a -G dialout "$SUDO_USER"
        print_success "User $SUDO_USER added to dialout group"
    fi

    # Raspberry Pi specific: Disable Bluetooth to free up PL011 UART
    if [[ -f /boot/firmware/config.txt ]]; then
        print_info "Configuring Raspberry Pi UART (disabling Bluetooth for PL011)..."

        # Disable Bluetooth services to free up hardware UART
        if systemctl is-enabled hciuart &>/dev/null; then
            systemctl disable hciuart
            print_success "Disabled hciuart service"
        else
            print_info "hciuart service not found or already disabled"
        fi

        if systemctl is-enabled bluetooth &>/dev/null; then
            systemctl disable bluetooth
            print_success "Disabled bluetooth service"
        else
            print_info "bluetooth service not found or already disabled"
        fi

        # Add dtoverlay=disable-bt to config.txt
        if ! grep -q "^dtoverlay=disable-bt" /boot/firmware/config.txt; then
            echo "dtoverlay=disable-bt" >> /boot/firmware/config.txt
            print_success "Added dtoverlay=disable-bt to config.txt"
        else
            print_info "dtoverlay=disable-bt already in config.txt"
        fi

        # Enable UART
        if ! grep -q "^enable_uart=1" /boot/firmware/config.txt; then
            echo "enable_uart=1" >> /boot/firmware/config.txt
            print_success "UART enabled in config.txt"
        else
            print_info "UART already enabled in config.txt"
        fi

        # CRITICAL: Remove serial console from cmdline.txt
        # This prevents kernel/getty from using ttyAMA0 for console
        if [[ -f /boot/firmware/cmdline.txt ]]; then
            print_info "Removing serial console from cmdline.txt..."

            # Backup original cmdline.txt
            if [[ ! -f /boot/firmware/cmdline.txt.backup ]]; then
                cp /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt.backup
                print_info "Backed up original cmdline.txt"
            fi

            # Remove console=serial0,115200 and console=ttyAMA0,115200
            if grep -q "console=serial0" /boot/firmware/cmdline.txt || grep -q "console=ttyAMA0" /boot/firmware/cmdline.txt; then
                sed -i 's/console=serial0,[0-9]\+ //g' /boot/firmware/cmdline.txt
                sed -i 's/console=ttyAMA0,[0-9]\+ //g' /boot/firmware/cmdline.txt
                print_success "Removed serial console from cmdline.txt"
                print_info "Original saved as cmdline.txt.backup"
            else
                print_info "Serial console not found in cmdline.txt (already removed)"
            fi
        fi

        # Disable serial-getty service (will be stopped after reboot)
        if systemctl is-enabled serial-getty@ttyAMA0.service &>/dev/null; then
            systemctl disable serial-getty@ttyAMA0.service
            print_success "Disabled serial-getty@ttyAMA0.service"
        else
            print_info "serial-getty@ttyAMA0 not enabled"
        fi

        # Stop serial-getty service if currently running
        if systemctl is-active serial-getty@ttyAMA0.service &>/dev/null; then
            systemctl stop serial-getty@ttyAMA0.service
            print_success "Stopped serial-getty@ttyAMA0.service"
        fi

        print_warning "Bluetooth and serial console disabled to free PL011 UART for flight controller"
        print_info "This ensures reliable serial communication at 115200 baud"
        print_info "Serial console will no longer be available on ttyAMA0"
    fi

    # Set UART permissions
    cat > /etc/udev/rules.d/99-serial.rules << EOF
KERNEL=="ttyAMA[0-9]*", GROUP="dialout", MODE="0660"
KERNEL=="ttyUSB[0-9]*", GROUP="dialout", MODE="0660"
KERNEL=="ttyACM[0-9]*", GROUP="dialout", MODE="0660"
EOF
    udevadm control --reload-rules
    udevadm trigger

    print_success "UART configured successfully"
}

################################################################################
# Network Configuration
################################################################################

configure_network() {
    print_header "Configuring Network Settings"

    local NETWORK_SETUP_SCRIPT="$WORKSPACE_DIR/network_setup_22_04.sh"

    if [[ ! -f "$NETWORK_SETUP_SCRIPT" ]]; then
        print_warning "Network setup script not found: $NETWORK_SETUP_SCRIPT"
        print_info "Skipping network configuration"
        return
    fi

    print_info "Running network setup script..."
    chmod +x "$NETWORK_SETUP_SCRIPT"
    bash "$NETWORK_SETUP_SCRIPT"

    if [[ $? -eq 0 ]]; then
        print_success "Network configuration completed successfully"
    else
        print_error "Network configuration failed"
        print_warning "Continuing with setup (network configuration is optional)"
    fi
}

################################################################################
# Hardware Checks
################################################################################

check_i2c_device() {
    print_header "Checking I2C LiDAR Connection"

    if [[ "$SKIP_HARDWARE_CHECK" = true ]]; then
        print_info "Skipping hardware check (--skip-hardware-check flag)"
        return
    fi

    # Check if I2C bus exists
    if [[ ! -e /dev/i2c-1 ]]; then
        print_warning "I2C bus /dev/i2c-1 not found"
        print_info "This is normal if not running on a Raspberry Pi or I2C hardware"
        return
    fi

    print_info "Scanning I2C bus 1 for devices..."

    if command -v i2cdetect &> /dev/null; then
        # Expected LiDAR address is 0x08 (from config)
        if i2cdetect -y 1 | grep -q "08"; then
            print_success "I2C device detected at address 0x08 (LiDAR)"
        else
            print_warning "No I2C device found at 0x08"
            print_info "LiDAR may not be connected or powered on"

            # Show all detected devices
            echo -e "\nDetected I2C devices:"
            i2cdetect -y 1
        fi
    else
        print_warning "i2cdetect not available, cannot verify I2C device"
    fi
}

check_uart_device() {
    print_header "Checking UART Flight Controller Connection"

    if [[ "$SKIP_HARDWARE_CHECK" = true ]]; then
        print_info "Skipping hardware check (--skip-hardware-check flag)"
        return
    fi

    # Expected UART device is /dev/ttyAMA0 (from config)
    if [[ -e /dev/ttyAMA0 ]]; then
        print_success "UART device /dev/ttyAMA0 found"

        # Check if it's accessible
        if [[ -r /dev/ttyAMA0 ]] && [[ -w /dev/ttyAMA0 ]]; then
            print_success "UART device is readable and writable"
        else
            print_warning "UART device exists but may not have correct permissions"
            print_info "User needs to be in 'dialout' group (requires re-login)"
        fi
    else
        print_warning "UART device /dev/ttyAMA0 not found"
        print_info "Flight controller may not be connected"

        # List available serial devices
        echo -e "\nAvailable serial devices:"
        ls -l /dev/ttyAMA* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No serial devices found"
    fi
}

################################################################################
# ROS2 Workspace Setup
################################################################################

setup_ros2_workspace() {
    print_header "Setting Up ROS2 Workspace"

    cd "$WORKSPACE_DIR"

    # Fix workspace ownership (in case script was run with sudo)
    print_info "Ensuring correct workspace permissions..."
    if [[ -n "$SUDO_USER" ]]; then
        sudo chown -R "$SUDO_USER:$SUDO_USER" "$WORKSPACE_DIR"
        print_success "Workspace ownership set to $SUDO_USER"
    elif [[ "$EUID" -ne 0 ]]; then
        # Running as normal user, ensure ownership
        sudo chown -R "$USER:$USER" "$WORKSPACE_DIR"
        print_success "Workspace ownership set to $USER"
    fi

    # Clean old build artifacts if they exist with wrong permissions
    if [[ -d "log" ]] || [[ -d "build" ]] || [[ -d "install" ]]; then
        print_info "Cleaning old build artifacts..."
        rm -rf log build install
        print_success "Old build artifacts removed"
    fi

    print_info "Installing ROS2 dependencies..."
    source /opt/ros/humble/setup.bash

    # Install dependencies using rosdep
    if [[ -f src/swarm_ai_integration/package.xml ]]; then
        rosdep install --from-paths src --ignore-src -r -y
        print_success "ROS2 dependencies installed"
    else
        print_warning "package.xml not found, skipping rosdep"
    fi

    print_info "Building ROS2 workspace..."
    colcon build --symlink-install

    if [[ $? -eq 0 ]]; then
        print_success "ROS2 workspace built successfully"
    else
        print_error "Failed to build ROS2 workspace"
        exit 1
    fi

    print_info "Setting execution permissions for ROS nodes..."
    chmod +x src/swarm_ai_integration/swarm_ai_integration/*.py
    print_success "Execution permissions set for Python nodes"
}

################################################################################
# Create Required Directories
################################################################################

create_required_directories() {
    print_header "Creating Required Directories"

    cd "$WORKSPACE_DIR"

    # Create flight-logs directory
    if [[ ! -d "flight-logs" ]]; then
        mkdir -p flight-logs
        print_success "Created flight-logs directory"
    else
        print_info "flight-logs directory already exists"
    fi

    # Create model directory
    if [[ ! -d "model" ]]; then
        mkdir -p model
        print_success "Created model directory"
    else
        print_info "model directory already exists"
    fi

    # Set correct ownership if running with sudo
    if [[ -n "$SUDO_USER" ]]; then
        chown -R "$SUDO_USER:$SUDO_USER" flight-logs model
        print_success "Set directory ownership to $SUDO_USER"
    fi
}

################################################################################
# Environment Configuration
################################################################################

configure_environment() {
    print_header "Configuring Environment"

    if [[ -z "$SUDO_USER" ]]; then
        print_warning "Cannot determine user, skipping bashrc setup"
        return
    fi

    USER_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
    BASHRC="$USER_HOME/.bashrc"

    print_info "Adding ROS2 environment to $BASHRC..."

    # Check if ROS2 setup is already sourced
    if ! grep -q "source /opt/ros/humble/setup.bash" "$BASHRC"; then
        cat >> "$BASHRC" << EOF

# ROS2 Humble Environment
source /opt/ros/humble/setup.bash
EOF
        print_success "Added ROS2 Humble to .bashrc"
    else
        print_info "ROS2 Humble already in .bashrc"
    fi

    # Add workspace setup
    if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" "$BASHRC"; then
        cat >> "$BASHRC" << EOF
source $WORKSPACE_DIR/install/setup.bash
EOF
        print_success "Added workspace to .bashrc"
    else
        print_info "Workspace already in .bashrc"
    fi

    # Add ROS domain ID if not set
    if ! grep -q "ROS_DOMAIN_ID" "$BASHRC"; then
        cat >> "$BASHRC" << EOF

# ROS2 Configuration
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
EOF
        print_success "Added ROS2 configuration to .bashrc"
    fi

    print_success "Environment configured"
}

################################################################################
# PM2 Process Manager (Optional)
################################################################################

install_pm2() {
    if [[ "$INSTALL_PM2" != true ]]; then
        print_info "Skipping PM2 installation (--skip-pm2 flag used)"
        return
    fi

    print_header "Installing PM2 Process Manager"

    # Install Node.js if not present
    if ! command -v node &> /dev/null; then
        print_info "Installing Node.js..."
        curl -fsSL https://deb.nodesource.com/setup_18.x | bash -
        apt install -y nodejs
        print_success "Node.js installed"
    else
        print_info "Node.js already installed"
    fi

    # Install PM2
    if ! command -v pm2 &> /dev/null; then
        print_info "Installing PM2..."
        npm install -g pm2
        print_success "PM2 installed"
    else
        print_info "PM2 already installed"
    fi

    # Setup PM2 startup script
    if [[ -n "$SUDO_USER" ]]; then
        sudo -u "$SUDO_USER" pm2 startup systemd -u "$SUDO_USER" --hp "$(getent passwd "$SUDO_USER" | cut -d: -f6)"
        print_success "PM2 startup configured"
    fi
}

################################################################################
# Final Summary
################################################################################

print_summary() {
    print_header "Installation Summary"

    echo -e "${GREEN}✓ Setup completed successfully!${NC}\n"

    # Check if Bluetooth was disabled (Raspberry Pi specific)
    local reboot_required=false
    if [[ -f /boot/firmware/config.txt ]]; then
        if grep -q "^dtoverlay=disable-bt" /boot/firmware/config.txt; then
            reboot_required=true
        fi
    fi

    echo "Next steps:"
    if [[ "$reboot_required" = true ]]; then
        echo "  ${RED}1. REBOOT THE SYSTEM to apply UART/Bluetooth changes${NC}"
        echo "     ${BLUE}sudo reboot${NC}"
        echo "  2. After reboot, verify hardware connections:"
        echo "     ${BLUE}i2cdetect -y 1${NC}  (check LiDAR at 0x08)"
        echo "     ${BLUE}ls -l /dev/ttyAMA0${NC}  (check flight controller UART)"
        echo "  3. Launch the system:"
    else
        echo "  1. Log out and log back in for group changes to take effect"
        echo "  2. Source the environment:"
        echo "     ${BLUE}source /opt/ros/humble/setup.bash${NC}"
        echo "     ${BLUE}source $WORKSPACE_DIR/install/setup.bash${NC}"
        echo "  3. Verify hardware connections:"
        echo "     ${BLUE}i2cdetect -y 1${NC}  (check LiDAR at 0x08)"
        echo "     ${BLUE}ls -l /dev/ttyAMA0${NC}  (check flight controller)"
        echo "  4. Launch the system:"
    fi
    echo "     ${BLUE}./launch.sh${NC}  (if using PM2)"
    echo "     or"
    echo "     ${BLUE}ros2 launch swarm_ai_integration swarm_ai_launch.py${NC}"
    echo ""

    print_info "Configuration files:"
    echo "  - ROS2 parameters: $WORKSPACE_DIR/src/swarm_ai_integration/config/swarm_params.yaml"
    echo "  - Launch script: $WORKSPACE_DIR/launch.sh"
    echo ""

    print_info "Workspace directories:"
    echo "  - Flight logs: $WORKSPACE_DIR/flight-logs"
    echo "  - Model files: $WORKSPACE_DIR/model"
    echo ""

    print_info "Python Virtual Environment:"
    echo "  - Location: ${HOME}/ai_flight_node_env"
    echo "  - Used by: AI Flight Node (ai_flight_node_wrapper.sh)"
    echo "  - Requirements: $WORKSPACE_DIR/requirements.txt"
    echo "  - Packages: numpy, typing-extensions, gymnasium, stable-baselines3, torch"
    echo ""

    if [[ "$INSTALL_PM2" = true ]]; then
        print_info "PM2 Commands:"
        echo "  - ${BLUE}pm2 list${NC}          Show running processes"
        echo "  - ${BLUE}pm2 logs${NC}          View logs"
        echo "  - ${BLUE}pm2 stop all${NC}      Stop all processes"
        echo "  - ${BLUE}pm2 restart all${NC}   Restart all processes"
        echo ""
    fi

    if [[ "$reboot_required" = true ]]; then
        print_warning "╔════════════════════════════════════════════════════════════════════╗"
        print_warning "║  REBOOT REQUIRED!                                                  ║"
        print_warning "║  Bluetooth has been disabled to free the PL011 UART.              ║"
        print_warning "║  You must reboot for these changes to take effect.                ║"
        print_warning "║  Command: sudo reboot                                              ║"
        print_warning "╚════════════════════════════════════════════════════════════════════╝"
    else
        print_warning "IMPORTANT: You must log out and log back in for group changes to take effect!"
    fi
}

################################################################################
# Main Execution
################################################################################

main() {
    print_header "Swarm ROS2 Environment Setup"

    check_root
    check_ubuntu_version

    install_ros2_humble
    install_system_dependencies
    install_python_dependencies

    configure_i2c
    configure_uart

    check_i2c_device
    check_uart_device

    setup_ros2_workspace
    create_required_directories
    configure_environment

    install_pm2
    
    configure_network

    print_summary
}

# Run main function
main "$@"