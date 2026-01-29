<a id="readme-top"></a>

# Scripts folder

Quick reference index for all utility scripts in this folder.

## Overview

This folder contains shell scripts for automating the setup, configuration, and launch of the Langostino drone system. Choose the appropriate scripts based on your Ubuntu version. During the development process, we used two types of Raspberry Pi; 3b+ and 5. We use version 22.04 on the Raspberry Pi 3b+ and version 24.04 on the Pi 5.

## Scripts Index

| Script | Description |
|--------|-------------|
| [**setup_22_04.sh**](setup_22_04.sh) | Full environment setup for Ubuntu 22.04: ROS2 Humble, Python dependencies, hardware config, and PM2 |
| [**setup_24_04.sh**](setup_24_04.sh) | Full environment setup for Ubuntu 24.04: ROS2 Jazzy, Python dependencies, hardware config, and PM2 |
| [**network_setup_22_04.sh**](network_setup_22_04.sh) | Network configuration for Ubuntu 22.04: static IP on eth0, WiFi AP mode with dnsmasq and hostapd |
| [**network_setup_24_04.sh**](network_setup_24_04.sh) | Network configuration for Ubuntu 24.04: static IP on eth0, WiFi AP mode with wpa_supplicant |
| [**verify_setup.sh**](verify_setup.sh) | Verification script to check that all system components are properly installed and configured |
| [**launch.sh**](launch.sh) | Quick launch script to start the ROS2 system using PM2 process manager |

## Usage

### Initial Setup

Run the setup script matching your Ubuntu version:

```bash
# Ubuntu server 22.04
sudo ./setup_22_04.sh

# Ubuntu server 24.04
sudo ./setup_24_04.sh
```

**Options:**
- `--skip-ros` — Skip ROS2 installation (if already installed)
- `--skip-hardware-check` — Skip hardware connectivity checks
- `--skip-pm2` — Skip PM2 installation

### Network Configuration

Configure the drone as a WiFi access point:

```bash
# Ubuntu server 22.04
sudo ./network_setup_22_04.sh

# Ubuntu server 24.04
sudo ./network_setup_24_04.sh
```

### Verify Installation

Check that everything is properly configured:

```bash
./verify_setup.sh
```

### Launch the System

Start the ROS2 nodes:

```bash
./launch.sh
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>
