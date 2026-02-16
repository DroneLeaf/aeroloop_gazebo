#!/bin/bash
# Installation script for Gazebo Harmonic on Ubuntu
# This script installs Gazebo Harmonic and all required development libraries
# for building the BetaflightPlugin

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Gazebo Harmonic Installation Script${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo -e "${RED}Error: Please do not run this script as root${NC}"
    echo "Run it as a regular user. It will prompt for sudo when needed."
    exit 1
fi

# Detect OS
echo -e "${YELLOW}[1/6] Detecting operating system...${NC}"
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS=$ID
    VER=$VERSION_ID
    CODENAME=$VERSION_CODENAME
    echo "Detected: $PRETTY_NAME (codename: $CODENAME)"
else
    echo -e "${RED}Error: Cannot detect OS. /etc/os-release not found${NC}"
    exit 1
fi

# Check if Ubuntu
if [ "$OS" != "ubuntu" ]; then
    echo -e "${RED}Error: This script is designed for Ubuntu.${NC}"
    echo "Detected OS: $OS"
    exit 1
fi

# Check Ubuntu version (Gazebo Harmonic officially supports Ubuntu 22.04+)
echo -e "${YELLOW}[2/6] Verifying Ubuntu version...${NC}"
if [ "$VER" != "24.04" ] && [ "$VER" != "22.04" ] && [ "$VER" != "23.04" ] && [ "$VER" != "23.10" ]; then
    echo -e "${YELLOW}Warning: Gazebo Harmonic is officially supported on Ubuntu 22.04 and later${NC}"
    echo "Your version: Ubuntu $VER"
    read -p "Do you want to continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Add OSRF repository
echo -e "${YELLOW}[3/6] Adding OSRF repository...${NC}"
if [ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then
    echo "Downloading OSRF GPG key..."
    sudo wget -q https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "OSRF GPG key added successfully"
else
    echo "OSRF GPG key already exists"
fi

if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
    echo "Adding Gazebo stable repository..."
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    echo "Repository added successfully"
else
    echo "Gazebo repository already exists"
fi

# Update package list
echo -e "${YELLOW}[4/6] Updating package list...${NC}"
sudo apt update

# Install Gazebo Harmonic
echo -e "${YELLOW}[5/6] Installing Gazebo Harmonic and development libraries...${NC}"
echo "This may take several minutes..."
echo ""

# Determine SDFormat version based on Ubuntu version
if [ "$VER" = "24.04" ]; then
    SDFORMAT_PKG="libsdformat14-dev"
elif [ "$VER" = "22.04" ]; then
    SDFORMAT_PKG="libsdformat13-dev"
else
    # Try to detect available version
    if apt-cache search libsdformat14-dev | grep -q libsdformat14-dev; then
        SDFORMAT_PKG="libsdformat14-dev"
    else
        SDFORMAT_PKG="libsdformat13-dev"
    fi
fi

echo "Using SDFormat package: $SDFORMAT_PKG"

PACKAGES=(
    "gz-harmonic"
    "libgz-sim8-dev"
    "libgz-plugin2-dev"
    "libgz-math7-dev"
    "libgz-common5-dev"
    "$SDFORMAT_PKG"
)

for package in "${PACKAGES[@]}"; do
    if dpkg -l | grep -q "^ii  $package "; then
        echo -e "${GREEN}✓${NC} $package already installed"
    else
        echo "Installing $package..."
        sudo apt install -y "$package"
    fi
done

# Verify installation
echo -e "${YELLOW}[6/6] Verifying installation...${NC}"
echo ""

# Check if gz command is available
if command -v gz &> /dev/null; then
    echo -e "${GREEN}✓${NC} gz command found"
    GZ_VERSION=$(gz sim --version 2>&1 | head -1 || echo "unknown")
    echo "  Version: $GZ_VERSION"
else
    echo -e "${RED}✗${NC} gz command not found"
    echo "  You may need to restart your shell or run: source ~/.bashrc"
fi

# Check for development libraries
echo ""
echo "Checking development libraries:"

check_header() {
    local header=$1
    local package=$2
    if [ -f "$header" ]; then
        echo -e "${GREEN}✓${NC} $package"
    else
        echo -e "${RED}✗${NC} $package (missing: $header)"
    fi
}

check_header "/usr/include/gz/sim8/gz/sim/System.hh" "gz-sim8-dev"
check_header "/usr/include/gz/plugin2/gz/plugin/Register.hh" "gz-plugin2-dev"
check_header "/usr/include/gz/math7/gz/math/PID.hh" "gz-math7-dev"

# Check for pkg-config
echo ""
if pkg-config --exists gz-sim8; then
    echo -e "${GREEN}✓${NC} gz-sim8 pkg-config found"
else
    echo -e "${YELLOW}!${NC} gz-sim8 pkg-config not found (may still work)"
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Installation Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Next steps:"
echo "1. Build the BetaflightPlugin:"
echo "   cd plugins"
echo "   mkdir -p build && cd build"
echo "   cmake .."
echo "   make"
echo ""
echo "2. Test Gazebo Harmonic:"
echo "   gz sim -v 4"
echo ""
echo -e "${YELLOW}Note: You may need to restart your terminal for all changes to take effect.${NC}"
