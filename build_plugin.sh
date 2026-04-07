#!/bin/bash
# Build script for BetaflightPlugin
# Run this after installing Gazebo Harmonic

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}BetaflightPlugin Build Script${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PLUGIN_DIR="$SCRIPT_DIR/plugins"
BUILD_DIR="$PLUGIN_DIR/build"

# Check if Gazebo Harmonic is installed
echo -e "${YELLOW}[1/4] Checking prerequisites...${NC}"
if ! command -v gz &> /dev/null; then
    echo -e "${RED}Error: gz command not found${NC}"
    echo "Please install Gazebo Harmonic first:"
    echo "  ./install_gazebo_harmonic.sh"
    exit 1
fi
echo -e "${GREEN}✓${NC} gz command found"

if ! pkg-config --exists gz-sim8 2>/dev/null; then
    echo -e "${YELLOW}Warning: gz-sim8 pkg-config not found${NC}"
    echo "Build may fail. Make sure libgz-sim8-dev is installed."
fi

# Check if CMakeLists.txt exists
if [ ! -f "$PLUGIN_DIR/CMakeLists.txt" ]; then
    echo -e "${RED}Error: CMakeLists.txt not found in $PLUGIN_DIR${NC}"
    exit 1
fi

# Create build directory
echo -e "${YELLOW}[2/4] Setting up build directory...${NC}"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
echo "Build directory: $BUILD_DIR"

# Run CMake
echo -e "${YELLOW}[3/4] Configuring with CMake...${NC}"
if cmake ..; then
    echo -e "${GREEN}✓${NC} CMake configuration successful"
else
    echo -e "${RED}✗${NC} CMake configuration failed"
    exit 1
fi

# Build
echo -e "${YELLOW}[4/4] Building plugin...${NC}"
if make -j$(nproc); then
    echo -e "${GREEN}✓${NC} Build successful"
else
    echo -e "${RED}✗${NC} Build failed"
    exit 1
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Build Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Check if plugin library was created
if [ -f "$BUILD_DIR/libBetaflightPlugin.so" ]; then
    echo -e "${GREEN}✓${NC} Plugin library created:"
    ls -lh "$BUILD_DIR/libBetaflightPlugin.so"
    echo ""
    echo "Plugin location: $BUILD_DIR/libBetaflightPlugin.so"
else
    echo -e "${RED}✗${NC} Plugin library not found"
    exit 1
fi

echo ""
echo "Next steps:"
echo "1. Test the plugin with Gazebo:"
echo "   python3 ~/betaflight-docker/betaloop/start.py"
echo ""
echo "2. Or test Gazebo manually:"
echo "   export GZ_SIM_SYSTEM_PLUGIN_PATH=$BUILD_DIR:\$GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "   gz sim -r -v 4 <world_file>"
echo ""
