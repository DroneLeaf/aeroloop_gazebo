# Gazebo Harmonic Migration Guide

This repository has been migrated from Gazebo Classic (Gazebo 8) to Gazebo Harmonic (gz-sim8).

## What Changed

### Plugin Architecture
- **Old:** `gazebo::ModelPlugin` with `Load()` and `OnUpdate()` callbacks
- **New:** `gz::sim::System` with `ISystemConfigure` and `ISystemPreUpdate` interfaces
- **Impact:** Plugin uses Entity-Component-System (ECS) architecture for better performance

### API Changes
- Headers: `gazebo/*` → `gz/sim/*`, `ignition/*` → `gz/math/*`
- Namespace: `gazebo::` → `gz::sim::systems::`
- Physics: Direct object pointers → Component queries
- Time: `gazebo::common::Time` → `std::chrono::steady_clock::duration`

### Environment Variables
- `GAZEBO_MODEL_PATH` → `SDF_PATH`
- `GAZEBO_RESOURCE_PATH` → `GZ_SIM_RESOURCE_PATH`
- `GAZEBO_PLUGIN_PATH` → `GZ_SIM_SYSTEM_PLUGIN_PATH`

### Commands
- `gazebo` / `gzserver` → `gz sim`
- `gzclient` → `gz sim` (GUI is default)

## What Stayed the Same

✅ **UDP Protocol (ports 9002/9003)** - Fully compatible with Betaflight SITL
✅ **Packet structures** - ServoPacket and fdmPacket unchanged
✅ **Configuration parameters** - SDF plugin configuration compatible
✅ **Motor mappings** - No changes to rotor configuration

## Installation

### Step 1: Install Gazebo Harmonic

Run the provided installation script from the repository root:

```bash
./install_gazebo_harmonic.sh
```

This will:
- Add the OSRF repository
- Install Gazebo Harmonic
- Install all required development libraries:
  - `gz-harmonic`
  - `libgz-sim8-dev`
  - `libgz-plugin2-dev`
  - `libgz-math7-dev`
  - `libgz-common5-dev`
  - `libsdformat13-dev`

### Step 2: Build the Plugin

After installation, build the BetaflightPlugin:

```bash
./build_plugin.sh
```

Or manually:

```bash
cd plugins
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### Step 3: Verify Installation

Check that the plugin was built successfully:

```bash
ls -lh plugins/build/libBetaflightPlugin.so
```

Test Gazebo Harmonic:

```bash
gz sim --version
```

## Running the Simulation

### Using the Python Launcher (Recommended)

```bash
python3 ~/betaflight-docker/betaloop/start.py
```

The launcher has been updated to:
- Use `gz sim` command instead of `gazebo`/`gzserver`
- Set correct environment variables for Harmonic
- Support both GUI and headless modes

### Manual Launch

```bash
# Set up environment (from repository root)
export SDF_PATH=${PWD}/models:$SDF_PATH
export GZ_SIM_RESOURCE_PATH=${PWD}/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=${PWD}/plugins/build:$GZ_SIM_SYSTEM_PLUGIN_PATH

# Run simulation (headless)
gz sim -s -r -v 4 ${PWD}/worlds/betaloop_iris_arducopter_demo.world

# Run simulation (with GUI)
gz sim -r -v 4 ${PWD}/worlds/betaloop_iris_arducopter_demo.world
```

## Troubleshooting

### Build Errors

**Error: `gz-sim8` not found**
```bash
# Install development libraries
sudo apt install libgz-sim8-dev libgz-plugin2-dev libgz-math7-dev
```

**Error: CMake version too old**
```bash
# Update CMake (requires 3.10.2+)
sudo apt install cmake
```

### Runtime Errors

**Error: Plugin not loaded**
- Check that `GZ_SIM_SYSTEM_PLUGIN_PATH` includes `plugins/build`
- Verify plugin exists: `ls plugins/build/libBetaflightPlugin.so`

**Error: IMU sensor not found**
- Verify IMU sensor name in SDF matches plugin configuration
- Check sensor scoped name format: `model_name::link_name::sensor_name`

**Error: Joints not responding**
- Ensure joint names in SDF match plugin configuration
- Check joint entities are being found (look for error messages on startup)

**Error: Betaflight not connecting**
- UDP ports 9002/9003 should work unchanged
- Check firewall settings
- Verify Betaflight SITL is running and configured for correct ports

## Testing Checklist

- [ ] Gazebo Harmonic installed and `gz sim --version` works
- [ ] Plugin compiles without errors
- [ ] Plugin loads in Gazebo (check verbose output)
- [ ] Model spawns correctly
- [ ] IMU sensor data is being read
- [ ] Motors respond to commands
- [ ] Betaflight SITL connects via UDP
- [ ] Motor commands are received and applied
- [ ] State data is sent back to Betaflight

## Known Issues

1. **First Launch Delay**: Gazebo may take longer to start on first run while downloading resources
2. **Component Access**: Some components may need additional EnableComponent calls for optimization
3. **Sensor Lookup**: IMU sensor name parsing is simplified - may need adjustment for complex model hierarchies

## References

- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [Migration Guide](https://gazebosim.org/docs/harmonic/migrating_gazebo_classic)
- [gz-sim API](https://gazebosim.org/api/sim/8/)
- [ECS Architecture](https://gazebosim.org/docs/harmonic/ecs)

## Support

For issues specific to the Betaloop port:
1. Check verbose output: `gz sim -v 4`
2. Enable plugin debug: Add `gzdbg` statements to plugin code
3. Verify component availability in ECM
4. Check Betaflight SITL logs for connection status
