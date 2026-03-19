# Vive Module

HTC Vive VR controller tracking and arm teleop for Viam robots. Uses [libsurvive](https://github.com/cntools/libsurvive) — no SteamVR required.

## Hardware

- HTC Vive 2.0 Controllers (one per arm)
- Watchman VR USB Dongles (one per controller)
- Lighthouse Base Stations (v1.0 or v2.0, at least two)

## Configure vive-controller

```jsonc
{
  "name": "left-vive",
  "api": "rdk:component:input_controller",
  "model": "viam:vive:vive-controller",
  "attributes": {
    // Serial number of the physical controller (stable across restarts).
    // Optional — if omitted, controllers are assigned in discovery order.
    "serial_number": "LHR-FEC592B1"
  }
}
```

### Controls

| Control | Type | Description |
|---------|------|-------------|
| `AbsoluteX` | axis | Trackpad X position |
| `AbsoluteY` | axis | Trackpad Y position |
| `AbsoluteZ` | axis | Trigger (0–1) |
| `ButtonSouth` | button | Trackpad press |
| `ButtonLT` | button | Grip |
| `ButtonMenu` | button | Menu |

### DoCommand

**Get 6DOF pose:**

```json
{"get_pose": true}
```

Returns:

```json
{"valid": true, "position": [x, y, z], "matrix": [[16 floats]]}
```

**Trigger haptic feedback:**

```json
{"haptic": {"amplitude": 0.5, "duration_ms": 100}}
```

**Get controller status:**

```json
{"status": true}
```

Returns:

```json
{"device_name": "WM0", "serial": "LHR-FEC592B1", "connected": true, "pose_valid": true}
```

## Configure teleop service

```jsonc
{
  "name": "teleop",
  "api": "rdk:service:generic",
  "model": "viam:vive:teleop",
  "attributes": {
    // Polling rate in Hz (default: 90)
    "hz": 90,
    "hands": [
      {
        "name": "left",
        // Name of the vive-controller component
        "controller": "left-vive",
        // Name of the arm component to control
        "arm": "right-arm",
        // Name of the gripper component (optional)
        "gripper": "right-gripper",
        // Position multiplier (default: 1.0). 2.0 = arm moves 2x your hand movement.
        "scale": 1.0,
        // Enable orientation tracking (default: true)
        "rotation_enabled": true,
        // Position dead-zone in mm (default: 0.5). Suppresses jitter.
        "pos_deadzone_mm": 0.5,
        // Rotation dead-zone in degrees (default: 1.0)
        "rot_deadzone_deg": 1.0,
        // EMA smoothing alpha, 0–1 (default: 0.5). Lower = smoother, higher = more responsive.
        "smooth_alpha": 0.5
      },
      {
        "name": "right",
        "controller": "right-vive",
        "arm": "left-arm",
        "gripper": "left-gripper"
      }
    ],
    // Directory for calibration/controller map files (default: executable directory)
    "calibration_dir": "/data"
  }
}
```

### VR Controls

- **Grip** — deadman's switch. Arm moves only while held.
- **Trigger** — proportional gripper control. Mirrors trigger pull.
- **Menu** — returns arm to previous start pose.
- **Trackpad up** — recalibrate forward direction to current controller heading.
- **Trackpad down** — toggle absolute/relative rotation tracking.

### DoCommand

**Calibrate forward direction:**

```json
{"calibrate": true}
```

**Get teleop status:**

```json
{"status": true}
```

Returns:

```json
{"hands": [{"name": "left", "controlling": false, "teleop_active": false}]}
```

**Toggle rotation mode:**

```json
{"toggle_rotation_mode": true}
```

Returns:

```json
{"rotation_mode": "absolute"}
```

**List tracked controllers:**

```json
{"list_controllers": true}
```

Returns all libsurvive tracked objects with serial numbers and assignment status.

**Assign controller to hand:**

```json
{"assign": {"serial": "LHR-FEC592B1", "hand": "left"}}
```

Manually assigns a controller by serial number. Persists to `controller_map.json`.

**Start dongle pairing:**

```json
{"pair_mode": true}
```

Starts the Watchman dongle pairing flow. Power on each controller one at a time during pairing.

## Getting Started

### 1. Install dependencies

```sh
make setup
```

### 2. Build

```sh
make build
```

Builds libsurvive from source (first time only) and compiles both binaries.

### 3. Pair controllers

Plug in Watchman dongles, then either:
- Use the `pair_mode` DoCommand from the Viam app, or
- Run `make pair` from the command line

### 4. Configure

Add the module, two `vive-controller` components, and one `teleop` service to your robot config (see examples above).

### CLI Mode

For development/testing, you can run the standalone CLI that connects to a remote robot:

```sh
cp .env.example .env   # fill in credentials
make dev
```
