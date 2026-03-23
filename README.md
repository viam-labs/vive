# Vive Module

HTC Vive VR controller tracking and arm teleop for Viam robots. Uses [libsurvive](https://github.com/cntools/libsurvive) — no SteamVR required.

## Hardware

- HTC Vive 2.0 Controllers (one per arm)
- Watchman VR USB Dongles (one per controller)
- Lighthouse Base Stations (v1.0 or v2.0, at least two)

## Configure vive-controller

```jsonc
{
  // optional — if omitted, controllers are assigned in discovery order
  "serial_number": "LHR-FEC592B1",
  // optional — typically auto-discovered (e.g. "WM0")
  "device_name": ""
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
  // default: 90 — polling rate in Hz
  "hz": 90,
  // required — at least one hand
  "hands": [
    {
      // required — hand identifier
      "name": "left",
      // required — name of the vive-controller component
      "controller": "left-vive",
      // required — name of the arm component to control
      "arm": "right-arm",
      // optional — gripper component, trigger controls proportional grip
      "gripper": "right-gripper",
      // default: 1.0 — position multiplier (2.0 = arm moves 2x your hand)
      "scale": 1.0,
      // default: true — enable orientation tracking
      "rotation_enabled": true,
      // default: 2.0 — position dead-zone in mm, suppresses jitter
      "pos_deadzone_mm": 2,
      // default: 2.0 — rotation dead-zone in degrees
      "rot_deadzone_deg": 2,
      // default: 0.2 — EMA smoothing alpha (0-1), lower = smoother
      "smooth_alpha": 0.2
    },
    {
      "name": "right",
      "controller": "right-vive",
      "arm": "left-arm",
      "gripper": "left-gripper"
    }
  ],
  // default: executable dir — directory for calibration and controller map files
  "calibration_dir": "/data"
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

**Recalibrate base stations:**

```json
{"recalibrate": true}
```

Clears cached base station positions and restarts tracking. Use this after a base station is bumped or moved. Blocks until base stations are re-solved (up to 30s). After it returns, use `calibrate` or trackpad-up to set the forward direction.

**Get current calibration:**

```json
{"get_calibration": true}
```

Returns:

```json
{"calibrated": true, "yaw_deg": 175.9}
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

**Set task description (for VLA training data):**

```json
{"set_task": "pick up the red block"}
```

Sets the task string used to tag captured data. Requires `capture_control_sensor` to be configured. The task persists until changed — call this from the Viam app control tab before each demonstration.

**Start dongle pairing:**

```json
{"pair_mode": true}
```

Starts the Watchman dongle pairing flow. Power on each controller one at a time during pairing.

### Troubleshooting

**Base station bumped / tracking is wrong:**
Run the `recalibrate` DoCommand to clear cached base station geometry and restart tracking. The command blocks until base stations are re-solved. Then recalibrate the forward direction with trackpad-up or `{"calibrate": true}`.

## Configure capture-control

The capture-control sensor enables automatic data capture during teleop sessions. When the grip is pressed (teleop starts), the sensor instructs the data manager to begin capturing arm pose and camera data. When the grip is released, capture stops. All captured data is tagged with a session timestamp and an optional task description for VLA training.

```jsonc
{
  // required — arm component to capture EndPosition from
  "arm_name": "right-arm",
  // optional — cameras to capture GetImages from
  "camera_names": ["wrist-cam", "overhead-cam"],
  // default: 10.0 — capture rate in Hz
  "capture_frequency_hz": 10.0
}
```

Wire it into the teleop service by adding `capture_control_sensor` to the teleop config:

```jsonc
{
  "hands": [...],
  "capture_control_sensor": "capture-ctrl"
}
```

### Data Manager Setup

The target resources (arm, cameras) must have data capture configured — even if initially disabled with `capture_frequency_hz: 0`. The capture-control sensor overrides the frequency at runtime.

**Arm:**
```json
{
  "name": "right-arm",
  "service_configs": [
    {
      "type": "data_manager",
      "attributes": {
        "capture_methods": [
          {"method": "EndPosition", "capture_frequency_hz": 0}
        ]
      }
    }
  ]
}
```

**Camera (repeat for each):**
```json
{
  "name": "wrist-cam",
  "service_configs": [
    {
      "type": "data_manager",
      "attributes": {
        "capture_methods": [
          {"method": "GetImages", "capture_frequency_hz": 0}
        ]
      }
    }
  ]
}
```

**Data manager service:**
```json
{
  "name": "data_manager-1",
  "api": "rdk:service:data_manager",
  "model": "rdk:builtin:builtin",
  "attributes": {
    "capture_control_sensor": {
      "name": "capture-ctrl",
      "key": "overrides"
    },
    "sync_interval_mins": 0.1,
    "sync_disabled": false
  }
}
```

### Workflow

1. Set the task description via DoCommand on the teleop service: `{"set_task": "pick up the red block"}`
2. Press grip — teleop starts and data capture begins automatically. All data is tagged with `session:<timestamp>` and `cmd:<task>`.
3. Release grip — teleop stops and data capture stops.
4. Change the task string with another `set_task` call before the next demonstration.

### DoCommand

**Set task (via sensor directly):**

```json
{"set_task": "pick up the red block"}
```

**Get status:**

```json
{"status": true}
```

Returns:

```json
{"capturing": false, "capture_frequency_hz": 10, "tags": null, "task": "pick up the red block"}
```

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
