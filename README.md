# Vive Module

HTC Vive VR controller tracking and arm teleop for Viam robots. Uses [libsurvive](https://github.com/cntools/libsurvive) — no SteamVR required.

## Hardware

- HTC Vive 2.0 Controllers (one per arm)
- Watchman VR USB Dongles (one per controller)
- Lighthouse Base Stations (v1.0 or v2.0, at least two)

## Configure vive-controller

```json
{
  "serial_number": "LHR-FEC592B1",
  "device_name": ""
}
```

| Attribute | Type | Description |
|-----------|------|-------------|
| `serial_number` | string | Serial number of the physical controller (stable across restarts). Optional — if omitted, controllers are assigned in discovery order. |
| `device_name` | string | Libsurvive device name (e.g. `"WM0"`). Optional — typically auto-discovered. |

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

```json
{
  "hz": 90,
  "hands": [
    {
      "name": "left",
      "controller": "left-vive",
      "arm": "right-arm",
      "gripper": "right-gripper",
      "scale": 1.0,
      "rotation_enabled": true,
      "pos_deadzone_mm": 2,
      "rot_deadzone_deg": 2,
      "smooth_alpha": 0.2
    },
    {
      "name": "right",
      "controller": "right-vive",
      "arm": "left-arm",
      "gripper": "left-gripper"
    }
  ],
  "calibration_dir": "/data"
}
```

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `hz` | int | `90` | Polling rate in Hz. |
| `hands` | array | *required* | List of hand configurations (at least one). |
| `calibration_dir` | string | executable dir | Directory for calibration and controller map files. |

**Hand attributes:**

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | string | *required* | Hand identifier (e.g. `"left"`, `"right"`). |
| `controller` | string | *required* | Name of the `vive-controller` component. |
| `arm` | string | *required* | Name of the arm component to control. |
| `gripper` | string | | Name of the gripper component. Trigger controls proportional grip. |
| `scale` | float | `1.0` | Position multiplier. `2.0` = arm moves 2x your hand movement. |
| `rotation_enabled` | bool | `true` | Enable orientation tracking. |
| `pos_deadzone_mm` | float | `2.0` | Position dead-zone in mm. Suppresses jitter. |
| `rot_deadzone_deg` | float | `2.0` | Rotation dead-zone in degrees. |
| `smooth_alpha` | float | `0.2` | EMA smoothing alpha (0-1). Lower = smoother, higher = more responsive. |

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
