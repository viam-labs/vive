# VR Teleop

Reads HTC Vive controller poses and button states from SteamVR/OpenVR and sends arm/gripper commands to a Viam robot at ~90 Hz. No frontend or WebSocket required.

## Hardware

- **Watchman VR Dongles** — USB receivers for the controllers (one per controller)
- **SteamVR Base Stations (Vive 2.0)** — at least two for full room-scale tracking
- **Vive 2.0 Controllers** — one per arm

### Controls

- **Trigger** — proportional gripper control. Gripper position mirrors trigger pull.
- **Grip** — deadman's switch. Arm moves only while held.
- **Menu** — returns arm to previous start pose. Each press steps further back.
- **Trackpad up** — recalibrate forward direction to current controller heading.
- **Trackpad down** — toggle absolute/relative rotation tracking.

## Software Requirements

- Go 1.21+
- SteamVR running on Linux
- A Viam robot with arm and (optionally) gripper components

## Setup

Download the OpenVR SDK and tidy Go modules:

```sh
make deps
```

## Build

```sh
make build
```

## Run

```sh
make run RUN_ARGS="--address <machine-address> --key-id <key-id> --key <key>"
```

Or run the binary directly:

```sh
./vr-teleop \
  --address <machine-address> \
  --key-id <key-id> \
  --key <key>
```

### Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--address` | (required) | Viam machine address |
| `--key-id` | | Viam API key ID |
| `--key` | | Viam API key |
| `--hz` | `90` | Polling rate in Hz |
| `--left-arm` | `right-arm` | Arm controlled by the left controller |
| `--right-arm` | `left-arm` | Arm controlled by the right controller |
| `--left-gripper` | `right-gripper` | Gripper controlled by the left controller (empty to disable) |
| `--right-gripper` | `left-gripper` | Gripper controlled by the right controller (empty to disable) |
| `--scale` | `1.0` | Position scale factor (0.1–3.0) |
| `--rotation` | `true` | Enable orientation tracking |

## Local dev overrides

Copy `Makefile.local.example` or create `Makefile.local` with machine-specific credentials and arm mappings. This file is gitignored.
