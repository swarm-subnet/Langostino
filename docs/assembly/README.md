# Build from scratch — Assembly guide 🦐

This guide is your “flight plan” to go from parts → build → first safe flight.

> ⚠️ **Safety first:** Langostino is a real flying machine. Follow local regulations, test in safe environments, and use prop guards + failsafes.

---

## Quick links

- ✅ Parts list (BOM): [`./BOM.md`](./BOM.md)
- 🧰 Raspberry Pi setup (Ubuntu 22.04 + ROS2 Humble): [`../SETUP_GUIDE.md#quick-setup`](../SETUP_GUIDE.md#quick-setup)
- 🧭 INAV configuration script (copy/paste): [`../INAV_GUIDE.md#complete-configuration-script-inav-cli-commands`](../INAV_GUIDE.md#complete-configuration-script-inav-cli-commands)
- 🛰️ INAV MSP / RC override params: [`../INAV_GUIDE.md#msp-configuration`](../INAV_GUIDE.md#msp-configuration)
- 🧯 Troubleshooting: [`../TROUBLESHOOTING_GUIDE.md`](../TROUBLESHOOTING_GUIDE.md)
- 🤝 Contributing: [`../../CONTRIBUTING.md`](../../CONTRIBUTING.md)
- 💬 Discord: https://discord.com/invite/bittensor


---

## What you’re building (plain words)

Langostino is a drone that:

- **senses** the world (dual LiDAR + GPS + IMU),
- **decides locally** on a Raspberry Pi (ROS2 + AI nodes),
- and **acts** through a flight controller running **INAV** (navigation/autonomy firmware).

---

## Your flight plan (zero → first flight)

1. **Get the parts** using the BOM
2. **Assemble the airframe** (frame + motors + FC/ESC stack)
3. **Add sensors + mounts** (dual LiDAR, GPS, receiver, buzzer, regulator)
4. **Wire power + data** (clean 5V, UART, I²C, optional Ethernet debug)
5. **Install software on the Pi** (setup + verify + launch)
6. **Configure INAV + safety checks**, then **manual flight first**, autonomy later

---

## Step 0 — Before you start

### Tools (typical)

- Hex drivers / screw set (FPV frame hardware)
- Soldering iron + solder + heatshrink (if your build requires it)
- Zip ties / cable sleeves (strain relief matters)
- Multimeter (recommended)
- Laptop with INAV Configurator installed

### Safety basics

- **Props off** for bench tests and when testing motors in configurators
- Use a **safe open area** for first outdoor tests
- Configure a **manual override** (radio receiver) before autonomy tests

---

## Step 1 — Parts (BOM)

Open: [`./BOM.md`](./BOM.md)

The BOM is your shopping list. It should include:

- Frame + hardware
- FC/ESC stack
- Motors + propellers
- Raspberry Pi + storage
- Dual LiDAR (front + down) + mounts
- GPS module
- 5V regulator + filtering capacitor
- Receiver (manual override) + buzzer

---

## Step 2 — The skeleton (frame) + propulsion

### 2.1 Build the frame

- Assemble the carbon frame (arms, plates, standoffs)
- Keep it **rigid**: vibrations = noisy data for sensors/AI
- Leave room for: FC stack, wiring, Pi, LiDAR mounts

### 2.2 Mount motors + props (props last)

- Mount motors securely (use thread lock if needed)
- **Do not install propellers yet** — leave them off until the end

### 2.3 Install the FC + ESC stack

- Mount the stack with vibration-damping hardware if available
- Ensure access to:
  - USB port (flashing/config)
  - UART pads/ports (GPS/MSP/etc.)
  - power leads (battery input)

> Why INAV: unlike Betaflight (manual-first), **INAV** is built for navigation/autonomy and supports MSP-based control paths.

---

## Step 3 — Sensors + “field” essentials

### 3.1 Dual LiDAR (front + downward)

- Install LiDAR front-facing + downward-facing
- Use stable mounts (3D-printed mounts help reduce vibration + keep alignment)
- Keep cables tidy and strain-relieved

### 3.2 GPS

- Mount the GPS with good sky visibility
- Keep it away from noisy power lines if possible

### 3.3 Receiver (manual override)

- Install a receiver so you can always take back control
- Confirm your “manual modes” plan before autonomy (ANGLE/manual/etc.)

### 3.4 Buzzer (highly recommended)

- Install an autonomous buzzer connected to the FC
- Purpose: finding the drone after unexpected landings/crashes

---

## Step 4 — Wiring (power + data)

This is where “circuits meet code”.

### 4.1 Clean power (very important)

- Battery feeds ESC → motors
- A regulator steps down high voltage to **clean 5V** for:
  - Raspberry Pi
  - LiDAR sensors
- Add a **filtering capacitor** on the power line:
  - reduces electrical noise from motors
  - helps protect electronics from voltage spikes

> If power is noisy, sensors can drift and the AI can “react” to bad data.

### 4.2 Data buses: UART + I²C

#### UART (direct “hotline”)

Used for FC ↔ Raspberry Pi (telemetry + commands).

- On the Pi, stable device paths often look like `/dev/ttyAMA0` / `/dev/ttyAMA1`
- Keep UART wiring short/clean where possible

#### I²C (shared “group chat”)

Used for multiple sensors on the same bus.

- Both LiDAR modules communicate via I²C
- Each has its own address so they don’t collide

### 4.3 Optional: Ethernet debug for field testing

- Ethernet can be more reliable than Wi-Fi outdoors
- Lets you read ROS2 logs and push quick updates between flights

---

## Step 5 — Raspberry Pi software (Ubuntu + ROS2 + Swarm stack)

Follow the quick setup:
[`../SETUP_GUIDE.md#quick-setup`](../SETUP_GUIDE.md#quick-setup)

In most cases you’ll run:

```bash
sudo ./setup.sh
./verify_setup.sh
./launch.sh
```

If you hit issues, go to:
[`../TROUBLESHOOTING_GUIDE.md`](../TROUBLESHOOTING_GUIDE.md)

---

## Step 6 — INAV setup (firmware + parameters + MSP path)

### 6.1 Flash INAV (if not already)

- Use INAV Configurator
- Flash a supported INAV build for your flight controller
- Reboot and confirm the FC connects properly

### 6.2 Apply baseline parameters (copy/paste script)

Use:
[`../INAV_GUIDE.md#complete-configuration-script-inav-cli-commands`](../INAV_GUIDE.md#complete-configuration-script-inav-cli-commands)

### 6.3 MSP / RC override

From [`../INAV_GUIDE.md#msp-configuration`](../INAV_GUIDE.md#msp-configuration):

- `msp_override_channels` defines which RC channels can be overridden via MSP.

**Build-guide checklist (repo-confirmed + typical practice):**

- ✅ Configure your INAV parameters (script above)
- ✅ Set `msp_override_channels` as documented (e.g. `127` or `255` depending on your channel plan)
- ✅ Ensure you have a working manual override (radio receiver) before autonomy tests

> Note: enabling MSP/override end-to-end often involves both **parameters** (like `msp_override_channels`) and **port configuration** in INAV Configurator. Port specifics vary by FC and wiring.

---

## How the autonomy stack works (ROS2 → AI → flight control)

Langostino runs a modular ROS2 graph. Hardware signals become ROS2 topics (virtual “wires”), then the AI outputs actions that are translated into flight commands.

### Core loop

1. **Sense** → LiDAR + GPS/IMU telemetry
2. **Observe** → nodes build a fixed-size observation vector (**131 values**)
3. **Decide** → `ai_flight_node` outputs velocity targets
4. **Translate** → `fc_adapter_node` converts targets to INAV-friendly commands
5. **Actuate** → `fc_comms_node` sends commands to the FC
6. **Safety** → `safety_monitor_node` watches anomalies and triggers safe behavior

### Nodes overview (prototype series)

- `lidar_reader_node` — reads front + down LiDAR over I²C
- `lidar_adapter_node` — normalizes raw readings for the AI
- `ai_flight_node` — takes 131-value observation vector, outputs velocity targets
- `fc_adapter_node` — converts AI actions to INAV flight commands
- `fc_comms_node` — handles serial comms between Pi and FC
- `safety_monitor_node` — detects anomalies and triggers emergency behaviors

### Hardware → Node → Topic (conceptual map)

_(Exact topic names may differ per version; this is the architecture pattern.)_

| Hardware / Line      | ROS2 Node           | Topic (example)                 | Purpose                               |
| -------------------- | ------------------- | ------------------------------- | ------------------------------------- |
| LiDAR (I²C)          | `lidar_reader_node` | `/lidar_distance`               | Obstacle + altitude distance readings |
| GPS/telemetry via FC | `fc_comms_node`     | `/fc/gps_fix`                   | Position + velocity                   |
| FC ↔ Pi (UART)       | `fc_adapter_node`   | `/fc/msp_command`               | Turns AI actions into flight commands |
| AI ↔ ROS             | `ai_flight_node`    | `/ai/observation`, `/ai/action` | Observation vector → control outputs  |

---

## First flights (do this in order)

### Bench test (props off)

- Power on, confirm FC boots normally
- Confirm sensors are detected (as per setup docs)
- Run [`../../scripts/verify_setup.sh`](../../scripts/verify_setup.sh)
- Confirm launch is stable and logs are clean

### Manual flight first

Before autonomy:

- Confirm arming checks pass
- Confirm vibrations are reasonable
- Confirm GPS meets minimum sats and is stable
- Confirm manual override works reliably

### Then autonomy experiments

- Start conservative
- Keep safe altitude and space
- Log everything and iterate

---

## Want to help improve this guide?

Documentation is part of the product.
If anything is unclear, missing, or outdated, open a PR or ask in Discord.

- Contributing: [`../../CONTRIBUTING.md`](../../CONTRIBUTING.md)
- Discord: https://discord.com/invite/bittensor

---

## Safety reminder

Langostino is a real flying machine.

- Follow local regulations
- Test in safe environments
- Use prop guards and appropriate failsafes
- Props off for bench testing
