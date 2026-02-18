# Parts list (BOM) — Langostino 🦐

This BOM is the parts list for a Langostino-compatible 6" quadcopter and the onboard autonomy stack (Raspberry Pi + ROS2).

## Scope

This list covers:
- All parts required to build the drone.
- Radio transmitter
- Onboard computer for ROS2 + AI stack
- Sensors and power supplier for reliable autonomy

## Core build (required)

### 1) Frame

| Part | Qty | Link | Notes |
|---|---:|---|---|
| FlyFishRC Volador II VD6 (6" carbon frame) | 1 | https://www.aliexpress.com/item/1005009244888358.html | 6" frame durable with enough room for onboard computer + sensors. |

### 2) Motors and propellers

| Part | Qty | Link | Notes |
|---|---:|---|---|
| iFlight XING-E Pro 2306 1700KV brushless motors | 4 | https://www.aliexpress.com/item/1005008310935904.html | Sized for 6" class builds. |
| HQProp 6x3x3 propellers (6CW/6CCW, 12pcs) | 1 pack | https://www.aliexpress.com/item/1005009859783537.html | Keep spare propellers for testing and tuning. |

### 3) Stack Flight controller + ESC

| Part | Qty | Link | Notes |
|---|---:|---|---|
| SpeedyBee F405 V4 + 55A 4-in-1 ESC stack (30x30) | 1 | https://www.aliexpress.com/item/1005010224729199.html | Flight controller + ESC stack. Compatible with INAV. |


### 4) GPS and compass module

| Part | Qty | Link | Notes |
|---|---:|---|---|
| HGLRC M10 GPS module + compass | 1 | https://www.aliexpress.com/item/1005008492407753.html | GPS/compass for navigation modes. Mount away from high-current wiring. |

### 5) ERLS radio transmitter and receiver

| Part | Qty | Link | Notes |
|---|---:|---|---|
| RadioMaster Boxer 2.4G ELRS transmitter | 1 | https://www.aliexpress.com/item/1005005476887648.html | Elrs and multichannel radio transmitter. |
| SpeedyBee Nano ExpressLRS receiver | 1 | https://www.aliexpress.com/item/1005009188594963.html| Receiver installed on the drone. |

### 6) Battery

| Part | Qty | Link | Notes |
|---|---:|---|---|
| Ovonic 4S LiPo 1300mAh 100C (XT60) | 1+ | https://www.aliexpress.com/item/1005009034486146.html| One battery is enough to start; multiple packs recommended for iteration. |

## Autonomy stack

### 7) Onboard computer

| Part | Qty | Link | Notes |
|---|---:|---|---|
| Raspberry Pi 5 (4GB/8GB) + case/fan | 1 | https://www.aliexpress.com/item/1005006268136678.html | Runs Ubuntu 24.04 + ROS2 Jazzy. Ensure adequate cooling. |

### 8) Clean 5V power (critical)

| Part | Qty | Link | Notes |
|---|---:|---|---|
| UBEC / switching regulator 5V (≥5A recommended) | 1 | https://www.aliexpress.com/item/1005009452713815.html | Stable 5V rail for Pi + sensors. Prefer ≥5A for Pi 5 headroom. |

## Sensors

| Part | Qty | Link | Notes |
|---|---:|---|---|
| LiDAR ToF (front-facing) | +3 | https://www.aliexpress.com/item/1005008738338495.html| Used for obstacle distance. Interface depends on model (I²C/UART). |

---

## Safety & field essentials

| Part | Qty | Link | Notes |
|---|---:|---|---|
| Autonomous buzzer / lost model buzzer | 1 | https://www.aliexpress.com/item/1005009383497649.html | Helps locate the drone after crashes or unexpected landings. It works even if the battery has been desconnected |
| Low-ESR filtering 35v 2200µf capacitor (power line) | 2 | https://www.aliexpress.com/item/1005003820087840.html| Reduces electrical noise and voltage spikes from motors/ESC. |

## Storage, wiring, and build consumables

| Part | Notes |
|---|---|
| microSD OS | 64GB min to store OS + proyect + logs
| Ethernet cable (field debugging) | Useful for direct connection to ethernet-ethernet. |
| Silicone wire awg 12 for and awg 16 for signals, heatshrink, zip ties | 
| Battery straps + foam pad | Secures LiPo during impact and vibration. |
| 3D printer | Recommended to print the files present in the `assets/3D_print_STL` folder. |

## Optional spares

| Part | Qty | Notes |
|---|---:|---|
| Extra propeller packs | 1+ | Propellers are consumables during testing. |
| Extra batteries | 1+ | More flight time per session. |

## Notes on compatibility

- This BOM targets a 6" class build (VD6 frame, 2306 motors, 6" props).
- INAV is used for navigation/autonomy features and MSP control paths.
- The autonomy stack assumes Ubuntu 25.04 + ROS2 Jazzy on a Raspberry Pi 5.


