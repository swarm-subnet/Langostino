# Parts list (BOM) — Langostino 🦐

This BOM is the parts list for a Langostino-compatible 6" quadcopter and the onboard autonomy stack (Raspberry Pi + ROS2).

> Safety: FPV drones can cause injury and property damage. Build and test responsibly. Keep propellers off during bench tests.

---

## Scope

This list covers:
- Airframe + propulsion + flight control electronics
- Radio control for manual override
- Onboard computer for ROS2 + AI stack
- Sensors and power conditioning required for reliable autonomy

---

## Core build (required)

### 1) Airframe

| Part | Qty | Link | Notes |
|---|---:|---|---|
| FlyFishRC Volador II VD6 (6" carbon frame) | 1 | https://es.aliexpress.com/item/1005009244888358.html?spm=a2g0o.order_list.order_list_main.88.6195194dFQg94A&gatewayAdapt=glo2esp | 6" frame with room for onboard computer + sensors. |

---

### 2) Propulsion

| Part | Qty | Link | Notes |
|---|---:|---|---|
| iFlight XING-E Pro 2306 1700KV brushless motors | 4 | https://es.aliexpress.com/item/1005008310935904.html?spm=a2g0o.order_list.order_list_main.10.6195194dFQg94A&gatewayAdapt=glo2esp | Sized for 6" class builds. |
| HQProp 6x3x3 propellers (6CW/6CCW, 12pcs) | 1 pack | https://es.aliexpress.com/item/1005009859783537.html?spm=a2g0o.order_list.order_list_main.76.6195194dFQg94A&gatewayAdapt=glo2esp | Keep spare propellers for testing and tuning. |

---

### 3) Flight controller + ESC

| Part | Qty | Link | Notes |
|---|---:|---|---|
| SpeedyBee F405 V4 + 55A 4-in-1 ESC stack (30x30) | 1 | https://es.aliexpress.com/item/1005010224729199.html?spm=a2g0o.order_list.order_list_main.22.6195194dFQg94A&gatewayAdapt=glo2esp | Flight controller + ESC stack. Compatible with INAV. |

---

### 4) GPS

| Part | Qty | Link | Notes |
|---|---:|---|---|
| HGLRC M10 GPS module + compass | 1 | https://es.aliexpress.com/item/1005008492407753.html?spm=a2g0o.order_list.order_list_main.16.6195194dFQg94A&gatewayAdapt=glo2esp | GPS/compass for navigation modes. Mount away from high-current wiring. |

---

### 5) Radio control (manual override)

| Part | Qty | Link | Notes |
|---|---:|---|---|
| RadioMaster Boxer 2.4G ELRS transmitter | 1 | https://es.aliexpress.com/item/1005005476887648.html?spm=a2g0o.order_list.order_list_main.5.6195194dFQg94A&gatewayAdapt=glo2esp | Manual control and safety override. |
| SpeedyBee Nano ExpressLRS receiver | 1 | https://es.aliexpress.com/item/1005009188594963.html?spm=a2g0o.order_list.order_list_main.34.6195194dFQg94A&gatewayAdapt=glo2esp | Receiver installed on the drone. |

---

### 6) Battery

| Part | Qty | Link | Notes |
|---|---:|---|---|
| Ovonic 4S LiPo 1300mAh 100C (XT60) | 1+ | https://es.aliexpress.com/item/1005009034486146.html?spm=a2g0o.order_list.order_list_main.58.6195194dFQg94A&gatewayAdapt=glo2esp | One battery is enough to start; multiple packs recommended for iteration. |

---

## Autonomy stack (required for Langostino-style autonomy)

### 7) Onboard computer

| Part | Qty | Link | Notes |
|---|---:|---|---|
| Raspberry Pi 5 (4GB/8GB) + case/fan | 1 | https://es.aliexpress.com/item/1005006268136678.html?spm=a2g0o.order_list.order_list_main.28.6195194dFQg94A&gatewayAdapt=glo2esp | Runs Ubuntu 22.04 + ROS2 Humble. Ensure adequate cooling. |

### 8) Clean 5V power (critical)

| Part | Qty | Link | Notes |
|---|---:|---|---|
| UBEC / switching regulator 5V (≥5A recommended) | 1 | https://es.aliexpress.com/item/1005009452713815.html?spm=a2g0o.order_list.order_list_main.40.6195194dFQg94A&gatewayAdapt=glo2esp | Stable 5V rail for Pi + sensors. Prefer ≥5A for Pi 5 headroom. |

---

## Sensors (required for the reference build)

| Part | Qty | Link | Notes |
|---|---:|---|---|
| LiDAR ToF (front-facing) | 1 | Link: TBD | Used for obstacle distance. Interface depends on model (I²C/UART). |
| LiDAR ToF (downward-facing) | 1 | Link: TBD | Used for altitude/ground distance. |
| LiDAR mounts (3D printed or equivalent) | 2 | Link: TBD | Improves alignment and reduces vibration. |

---

## Safety & field essentials

| Part | Qty | Link | Notes |
|---|---:|---|---|
| Autonomous buzzer / lost model buzzer | 1 | Link: TBD | Helps locate the drone after crashes or unexpected landings. |
| Low-ESR filtering capacitor (power line) | 1 | Link: TBD | Reduces electrical noise and voltage spikes from motors/ESC. |

---

## Storage, wiring, and build consumables

| Part | Qty | Link | Notes |
|---|---:|---|---|
| microSD or SSD (depending on Pi setup) | 1 | Link: TBD | OS + logs. Prefer reliable storage. |
| Ethernet cable (field debugging) | 1 | Link: TBD | Useful for stable debugging outdoors. |
| Silicone wire (power + signal), heatshrink, zip ties | 1 set | Link: TBD | Cable management and strain relief are part of reliability. |
| Battery straps + foam pad | 1–2 | Link: TBD | Secures LiPo during impact and vibration. |
| Threadlocker (medium) | 1 | Link: TBD | Helps motor screws stay secure (if required). |

---

## Optional spares

| Part | Qty | Notes |
|---|---:|---|
| Extra propeller packs | 1+ | Propellers are consumables during testing. |
| Extra batteries | 1+ | More flight time per session. |

---

## Notes on compatibility

- This BOM targets a 6" class build (VD6 frame, 2306 motors, 6" props).
- INAV is used for navigation/autonomy features and MSP control paths.
- The autonomy stack assumes Ubuntu 22.04 + ROS2 Humble on Raspberry Pi.

