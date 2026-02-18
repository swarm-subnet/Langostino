# 3D Printed Parts Guide

Custom 3D printed parts for mounting sensors and the companion computer on the drone frame.

> **Note:** The Raspberry Pi mount is designed for **Raspberry Pi 5**. A Raspberry Pi 3 does not fit due to the audio jack interfering with the case geometry. A version compatible with the Raspberry Pi 3 is coming soon.

---

## STL Files Location

All printable files are located in:

```
/assets/3D_print_STL/
```

---

## Parts List

### `drone-down-lidar-base.stl`

Base mount for the downward-facing LiDAR sensor. Attaches to the bottom of the drone frame.

---

### `drone-front-lidar-base.stl`

Base mount for the front-facing LiDAR sensor. On this frame, it mounts in the position originally designed for the GoPro mount.

---

### `drone-leg.stl`

Landing leg extension that fits onto the frame arm. Prevents the sensors mounted underneath from touching the ground during landing.

---

### `drone-lidar-mount.stl`

Universal LiDAR sensor mount used in both the downward and front positions. Designed so the sensor can be articulated, allowing its angle to be adjusted.

---

### `drone-raspberry-onboard-case.stl`

Case for mounting the Raspberry Pi 5 on the underside of the drone. It attaches using the screws on the lower front of the frame — the same screws that hold the metal parts originally designed for the FPV video system.

---

## Assembly Notes

The remaining components (ESCs, flight controller, receiver, etc.) should be assembled using **cable ties** and **hot silicone**, always ensuring that all parts are securely attached.

**Electromagnetic interference:** Components sensitive to electromagnetic fields, such as the **GPS** and **magnetometer**, should be mounted as far away as possible from the battery cables. For this reason:

- The **XT60 connector** is placed on the **side of the frame**.
- The **battery** is positioned **as far forward as possible**.
