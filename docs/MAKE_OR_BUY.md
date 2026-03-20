# Make-or-Buy Decision Analysis

**Date:** 2026-03-20
**Author:** CTO Office
**Scope:** Complete autonomous drone system — hardware, sensors, software, ground segment, compliance
**Assumption:** Building an autonomous drone stack from scratch. For every component we decide whether to develop in-house ("Make") or procure from the market ("Buy"), using **time-to-flight, cost, and engineering effort** as the primary metrics.

---

## Executive Summary

Building an autonomous drone from scratch requires decisions across **~30 component categories** spanning hardware, sensors, software, ground systems, and compliance. The wrong make/buy split is the #1 startup killer in this space: over-building burns runway, over-buying creates vendor lock-in and integration nightmares.

**Our guiding principles:**

1. **Buy commodity hardware** — flight controllers, IMUs, GPS, cameras, radios. These are mature markets with decade-long iteration. You cannot out-engineer a $300 Pixhawk with $300K of engineering effort.
2. **Buy infrastructure middleware** — IPC, logging, linear algebra, JSON parsing. These are solved problems. Integration effort is minimal compared to building from scratch.
3. **Make the differentiating software** — perception pipeline, mission planner, fault management, watchdog, sensor fusion. This is where your competitive moat lives. Buying these means you're reselling someone else's drone.
4. **Evaluate the hard middle** — VIO/SLAM, object detection models, path planning. These have strong open-source options that may be better than what you can build in-house, but choosing wrong creates deep dependencies.

**Bottom line:** Of ~30 components, we recommend **Make for 12** (all core software), **Buy for 14** (all hardware + infrastructure), and **Evaluate for 4** (components where the right answer depends on your specific mission profile).

### Cost Summary

| Scenario | Annual Software Cost | Hardware Cost (per drone) | Engineering Months |
|----------|---------------------|--------------------------|-------------------|
| **A: Maximum Open-Source** | $768–2,652 | ~$3,500–6,000 | 36–48 |
| **B: Pragmatic Hybrid** (recommended) | $768–3,651 | ~$5,000–10,000 | 24–30 |
| **C: Maximum Commercial** | $50,000–120,000 | ~$8,000–20,000 | 12–18 |

*Annual software costs include CI/CD infrastructure (GitHub Actions runners, self-hosted hardware). See [Section 32b: Software Licensing & Compliance](#32b-software-licensing--compliance) for the full breakdown. All open-source components are $0 license cost; the floor is set by CI/CD compute.*

---

## Table of Contents

- [Part I: Hardware Platform](#part-i-hardware-platform)
- [Part II: Sensors](#part-ii-sensors)
- [Part III: Communication](#part-iii-communication)
- [Part IV: Payload](#part-iv-payload)
- [Part V: Propulsion & Power](#part-v-propulsion--power)
- [Part VI: Software — Middleware & Infrastructure](#part-vi-software--middleware--infrastructure)
- [Part VII: Software — Perception](#part-vii-software--perception)
- [Part VIII: Software — Navigation & Localization](#part-viii-software--navigation--localization)
- [Part IX: Software — Mission & Control](#part-ix-software--mission--control)
- [Part X: Software — Reliability & Monitoring](#part-x-software--reliability--monitoring)
- [Part XI: Simulation & Testing](#part-xi-simulation--testing)
- [Part XII: Ground Segment](#part-xii-ground-segment)
- [Part XIII: Compliance & Security](#part-xiii-compliance--security)
- [Part XIV: Additional Components](#part-xiv-additional-components)
- [32b. Software Licensing & Compliance](#32b-software-licensing--compliance)
- [Appendix A: Full Bill of Materials](#appendix-a-full-bill-of-materials)
- [Appendix B: Priority Actions](#appendix-b-priority-actions)

---

# Part I: Hardware Platform

## 1. Companion Computer

**What it does:** Runs the entire autonomy stack (perception, planning, SLAM, comms). The brain of the drone.

**Verdict: BUY** — No startup should design custom compute hardware. The market offers excellent SWaP-optimized (size, weight, power) options purpose-built for edge AI on drones.

**CTO reasoning:** The companion computer is a commodity with a 2-year refresh cycle. Your software must be portable across compute platforms — designing around one vendor's silicon is acceptable, designing your own silicon is not. Choose based on AI inference TOPS/watt, camera interface count, and ecosystem maturity.

| Option | Vendor | Price | AI Performance | Power | Weight | Pros | Cons |
|--------|--------|-------|----------------|-------|--------|------|------|
| **NVIDIA Jetson Orin NX 16GB** | NVIDIA | ~$600 (module) + $250 (carrier) | 100 TOPS (INT8) | 10–25W | ~60g (module) | Best AI inference ecosystem (TensorRT, CUDA, cuDNN), massive developer community, JetPack 6.x, native support for 6 CSI cameras, 8 GB/16 GB LPDDR5 | Higher power draw than Qualcomm, thermal management needed, NVIDIA proprietary toolchain |
| **Qualcomm RB5 / QRB5165** | Qualcomm / ModalAI | ~$500 (VOXL 2 dev kit ~$800) | 15 TOPS (DSP+GPU) | 5–8W | ~40g | Lowest power, integrated 5G modem option, Hexagon DSP for VIO, ModalAI has flight-ready carrier boards (VOXL 2), smaller/lighter | Weaker GPU than Jetson, smaller community, Qualcomm SDK ecosystem fragmented, limited CUDA-equivalent tooling |
| **Intel NUC / x86 Edge** | Intel / various | ~$400–800 | Varies (iGPU ~5 TOPS, discrete ~30 TOPS) | 15–45W | ~200g+ | Familiar x86 dev environment, easy prototyping, wide peripheral support | Too heavy and power-hungry for most drones, no competitive edge AI, not flight-optimized |

**Recommendation:** It is recommended to select the **NVIDIA Jetson Orin NX 16GB.** The AI inference performance (100 TOPS) at 10–25W is unmatched. The CUDA/TensorRT ecosystem is the industry standard for drone perception. Qualcomm RB5 (via ModalAI VOXL 2) is the only real alternative if power budget is extremely tight (<8W) or integrated 5G is needed. x86 should be avoided — too heavy, too power-hungry.

---

## 2. Flight Controller

**What it does:** Low-level attitude control, motor mixing, sensor fusion (IMU + baro + mag + GPS), failsafe logic. Runs a real-time control loop at 400–1000 Hz.

**Verdict: BUY** — Flight controllers are safety-critical, real-time embedded systems with decades of community testing. Building one from scratch would take 2+ years and produce an inferior, untested product. The open-source PX4/ArduPilot ecosystem is the industry standard.

**CTO reasoning:** Your companion computer sends high-level commands (velocity, waypoints) to the FC via MAVLink. The FC handles the hard real-time control loop. This separation of concerns is architecturally correct. Never mix 30 Hz perception code with 1000 Hz PID loops on the same processor.

| Option | Vendor | Price | Processor | IMU | Firmware | Pros | Cons |
|--------|--------|-------|-----------|-----|----------|------|------|
| **Pixhawk 6X** | Holybro | ~$300 | STM32H753 (480 MHz Cortex-M7) | Triple-redundant IMU (ICM-42688-P × 2, ICM-20649) | PX4 / ArduPilot | Gold standard, triple-redundant IMU, Ethernet port for Jetson, vibration-isolated IMU, FMUv6X standard, massive community | Larger/heavier than mini FCs, overkill for small drones |
| **CubePilot Cube Orange+** | CubePilot / Hex | ~$350 | STM32H757 (480 MHz dual-core) | Triple IMU (ICM-42688 × 2, ICM-20649), heated IMU | PX4 / ArduPilot | Heated IMU (critical for high altitude), modular carrier board design, ADS-B In receiver option, proven in defense/enterprise | Heavier than Pixhawk 6X, carrier board adds cost, CubePilot ecosystem slightly fragmented |
| **Holybro Kakute H7 V2** | Holybro | ~$60 | STM32H743 (480 MHz) | Single IMU (BMI270) | ArduPilot (Betaflight for FPV) | Tiny, lightweight, cheap, good for sub-250g drones | Single IMU (no redundancy), no vibration isolation, fewer interfaces, not suitable for enterprise/safety-critical |

**Recommendation:** It is recommended to select the **Pixhawk 6X** for any drone >250g where reliability matters. The triple-redundant IMU and Ethernet interface to the companion computer are worth the $300. **PX4** firmware is recommended — it has the cleanest MAVLink companion computer integration, the best SITL simulation (Gazebo), and Auterion as a commercial backer. ArduPilot is the fallback if specific frame types or ground vehicle support are needed.

---

## 3. Airframe / Drone Platform

**What it does:** The physical drone — frame, arms, landing gear. Determines payload capacity, flight time, and operating envelope.

**Verdict: BUY** — Designing airframes is mechanical engineering, not software. Buy a proven frame and focus engineering effort on the autonomy stack.

**CTO reasoning:** Airframe design requires wind tunnel testing, FEA, vibration analysis, and FAA certification (for commercial operations). A startup should buy a frame that is well-characterized and has a large user community for troubleshooting. Custom frames only make sense at scale (>1000 units) or for unique form factors.

| Option | Vendor | Price | Size / Class | Payload Capacity | Flight Time | Pros | Cons |
|--------|--------|-------|-------------|-----------------|-------------|------|------|
| **Holybro X500 V2** | Holybro | ~$320 (frame kit) | 500mm wheelbase, ~2 kg AUW | ~600g useful payload | ~25 min (4S 5200 mAh) | PX4-native development platform, well-documented, modular, large community, spare parts available, designed for Pixhawk | Not waterproof, limited payload for heavy sensors (LiDAR), plastic arms |
| **Freefly Alta X** | Freefly | ~$8,500 | 1048mm, ~10 kg AUW | 4.5 kg payload | ~28 min (loaded) | Industrial-grade, folding arms, IP43, designed for heavy payloads (cinema cameras, LiDAR), travel case, proven in commercial ops | Expensive, heavier than needed for dev, proprietary motor/ESC |
| **DJI Matrice 350 RTK** | DJI | ~$6,500 | 670mm, ~6.3 kg AUW | 2.7 kg payload | ~41 min (no payload) | Best-in-class flight time, RTK GPS built-in, IP55 weatherproof, DJI reliability, obstacle avoidance sensors built-in | Closed ecosystem (DJI SDK only), NDAA-restricted (cannot sell to US government), cannot run PX4, locked to DJI software |

**Recommendation:** It is recommended to select the **Holybro X500 V2** for development. It is the PX4 reference platform — every PX4 tutorial and Gazebo model is built for it. A custom or industrial frame (Freefly Alta X class) should be considered when the mission profile demands heavier payloads. **DJI should be avoided for an autonomous stack** — their closed ecosystem prevents running custom companion software and PX4 firmware.

---

# Part II: Sensors

## 4. Mission Camera (Electro-Optical)

**What it does:** Primary visual sensor for object detection. Feeds the perception pipeline (P2) at 1920×1080 @ 30 Hz.

**Verdict: BUY** — Camera modules are commodity hardware. Your value-add is the perception software, not the sensor.

| Option | Vendor | Price | Resolution | Interface | Lens | Pros | Cons |
|--------|--------|-------|-----------|-----------|------|------|------|
| **Arducam IMX477 (HQ)** | Arducam | ~$50–80 | 12.3 MP (4056×3040) | MIPI CSI-2 | Interchangeable C/CS mount | High resolution, global shutter option, Jetson-native CSI, cheap, interchangeable lenses | No autofocus (manual lens), requires CSI cable routing, rolling shutter on base model |
| **FLIR Blackfly S (BFS-U3-16S2C)** | Teledyne FLIR | ~$500 | 1.6 MP (1440×1080) @ 226 FPS | USB3 | C-mount | Machine vision grade, global shutter, hardware trigger for sync, SDK (Spinnaker), excellent low-light | Expensive for a single camera, USB3 latency vs CSI, heavier, requires FLIR SDK |
| **Sony IMX296 (global shutter)** | e-con Systems / Leopard Imaging | ~$150–300 (module) | 1.6 MP (1440×1080) | MIPI CSI-2 | Board-level options | Global shutter (critical for high-speed flight), Jetson CSI native, compact, low-latency | Lower resolution than IMX477, fewer lens options, less community support |

**Recommendation:** It is recommended to use the **Arducam IMX477 for development** ($50, 12 MP, CSI). For production, it is recommended to switch to a **global shutter sensor** (Sony IMX296 or FLIR Blackfly S) — rolling shutter causes motion artifacts at drone speeds that degrade detection accuracy. The $150–500 premium is worth it.

---

## 5. Stereo Camera (for VIO/SLAM)

**What it does:** Provides depth and visual features for Visual-Inertial Odometry. Feeds P3 at 640×480 @ 30 Hz (stereo pair).

**Verdict: BUY** — Stereo camera calibration and baseline precision are critical. Pre-calibrated stereo modules save weeks of calibration effort and produce better results.

| Option | Vendor | Price | Resolution | Baseline | Interface | Pros | Cons |
|--------|--------|-------|-----------|----------|-----------|------|------|
| **Intel RealSense D455** | Intel | ~$350 | 1280×800 stereo + RGB | 95mm | USB3 | Pre-calibrated, built-in IMU (BMI055), hardware stereo depth processor, ROS/SDK support, proven in drone VIO (PX4 Avoidance) | Intel discontinued RealSense division (2025) — long-term supply risk, USB3 latency, 95mm baseline limits close-range depth |
| **Stereolabs ZED 2i** | Stereolabs | ~$500 | 2208×1242 stereo | 120mm | USB3 | Built-in IMU, IP66 weatherproof, integrated neural depth, SDK includes VIO/SLAM/object detection, barometer | Requires ZED SDK (proprietary, free with camera), heavier (~170g), higher power (~4W), vendor lock-in for VIO features |
| **Luxonis OAK-D Pro** | Luxonis | ~$300 | 1280×800 stereo + 12 MP RGB | 75mm | USB3 / USB2 | On-chip AI (Myriad X VPU, 4 TOPS), open-source DepthAI SDK, compact, active IR projector for structured light, low power (~2.5W) | Myriad X less powerful than Jetson GPU (redundant if using Jetson for AI), smaller baseline limits outdoor depth |

**Recommendation:** It is recommended to select the **Intel RealSense D455** if available in the supply chain — it is the most widely used stereo camera in the PX4/ROS drone ecosystem. **ZED 2i** is the alternative if an integrated VIO solution is desired (their SDK includes VIO, but that creates vendor lock-in). **OAK-D Pro** is suitable for budget-constrained projects. Given this stack runs custom VIO on the Jetson, only calibrated stereo frames are needed — the D455 raw stream is ideal.

**Supply risk note:** Intel shuttered the RealSense division. Existing stock is available, but a migration path to OAK-D or custom stereo pairs should be planned for production.

---

## 6. Inertial Measurement Unit (IMU)

**What it does:** Measures acceleration (3-axis) and angular rate (3-axis) at 200–400 Hz. Critical for VIO and attitude estimation.

**Verdict: BUY** — IMU design requires MEMS fabrication expertise. No software company should build IMUs.

**CTO reasoning:** Your flight controller already has a triple-redundant IMU for attitude control. The question is whether you need a *separate* IMU on the companion computer for VIO. Yes — if your VIO pipeline (P3) needs tight time-synchronization between stereo frames and IMU samples. The FC's IMU is accessed via MAVLink at ~100 Hz with variable latency, which is insufficient for high-quality VIO.

| Option | Vendor | Price | Axes | Sample Rate | Noise (Gyro) | Interface | Pros | Cons |
|--------|--------|-------|------|-------------|---------------|-----------|------|------|
| **Bosch BMI088** | Bosch | ~$8 (chip), ~$30 (breakout) | 6-DOF (accel + gyro) | Up to 2000 Hz | 0.014 °/s/√Hz | SPI / I2C | Automotive-grade, low noise, proven in PX4 (Pixhawk 6X uses it), low power (0.9 mA), tiny | Requires SPI driver on Jetson, no magnetometer, no built-in calibration |
| **VectorNav VN-100** | VectorNav | ~$500 | 9-DOF (accel + gyro + mag) | Up to 800 Hz | 0.0035 °/s/√Hz | UART / SPI | Industrial-grade, on-board Kalman filter, factory-calibrated, temperature-compensated, excellent bias stability | Expensive for a drone, overkill if you do your own fusion, 9g weight |
| **InvenSense ICM-42688-P** | TDK InvenSense | ~$5 (chip), ~$25 (breakout) | 6-DOF | Up to 32 kHz (accel), 32 kHz (gyro) | 0.0028 °/s/√Hz (low-noise mode) | SPI / I2C | Best-in-class MEMS gyro noise, used in Pixhawk 6X, ultra-low power (0.5 mA), AEC-Q100 automotive qualified | Chip-level — requires PCB integration, no magnetometer, no built-in filter |

**Recommendation:** If the stereo camera has a built-in IMU (D455's BMI055, ZED 2i's BMI085), it is recommended to **use that first** — hardware time-sync between camera and IMU frames is more important than raw IMU quality. If a standalone IMU is needed for the companion computer, **BMI088** is the recommended choice: proven, cheap, drone-grade. VectorNav VN-100 is only recommended if factory calibration is required and custom bias estimation is not desired.

---

## 7. GPS / GNSS Module

**What it does:** Global position fix. Essential for waypoint navigation, geofencing, and return-to-launch. Accuracy: ~2.5m (standard), ~2cm (RTK).

**Verdict: BUY** — GNSS receivers are specialized RF silicon. Buy the module.

| Option | Vendor | Price | Constellations | Accuracy | RTK | Pros | Cons |
|--------|--------|-------|---------------|----------|-----|------|------|
| **u-blox M9N** | u-blox | ~$50–80 (module) | GPS + GLONASS + Galileo + BeiDou | 2.5m CEP | No (M9N), Yes (F9P) | Industry standard for drones, PX4/ArduPilot native support, tiny, low power, concurrent multi-GNSS | No RTK on M9N (need F9P for ~$200), single-frequency |
| **u-blox F9P** | u-blox | ~$200 (module), ~$300 (Holybro H-RTK F9P kit) | GPS + GLONASS + Galileo + BeiDou (L1/L2 dual-freq) | 1cm + 1ppm (RTK), 2.5m (standalone) | Yes (RTK) | Centimeter-accurate RTK, dual-frequency (robust to ionospheric error), PX4-native, RTCM correction input via MAVLink, widely used | Requires RTK base station or NTRIP service for corrections, ~$200 more than M9N, slightly higher power |
| **Septentrio mosaic-go** | Septentrio | ~$600 (evaluation kit) | GPS + GLONASS + Galileo + BeiDou + NavIC (triple-freq) | 0.6m (standalone), 0.8cm (RTK) | Yes (RTK) | Professional-grade, anti-jamming, anti-spoofing, triple-frequency, AIM+ interference mitigation, smallest RTK receiver, web interface for config | 3× price of F9P, overkill for most drone applications, smaller community |

**Recommendation:** It is recommended to use the **u-blox M9N** for development and basic missions (GPS is a secondary input to VIO). An upgrade to the **u-blox F9P** is recommended when centimeter-accurate positioning is needed for precision landing, surveying, or mapping missions. Septentrio is only recommended for contested RF environments (defense, critical infrastructure) where anti-jamming is required.

---

## 8. LiDAR (Optional)

**What it does:** 3D point cloud for obstacle detection, terrain mapping, and SLAM. Enables reliable obstacle avoidance in GPS-denied or vision-degraded environments.

**Verdict: BUY (when needed)** — LiDAR is not required for basic autonomy (camera + VIO is sufficient for most missions), but becomes critical for BVLOS operations and GPS-denied flight in complex environments.

**CTO reasoning:** LiDAR adds $500–2,000 to each drone, plus 100–400g of weight and 5–15W of power. Only add it when your mission profile demands it (BVLOS, indoor inspection, mining/construction mapping). For visual-only autonomy, skip LiDAR in V1.

| Option | Vendor | Price | Points/sec | Range | Weight | Power | Pros | Cons |
|--------|--------|-------|-----------|-------|--------|-------|------|------|
| **Livox Mid-360** | Livox (DJI subsidiary) | ~$600 | 200,000 pts/s | 40m (10% reflectivity) | 265g | ~9W | Best price-to-performance for drones, non-repetitive scan pattern (higher effective resolution), Ethernet interface, compact, used in DJI L2 payload | Non-standard scan pattern requires custom processing (not spinning LiDAR), DJI subsidiary (NDAA consideration), limited FOV (360°×59°) |
| **Ouster OS0-32** | Ouster | ~$2,500 | 655,360 pts/s | 50m | 447g | ~14–20W | True 360° FOV, digital lidar (solid-state, no moving parts internally), excellent ambient light rejection, REV7 smallest form factor, open-source drivers | Heavier and more power-hungry than Livox, more expensive, overkill for small drones |
| **Intel RealSense L515 (LiDAR Camera)** | Intel | ~$350 (while stock lasts) | 23M pixels/s | 9m | 100g | ~3.5W | Extremely lightweight, low power, solid-state, good for indoor mapping, USB3 interface | Very short range (9m), discontinued (Intel RealSense shutdown), indoor-only |

**Recommendation:** It is recommended to skip LiDAR for V1. When needed, the **Livox Mid-360** is the best drone-optimized option at $600, 265g, 9W. Ouster OS0 is recommended for professional surveying/mapping where 360° coverage and range matter more than weight.

---

## 9. Barometer / Magnetometer

**What it does:** Barometer provides altitude hold (relative altitude via air pressure). Magnetometer provides compass heading. Both feed the flight controller's EKF.

**Verdict: BUY (included with flight controller)** — The Pixhawk 6X includes an MS5611 barometer and IST8310 magnetometer. No additional procurement needed unless you need an external magnetometer (recommended for accuracy).

| Option | Vendor | Price | Type | Notes |
|--------|--------|-------|------|-------|
| **External GPS + Mag combo** (e.g., Holybro M9N GPS) | Holybro | ~$50 | Magnetometer (IST8310/QMC5883L) + GPS | Mount away from motors/ESCs to avoid magnetic interference. All Pixhawk GPS modules include a magnetometer |
| **Standalone barometer (BMP388)** | Bosch / SparkFun | ~$10 | Barometric pressure | Only if the FC's built-in baro has vibration issues. Most FCs have foam-isolated baro already |

**Recommendation:** It is recommended to use what comes with the **Pixhawk 6X + external GPS/mag module**. No additional procurement is needed. The external magnetometer should be mounted as far from motors as possible (GPS mast).

---

# Part III: Communication

## 10. Telemetry Radio (FC <-> GCS)

**What it does:** Low-bandwidth, long-range radio link between the drone and ground control station. Carries MAVLink telemetry (position, battery, health) and commands (RTL, mission upload).

**Verdict: BUY** — Radio hardware requires RF certification (FCC/CE). Buy certified modules.

| Option | Vendor | Price | Range (LOS) | Bandwidth | Frequency | Pros | Cons |
|--------|--------|-------|-------------|-----------|-----------|------|------|
| **Holybro SiK Radio V3** | Holybro | ~$60 (pair) | 1–2 km | 57.6–250 kbps | 915 MHz / 433 MHz | PX4/ArduPilot native, transparent serial bridge, FHSS, FCC certified, cheapest option, plug-and-play | Short range, low bandwidth (no video), half-duplex |
| **RFD900x** | RFDesign | ~$250 (pair) | 40+ km | Up to 750 kbps | 900 MHz | Long range (40+ km with antenna), high bandwidth, FHSS, mesh networking option, used in commercial drones, PPM passthrough for RC | Larger antenna, higher power draw, 900 MHz (may interfere with other equipment), requires antenna pointing for max range |
| **Doodle Labs Smart Radio (Helix Mini)** | Doodle Labs | ~$1,500 (pair) | 10+ km | Up to 20 Mbps | 2.4 GHz / 5 GHz / 1.6 GHz (custom) | High bandwidth (video + telemetry), MIMO, mesh networking, AES-256 encryption, IP67, military-grade, MANET | Expensive, higher power draw, 2.4/5 GHz range limited by interference, overkill for telemetry-only |

**Recommendation:** It is recommended to use the **SiK Radio V3** for development ($60). The **RFD900x** is recommended for field operations requiring >2 km range. Doodle Labs is only recommended for defense/enterprise customers needing encrypted, high-bandwidth, mesh-networked comms.

---

## 11. Datalink (High-Bandwidth Video + Telemetry)

**What it does:** Streams live video and high-rate telemetry from the drone to the ground. Separate from (or combined with) the MAVLink telemetry radio.

**Verdict: BUY (when needed)** — Only required if your mission needs live video downlink to a human operator. Fully autonomous drones that make all decisions on-board can operate with telemetry-only radio.

| Option | Vendor | Price | Range | Bandwidth | Latency | Pros | Cons |
|--------|--------|-------|-------|-----------|---------|------|------|
| **Herelink** | CubePilot | ~$500 (air unit + ground controller) | 20 km | H.265 video + MAVLink | ~150 ms (video) | Integrated solution (radio + ground controller + Android tablet), PX4/ArduPilot compatible, dual HDMI input, FCC certified | Proprietary ground unit, limited to 1080p@30fps, 2.4 GHz (interference-prone), no mesh |
| **DJI O3 Air Unit** | DJI | ~$230 (air unit) | 13 km (FCC) | 1080p@60fps + MAVLink via UART | ~28 ms (video) | Incredible latency and video quality, tiny/lightweight (30g), digital FPV | DJI ecosystem lock-in, no encryption (consumer-grade), NDAA restricted, requires DJI goggles/controller |
| **Silvus StreamCaster 4200** | Silvus Technologies | ~$5,000+ (per radio) | 80+ km (with directional ant.) | 100+ Mbps | Low | Military-grade MIMO mesh, AES-256, anti-jam, MANET, multiple video streams, IP mesh networking | Very expensive, heavy, defense-focused, US ITAR-controlled |

**Recommendation:** It is recommended to use **Herelink** for commercial operations needing video downlink with MAVLink integration. DJI O3 is suitable for FPV/development if the ecosystem lock-in is acceptable. Silvus is only recommended for defense programs. For a fully autonomous stack, it is recommended to **defer video downlink** — the focus should be on on-board decision-making first.

---

## 12. Multi-Drone Mesh Networking (Future)

**What it does:** Ad-hoc mesh network between multiple drones and ground stations. Enables swarm coordination, distributed task allocation, and resilient communication.

**Verdict: BUY (future)** — Mesh networking is radio engineering. Buy when you need multi-drone operations.

| Option | Vendor | Price | Range | Bandwidth | Pros | Cons |
|--------|--------|-------|-------|-----------|------|------|
| **Rajant Peregrine** | Rajant | ~$3,000–5,000 per node | 2+ km (per hop) | Up to 100 Mbps | InstaMesh (self-healing, zero-config), multi-frequency, proven in mining/construction, military customers | Expensive per node, heavy, high power |
| **Doodle Labs Mesh Rider** | Doodle Labs | ~$1,500 per node | 10+ km | Up to 20 Mbps | Purpose-built for drones, lightweight, MANET, AES-256, IP67, multiple frequency bands | Still expensive at scale, requires network planning |
| **Zenoh (software mesh)** | ZettaScale | Free (software) | N/A (runs over any IP network) | Limited by transport layer | Already in your stack, zero-copy, pub/sub, peer-to-peer or routed, runs over WiFi/LTE/custom radio | Not a radio — still needs underlying physical transport, not a replacement for dedicated mesh radios |

**Recommendation:** It is recommended to use **Zenoh over WiFi** for development and close-range multi-drone testing. When deploying multi-drone operations, **Doodle Labs Mesh Rider** ($1,500/node) should be evaluated as the best drone-optimized solution. Note that Zenoh's network transport layer can run over any mesh radio, so the software layer is already solved.

---

# Part IV: Payload

## 13. Gimbal System

**What it does:** Mechanically stabilizes the mission camera against drone vibration and attitude changes. Enables pointing the camera at specific targets.

**Verdict: BUY** — Gimbal stabilization requires precision brushless motors, IMU-based feedback loops, and vibration-isolated mounts. Mature market.

| Option | Vendor | Price | Axes | Payload | Interface | Pros | Cons |
|--------|--------|-------|------|---------|-----------|------|------|
| **SIYI A8 Mini** | SIYI | ~$260 | 3-axis | Up to 110g | UART / UDP (Ethernet) | Lightweight (78g), integrated 4K camera (Sony IMX415), wide-angle, UART/UDP control protocol, PX4 compatible, affordable | Small payload capacity (only for built-in camera), limited zoom, newer brand (less track record) |
| **Gremsy Pixy U** | Gremsy | ~$1,200 | 3-axis | Up to 500g | UART / CAN (MAVLink) | MAVLink-native gimbal protocol, carries medium cameras (Sony Alpha, FLIR Vue), vibration-isolated, proven in commercial drones | Expensive, heavier (320g gimbal weight), requires separate camera |
| **DJI Zenmuse H30** | DJI | ~$9,800 | 3-axis | Integrated multi-sensor | DJI SDK (PSDK) | Thermal + zoom + wide + laser rangefinder in one unit, best stabilization, IP55, enterprise-grade | Only works with DJI M350 RTK, extremely expensive, closed ecosystem |

**Recommendation:** It is recommended to select the **SIYI A8 Mini** for development and lightweight missions ($260, built-in 4K camera). The **Gremsy Pixy U** is recommended when interchangeable cameras (thermal, zoom) need to be carried. The gimbal HAL interface (`IGimbal`) in the stack already supports SIYI's UART protocol as a planned backend.

---

## 14. Thermal Camera (LWIR)

**What it does:** Detects thermal radiation (8–14 μm). Essential for search & rescue, inspection, security, and wildlife monitoring. Sees through smoke/haze, works in complete darkness.

**Verdict: BUY** — FLIR/DRS dominance means you buy or you don't have thermal. ITAR-controlled above 9 Hz (drone-grade is usually 30 Hz with export license).

| Option | Vendor | Price | Resolution | Frame Rate | Interface | Pros | Cons |
|--------|--------|-------|-----------|------------|-----------|------|------|
| **FLIR Boson 640** | Teledyne FLIR | ~$3,500 | 640×512 | 60 Hz | USB / CMOS parallel | Industry standard drone thermal core, tiny (21×21×11 mm, 7.5g), low power (0.5W), radiometric, VGA resolution | Expensive, ITAR-controlled, requires integration (no housing/lens included in core price) |
| **FLIR Lepton 3.5** | Teledyne FLIR | ~$250 | 160×120 | 9 Hz | SPI | Incredibly cheap, tiny, low power, good for presence detection | Very low resolution (160×120), 9 Hz frame rate, short range, not suitable for serious perception |
| **DJI Zenmuse H20T (thermal payload)** | DJI | ~$6,200 | 640×512 | 30 Hz | DJI SDK | Integrated with zoom + wide-angle camera, radiometric, excellent image quality | DJI-only, expensive, closed ecosystem |

**Recommendation:** It is recommended to use the **FLIR Lepton 3.5** for development ($250, validates thermal pipeline). The **FLIR Boson 640** is recommended for production ($3,500, full VGA thermal at 60 Hz). The stack already has a `SimulatedThermalCamera` backend — Boson 640 integration is straightforward via USB.

---

# Part V: Propulsion & Power

## 15. Motors, ESCs, and Propellers

**What it does:** Converts electrical energy into thrust. The motor-ESC-propeller combination determines thrust-to-weight ratio, flight time, and responsiveness.

**Verdict: BUY** — Commodity components. Match to your airframe's weight class.

| Option | Vendor | Price (set of 4) | Class | Thrust (per motor) | Pros | Cons |
|--------|--------|------------------|-------|--------------------|----- |------|
| **T-Motor U5 + T-Motor Alpha 40A ESC** | T-Motor | ~$400 (4 motors + 4 ESCs) | 500–650mm quad | 2.1 kg @ 100% (6S) | Premium quality, efficient, low vibration, used in DJI and professional drones, BLHeli_32 ESC firmware | Expensive, requires matching props (T-Motor 15×5) |
| **Holybro X500 kit motors (2216-880KV)** | Holybro / generic | ~$100 (4 motors + 4 ESCs) | 500mm quad | ~1.0 kg @ 100% (4S) | Matched to X500 frame, cheap, well-tested with PX4, good for development | Lower quality than T-Motor, less efficient, noisier |
| **KDE Direct 2315XF-885 + KDE 35A ESC** | KDE Direct | ~$550 (4 motors + 4 ESCs) | 500–700mm quad | 2.0 kg @ 100% (6S) | Made in USA, defense-grade, excellent quality, long lifespan, KDE Direct is a known defense supplier | Most expensive, limited distribution, requires KDE props |

**Recommendation:** It is recommended to use **Holybro kit motors** for development ($100). An upgrade to **T-Motor** or **KDE Direct** is recommended for production. Propeller size should be matched to motor KV and battery voltage using the manufacturer's thrust tables. A >2:1 thrust-to-weight ratio should be targeted.

---

## 16. Battery & Power Distribution

**What it does:** Stores energy and distributes it to motors, FC, companion computer, and payloads. Determines flight time.

**Verdict: BUY** — LiPo battery chemistry and power distribution boards are commodity.

| Option | Vendor | Price | Capacity | Voltage | Weight | Pros | Cons |
|--------|--------|-------|----------|---------|--------|------|------|
| **Tattu 6S 5200 mAh 35C** | Tattu (Gens Ace) | ~$90 | 5200 mAh | 22.2V (6S) | ~680g | Reliable, consistent discharge, popular in commercial drones, wide availability | Standard LiPo (no smart BMS), requires external voltage/current monitoring |
| **Tattu Plus Smart Battery 6S 10000 mAh** | Tattu | ~$200 | 10,000 mAh | 22.2V (6S) | ~1,200g | Built-in smart BMS (cell balancing, voltage/temp monitoring, CAN bus), longer flight time, battery health reporting | Heavier, expensive, requires CAN integration for smart features |
| **Maxamps LiPo 4S 5450 mAh** | MaxAmps | ~$120 | 5450 mAh | 14.8V (4S) | ~480g | Lightweight, Made in USA, custom configurations available, excellent customer support | 4S (lower voltage = less thrust headroom), smaller brand |

**Power Distribution:**

| Option | Vendor | Price | Notes |
|--------|--------|-------|-------|
| **Holybro PM02 V3 Power Module** | Holybro | ~$35 | Voltage + current sensing for Pixhawk, up to 60A, analog output. Standard choice for PX4 |
| **Mauch Power Module** | Mauch Electronics | ~$80 | Higher accuracy (±0.5%), hall-effect current sensor, multiple models up to 200A. For precision battery estimation |

**Recommendation:** It is recommended to use the **Tattu 6S 5200 mAh** for development. The **PM02 V3 Power Module** provides battery voltage/current to the Pixhawk for battery failsafe logic. The stack's `FCState` already receives battery data via MAVLink — no custom power monitoring is needed.

---

# Part VI: Software — Middleware & Infrastructure

## 17. IPC / Middleware

**What it does:** Inter-process communication between the 7 processes. Carries video frames (~6 MB), pose data, commands, and health telemetry across process boundaries.

**Verdict: MAKE (integration) + BUY (engine) — already done.** We use Eclipse Zenoh as the transport engine (buy), wrapped in our own `IPublisher<T>` / `ISubscriber<T>` abstraction layer with liveliness tokens and fault-tolerant channel design (make). This is the correct architecture — don't build a pub/sub engine, but do own the abstraction.

**CTO reasoning:** The IPC layer is infrastructure. What matters is the abstraction. By wrapping Zenoh behind `IPublisher<T>` / `ISubscriber<T>`, we can swap the engine without touching process code. The Zenoh engine itself is world-class (15 μs latency, zero-copy SHM) — no point rebuilding it.

| Option | License | Annual Cost | Latency | Network Transport | Pros | Cons |
|--------|---------|-------------|---------|-------------------|------|------|
| **Eclipse Zenoh** (current) | EPL 2.0 / Apache 2.0 (dual) | $0 | ~15 μs | Yes (TCP/UDP/QUIC, peer-to-peer + routed) | Best latency+throughput combo, zero-copy SHM, network-transparent (drone↔GCS over any IP), Rust core (memory-safe), cloud-to-edge topology | Smaller community than ROS 2, must explicitly select Apache 2.0 license for commercial use (see Licensing section) |
| **Eclipse iceoryx** | Apache 2.0 | $0 | ~1 μs (true zero-copy) | **No** (single-machine only) | Lowest possible latency (~1 μs, kernel-bypass zero-copy via shared memory), lock-free design, deterministic real-time performance, clean Apache 2.0 license, used in AUTOSAR Adaptive (automotive), Eclipse Foundation backed | **No network transport** — cannot send data between drone and GCS, no built-in discovery across machines, requires a separate network layer (e.g., Zenoh, DDS, or custom) for any off-board communication |
| **RTI Connext DDS** | Commercial | ~EUR 4,500/dev/yr | ~50–100 μs | Yes (UDP multicast, TCP, shared memory) | DO-178C / ISO 26262 certified, DDS standard, used in F-35/MQ-25, QoS policies | Expensive, complex XML configuration, overkill for prototype |
| **eProsima Fast-DDS** | Apache 2.0 | $0 | ~50–100 μs | Yes (UDP multicast, TCP, shared memory) | ROS 2 default middleware, large community, DDS standard | 3× higher latency than Zenoh, heavier memory footprint |

**iceoryx deep-dive:** Eclipse iceoryx deserves serious consideration for on-board IPC. Its true zero-copy architecture (publisher writes directly into shared memory, subscriber reads without any copy or serialization) achieves ~1 μs latency — roughly 15× faster than Zenoh's SHM mode. For our stack's 6 MB video frames flowing P1→P2 and P1→P3, iceoryx would eliminate all copy overhead. However, iceoryx is **fundamentally single-machine**: it has no network transport layer. Our architecture requires drone↔GCS communication (P5 `comms` ↔ ground station) over TCP/UDP, which iceoryx cannot provide. This means adopting iceoryx would require **two middleware layers** — iceoryx for on-board IPC + a separate network transport (Zenoh, DDS, or custom UDP) for GCS/fleet comms. This dual-stack approach adds complexity, doubles the abstraction surface, and creates a synchronization boundary between local and network messages. Zenoh's SHM mode delivers ~15 μs latency (sufficient for 30 Hz video at 33 ms frame intervals) while also providing network transport — one middleware for both on-board and off-board communication.

**Recommendation:** It is recommended to keep Zenoh as the sole IPC middleware. The ~14 μs latency premium over iceoryx is negligible relative to frame intervals (33 ms at 30 Hz), and the architectural simplicity of a single middleware layer outweighs the raw performance advantage. If profiling reveals that IPC copy overhead is a bottleneck (unlikely given Zenoh's SHM mode), iceoryx could be evaluated as an on-board fast-path behind the existing `IPublisher<T>` / `ISubscriber<T>` abstraction, with Zenoh retained for network transport. The only reason to switch entirely is if safety certification (RTI Connext DDS) is required for defense contracts needing DO-178C evidence.

---

## 18. Flight Controller Firmware

**What it does:** Real-time autopilot running on the FC hardware. Handles attitude control, motor mixing, failsafes, MAVLink communication.

**Verdict: BUY (open-source)** — PX4 and ArduPilot are the two options. Do not build your own autopilot.

**CTO reasoning:** Writing a flight controller firmware is a multi-year effort requiring real-time control theory, motor dynamics modeling, extensive PID tuning, and safety certification. PX4 has >10 years of development and runs on thousands of commercial drones. It's the single most important "buy" decision in this stack.

| Option | License | Cost | Vehicles | SITL | Pros | Cons |
|--------|---------|------|----------|------|------|------|
| **PX4 Autopilot** (current) | BSD 3-Clause | $0 | Multi-rotor, VTOL, fixed-wing | Gazebo, jMAVSim | Cleanest companion computer integration (MAVSDK), best SITL (Gazebo Harmonic), Auterion commercial backing, used by defense (Auterion, Shield AI), modular architecture | Fewer vehicle types than ArduPilot, smaller hobbyist community |
| **ArduPilot** | GPL 2.0+ | $0 | Multi-rotor, VTOL, fixed-wing, rover, submarine, boat | Gazebo, RealFlight, X-Plane | Largest community, most vehicle types, most peripheral drivers, longest history (2009), MAVROS ROS integration | GPL license (copyleft — may affect proprietary companion software), SITL less polished than PX4, less clean offboard API |
| **Auterion OS** | Commercial (PX4-based) | Per-unit licensing (contact sales) | PX4-compatible vehicles | Gazebo | PX4 + fleet management + OTA updates + analytics + security, commercial support, used by US Army (FTUAS) | Vendor lock-in (requires Auterion Skynode hardware or software license), per-unit cost |

**Recommendation:** It is recommended to keep PX4. BSD license, best companion computer integration via MAVSDK, best Gazebo SITL. ArduPilot is the alternative if niche vehicle types are needed. Auterion OS is recommended when commercial support + fleet management is required for defense/enterprise.

---

## 19. FC Communication SDK

**What it does:** C++ library for sending commands to and receiving telemetry from the flight controller over MAVLink.

**Verdict: BUY (open-source) — already done.** MAVSDK is the correct choice.

| Option | License | Cost | Language | Pros | Cons |
|--------|---------|------|----------|------|------|
| **MAVSDK** (current) | BSD 3-Clause | $0 | C++ (also Python, Swift, Java) | PX4-recommended, clean async C++ API, no ROS dependency, Action/Offboard/Telemetry plugins, cross-platform | Limited ArduPilot support, some missing MAVLink message types |
| **MAVROS** | BSD 3-Clause | $0 | C++ (ROS 2 node) | Full MAVLink bridge, PX4 + ArduPilot, ROS 2 ecosystem | Requires ROS 2 (heavy dependency), not standalone |
| **pymavlink** | LGPL 3.0 | $0 | Python (C bindings) | Maximum MAVLink control, all message types, raw protocol access | Python (not suitable for C++ stack), LGPL license, no high-level API |

**Recommendation:** It is recommended to keep MAVSDK. No reason to change.

---

## 20. Logging & Linear Algebra Libraries

**What it does:** Structured logging (spdlog), matrix/vector math (Eigen3), config parsing (nlohmann/json). Foundation libraries used by every process.

**Verdict: BUY (open-source) — already done.** These are the industry-standard choices. Alternatives exist but offer no advantage.

| Library | License | Purpose | Why this one |
|---------|---------|---------|-------------|
| **spdlog** | MIT | Structured logging | Fastest C++ logger, header-only, fmt-based, compile-time format checking |
| **Eigen3** | MPL 2.0 | Linear algebra | De facto standard in robotics, SIMD-optimized, header-only, used by GTSAM/Ceres/g2o |
| **nlohmann/json** | MIT | JSON config parsing | Most popular C++ JSON library, intuitive API, header-only |
| **GTest** | BSD 3-Clause | Unit testing | Google Test — standard C++ testing framework |

**Recommendation:** It is recommended to keep all four. No action needed.

---

# Part VII: Software — Perception

## 21. Object Detection / AI Inference

**What it does:** Detects objects (people, vehicles, drones) in camera frames. The input to the tracking pipeline. Currently YOLOv8n via OpenCV DNN — **AGPL 3.0 license risk**.

**Verdict: EVALUATE — switch model, keep inference pipeline.**

**CTO reasoning:** The detection *pipeline* (frame ingestion, pre-processing, NMS, class mapping) is ours and should stay ours. The *model* is the question. AGPL is unacceptable for a commercial product. We need an Apache 2.0 or MIT-licensed model that runs at 20+ FPS on Jetson Orin with TensorRT.

| Option | Vendor | License | Cost | Jetson Orin FPS (est.) | mAP (COCO) | Pros | Cons |
|--------|--------|---------|------|----------------------|------------|------|------|
| **RF-DETR (Nano/Large)** | Roboflow | Apache 2.0 | $0 | ~20–30 FPS (TensorRT FP16) | 53.3 mAP (Large) | Transformer-based (no NMS needed), permissive license, ICLR 2026, actively developed, fine-tunable on custom datasets | Newer (less battle-tested), higher VRAM than YOLO, transformer warmup |
| **RT-DETR v2** | Baidu / PaddlePaddle | Apache 2.0 | $0 | ~40–50 FPS (TensorRT) | 54.3 mAP (RT-DETR-X) | Proven on aerial datasets (VisDrone), NMS-free, CVPR 2024, multiple size variants (S/M/L/X), exportable to ONNX→TensorRT | PaddlePaddle ecosystem less familiar, fewer drone-specific fine-tuned weights |
| **YOLOv9 / YOLO11 (Ultralytics Enterprise)** | Ultralytics | Commercial ($5K/yr) or AGPL 3.0 | $5,000/yr | ~60+ FPS (TensorRT INT8) | 55.4 mAP (YOLO11x) | Highest FPS, most mature ecosystem, best tooling (export, benchmark, dataset management), largest model zoo | AGPL if license lapses, ongoing cost, vendor dependency |

**Inference Engine:**

| Engine | License | Cost | Notes |
|--------|---------|------|-------|
| **NVIDIA TensorRT** | NVIDIA EULA (free on Jetson) | $0 | 2–5× faster than ONNX Runtime on Jetson, FP16/INT8 quantization, essential for production |
| **ONNX Runtime** | MIT | $0 | Cross-platform, good for development, CUDA EP for GPU |
| **OpenCV DNN** (current) | Apache 2.0 | $0 | Simplest integration, but 2–3× slower than TensorRT on Jetson |

**Recommendation:** It is recommended to switch to **RT-DETR v2 (Apache 2.0) + TensorRT for production.** RT-DETR is more proven than RF-DETR and runs faster on Jetson. OpenCV DNN should be used during development (as today), TensorRT for deployment. This eliminates the AGPL risk at zero cost. A budget of $5K/yr for Ultralytics Enterprise is only recommended if their tooling ecosystem (auto-labeling, Roboflow integration) is needed.

---

## 22. Multi-Object Tracking (MOT)

**What it does:** Associates detections across frames to maintain persistent object identities. Handles occlusion, ID re-assignment, and track lifecycle.

**Verdict: MAKE — already done.** Our ByteTrack implementation is competitive, C++ native, and carries zero license risk. (SORT was removed in Issue #205 — ByteTrack strictly supersedes it.)

**CTO reasoning:** MOT algorithms (SORT, ByteTrack, BoT-SORT) are well-documented in papers with clear pseudocode. The implementation is ~500 lines of C++. The value is in tuning for your specific detection pipeline and mission profile. Buying a tracking library would add a dependency for something we already have and understand completely.

| Option | License | Cost | Pros | Cons |
|--------|---------|------|------|------|
| **Keep current (ByteTrack)** | Project-original | $0 | Full control, C++ native, zero dependencies, tested, tuned for our detection pipeline, two-stage occlusion recovery | No re-identification (appearance model) |
| **NVIDIA NvDCF/NvSORT (DeepStream)** | Free (with DeepStream) | $0 | GPU-accelerated, production-grade, multi-stream | Requires GStreamer/DeepStream pipeline (major architecture change), vendor lock-in |
| **BoT-SORT / StrongSORT** | Academic | $0 | Camera motion compensation, appearance re-ID, higher MOTA | Python reference impl only, license unclear, requires CNN feature extractor (adds latency) |

**Recommendation:** It is recommended to keep the current implementation. ByteTrack with high/low confidence two-stage association is state-of-the-art for real-time MOT. Appearance-based re-ID should only be added if track fragmentation becomes a problem in field testing.

---

## 23. Sensor Fusion (Multi-Sensor State Estimation)

**What it does:** Fuses data from multiple sensors (camera, IMU, GPS, LiDAR) into a unified state estimate. Currently an original UKF implementation.

**Verdict: MAKE — already done.** The UKF is tailored to our state vector and sensor configuration.

**CTO reasoning:** Sensor fusion is core IP. The state vector, process model, and measurement model are specific to your drone's sensor configuration. Generic fusion libraries (GTSAM, robot_localization) are designed for different state vectors and require significant adaptation. Our UKF is lightweight, real-time, and we understand every line.

| Option | License | Cost | Pros | Cons |
|--------|---------|------|------|------|
| **Keep current (UKF)** | Project-original | $0 | Tailored state vector, real-time, lightweight, fully understood | No graph optimization |
| **GTSAM** | BSD 3-Clause | $0 | Factor graph optimization, incremental (iSAM2), loop closure, proven | Heavy library (~100 MB), learning curve, designed for SLAM backend not real-time filter |
| **Ceres Solver** | BSD 3-Clause | $0 | General nonlinear least-squares, Google-maintained | Batch optimizer (not streaming), not designed for real-time fusion |

**Recommendation:** It is recommended to keep the current UKF. **GTSAM (BSD)** should only be added later if a graph-based SLAM backend is needed for loop closure. The UKF handles real-time streaming fusion perfectly.

---

# Part VIII: Software — Navigation & Localization

## 24. Visual-Inertial Odometry (VIO) / SLAM

**What it does:** Estimates drone position and orientation from stereo camera images and IMU data. Critical for GPS-denied flight. Currently we have IMU pre-integration only — **no visual component in production**.

**Verdict: EVALUATE — the most important technical decision in the stack.**

**CTO reasoning:** VIO is the hardest component to build well. It requires feature extraction, stereo matching, IMU pre-integration, marginalization, and a tightly-coupled optimization backend. Building production-grade VIO from scratch is a 12–18 month effort for a team of 2–3 robotics PhDs. The open-source options are strong. But choosing wrong creates a deep dependency on code you don't fully understand.

**Decision framework:**
- If GPS is always available: skip VIO, use GPS + IMU EKF (what we have via PX4)
- If GPS-denied flight is required: VIO is mandatory, evaluate below
- If BVLOS certification is the goal: VIO + GPS fusion is required (redundant positioning)

| Option | License | Cost | Accuracy (EuRoC ATE) | Jetson Performance | Pros | Cons |
|--------|---------|------|---------------------|-------------------|------|------|
| **Basalt VIO** (TUM) | BSD 3-Clause | $0 | 0.012m (best OSS) | ~300 Hz (claimed) | Best open-source VIO accuracy, permissive license, stereo + IMU, visual-inertial bundle adjustment, actively maintained | Research-grade code quality, limited documentation, integration effort ~4–6 weeks |
| **NVIDIA cuVSLAM** | NVIDIA Community License | $0 (dev), unclear (prod) | <5 cm position error | 232 FPS @ 720p | Fastest VIO (GPU-accelerated), unmatched throughput, Isaac ROS integration | Proprietary binary blob, license unclear for production, NVIDIA lock-in, no source code access |
| **Kimera-VIO** (MIT SPARK Lab) | BSD 2-Clause | $0 | Good (not top-tier) | Tested on older Jetsons | Modular (VIO + meshing + SLAM), permissive license, academic pedigree (MIT), 3D mesh output | Lower accuracy than Basalt, heavier, research-grade |

**GPL options (incompatible with BSD project without commercial license):**

| Option | License | Cost | Notes |
|--------|---------|------|-------|
| **ORB-SLAM3** | GPL 3.0 (commercial available) | Contact Univ. Zaragoza | Gold-standard visual SLAM, monocular/stereo/stereo-inertial, loop closure. Commercial license pricing is opaque (~$10K–50K estimated) |
| **VINS-Fusion** | GPL 3.0 | $0 | Excellent stereo+IMU VIO, GPS fusion, proven on drones. No commercial license option |
| **OpenVINS** | GPL 3.0 | $0 | State-of-the-art monocular VIO, best documentation of any VIO library |

**Commercial SDKs:**

| Option | Vendor | Est. Cost | Notes |
|--------|--------|-----------|-------|
| **Spleenlab visonAIry** | Spleenlab | Custom (est. $20K–50K/yr) | Top drone customers (Quantum Systems, DroneUp), PX4/Auterion partner, Orin-compatible |
| **SLAMcore** | SLAMcore | Per-seat + per-robot/month | RealSense D435i/D455 out-of-box, warehouse/consumer focus |
| **ZED SDK (built-in VIO)** | Stereolabs | Free (with ZED camera, ~$500) | Excellent VIO quality, but locked to ZED cameras |

**Recommendation:** It is recommended to integrate **Basalt VIO (BSD 3-Clause).** It offers the best accuracy in any permissively-licensed VIO. A budget of 4–6 weeks of integration effort should be allocated for the VIO HAL backend. If faster time-to-flight is desired and NVIDIA lock-in is acceptable, **cuVSLAM** is the performance king but the proprietary license is a long-term risk. If a ZED 2i camera is procured, the built-in VIO is an excellent zero-effort option — but that locks the project to Stereolabs hardware.

---

## 25. Path Planning

**What it does:** Computes a collision-free path from the current position to the next waypoint, avoiding known obstacles (HD map) and dynamic obstacles (perception).

**Verdict: MAKE — already done.** A*, D* Lite, and potential field planners are implemented and tested.

**CTO reasoning:** Path planning is core autonomy IP. Our A* and D* Lite run on a 3D occupancy grid that we populate from config (HD map) and perception (dynamic obstacles). This tight integration with our specific IPC types, coordinate frame, and mission planner FSM would be lost by adopting an external library. The algorithms themselves are textbook — the value is in the integration.

| Option | License | Cost | Pros | Cons |
|--------|---------|------|------|------|
| **Keep current (A* + D* Lite + potential field)** | Project-original | $0 | Tight integration, 3D grid, configurable, tested with full stack, D* Lite incremental replanning | No sampling-based planners (RRT*) |
| **OMPL** (Open Motion Planning Library) | BSD 3-Clause (core) | $0 | 50+ planners (RRT*, PRM, BIT*), well-tested, standard in manipulation | Designed for robotic arms, requires adaptation for 3D flight, adds dependency |
| **Nav2** (ROS 2 Navigation) | Apache 2.0 | $0 | Complete navigation stack, behavior trees, Smac planner | Massive ROS 2 dependency, ground-robot focused, overkill |

**Recommendation:** It is recommended to keep the current implementation. If complex 3D environments (indoor, urban canyon) demand sampling-based planning, **OMPL (BSD)** should be added as a library — the full ROS 2 Nav2 stack should not be adopted. The `IPathPlanner` HAL interface already supports swappable backends.

---

## 26. Obstacle Avoidance

**What it does:** Real-time reactive avoidance of obstacles detected during flight. Modifies the planned trajectory to avoid collisions.

**Verdict: MAKE — already done.** Potential field (2D + 3D) and integration with the path planner.

**CTO reasoning:** Obstacle avoidance is tightly coupled to your path planner, perception pipeline, and drone dynamics. It's 200–500 lines of reactive control logic. The value is in tuning for your specific platform's dynamics (stopping distance, turn rate, sensor FOV).

| Option | License | Cost | Pros | Cons |
|--------|---------|------|------|------|
| **Keep current (potential field 2D/3D)** | Project-original | $0 | Integrated with planner, configurable influence radius, tested in Gazebo SITL, 3D vertical avoidance | Simple model — may need VFH+ for cluttered environments |
| **PX4 Avoidance (local_planner)** | BSD 3-Clause | $0 | Stereo-camera based, PX4-native, RealSense D435 integration | Requires ROS 2, discontinued (minimal maintenance since 2022), depth-camera only |
| **VFH+ (Vector Field Histogram)** | Academic (implement from paper) | 2–3 weeks eng | Better than potential field in cluttered environments, proven on ground robots | Must implement from paper, no production C++ implementation available |

**Recommendation:** It is recommended to keep the current implementation for open-air operations. **VFH+** should be implemented (from paper, ~2–3 weeks) when obstacle avoidance in cluttered environments (indoor, forest) is needed. This is a "make" — the algorithm is well-documented but no production C++ library exists.

---

# Part IX: Software — Mission & Control

## 27. Mission Planner / Finite State Machine

**What it does:** The central decision-maker. Manages the mission lifecycle (IDLE → PREFLIGHT → TAKEOFF → NAVIGATE → LOITER → RTL → LAND → EMERGENCY), processes faults, executes contingency actions, handles GCS commands.

**Verdict: MAKE — this is your core product.**

**CTO reasoning:** The mission planner is the most differentiating component in the stack. It's where all sensor data converges, all decisions are made, and all safety logic lives. Buying a mission planner means you're reselling someone else's drone. Every drone company's competitive advantage lives in their mission planner + fault management logic.

| Option | License | Cost | Pros | Cons |
|--------|---------|------|------|------|
| **Keep current (custom FSM + FaultManager)** | Project-original | $0 | Full control over safety logic, tight integration with all subsystems, configurable fault escalation, geofencing, GCS command handling | Single-threaded, no behavior tree (pure FSM) |
| **BehaviorTree.CPP** | MIT | $0 | Behavior tree framework (more flexible than FSM for complex behaviors), visual editor, used in ROS 2 Nav2 | Adds dependency, learning curve, may be over-engineered for current mission complexity |
| **Auterion Mission Computer SDK** | Commercial | Custom pricing | Full autonomy framework, PX4-native, defense-grade, BVLOS-certified missions | Complete vendor lock-in, Skynode hardware required, you lose all control |

**Recommendation:** It is recommended to keep the current FSM. Adding **BehaviorTree.CPP (MIT)** as a library should be considered when mission complexity grows (multi-phase missions, conditional behaviors, parallel tasks). The FSM is sufficient for waypoint navigation + fault management.

---

## 28. Geofencing

**What it does:** Enforces spatial boundaries (polygon + altitude floor/ceiling). Triggers warnings and contingency actions (LOITER → RTL) when the drone approaches or breaches boundaries.

**Verdict: MAKE — already done.** The geofence implementation is integrated with the mission planner's fault system.

**CTO reasoning:** Geofencing is 200–300 lines of point-in-polygon + altitude checks. It's tightly coupled to your fault management system. There's no advantage to buying it.

| Option | License | Cost | Notes |
|--------|---------|------|-------|
| **Keep current** | Project-original | $0 | Polygon + altitude, integrated with FaultManager, configurable warning margin |
| **PX4 built-in geofence** | BSD 3-Clause | $0 | PX4's own geofence (on FC). Use as a backup layer — if companion computer fails, PX4 still enforces geofence |
| **Airmap SDK** | Commercial | Contact sales | Dynamic geofencing from airspace databases (LAANC). Different from static geofence — adds regulatory awareness |

**Recommendation:** It is recommended to keep the current implementation and enable **PX4's built-in geofence** as a defense-in-depth backup (so even if the companion computer crashes, the FC enforces boundaries). Airmap should be added when LAANC/UTM integration is needed.

---

# Part X: Software — Reliability & Monitoring

## 29. System Monitoring / Watchdog

**What it does:** Three-layer watchdog (thread → process → OS), health monitoring (CPU, memory, temperature, disk), process supervision with exponential backoff restart.

**Verdict: MAKE — already done.** This is purpose-built and has no equivalent to buy.

**CTO reasoning:** The three-layer watchdog architecture (ThreadHeartbeat + ProcessManager + systemd) is unique to our stack's multi-process architecture. No commercial product monitors 21 threads across 7 processes with the same granularity. This is core reliability engineering.

| Option | License | Cost | Notes |
|--------|---------|------|-------|
| **Keep current** | Project-original | $0 | ThreadHeartbeat (~1ns), ThreadWatchdog, ProcessManager (fork+exec, dependency graph, exponential backoff), systemd service units |
| **Prometheus + Grafana** | Apache 2.0 / AGPL | $0 (self-hosted) | Industry-standard monitoring. Use as a *complement* for fleet-level dashboards, not a replacement for on-board watchdog |
| **systemd alone** | LGPL | $0 | Already used as the OS layer. Could replace ProcessManager, but loses thermal gating, cascade restarts, and dependency-graph ordering |

**Recommendation:** It is recommended to keep the current implementation. The on-board watchdog is irreplaceable. Prometheus/Grafana should be added for fleet-level observability when multiple drones are in operation.

---

## 30. Fault Injection & Testing

**What it does:** Injects faults (battery drain, FC link loss, thermal throttle, VIO degradation) into the running stack via IPC to validate fault response logic.

**Verdict: MAKE — already done.** The `fault_injector` CLI tool publishes `FaultOverrides` over Zenoh.

**CTO reasoning:** Fault injection is specific to your fault taxonomy and IPC types. Generic fault injection tools (Chaos Monkey, LitmusChaos) are designed for cloud microservices, not real-time drone systems. Our tool is 200 lines of C++ and exercises every fault path in the stack.

---

# Part XI: Simulation & Testing

## 31. Simulation Environment

**What it does:** Full closed-loop simulation of the drone, sensors, and environment. Enables testing the entire autonomy stack without flying.

**Verdict: BUY (open-source) — already done.** Gazebo Harmonic + PX4 SITL is the PX4 ecosystem standard.

| Option | License | Cost | Physics | Graphics | PX4 Integration | Pros | Cons |
|--------|---------|------|---------|----------|----------------|------|------|
| **Gazebo Harmonic + PX4 SITL** (current) | Apache 2.0 + BSD | $0 | ODE/Bullet | Moderate (Ogre2) | Native (built-in) | Tightest PX4 integration, full sensor plugins (camera, IMU, GPS, LiDAR), free, largest drone sim community, gz-transport for our Gazebo HAL backends | Lower visual fidelity than Unreal/Unity, learning curve for SDF models |
| **NVIDIA Isaac Sim** | Free dev; Omniverse Enterprise $4,500/yr | $0–4,500/yr | PhysX 5 | Photorealistic (RTX) | Via ROS 2 bridge | Photorealistic rendering (synthetic data generation), domain randomization for training, USD scene format, GPU-accelerated physics | Not drone-focused, requires beefy GPU (RTX 3090+), ROS 2 dependency, steep learning curve |
| **AirSim → Colosseum (community fork)** | MIT | $0 | Unreal Engine physics | Photorealistic (Unreal 5) | MAVLink (PX4/ArduPilot) | Best visual fidelity for drone sim, Unreal Engine 5, weather effects, MAVLink native | Microsoft archived AirSim (2022), Colosseum fork has limited maintenance, UE5 is heavyweight |

**Recommendation:** It is recommended to keep **Gazebo + PX4 SITL** as the primary simulation. **Isaac Sim** should be considered as a complement for synthetic training data generation (photorealistic images to train detection models). Gazebo should not be replaced — its PX4 SITL integration is unmatched.

---

## 32. CI/CD & Build System

**What it does:** Continuous integration pipeline — format checking, multi-config builds, sanitizers, coverage.

**Verdict: MAKE (pipeline config) + BUY (infrastructure) — already done.** GitHub Actions for CI, CMake for build.

| Component | Choice | License | Cost | Notes |
|-----------|--------|---------|------|-------|
| **CI** | GitHub Actions | Proprietary | Free (public repos) / $0.008/min (private) | 5-job pipeline: format + build + ASan + TSan + UBSan + coverage |
| **Build** | CMake 3.16+ | BSD 3-Clause | $0 | Industry standard for C++ |
| **Sanitizers** | ASan/TSan/UBSan (Clang/GCC) | Open source | $0 | Built into the compiler |
| **Coverage** | lcov + gcov | GPL 2.0 | $0 | Standard C++ coverage tools |
| **Formatting** | clang-format-18 | Apache 2.0 | $0 | CI-enforced |
| **Static analysis** | clang-tidy-18 | Apache 2.0 | $0 | CI-enforced |

**Recommendation:** It is recommended to keep the current CI pipeline configuration. No changes needed to tooling.

### CI/CD Infrastructure Costs

The table above covers *tooling* (all free/open-source). But running CI/CD has real infrastructure costs that scale with build frequency, test suite size, and hardware-in-the-loop (HIL) requirements. These costs are often overlooked in make-or-buy analyses.

**Cost drivers:**
- **GitHub Actions minutes:** Free for public repos; $0.008/min for private repo Linux runners (post-Jan 2026). Our 5-job pipeline (format + build + ASan + TSan + UBSan + coverage) takes ~25 min per push.
- **Self-hosted runners:** $0.002/min platform charge (March 2026+) + hardware cost. Required for Jetson cross-compilation and GPU-accelerated tests.
- **Nightly test runs:** Full sanitizer + coverage suite running nightly on main, even without pushes.
- **Jetson HIL testing:** Running tests on real Jetson Orin hardware requires a dedicated lab machine (~$850 one-time + power/network).

| Scenario | Description | Est. Annual Cost |
|----------|-------------|-----------------|
| **A: Minimal (startup)** | GitHub Actions hosted runners only, ~50 pushes/month, no nightly, no HIL | ~$768/yr ($0.008/min × 25 min × 50 pushes × 12 months + overhead) |
| **B: Medium (active dev)** | Hosted runners + 1 self-hosted Jetson runner, nightly builds, ~100 pushes/month | ~$2,256/yr (hosted: ~$1,440 + self-hosted platform: ~$216 + Jetson hardware amortized: ~$300 + network/power: ~$300) |
| **C: Full (production)** | Hosted + 2 self-hosted runners (x86 + Jetson), nightly sanitizers, GPU instance for TensorRT CI, HIL tests | ~$2,652/yr (hosted: ~$1,440 + self-hosted: ~$432 + GPU cloud instance ~$480/yr spot + Jetson lab: ~$300) |

**Note:** These estimates assume private repositories. Public repositories get 2,000 free GitHub Actions minutes/month, which would reduce Scenario A to near-zero.

**Recommendation:** It is recommended to start with Scenario A (hosted runners only) and migrate to Scenario B when the team exceeds 3 developers or when Jetson-specific CI becomes necessary. Budget $2,000–3,000/yr for CI/CD infrastructure in production planning.

---

## 32b. Software Licensing & Compliance

**What it does:** Ensures all third-party software dependencies are legally cleared for commercial distribution, identifies hidden production costs, and manages license obligations.

**Verdict: EVALUATE — most dependencies are genuinely free, but several have production gotchas.**

**CTO reasoning:** Open-source does not always mean free-for-commercial-use. License terms, contributor agreements, and vendor EULAs can impose obligations that cost money, restrict distribution, or require legal review. This section audits every software dependency in the stack for production readiness.

### License Audit Summary

| Dependency | License | Production Cost | Gotcha |
|------------|---------|----------------|--------|
| **Eclipse Zenoh** | EPL 2.0 / Apache 2.0 (dual) | $0 | **Must explicitly choose Apache 2.0.** EPL 2.0 has weak copyleft — modifications to Zenoh itself must be published. Apache 2.0 has no such obligation. Set license choice in procurement/legal records. Optional commercial support from ZettaScale: ~$20K–100K+/yr |
| **PX4 Autopilot** | BSD 3-Clause | $0 | No restrictions. Attribution required |
| **MAVSDK** | BSD 3-Clause | $0 | No restrictions |
| **Eigen3** | MPL 2.0 | $0 | **Define `EIGEN_MPL2_ONLY`** in build to exclude LGPL/GPL-licensed modules (some advanced decompositions). MPL 2.0 is file-level copyleft — modifications to Eigen source files must be published, but your own code is unaffected |
| **spdlog** | MIT | $0 | No restrictions |
| **nlohmann/json** | MIT | $0 | No restrictions |
| **Google Test** | BSD 3-Clause | $0 | No restrictions (test-only, not distributed) |
| **Gazebo Harmonic** | Apache 2.0 | $0 | No restrictions (dev/test only, not distributed) |
| **OpenCV** | Apache 2.0 | $0 | Some modules (face, SIFT pre-2020) had patent issues — resolved. DNN module is clean |
| **NVIDIA TensorRT** | NVIDIA EULA | $0 (on Jetson) | **Hardware-locked:** free on Jetson via JetPack, but EULA prohibits use on non-NVIDIA hardware. Contains IP grant-back clause — NVIDIA gets a license to any patents you assert against them. Legal review recommended before production deployment |
| **NVIDIA JetPack / CUDA** | NVIDIA EULA | $0 (on Jetson) | Same hardware lock-in as TensorRT. CUDA code cannot be deployed on AMD/Intel GPUs |
| **Linux kernel** | GPL 2.0 | $0 | **Must provide kernel source** to customers receiving the drone (or offer written source access for 3 years). Standard GPL compliance — most SBC vendors (NVIDIA, Qualcomm) provide BSP source already |
| **Ubuntu** | Mixed (mostly GPL/MIT) | $0 | Canonical's trademark policy: cannot distribute modified Ubuntu and call it "Ubuntu" without permission. Use a custom image name |
| **QGroundControl** | Apache 2.0 (core) / GPL 3.0 (some plugins) | $0 | If modifying and redistributing QGC, **Qt commercial license** may be required (~$4,000/dev/yr). Using unmodified QGC is fine |
| **RT-DETR v2** (recommended model) | Apache 2.0 | $0 | No restrictions. Weights are Apache 2.0 |
| **Basalt VIO** (recommended) | BSD 3-Clause | $0 | No restrictions |
| **Mender.io** | Apache 2.0 (self-hosted) | $0 self-hosted / $2,500/yr hosted | Enterprise features (RBAC, monitoring) require paid plan |

### Optional Commercial Support Contracts

These are not required but may be valuable for production deployments:

| Vendor | What You Get | Est. Annual Cost |
|--------|-------------|-----------------|
| **ZettaScale (Zenoh)** | Priority support, SLA, custom features, deployment guidance | $20,000–100,000+/yr (depends on scope) |
| **NVIDIA AI Enterprise** | Enterprise TensorRT support, training, certified containers | $4,500/yr per GPU |
| **Auterion (PX4)** | Commercial PX4 support, fleet management, OTA, certification assistance | Per-unit licensing (contact sales) |
| **Northern.tech (Mender)** | Hosted OTA, monitoring, RBAC, SLA | $2,500–10,000/yr |

### Updated Annual Software Cost Summary

| Component | Recommended | Annual Cost | Notes |
|-----------|-------------|-------------|-------|
| OS / Firmware | Ubuntu + PX4 | $0 | GPL compliance required for kernel |
| IPC Middleware | Zenoh (Apache 2.0) | $0 | Must select Apache 2.0 explicitly |
| FC SDK | MAVSDK (BSD) | $0 | |
| Detection Model | RT-DETR v2 (Apache 2.0) | $0 | |
| Inference Engine | TensorRT (NVIDIA EULA) | $0 | Hardware-locked to NVIDIA, IP grant-back |
| VIO | Basalt VIO (BSD) | $0 | |
| Simulation | Gazebo + PX4 SITL | $0 | |
| GCS | QGroundControl | $0 | Qt license if modifying/redistributing |
| Libraries | spdlog, Eigen3, nlohmann/json, GTest | $0 | Define EIGEN_MPL2_ONLY |
| CI/CD Infrastructure | GitHub Actions + self-hosted | $768–2,652/yr | See CI/CD cost scenarios above |
| Fleet Management | FlytBase (when needed) | $999/yr | |
| OTA Updates | Mender.io (self-hosted) | $0 | |
| **Total (no support contracts)** | | **$768–3,651/yr** | |
| **Total (with Zenoh support)** | | **$20,768–103,651/yr** | Only if commercial SLA needed |

**Recommendation:** It is recommended to conduct a formal license audit before first commercial shipment. The immediate actions are: (1) set `EIGEN_MPL2_ONLY` in CMakeLists.txt, (2) document the Apache 2.0 license selection for Zenoh in procurement records, (3) have legal review the NVIDIA TensorRT EULA IP grant-back clause, and (4) establish a GPL compliance process for Linux kernel source distribution. Budget $768–2,652/yr for CI/CD infrastructure and $0 for software licensing in the base case, with optional support contracts adding $20K–100K+/yr if commercial SLAs are required.

---

# Part XII: Ground Segment

## 33. Ground Control Station (GCS)

**What it does:** Operator interface for mission planning, real-time monitoring, and command/control. Displays map, telemetry, video feed, and alerts.

**Verdict: BUY (open-source) for V1, MAKE for custom features later.**

**CTO reasoning:** QGroundControl is the PX4-native GCS and handles 90% of operational needs (mission planning, telemetry, parameters, firmware update). Building a custom GCS is a separate product — 6–12 months of Qt/web development. Only invest in custom GCS when QGC's limitations become a blocker for your specific mission profile.

| Option | License | Cost | Platform | Pros | Cons |
|--------|---------|------|----------|------|------|
| **QGroundControl (QGC)** | Apache 2.0 / GPL 3.0 | $0 | Windows, Mac, Linux, Android, iOS | PX4-native, full mission planning, parameter editing, firmware update, video streaming, MAVLink log analysis, open-source | UI is dated, limited customization without forking, no fleet management, single-vehicle focused |
| **Auterion Suite** | Commercial SaaS | Free tier → paid per-vehicle/yr | Web + mobile | Modern web UI, fleet management, analytics, OTA updates, PX4-native, defense-grade | Requires Auterion ecosystem, vendor lock-in, pricing scales with fleet size |
| **FlytBase** | Commercial SaaS | $999/yr per org | Web + mobile + API | Hardware-agnostic (DJI, PX4, ArduPilot), fleet management, DJI Dock integration, REST API, white-label option | Monthly subscription, less PX4-native than QGC, cloud-dependent |

**Recommendation:** It is recommended to use **QGroundControl** for development and single-vehicle operations. **Auterion Suite** or **FlytBase** should be evaluated when fleet management is needed. The stack's GCS link (`IGCSLink`) is independent of the GCS software — any MAVLink-compatible GCS works.

---

## 34. Fleet Management

**What it does:** Manages multiple drones: mission dispatch, health monitoring, geofencing, regulatory compliance, data management.

**Verdict: BUY (when needed)** — Building fleet management is a separate product. Buy when you have >5 drones.

| Option | Vendor | Price | Pros | Cons |
|--------|--------|-------|------|------|
| **Auterion Suite** | Auterion | Free tier → paid per-vehicle/yr | PX4-native, defense-grade, OTA, analytics, used by US Army | Requires Auterion ecosystem |
| **FlytBase** | FlytBase | $999/yr per org | Hardware-agnostic, REST API, DJI Dock support | Cloud-dependent |
| **Custom (Zenoh + web dashboard)** | In-house | 3–6 months eng | Full control, Zenoh network transport is already multi-node capable | Significant engineering investment, not core business |

**Recommendation:** It is recommended to defer fleet management procurement. QGroundControl should be used for single-vehicle operations. **FlytBase** ($999/yr, hardware-agnostic) should be evaluated when fleet operations begin.

---

# Part XIII: Compliance & Security

## 35. Remote ID

**What it does:** Broadcasts drone identification and location over Bluetooth/WiFi per FAA/EASA regulation. **Required by law** for all drones >250g in the US (since March 2024) and EU.

**Verdict: BUY** — Remote ID is a regulatory requirement with specific RF broadcast standards. Buy a certified module.

| Option | Vendor | Price | Standard | Interface | Pros | Cons |
|--------|--------|-------|----------|-----------|------|------|
| **Cube ID** | CubePilot | ~$50 | FAA Remote ID (ASTM F3411) | CAN / Serial | Integrated with CubePilot ecosystem, small, FAA-certified, supports Bluetooth 5 + WiFi NaN | CubePilot ecosystem, limited standalone documentation |
| **Dronetag Mini** | Dronetag | ~$150 | FAA + EU Remote ID | Bluetooth / UART | Universal (works with any drone), FAA + EU dual-certified, app for monitoring, over-the-air firmware updates | More expensive than Cube ID, separate hardware |
| **PX4 built-in Remote ID** | PX4 (firmware) | $0 (software) | ASTM F3411 (if FC has BT/WiFi) | Via FC's Bluetooth | Free if your FC has Bluetooth hardware, no additional module needed | Not all FCs have Bluetooth, less reliable than dedicated module |

**Recommendation:** It is recommended to select **Cube ID** ($50) if using CubePilot hardware, or **Dronetag Mini** ($150) for universal compatibility. PX4's built-in Remote ID should also be enabled as a backup (belt and suspenders — regulatory compliance is not optional).

---

## 36. UTM / Airspace Management

**What it does:** Integration with Unmanned Traffic Management systems for automated airspace authorization (LAANC in the US), flight plan filing, and deconfliction.

**Verdict: BUY (when needed)** — UTM is a regulatory service, not software you build.

| Option | Vendor | Price | Region | Pros | Cons |
|--------|--------|-------|--------|------|------|
| **Airmap** | Airmap | Free API (basic) → enterprise pricing | US (LAANC), EU | FAA LAANC-authorized, airspace maps, automated authorization, API + SDK | Company has had financial instability, API can be slow |
| **Aloft (fmr. Kittyhawk)** | Aloft (Versaterm) | Contact sales | US (LAANC), EU | LAANC-authorized, compliance focus, acquired by Versaterm (public safety), flight logging | Less developer-friendly API than Airmap |
| **Wing UTM (Google)** | Wing (Alphabet) | Free (OpenSky) | US, Australia | Google-backed, OpenSky standard, modern API | Limited coverage, focused on Wing's delivery operations |

**Recommendation:** It is recommended to defer until BVLOS operations require LAANC authorization. **Airmap** (largest API) or **Aloft** (compliance-focused) should then be evaluated.

---

## 37. Cybersecurity

**What it does:** Secures the drone against data interception, command injection, GPS spoofing, and firmware tampering.

**Verdict: MAKE (most of it) — security is architecture, not a product you buy.**

**CTO reasoning:** Drone cybersecurity is primarily about architectural decisions: encrypted IPC (Zenoh TLS), authenticated MAVLink (MAVLink 2 signing), secure boot (Jetson Secure Boot), encrypted storage, access control. These are configuration and integration tasks, not separate products.

| Layer | Current State | Action | Cost |
|-------|--------------|--------|------|
| **IPC encryption** | Zenoh TLS support (disabled in dev, `ALLOW_INSECURE_ZENOH=ON`) | Enable TLS in production config, generate certificates | $0 |
| **MAVLink signing** | MAVLink 2 supports message signing | Enable in MAVSDK config | $0 |
| **Secure boot** | Jetson Orin supports Secure Boot via JetPack | Enable in production deployment | $0 |
| **OTA updates** | Not implemented | Build or buy (Mender.io — Apache 2.0, $0 self-hosted) | $0–2,500/yr (Mender hosted) |
| **GPS spoofing detection** | Not implemented | Implement IMU/GPS consistency check in sensor fusion | 2–3 weeks eng |
| **Intrusion detection** | Not implemented | Evaluate drone-specific IDS (academic, limited commercial options) | Research phase |

| Option | Vendor | Price | Notes |
|--------|--------|-------|-------|
| **Mender.io (OTA updates)** | Northern.tech | $0 (self-hosted) / $2,500/yr (hosted) | Open-source OTA update framework, Yocto/Debian support, rollback, A/B partitioning |
| **SDLE SecureDrone** | Shield Defence & Logistics | Custom pricing | Drone-specific encryption, anti-jamming, counter-UAS integration. Defense-focused |
| **DroneShield Counter-UAS** | DroneShield | Custom pricing | Detection + mitigation, but focuses on *defending against* hostile drones, not securing your own |

**Recommendation:** It is recommended to enable Zenoh TLS, MAVLink signing, and Jetson Secure Boot in production configs ($0 cost). GPS spoofing detection should be implemented in-house (2–3 weeks). **Mender.io** should be added for OTA updates when deploying to the field. Drone-specific security products are immature and defense-focused — not needed for commercial operations.

---

# Part XIV: Additional Components

## 38. Data Recording / Black Box

**What it does:** Records all sensor data, IPC messages, and decisions for post-flight analysis, debugging, and regulatory compliance.

**Verdict: MAKE** — Tight integration with our IPC types. PX4's ULog handles FC-side logging.

| Option | Notes |
|--------|-------|
| **PX4 ULog** (FC-side) | Already recording FC data (IMU, GPS, attitude, battery) on the SD card. Free, automatic |
| **Custom IPC recorder** (companion-side) | Subscribe to all Zenoh topics, write to disk with timestamps. ~1–2 weeks eng. Enables full replay |
| **ROS 2 rosbag** | Standard recording format, but requires ROS 2 dependency. Not appropriate for our non-ROS stack |

**Recommendation:** PX4 ULog handles FC-side logging. It is recommended to build a simple Zenoh topic recorder for companion-side data (~1–2 weeks). This enables full mission replay for debugging.

---

## 39. OTA (Over-the-Air) Update System

**What it does:** Remotely updates drone firmware and companion software in the field without physical access.

**Verdict: BUY** — OTA is a solved problem. Don't build your own update system.

| Option | Vendor | License | Cost | Pros | Cons |
|--------|--------|---------|------|------|------|
| **Mender.io** | Northern.tech | Apache 2.0 (self-hosted) / Commercial (hosted) | $0–2,500/yr | Open-source, A/B partition rollback, Yocto/Debian, artifact signing, fleet management | Self-hosted requires server infrastructure |
| **SWUpdate** | Community | GPL 2.0 | $0 | Lightweight, Yocto-native, dual-copy/rescue modes, Lua scripting | GPL license, less polished UI, no hosted service |
| **Balena** | Balena | Free (≤10 devices) / $1.15/device/mo | Container-based updates, Docker workflow, dashboard, fleet management | Container overhead on embedded, less suitable for bare-metal C++ |

**Recommendation:** It is recommended to use **Mender.io** (Apache 2.0 self-hosted). A/B partition rollback is critical for drones — a bad update must never brick a drone mid-field.

---

## 40. Mapping / Photogrammetry (Post-Processing)

**What it does:** Generates orthomosaics, 3D models, and point clouds from drone imagery. Used in surveying, agriculture, construction.

**Verdict: BUY (when needed)** — This is a downstream application, not part of the autonomy stack.

| Option | Vendor | License | Cost | Pros | Cons |
|--------|--------|---------|------|------|------|
| **OpenDroneMap (ODM)** | Community | AGPL 3.0 | $0 (self-hosted) | Open-source, full photogrammetry pipeline, Docker deployment, WebODM UI | AGPL (copyleft), requires significant compute, slower than commercial |
| **Pix4Dmapper** | Pix4D | Commercial | ~$4,500/yr (perpetual ~$8,700) | Industry standard, RTK support, excellent quality, cloud processing option | Expensive, desktop-only processing |
| **DroneDeploy** | DroneDeploy | Commercial | ~$4,188/yr (Business) | Cloud-based, easy, integrates with DJI, analytics, AI defect detection | Cloud-dependent, expensive, less control |

**Recommendation:** It is recommended to defer mapping procurement. Mapping is a mission-specific application. If needed, **OpenDroneMap** ($0) should be the starting point, with an upgrade to **Pix4D** for professional surveying.

---

## 41. AI Training Pipeline / MLOps

**What it does:** Manages the lifecycle of AI models — data collection, labeling, training, evaluation, deployment to edge.

**Verdict: BUY (when needed)** — Only relevant when you start training custom detection models.

| Option | Vendor | Cost | Pros | Cons |
|--------|--------|------|------|------|
| **Roboflow** | Roboflow | Free tier → $249/mo (Pro) | Labeling + training + deployment, exports to TensorRT/ONNX, hosted training, dataset management | Cloud-dependent for training, AGPL for some models |
| **NVIDIA TAO Toolkit** | NVIDIA | $0 (on Jetson) | Transfer learning, INT8 quantization, TensorRT-native, 100+ pre-trained models | NVIDIA ecosystem only, steeper learning curve |
| **Label Studio + W&B** | Community / Weights & Biases | $0 (OSS) / $0–50/mo | Open-source labeling, experiment tracking, model registry, flexible | More DIY, requires ML engineering expertise |

**Recommendation:** It is recommended to defer until custom model training is needed. **Roboflow** should then be used for rapid prototyping or **NVIDIA TAO** for production TensorRT deployment.

---

## 42. Anti-Collision Lights / Strobes

**What it does:** FAA-required anti-collision lighting for night operations and visibility.

**Verdict: BUY** — Regulatory requirement, commodity hardware.

| Option | Vendor | Price | Notes |
|--------|--------|-------|-------|
| **Lume Cube Strobe** | Lume Cube | ~$30 | FAA Part 107.29 compliant, 3 statute miles visibility, 6g, USB-C rechargeable |
| **Firehouse Technology Arc V** | Firehouse Technology | ~$40 | FAA compliant, multiple colors, visible 5+ miles, lightweight |
| **uAvionix Ping200X (combo ADS-B + strobe)** | uAvionix | ~$300 | ADS-B Out + anti-collision strobe in one unit. Useful for BVLOS |

**Recommendation:** It is recommended to use the **Lume Cube Strobe** ($30) for basic compliance. An upgrade to the **uAvionix Ping200X** is recommended if ADS-B Out is needed for BVLOS operations.

---

## 43. Parachute / Recovery System

**What it does:** Deploys a parachute to protect people and property in case of catastrophic failure. **Required by some regulators** for flights over people (FAA Part 107.39 waiver).

**Verdict: BUY** — Safety-critical hardware with certification requirements.

| Option | Vendor | Price | Drone Weight Class | Deploy Time | Pros | Cons |
|--------|--------|-------|-------------------|-------------|------|------|
| **ParaZero SafeAir** | ParaZero | ~$500–1,500 | 1–30 kg | <1 second | FAA-accepted (ASTM F3322), autonomous deployment (detects free-fall), MAVLink integration, SmartAir trigger | Adds weight (150–400g), expensive for small drones |
| **MARS Parachute (DJI)** | DJI / third-party | ~$200–400 | 1–7 kg | ~1 second | Affordable, lightweight, spring-loaded | Less certification than ParaZero, manual trigger on some models |
| **Indemnis Nexus** | Indemnis | ~$800 | 3–15 kg | ~0.5 seconds | FAA-accepted, fastest deploy, drag-based safe landing speed calculation | Expensive, heavier than ParaZero for same weight class |

**Recommendation:** It is recommended to procure **ParaZero SafeAir** when flying over people or for BVLOS certification. MAVLink integration means the stack can trigger deployment via `FCCommand`. It is not needed for development/testing over open fields.

---

## 44. Digital Twin

**What it does:** A real-time virtual replica of the physical drone, mirroring sensor data, state, and environment for remote monitoring, predictive maintenance, and simulation replay.

**Verdict: MAKE (lightweight) / BUY (full platform) — future.**

**CTO reasoning:** A basic digital twin is just our Gazebo simulation consuming real telemetry instead of simulated data. Full enterprise digital twins (3D visualization, predictive maintenance, fleet analytics) are a separate product category.

| Option | Vendor | Cost | Notes |
|--------|--------|------|-------|
| **Gazebo + real telemetry replay** | In-house | ~2 weeks eng | Feed recorded Zenoh topics into Gazebo visualization. Lightweight, uses existing infra |
| **NVIDIA Omniverse** | NVIDIA | $0 (dev) / $4,500/yr (enterprise) | Full 3D digital twin platform, photorealistic, USD format, real-time sync |
| **Azure Digital Twins** | Microsoft | Pay-per-use (~$0.001/message) | Cloud-based, IoT integration, graph-based modeling, fleet-scale |

**Recommendation:** It is recommended to defer digital twin procurement. A lightweight replay tool using Gazebo + recorded Zenoh data should be built when post-flight analysis becomes a regular workflow.

---

## 45. Edge AI Accelerator (Co-Processor)

**What it does:** Dedicated AI inference hardware to offload the Jetson GPU for other tasks (VIO, image processing). Or to add AI capability to a lower-power companion computer.

**Verdict: BUY (when needed)** — Only if the Jetson GPU becomes a bottleneck running both VIO and detection simultaneously.

| Option | Vendor | Price | Performance | Power | Pros | Cons |
|--------|--------|-------|-------------|-------|------|------|
| **Hailo-8 M.2** | Hailo | ~$180 | 26 TOPS | 2.5W | Ultra-efficient (10.4 TOPS/W), M.2 form factor, drone-friendly, SDK supports YOLO/DETR | Requires Hailo runtime (new toolchain), not CUDA-compatible |
| **Google Coral M.2** | Google | ~$30–75 | 4 TOPS | 2W | Cheapest AI accelerator, TFLite delegate, tiny | Only 4 TOPS (too weak for VGA detection), INT8 only, limited model support |
| **Intel Movidius Myriad X** (in OAK-D) | Intel / Luxonis | ~$300 (OAK-D) | 4 TOPS | 2.5W | Integrated with camera (OAK-D), OpenVINO | Discontinued Intel VPU line (2023), limited to OAK-D hardware |

**Recommendation:** The Jetson Orin NX (100 TOPS) can handle both detection and VIO simultaneously. An edge AI co-processor is **not needed** unless a lower-power companion computer is selected. If so, **Hailo-8 M.2** at 26 TOPS / 2.5W is the recommended option.

---

# Appendix A: Full Bill of Materials

## Per-Drone Hardware Cost (Recommended Configuration)

| # | Component | Recommended Option | Unit Cost |
|---|-----------|-------------------|-----------|
| 1 | Companion Computer | NVIDIA Jetson Orin NX 16GB + carrier | ~$850 |
| 2 | Flight Controller | Holybro Pixhawk 6X | ~$300 |
| 3 | Airframe | Holybro X500 V2 | ~$320 |
| 4 | Mission Camera | Arducam IMX477 (dev) / Sony IMX296 (prod) | ~$50–300 |
| 5 | Stereo Camera | Intel RealSense D455 | ~$350 |
| 6 | GPS Module | u-blox M9N (dev) / F9P RTK (prod) | ~$50–300 |
| 7 | Telemetry Radio | Holybro SiK V3 (pair) | ~$60 |
| 8 | Gimbal | SIYI A8 Mini | ~$260 |
| 9 | Motors + ESCs (4×) | Holybro kit (dev) / T-Motor U5 (prod) | ~$100–400 |
| 10 | Battery | Tattu 6S 5200 mAh | ~$90 |
| 11 | Power Module | Holybro PM02 V3 | ~$35 |
| 12 | Remote ID | Cube ID | ~$50 |
| 13 | Anti-Collision Light | Lume Cube Strobe | ~$30 |
| | **Development Total** | | **~$2,545** |
| | **Production Total** (upgraded sensors) | | **~$5,000–7,000** |

### Optional Add-Ons

| Component | Option | Unit Cost | When Needed |
|-----------|--------|-----------|-------------|
| Thermal Camera | FLIR Boson 640 | ~$3,500 | Search & rescue, inspection |
| LiDAR | Livox Mid-360 | ~$600 | BVLOS, GPS-denied, mapping |
| Video Datalink | Herelink | ~$500 | Live video downlink |
| RTK Base Station | u-blox F9P base + antenna | ~$400 | Precision operations |
| Parachute | ParaZero SafeAir | ~$500–1,500 | Flights over people |
| Mesh Radio | Doodle Labs Mesh Rider | ~$1,500/node | Multi-drone operations |

---

## Annual Software Cost Summary

| Component | Recommended | Annual Cost |
|-----------|-------------|-------------|
| OS / Firmware | Ubuntu + PX4 | $0 |
| IPC Middleware | Zenoh (Apache 2.0) | $0 |
| FC SDK | MAVSDK (BSD) | $0 |
| Detection Model | RT-DETR v2 (Apache 2.0) | $0 |
| VIO | Basalt VIO (BSD) | $0 |
| Simulation | Gazebo + PX4 SITL | $0 |
| GCS | QGroundControl | $0 |
| Libraries | spdlog, Eigen3, nlohmann/json, GTest | $0 |
| CI/CD Infrastructure | GitHub Actions + self-hosted runners | $768–2,652 |
| Fleet Management | FlytBase (when needed) | $999 |
| OTA Updates | Mender.io (self-hosted) | $0 |
| **Total** | | **$768–3,651/yr** |

### With Optional Commercial Support Contracts

For production fleets requiring SLA-backed vendor support, the following contracts may apply:

| Vendor | Product | Annual Cost | When Needed |
|--------|---------|-------------|-------------|
| ZettaScale | Zenoh Commercial Support | $20,000–100,000+ | Production SLA for IPC middleware bugs/patches |
| NVIDIA | AI Enterprise (TensorRT support) | $4,500/GPU/yr | Priority DNN inference support on Jetson |
| Auterion | PX4 Enterprise Support | $10,000–50,000 | Flight controller firmware SLA |
| Dronecode | PX4 Contributor License | $5,000–15,000 | Upstream influence + priority review |

| Scenario | Base Cost | + Support Contracts | Total |
|----------|-----------|---------------------|-------|
| **Minimal** (community-only) | $768–3,651/yr | $0 | **$768–3,651/yr** |
| **Production** (Zenoh SLA only) | $768–3,651/yr | $20,000–100,000 | **$20,768–103,651/yr** |
| **Full Enterprise** (all vendors) | $768–3,651/yr | $39,500–169,500 | **$40,268–173,151/yr** |

*Most startups operate at the "Minimal" tier through first flight and early customer deployments. Commercial support contracts are typically justified only after fleet scale exceeds ~10 drones or when contractual SLAs with end-customers demand vendor-backed uptime guarantees. See [Section 32b](#32b-software-licensing--compliance) for the full license audit and CI/CD cost breakdown.*

---

# Appendix B: Priority Actions

### Immediate (before next flight test)
1. **Replace YOLOv8n with RT-DETR v2 (Apache 2.0)** — eliminates sole license risk, zero cost
2. **Enable PX4 built-in geofence** as defense-in-depth backup to companion geofence
3. **Procure Remote ID module** (Cube ID, $50) — legal requirement

### Short-Term (next 1–3 months)
4. **Integrate Basalt VIO (BSD)** as VIO HAL backend — enables GPS-denied flight (Issue #169)
5. **Switch inference to TensorRT** — 2–5× FPS improvement over OpenCV DNN on Jetson
6. **Build Zenoh topic recorder** — enables post-flight replay and debugging (~2 weeks eng)

### Medium-Term (3–6 months)
7. **Procure Intel RealSense D455** (or OAK-D Pro if D455 supply issues) for real stereo VIO
8. **Evaluate FLIR Lepton 3.5** ($250) to validate thermal perception pipeline
9. **Add Mender.io OTA** for field deployments
10. **Implement GPS spoofing detection** in sensor fusion (~2–3 weeks eng)

### Long-Term (6–12 months)
11. **Evaluate FlytBase or Auterion Suite** for fleet management
12. **Add Livox Mid-360 LiDAR** for BVLOS obstacle avoidance
13. **Implement VFH+** for cluttered environment avoidance
14. **Evaluate ParaZero SafeAir** for flights-over-people certification
15. **Add BehaviorTree.CPP** if mission complexity outgrows FSM

---

## Make vs Buy Summary Matrix

| # | Component | Verdict | Reasoning |
|---|-----------|---------|-----------|
| 1 | Companion Computer | **BUY** | NVIDIA Jetson Orin NX — no one builds custom compute |
| 2 | Flight Controller | **BUY** | Pixhawk 6X — 10+ years of community testing, real-time control |
| 3 | Airframe | **BUY** | Holybro X500 V2 — mechanical engineering, not software |
| 4 | Mission Camera | **BUY** | Commodity sensor, value is in perception software |
| 5 | Stereo Camera | **BUY** | Pre-calibrated saves weeks, precision matters for VIO |
| 6 | IMU | **BUY** | MEMS fabrication, use camera-integrated or BMI088 |
| 7 | GPS/GNSS | **BUY** | RF silicon, u-blox M9N/F9P |
| 8 | LiDAR | **BUY** | When needed — Livox Mid-360 |
| 9 | Barometer/Magnetometer | **BUY** | Included with FC + GPS module |
| 10 | Telemetry Radio | **BUY** | RF certification required |
| 11 | Video Datalink | **BUY** | When needed — Herelink |
| 12 | Mesh Networking | **BUY** | When needed — Doodle Labs |
| 13 | Gimbal | **BUY** | Precision mechatronics — SIYI A8 Mini |
| 14 | Thermal Camera | **BUY** | FLIR dominance — Boson 640 |
| 15 | Motors/ESCs/Props | **BUY** | Commodity propulsion |
| 16 | Battery & Power | **BUY** | Commodity energy storage |
| 17 | IPC Middleware | **BUY** engine, **MAKE** abstraction | Zenoh engine + our IPublisher/ISubscriber |
| 18 | FC Firmware | **BUY** | PX4 — never build your own autopilot |
| 19 | FC Communication SDK | **BUY** | MAVSDK (BSD) |
| 20 | Logging/Math Libraries | **BUY** | spdlog, Eigen3, nlohmann/json |
| 21 | Object Detection Model | **EVALUATE** | Switch to RT-DETR v2 (Apache 2.0) |
| 22 | Multi-Object Tracking | **MAKE** | Core IP — ByteTrack, competitive, zero risk |
| 23 | Sensor Fusion | **MAKE** | Core IP — UKF tailored to our state vector |
| 24 | VIO / SLAM | **EVALUATE** | Basalt VIO (BSD) — hardest component, strong OSS options |
| 25 | Path Planning | **MAKE** | Core IP — D* Lite, tight integration |
| 26 | Obstacle Avoidance | **MAKE** | Core IP — potential field, platform-specific tuning |
| 27 | Mission Planner / FSM | **MAKE** | **Core product** — this IS the autonomy |
| 28 | Geofencing | **MAKE** | 300 lines, integrated with fault management |
| 29 | System Monitoring | **MAKE** | Purpose-built 3-layer watchdog, no equivalent to buy |
| 30 | Fault Injection | **MAKE** | Specific to our fault taxonomy and IPC |
| 31 | Simulation | **BUY** | Gazebo + PX4 SITL — ecosystem standard |
| 32 | CI/CD | **BUY** infra, **MAKE** config | GitHub Actions + CMake |
| 33 | GCS | **BUY** | QGroundControl for V1 |
| 34 | Fleet Management | **BUY** | When needed — FlytBase |
| 35 | Remote ID | **BUY** | Regulatory — Cube ID |
| 36 | UTM / Airspace | **BUY** | When needed — Airmap |
| 37 | Cybersecurity | **MAKE** mostly | Enable Zenoh TLS, MAVLink signing, Secure Boot |
| 38 | Data Recording | **MAKE** + PX4 ULog | Zenoh topic recorder ~2 weeks |
| 39 | OTA Updates | **BUY** | Mender.io (Apache 2.0) |
| 40 | Mapping/Photogrammetry | **BUY** | When needed — OpenDroneMap |
| 41 | AI Training Pipeline | **BUY** | When needed — Roboflow or NVIDIA TAO |
| 42 | Anti-Collision Lights | **BUY** | Regulatory — Lume Cube |
| 43 | Parachute | **BUY** | When needed — ParaZero SafeAir |
| 44 | Digital Twin | **MAKE** lightweight | Gazebo + Zenoh replay |
| 45 | Edge AI Accelerator | **BUY** | Only if Jetson GPU insufficient — Hailo-8 |

---

*Last updated: 2026-03-20*
