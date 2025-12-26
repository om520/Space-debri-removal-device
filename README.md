# Space-debri-removal-device
# Space Debris Removal Device — Simulation Toolbox

## Overview

This repository includes MATLAB and GMAT simulation scripts developed for validating core subsystems of a debris-removal or inspection spacecraft in Low Earth Orbit (LEO). The suite covers end-to-end onboard navigation, radar-only relative motion estimation, multi-hop communication protocols, plus a GMAT-based descent simulation using xenon propulsion.

---

## Contents

### `matlab_simulation/`

- **end_to_end_ekf.m**
  
  Simulates the complete truth-sensor-estimator chain for onboard relative navigation.
  
  - **Input:** Raw radar measurements, orbital dynamics.
  - **Functionality:** Corrects for ion thruster-induced perturbations. Delivers high-precision state estimates for guidance and control.
  - **Output:**  
    - End-to-end Position RMS error: **646.867 m**
    - End-to-end Velocity RMS error: **11.8955 m/s**

- **radar_relative_navigation.m**
  
  Implements and validates radar-only relative navigation.
  
  - **Use Case:** Operates in GPS-denied, vision-sensor-limited regimes. Core layer for close-proximity ops, rendezvous/docking, servicing/formation maintenance.

## Video Demonstration

[![Space Debris Removal Device - Video](https://img.youtube.com/vi/LjgtK_CmWN0/0.jpg)](https://youtu.be/LjgtK_CmWN0)
  - **Output:**  
    - Position RMS error: **7.346 m**
    - Velocity RMS error: **0.0102 m/s**

- **comm_sim_satrelay.m**
  
  Simulates multi-hop DTN protocol for LEO communications.
  
  - **Features:** Physics modeling (link budgets, BER, noise), realistic protocol stack (fragmentation, retransmits, buffering), tunable parameters for orbital altitude, antenna gain, bandwidth.
  - **Applications:** Satellite constellations, Lunar Gateway relays, Deep-space store-and-forward (CFDP, DTN).
  - **Results:**  
    - Delivered bundles: **427**
    - Median latency: **0.0 s**, 90th percentile: **1.0 s**

---

### `gmat_simulation/`

- **device_descent_simulation.gmat**
  
  GMAT scenario for spacecraft deorbiting: simulates retrograde thrust maneuvers from 800 km to 100 km altitude using a xenon thruster.

---

## Getting Started

1. **Clone the repository:**
