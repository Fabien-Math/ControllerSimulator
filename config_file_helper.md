# **Configuration File Documentation**

This document describes the structure and meaning of the configuration YAML file used to define simulation parameters, robot settings, control systems, thrusters, and environmental properties for the underwater robotics simulation.

A full exemple can be found in `config/exemple.yaml`.

---

## 🧠 **Simulation Settings**

```yaml
simulation:
  name: <string>         # Name of the simulation.
  timestep: <float>       # Duration of each step (seconds).
  end_time: <float>       # Total simulation time (seconds).
  graphical: <bool>       # Enable 3D visualization.
  show_graphs: <bool>     # Display simulation graphs interactively.
  save_graphs: <bool>     # Save graphs to disk.
  save_csv: <bool>        # Save simulation logs as CSV.
  save_output: <string>   # Output folder for all files.
```

---

## 🤖 **Robot Definition**

```yaml
robot:
  name: <string>     # Robot model or identifier
```

---

### 🎯 **Mission Configuration**

The mission block selects the robot’s path-planning strategy.
The loader expects:

```yaml
mission:
  type: <waypoints | trajectory | spiral | lawnmower>
  params: <parameters depending on mission type>
```

---

#### **1. `waypoints` — List of 6-DoF waypoints**

The simplest mission: the robot follows a sequence of poses.

```yaml
mission:
  type: waypoints
  params:
    - [x, y, z, roll, pitch, yaw]
    - [x, y, z, roll, pitch, yaw]
    - ...
```

---

#### **2. `trajectory` — Time-parameterized spline path**

Uses cubic splines to generate a smooth trajectory with automatic velocity & acceleration.

```yaml
mission:
  type: trajectory
  params:
    waypoints:
      - [x, y, z, roll, pitch, yaw]
      - [x, y, z, roll, pitch, yaw]
      - ...
    times:
      - t0
      - t1
      - ...
```

**Notes:**

* `waypoints` and `times` must have the same length.
* `times` must be strictly increasing.
* A C²-continuous trajectory is generated.

---

#### **3. `spiral` — Outward spiral survey pattern**

Automatically generates a spiral trajectory expanding with each turn.

```yaml
mission:
  type: spiral
  params:
    radius: <float>             # Final radius of the spiral
    n_turn: <int>               # Number of turns
    n_points_per_turn: <int>    # Resolution of each turn
    offset: [x, y, z, roll, pitch, yaw]   # Global offset for the entire spiral
```

---

#### **4. `lawnmower` — Zig-zag coverage pattern**

A typical survey pattern for AUVs/ROVs.

```yaml
mission:
  type: lawnmower
  params:
    width: <float>              # Coverage width in X direction
    height: <float>             # Total height in Y direction
    line_spacing: <float>       # Distance between sweep lines
    offset: [x, y, z, roll, pitch, yaw]   # Starting 6-DoF pose offset
```

**Behavior:**

* Robot moves back and forth in alternating directions.
* Generates a transition waypoint at the end of each line.
* Yaw flips between 0 and π depending on sweep direction.

---

### 🧭 **Initial Conditions**

```yaml
  initial_conditions:
    eta: [x, y, z, roll, pitch, yaw]    # Initial 6D pose
    nu:  [u, v, w, p, q, r]              # Initial 6D velocity
```

---

### ⚖️ **Mass and Inertia**

```yaml
  mass_properties:
    m: <float>           # Mass in kg
    rg: [x, y, z]        # Center of gravity
    I0:                  # 3×3 inertia matrix
      - [ , , ]
      - [ , , ]
      - [ , , ]
    Ma:                  # 6×6 added mass matrix
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
```

---

### 🌊 **Hydrodynamic Damping**

```yaml
  damping:
    Dl:   # Linear damping (6x6)
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
    Dq:   # Quadratic damping (6x6)
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
```

---

## 🚀 **Thruster Configuration**

From the loader code:

* Thrusters are defined under `thrusters:`
* Count is stored in `n_thrusters`
* Individual thrusters use keys `thruster0`, `thruster1`, …
* Each thruster includes `name`, `position`, `thrust_limits`, `wn`, and `zeta`

```yaml
  thrusters:
    n_thrusters: <int>

    thruster0:
      name: <string>
      position: [x, y, z]                 # NOTE: orientation is NOT used in loader
      thrust_limits: [min, max]           # Newtons
      wn: <float>                         # Natural frequency
      zeta: <float>                       # Damping ratio

    thruster1:
      name: <string>
      position: [x, y, z]
      thrust_limits: [min, max]
      wn: <float>
      zeta: <float>

    # Continue for all thrusters
```

⚠ **Important:**
The loader treats `position` as a 3D vector only (no roll/pitch/yaw), because it calls:

```python
np.array(thruster_cfg.get('position'))
```

---

## 🎮 **Controller Settings**

The loader supports exactly 5 controller types:

* `"PID"`
* `"SMC"`
* `"FDB"`
* `"PPC"`
* `"MPC"`

Any other type raises:

```
Unsupported controller type
```

### **Base controller structure**

```yaml
  controller:
    type: <PID | SMC | FDB | PPC | MPC>

    eta_tol: [ , , , , , ]     # Pose tolerance
    nu_tol:  [ , , , , , ]     # Velocity tolerance
    cmd_offset: [ , , , , , ]  # Offset on commands
```

---

### ⚙️ **SMC Parameters (if type = "SMC")**

```yaml
    smc_params:
      k_smc:       [ , , , , , ]
      lambda_smc:  [ , , , , , ]
      phi:         [ , , , , , ]
```

---

### ⚙️ **PID Parameters (if type = "PID")**

```yaml
    pid_params:
      kp: [ , , , , , ]
      ki: [ , , , , , ]
      kd: [ , , , , , ]
```

---

### ⚙️ **FDB Parameters (if type = "FDB")**

```yaml
    feedback_params:
      kp:     [ , , , , , ]
      kd:     [ , , , , , ]
      noises: [ , , , , , ]
```

---

### ⚙️ **PPC Parameters (if type = "PPC")**

```yaml
    pole_placement_params:
      p: [ , , , , , ]
```

---

### ⚙️ **MPC Parameters (if type = "MPC")**

```yaml
    mpc_params:
      k: <float>    # MPC gain or horizon parameter
```

---

## 🌊 **Environment Settings**

```yaml
environment:
  properties:
    gravity: [gx, gy, gz]
    water_density: <float>
    water_viscosity: <float>
```

---

## 🌪️ **Current Modeling**

The loader reads:

```python
current_types = "normal,jet,constant,time_series,depth_profile"
```

— all types may be combined and separated by commas.

Example:

```yaml
current:
  types: "constant, jet"
```

---

### **normal — Gaussian random current**

```yaml
  normal:
    speed: [u, v, w, p, q, r]
    std:   [std_u, std_v, std_w, std_p, std_q, std_r]
```

---

### **jet — periodic on/off jet current**

Loader structure:

```python
current_params['jet']['vector']
current_params['jet']['period']
current_params['jet']['duty']
```

Therefore YAML:

```yaml
  jet:
    vector: [u, v, w, p, q, r]   # Or 6 values depending on your model
    period: <float>     # seconds
    duty: <float>       # 0–1 duty cycle
```

---

### **constant — fixed current vector**

```yaml
  constant:
    vector: [u, v, w, p, q, r]
```

---

### **time_series — time-dependent current**

Loader expects a list of objects:

```python
{'time': <float>, 'vector': np.array([...])}
```

YAML:

```yaml
  time_series:
    - time: 0.0
      vector: [u, v, w, p, q, r]
    - time: 5.0
      vector: [u, v, w, p, q, r]
```

---

### **depth_profile — varies with depth**

```yaml
  depth_profile:
    - depth: <float>
      vector: [u, v, w, p, q, r]
    - depth: <float>
      vector: [u, v, w, p, q, r]
```
