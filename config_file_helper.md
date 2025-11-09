# Configuration File Documentation

This document describes the structure and meaning of the configuration YAML file used to define the simulation parameters, robot settings, control systems, and environmental properties for underwater robotics simulations.

---

## 🧠 Simulation Settings

```yaml
simulation:
  timestep: <float>       # The duration of each simulation step (in seconds).
  end_time: <float>       # Total time the simulation should run (in seconds).
  graphical: <bool>       # Enable or disable 3D visualization during simulation.
  show_graphs: <bool>     # Whether simulation graphs are shown.
  save_graphs: <bool>     # Whether simulation graphs are saved.
  save_csv: <bool>        # Whether the simulation is saved in a csv.
  save_output: <string>   # Folder where all results are saved.
```

This section sets up global options for the simulation time, rendering, and training mode.

---

## 🤖 Robot Definition

```yaml
robot:
  name: <string>          # Identifier or model name of the robot.
```

### 🎯 Mission Configuration

```yaml
  mission:
    waypoints:
      - [x, y, z, roll, pitch, yaw]
```

Defines the sequence of 6D waypoints (position and orientation) the robot must follow.

---

### 🧭 Initial Conditions

```yaml
  initial_conditions:
    eta: [x, y, z, roll, pitch, yaw]   # Initial pose of the robot.
    nu: [u, v, w, p, q, r]             # Initial linear and angular velocity.
```

---

### ⚖️ Mass and Inertia

```yaml
  mass_properties:
    m: <float>                        # Robot mass in kilograms.
    rg: [x, y, z]                     # Position of center of gravity in meters.
    I0:                               # 3x3 Inertia matrix about the center of gravity.
      - [ , , ]
      - [ , , ]
      - [ , , ]
    Ma:                               # 6x6 Added mass matrix (models hydrodynamic added mass).
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
      - [ , , , , , ]
```

This section describes the physical parameters used for dynamics modeling.

---

### 🌊 Hydrodynamic Damping

```yaml
  damping:
    Dl:   # Linear damping (6x6 matrix)
      - [ , , , , , ]
    Dq:   # Quadratic damping (6x6 matrix)
      - [ , , , , , ]
```

Damping coefficients help simulate realistic underwater drag forces.

---

### 🚀 Thruster Configuration

```yaml
  thruster:
    T:                      # 6xN Thruster allocation matrix
      - [ , , , , , , , ]
    n_thrusters: <int>      # Total number of thrusters.
    thruster_limits: [min, max]  # Force or speed limits of each thruster.
    wn: <float>             # Natural frequency of thruster dynamics.
    zeta: <float>           # Damping ratio for thruster response modeling.
```

This section defines how thruster outputs translate into robot forces and torques.

---

### 🎮 Controller Settings

```yaml
  controller:
    type: <string>          # Control method ("PID", "SMC", etc.)

    eta_tol: [ , , , , , ]  # Pose tolerance for each axis (x, y, z, roll, pitch, yaw).
    nu_tol: [ , , , , , ]   # Velocity tolerance for each axis (u, v, w, p, q, r).
```

Defines the control strategy and the acceptable error thresholds for pose and velocity.

#### ⚙️ SMC Parameters (Taken into account only if controller type is `"SMC"`)

```yaml
    smc_params:
      k_smc: [ , , , , , ]       # Sliding mode control gains.
      lambda_smc: [ , , , , , ]  # Exponential convergence rates.
      phi: [ , , , , , ]         # Boundary layer thickness (reduces chattering).
```

#### ⚙️ PID Parameters (Taken into account only if controller type is `"PID"`)

```yaml
    smc_params:
      kp: [ , , , , , ]   # Proportionnal parameters.
      ki: [ , , , , , ]   # Integral parameters.
      kd: [ , , , , , ]   # Derivative parameters.
```


---

## 🌊 Environment Settings

```yaml
environment:
  properties:
    gravity: [gx, gy, gz]       # Gravitational acceleration vector.
    water_density: <float>      # Density of water in kg/m^3.
    water_viscosity: <float>    # Dynamic viscosity of water in Pa·s.
```

These parameters define the base environmental conditions the robot is subjected to.

---

### 🌪️ Current Modeling

```yaml
  current:
    type: <string>   # One of: "normal", "jet", "constant", "time_series", "depth_profile"
```

#### `normal` – Gaussian distributed current

```yaml
    normal:
      speed: [u, v, w]             # Mean linear speeds.
      std: [std_u, std_v, std_w]   # Standard deviations.
```

#### `jet` – Fixed 6D velocity (current + angular)

```yaml
    jet:
      vector: [u, v, w, p, q, r]   # Constant current vector.
```

#### `constant` – Fixed linear velocity only

```yaml
    constant:
      vector: [u, v, w]            # Constant flow vector.
```

#### `time_series` – Time-dependent current

```yaml
    time_series:
      - [u, v, w, p, q, r]         # Sequence of current vectors over time.
      - [u, v, w, p, q, r]
```

#### `depth_profile` – Varies by depth

```yaml
    depth_profile:
      - depth: <float>            # Depth in meters.
        vector: [u, v, w]         # Velocity at that depth.
      - depth: <float>
        vector: [u, v, w]
```
