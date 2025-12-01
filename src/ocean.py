import numpy as np

class Ocean:
    def __init__(self, environment_params):
        # Physical environment properties (required)
        self.gravity = np.array(environment_params["gravity"])
        self.water_density = environment_params["water_density"]
        self.water_viscosity = environment_params["water_viscosity"]

        # Current configuration (type is required)
        self.current_types = environment_params["current"]["type"].split(',')
        self.current_params = environment_params["current"]

        self.fluid_vel = np.zeros(6)

        # Preprocess data depending on current type
        if "time_series" in self.current_types:
            self.time_series = np.array(self.current_params["time_series"])
            self.current_step = 0

        elif "depth_profile" in self.current_types:
            self.depth_profile = sorted(self.current_params["depth_profile"], key=lambda d: d["depth"])

    def init_fluid_vel(self):
        for current_type in self.current_types:
            if current_type == "normal":
                mean = np.array(self.current_params["speed"])
                std = np.array(self.current_params["std"])
                self.fluid_vel = np.random.normal(mean, std)
            elif current_type == "constant":
                self.fluid_vel = np.array(self.current_params["vector"])


    def compute_fluid_vel(self, pos, dt, step=None):
        """
        Compute the 6D fluid velocity vector at a given position.
        pos: [x, y, z] as numpy array
        step: optional time step for time_series currents
        """
        for current_type in self.current_types:
            if current_type == "null":
                return

            if current_type == "normal":
                mean = np.array(self.current_params["speed"])
                std = np.array(self.current_params["std"])
                self.fluid_vel[:3] += (mean - np.random.normal(mean, std)) * dt

            elif current_type == "constant":
                linear = np.array(self.current_params["vector"])
                self.fluid_vel[:3] = linear

            elif current_type == "time_series":
                i = step if step is not None else self.current_step
                i = min(i, len(self.time_series) - 1)
                self.fluid_vel = self.time_series[i]

            elif current_type == "depth_profile":
                z = pos[2]
                for i in range(len(self.depth_profile) - 1):
                    d0 = self.depth_profile[i]["depth"]
                    d1 = self.depth_profile[i + 1]["depth"]
                    if d0 <= z <= d1:
                        v0 = np.array(self.depth_profile[i]["vector"])
                        v1 = np.array(self.depth_profile[i + 1]["vector"])
                        ratio = (z - d0) / (d1 - d0)
                        return np.concatenate((v0 + ratio * (v1 - v0), np.zeros(3)))
                if z <= self.depth_profile[0]["depth"]:
                    self.fluid_vel[:3] = np.array(self.depth_profile[0]["vector"])
                self.fluid_vel[:3] = np.array(self.depth_profile[-1]["vector"])
            
            elif current_type == "jet":
                jet_force =  np.array(self.current_params["vector"])
                jet_period   = self.current_params.get("period", 10.0)
                jet_duty     = self.current_params.get("duty", 0.1)  # percentage of period active

                phase = (step % jet_period) / jet_period
                jet_vector = jet_force * (phase < jet_duty)

                self.fluid_vel[:3] += jet_vector

            else:
                raise ValueError(f"Unsupported current type: {current_type}")
