import numpy as np

class Ocean:
    def __init__(self, environment_params):
        # Physical environment properties (required)
        self.gravity = np.array(environment_params["gravity"])
        self.water_density = environment_params["water_density"]
        self.water_viscosity = environment_params["water_viscosity"]

        # Current configuration (type must be present)
        self.current_type = environment_params["current"]["type"]
        self.current_params = environment_params["current"]

        # Preprocess data depending on current type
        if self.current_type == "time_series":
            self.time_series = np.array(self.current_params["time_series"])
            self.current_step = 0

        elif self.current_type == "depth_profile":
            self.depth_profile = sorted(self.current_params["depth_profile"], key=lambda d: d["depth"])


    def compute_fluid_vel(self, pos, step=None):
        """
        Compute the 6D fluid velocity vector at a given position.
        pos: [x, y, z] as numpy array
        step: optional time step for time_series currents
        """
        if self.current_type == "normal":
            mean = np.array(self.current_params["speed"])
            std = np.array(self.current_params["std"])
            linear = np.random.normal(mean, std)
            return np.concatenate((linear, np.zeros(3)))

        elif self.current_type == "jet":
            return np.array(self.current_params["vector"])

        elif self.current_type == "constant":
            linear = np.array(self.current_params["vector"])
            return np.concatenate((linear, np.zeros(3)))

        elif self.current_type == "time_series":
            i = step if step is not None else self.current_step
            i = min(i, len(self.time_series) - 1)
            return self.time_series[i]

        elif self.current_type == "depth_profile":
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
                return np.concatenate((np.array(self.depth_profile[0]["vector"]), np.zeros(3)))
            return np.concatenate((np.array(self.depth_profile[-1]["vector"]), np.zeros(3)))

        else:
            raise ValueError(f"Unsupported current type: {self.current_type}")
