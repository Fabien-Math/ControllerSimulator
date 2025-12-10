import numpy as np
from scipy.interpolate import CubicSpline



class MissionSystem:
    def __init__(self, config_node):
        self.times = None
        self.waypoints = None

        self.type = config_node["mission"]["type"]
        
        if self.type == "waypoints":
            self.waypoints = np.array(config_node["mission"]["params"])
        
        if self.type == "trajectory":
            self.waypoints = config_node["mission"]["params"]["waypoints"]
            self.times = config_node["mission"]["params"]["times"]

            self.traj_pos, self.traj_vel, _ = self.compute_spline_trajectory(self.waypoints, self.times)
        
        if self.type == "spiral":
            self.radius = config_node["mission"]["params"]["radius"]
            self.n_turn = config_node["mission"]["params"]["n_turn"]
            self.n_points_per_turn = config_node["mission"]["params"]["n_points_per_turn"]
            self.offset = config_node["mission"]["params"]["offset"]
            self.waypoints = self.compute_spiral_waypoints()
            
        if self.type == "lawnmower":
            self.width = config_node["mission"]["params"]["width"]
            self.height = config_node["mission"]["params"]["height"]
            self.line_spacing = config_node["mission"]["params"]["line_spacing"]
            self.offset = config_node["mission"]["params"]["offset"]
            self.waypoints = self.compute_lawnmower_waypoints()


            
    def compute_spiral_waypoints(self):
        """
        Generate spiral-like 6-DoF waypoints.
        """
        points = []
        total_points = self.n_turn * self.n_points_per_turn
        for i in range(total_points):
            angle = 2 * np.pi * (i / self.n_points_per_turn)
            r = self.radius * (i / total_points)
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            yaw = angle
            points.append(self.offset + np.array([x, y, 0, 0, 0, yaw]))
        return np.array(points)
    
    def compute_lawnmower_waypoints(self):
        """
        Generate a 6-DoF lawn-mower (zig-zag) waypoints over a rectangular area.
        width: length of each sweep (X direction)
        height: total coverage in Y direction
        line_spacing: distance between lines
        """
        num_lines = int(self.height / self.line_spacing)
        points = []

        for i in range(num_lines):
            y = i * self.line_spacing
            # Alternate direction for each line
            if i % 2 == 0:
                x_line = np.linspace(0, self.width, 10) + self.offset[0]
                yaw = 0.0 + self.offset[5]
            else:
                x_line = np.linspace(self.width, 0, 10) + self.offset[0]
                yaw = np.pi + self.offset[5]  # turn around

            for x in x_line:
                points.append([x, y, self.offset[2], 0.0, 0.0, yaw])

            # Move to next line (transition step)
            if i < num_lines - 1:
                next_y = (i + 1) * self.line_spacing + self.offset[1]
                points.append([x_line[-1], next_y, self.offset[2], 0.0, 0.0, yaw + np.pi])

        return np.array(points)
    

    def compute_spline_trajectory(self, waypoints, timestamps):
        """
        Create a C2-continuous trajectory from waypoints and timestamps.

        Args:
            waypoints: np.array of shape (N, dim) containing N waypoints in 'dim' dimensions.
            timestamps: np.array of shape (N,) containing strictly increasing times for each waypoint.

        Returns:
            traj_func: function(t) -> position at time t
            traj_vel: function(t) -> velocity at time t
            traj_acc: function(t) -> acceleration at time t
        """
        waypoints = np.array(waypoints)
        timestamps = np.array(timestamps)

        if waypoints.shape[0] != timestamps.shape[0]:
            raise ValueError("Number of waypoints must match number of timestamps")

        # Create a cubic spline for each dimension
        self.splines = []
        for dim in range(waypoints.shape[1]):
            # 'natural' boundary condition gives zero acceleration at endpoints
            cs = CubicSpline(timestamps, waypoints[:, dim], bc_type='natural')
            self.splines.append(cs)

        # Function to evaluate position, velocity, acceleration
        def traj_func(t):
            return np.array([cs(t) for cs in self.splines])

        def traj_vel(t):
            return np.array([cs(t, 1) for cs in self.splines])

        def traj_acc(t):
            return np.array([cs(t, 2) for cs in self.splines])

        return traj_func, traj_vel, traj_acc
    