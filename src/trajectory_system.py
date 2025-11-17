import numpy as np
import yaml


class TrajectorySystem:
    def __init__(self, config_node):
        self.trajectory = None
        
        self.traj_type = config_node["mission"]["type"]
        if self.traj_type == "waypoints":
            self.trajectory = np.array(config_node["mission"]["params"])
        
        if self.traj_type == "spiral":
            self.radius = config_node["mission"]["params"]["radius"]
            self.n_turn = config_node["mission"]["params"]["n_turn"]
            self.n_points_per_turn = config_node["mission"]["params"]["n_points_per_turn"]
            self.offset = config_node["mission"]["params"]["offset"]
            self.trajectory = self.compute_spiral_trajectory()
            
        if self.traj_type == "lawnmower":
            self.width = config_node["mission"]["params"]["width"]
            self.height = config_node["mission"]["params"]["height"]
            self.line_spacing = config_node["mission"]["params"]["line_spacing"]
            self.offset = config_node["mission"]["params"]["offset"]
            self.trajectory = self.compute_lawnmower_trajectory()


            
    def compute_spiral_trajectory(self):
        """
        Generate spiral-like 6-DoF trajectory.
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
    
    def compute_lawnmower_trajectory(self):
        """
        Generate a 6-DoF lawn-mower (zig-zag) trajectory over a rectangular area.
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