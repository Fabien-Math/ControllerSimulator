import numpy as np
import matplotlib.pyplot as plt
import os
import re
from logging_system import LoggingSystem

plt.rcParams.update({
        "figure.constrained_layout.use": True,  # auto layout adjustment
        "font.size": 14,                        # base font size
        "axes.titlesize": 16,                   # title font
        "axes.labelsize": 14,                   # axis labels
        "legend.fontsize": 12,                  # legend font
        "xtick.labelsize": 12,                  # tick labels
        "ytick.labelsize": 12,
        "figure.titlesize": 18,                 # suptitle font
        "lines.linewidth": 1.8,                 # slightly thicker lines
        "grid.alpha": 0.4,                      # subtle grid
        "axes.grid": True,                      # grid enabled by default
    })

plt.style.use("seaborn-v0_8")

class GraphSystem:
    def __init__(self, logger, show_graph, save_graph, folder):
        self.logger: LoggingSystem = logger
        self.show_graph = show_graph
        self.save_graph = save_graph
        self.folder = folder
        
        logger.np_format()

        timestamps = self.logger.timestamps
        etas = self.logger.etas  # actual pose [x, y, z, roll, pitch, yaw]
        nus = self.logger.nus    # actual velocities
        etas_err = self.logger.etas_err
        etas_err_world = self.logger.etas_err_world
        nus_err = self.logger.nus_err
        thrust_forces = self.logger.thrust_forces
        thruster_forces = self.logger.thruster_forces
        commands = self.logger.commands
        etas_desired = self.logger.etas_desired
        nus_desired = self.logger.nus_desired
        
        eta_labels = ["X", "Y", "Z", "P", "Q", "R"]
        nu_labels = ["U", "V", "W", "P", "Q", "R"]
        
        # Plot 1: Actual vs Desired
        self.plot_dof_grid(timestamps=timestamps, data_actual=etas, data_desired=etas_desired, labels=eta_labels, title="Pose DoF overview")
        self.plot_dof_grid(timestamps=timestamps, data_actual=nus, data_desired=nus_desired, labels=nu_labels, title="Velocity DoF overview")
        
        # Plot 2: Errors
        self.plot_dof_grid(timestamps=timestamps, data_actual=etas_err, title=r"Robot Pose Error ($\eta$_err)", labels=eta_labels)
        self.plot_dof_grid(timestamps=timestamps, data_actual=etas_err_world, title=r"World Pose Error $\eta$_err", labels=eta_labels)

        # Plot 3: Velocity Error and Commands
        self.plot_dof_grid(timestamps=timestamps, data_actual=nus_err, title=r"Velocity Error ($\nu$_err)", labels=nu_labels)
        self.plot_dof_grid(timestamps=timestamps, data_actual=commands, title="Controller Commands")

        # Plot 4: Forces
        self.plot_dof_grid(timestamps=timestamps, data_actual=thrust_forces, title="Thrust Forces (Total)", labels=eta_labels)
        self.plot_dof_grid(timestamps=timestamps, data_actual=thruster_forces, title="Individual Thruster Forces", n=2, m=4)
        
        # Plot 5: Thruster forces vs Command
        self.plot_command_vs_thrust(timestamps=timestamps, commands=commands, thrust_forces=thrust_forces)
        

        
    
    def plot_dof_grid(self, timestamps, data_actual, data_desired=None, data_error=None, 
                    labels=None, n=2, m=3, title="DoF Overview"):
        """
        General-purpose function to plot multi-DoF data in an n×m grid.
        
        Args:
            timestamps (array): Time vector of shape (N,)
            data_actual (array): Actual data (N, D)
            data_desired (array): Desired data (N, D), optional
            data_error (array): Error data (N, D), optional
            labels (list): List of DoF labels of length D
            n (int): Number of subplot rows
            m (int): Number of subplot columns
            title (str): Figure title
        """
        # --- Validation ---
        if timestamps is None or len(timestamps) == 0:
            print("⚠️ No timestamps provided.")
            return
        
        timestamps = np.array(timestamps)
        data_actual = np.array(data_actual)
        D = data_actual.shape[1] if data_actual.ndim > 1 else 1

        if labels is None:
            labels = [f"DoF {i+1}" for i in range(D)]

        total_subplots = n * m
        if total_subplots < D:
            print(f"⚠️ Not enough subplots ({total_subplots}) for {D} DoFs. Expanding grid...")
            # Automatically expand the grid
            n = int(np.ceil(D / m))
            total_subplots = n * m
            print(f"→ Adjusted grid: {n} rows × {m} cols")

        # --- Create figure ---
        fig, axes = plt.subplots(n, m, figsize=(5*m, 3.5*n))
        fig.suptitle(title, fontsize=18, fontweight='bold')
        axes = np.atleast_2d(axes)  # ensure 2D array for consistency

        # --- Plot each DoF ---
        for i in range(total_subplots):
            row, col = divmod(i, m)
            ax = axes[row, col]
            if i >= D:
                ax.axis('off')  # hide unused plots
                continue

            # Extract data
            y_actual = data_actual[:, i] if data_actual.ndim > 1 else data_actual
            y_desired = data_desired[:, i] if data_desired is not None and data_desired.ndim > 1 else None
            y_error = data_error[:, i] if data_error is not None and data_error.ndim > 1 else None

            # Plot actual & desired
            ax.plot(timestamps, y_actual, label="Actual", color='tab:blue', lw=1.8)
            if y_desired is not None:
                ax.plot(timestamps, y_desired, '--', label="Desired", color='tab:orange', lw=1.5)

            # Secondary axis for error
            if y_error is not None:
                ax2 = ax.twinx()
                ax2.plot(timestamps, y_error, color='tab:red', alpha=0.6, label="Error")
                ax2.set_ylabel("Error", color='tab:red')
                ax2.tick_params(axis='y', labelcolor='tab:red')
                lines, labels1 = ax.get_legend_handles_labels()
                lines2, labels2 = ax2.get_legend_handles_labels()
                ax.legend(lines + lines2, labels1 + labels2, loc='upper right', fontsize=8)
            else:
                ax.legend(loc='upper right', fontsize=8)

            ax.set_title(labels[i])
            ax.set_xlabel("Time [s]")
            ax.set_ylabel("Value")
            ax.grid(True)

        if self.save_graph:
            self.save_figure(fig, self.folder, title)
        if self.show_graph:
            plt.show()

        
    def plot_command_vs_thrust(self, timestamps, commands, thrust_forces, n_rows=2, n_cols=None):
        """
        Plot commanded vs actual thrust forces for each thruster.

        Args:
            robot: Robot object with .timestamps, .commands, and .thrust_forces
            n_rows (int): Number of subplot rows (default = 2)
            n_cols (int): Number of subplot columns (auto if None)
        """
        # --- Validation ---
        if timestamps.size == 0 or commands.size == 0 or thrust_forces.size == 0:
            print("Missing data: timestamps, commands, or thruster forces.")
            return

        n_thrusters = commands.shape[1]
        if thrust_forces.shape[1] != n_thrusters:
            print(f"Mismatch: {thrust_forces.shape[1]} thrust forces for {n_thrusters} commands.")
            return

        # --- Automatic grid calculation ---
        if n_cols is None:
            n_cols = int(np.ceil(n_thrusters / n_rows))

        fig, axes = plt.subplots(n_rows, n_cols, figsize=(5 * n_cols, 3.5 * n_rows))
        fig.suptitle("Thruster Commands vs Actual Thrust Forces", fontsize=18, fontweight='bold')
        axes = np.atleast_2d(axes)

        for i in range(n_rows * n_cols):
            row, col = divmod(i, n_cols)
            ax = axes[row, col]

            if i >= n_thrusters:
                ax.axis("off")
                continue

            ax.plot(timestamps, commands[:, i], label="Command", color='tab:blue', lw=1.8)
            ax2 = ax.twinx()
            ax2.plot(timestamps, thrust_forces[:, i], label="Thrust", color='tab:orange', lw=1.8)

            ax.set_title(f"Thruster {i+1}")
            ax.set_xlabel("Time [s]")
            ax.set_ylabel("Command", color='tab:blue')
            ax2.set_ylabel("Thrust [N]", color='tab:orange')
            ax.grid(True)

            # Combine legends from both axes
            lines, labels_ = ax.get_legend_handles_labels()
            lines2, labels2_ = ax2.get_legend_handles_labels()
            ax.legend(lines + lines2, labels_ + labels2_, loc="upper right", fontsize=8)

        if self.save_graph:
            self.save_figure(fig, self.folder, "ThrustersForceVSCommands")
        if self.show_graph:
            plt.show()

    
    def save_figure(self, fig, folder=None, name="plot", file_format="png", dpi=300):
        """
        Save a matplotlib figure with a timestamped filename.
        
        Args:
            fig (matplotlib.figure.Figure): The figure to save.
            folder (str): Directory where to save the plot (created if missing).
            name (str): Base name for the file.
            file_format (str): File extension ('png', 'jpg', 'pdf', etc.).
            dpi (int): Resolution for raster formats.
        """
        folder_plot = folder + "/plots/"
        # Create output directory if needed
        os.makedirs(folder_plot, exist_ok=True)
        
        # Generate timestamp
        safe_name = re.sub(r'[^A-Za-z0-9_\-]', '_', name)
        
        # Construct filename
        filename = f"{safe_name}.{file_format}"
        filepath = os.path.join(folder_plot, filename)
        
        # Save the figure
        fig.savefig(filepath, format=file_format, dpi=dpi, bbox_inches="tight")
        print(f"Figure saved as: {filepath}")

        return filepath