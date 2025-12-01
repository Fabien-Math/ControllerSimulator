import numpy as np
import os

class LoggingSystem:
	def __init__(self, robot):
		self.launch_time = None 
		self.robot = robot
		self.etas = []
		self.etas_desired = []
		self.nus = []
		self.nus_desired = []
		self.fluid_vels = []
		self.thrust_forces = []
		self.forces = []
		self.thruster_forces = []
		self.hydro_forces = []
		self.commands = []
		self.etas_err = []
		self.etas_err_world = []
		self.nus_err = []
		self.desired_etas = np.array(robot.controller.desired_etas)
		self.timestamps = []

		self.errs = False
		self.thrust_force = False

	def log_state(self, timestamp):
		"""Append the current robot state to logs."""
		self.etas.append(np.array(self.robot.eta))
		self.nus.append(np.array(self.robot.nu))
		self.fluid_vels.append(np.array(self.robot.fluid_vel))
		self.forces.append(np.array(self.robot.forces))
		self.hydro_forces.append(np.array(self.robot.hydro_forces))
		self.etas_desired.append(np.array(self.robot.controller.desired_eta))
		self.nus_desired.append(np.zeros(6))
		
		if self.thrust_force is not None:
			self.thrust_forces.append(np.array(self.robot.thrusters.force))
			self.thruster_forces.append(np.array(self.robot.thrusters.thrust))
	
		if self.robot.controller is not None:
			self.commands.append(self.robot.controller.controller.cmd)
		
		if self.errs is not None:
			self.etas_err.append(np.array(self.robot.controller.etas_err))
			self.etas_err_world.append(np.array(self.robot.controller.etas_err_world))
			self.nus_err.append(np.array(self.robot.controller.nus_err))
		
		self.timestamps.append(timestamp)

	def clear_logs(self):
		"""Clear all logs."""
		self.etas.clear()
		self.nus.clear()
		self.fluid_vels.clear()
		self.forces.clear()
		self.hydro_forces.clear()
		self.thrust_forces.clear()
		self.thruster_forces.clear()
		self.commands.clear()
		self.etas_err.clear()
		self.nus_err.clear()
		self.timestamps.clear()

	def np_format(self):
		self.etas = np.array(self.etas)
		self.etas_desired = np.array(self.etas_desired)
		self.nus = np.array(self.nus)
		self.nus_desired = np.array(self.nus_desired)
		self.fluid_vels = np.array(self.fluid_vels)
		self.thrust_forces = np.array(self.thrust_forces)
		self.thruster_forces = np.array(self.thruster_forces)
		self.forces = np.array(self.forces)
		self.hydro_forces = np.array(self.hydro_forces)
		self.commands = np.array(self.commands)
		self.etas_err = np.array(self.etas_err)
		self.nus_err = np.array(self.nus_err)
		self.timestamps = np.array(self.timestamps)
		
			
	def save_to_csv(self, folder):
		import csv
		
		# Construct filename
		filename = f"output.csv"
		filepath = os.path.join(folder, filename)
		
		with open(filepath, 'w', newline='') as csvfile:
			writer = csv.writer(csvfile)
			
			# Create header with dynamic sizes based on first entries
			header = ['timestamp']
			
			if len(self.etas) and len(self.etas[0]) > 0:
				header += [f'eta_{i}' for i in range(len(self.etas[0]))]
			if len(self.nus) and len(self.nus[0]) > 0:
				header += [f'nu_{i}' for i in range(len(self.nus[0]))]
			if len(self.fluid_vels) and len(self.fluid_vels[0]) > 0:
				header += [f'fluid_vel_{i}' for i in range(len(self.fluid_vels[0]))]
			if len(self.thrust_forces) and self.thrust_forces[0] is not None:
				header += [f'thrust_force_{i}' for i in range(len(self.thrust_forces[0]))]
			if len(self.thruster_forces) and self.thruster_forces[0] is not None:
				header += [f'thruster_force_{i}' for i in range(len(self.thruster_forces[0]))]
			if len(self.forces) and len(self.forces[0]) > 0:
				header += [f'force_{i}' for i in range(len(self.forces[0]))]
			if len(self.commands) and self.commands[0] is not None:
				header += [f'commands_{i}' for i in range(len(self.commands[0]))]
			if len(self.etas_err) and len(self.etas_err[0]) > 0:
				header += [f'etas_err_{i}' for i in range(len(self.etas_err[0]))]
			if len(self.etas_err_world) and len(self.etas_err_world[0]) > 0:
				header += [f'etas_err_world{i}' for i in range(len(self.etas_err_world[0]))]
			if len(self.nus_err) and len(self.nus_err[0]) > 0:
				header += [f'nus_err_{i}' for i in range(len(self.nus_err[0]))]

			writer.writerow(header)

			for i in range(len(self.timestamps)):
				row = [self.timestamps[i]]
				
				if self.etas is not None and i < len(self.etas):
					row += list(self.etas[i])
				if self.nus is not None and i < len(self.nus):
					row += list(self.nus[i])
				if self.fluid_vels is not None and i < len(self.fluid_vels):
					row += list(self.fluid_vels[i])
				if self.thrust_forces is not None and i < len(self.thrust_forces) and self.thrust_forces[i] is not None:
					row += list(self.thrust_forces[i])
				else:
					row += []
				if self.thruster_forces is not None and i < len(self.thruster_forces) and self.thruster_forces[i] is not None:
					row += list(self.thruster_forces[i])
				else:
					row += []
				if self.forces is not None and i < len(self.forces):
					row += list(self.forces[i])
				if self.commands is not None and i < len(self.commands):
					row += list(self.commands[i])
				if self.etas_err is not None and i < len(self.etas_err):
					row += list(self.etas_err[i])
				if self.etas_err_world is not None and i < len(self.etas_err_world):
					row += list(self.etas_err_world[i])
				if self.nus_err is not None and i < len(self.nus_err):
					row += list(self.nus_err[i])

				writer.writerow(row)
    
		print("Simulation saved!")
