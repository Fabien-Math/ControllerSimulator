import numpy as np

# Define thruster system
class ThrusterSystem:
	def __init__(self, thruster_params):
		self.T = thruster_params['T']
		self.T_inv = self.T.T @ np.linalg.inv(self.T @ self.T.T)
		self.n_thrusters = thruster_params['n_thrusters']
		self.min_force, self.max_force = thruster_params['thruster_limits']

		# 2nd-order actuator model for each thruster: x'' + 2ζωx' + ω²x = ω²u
		self.wn = thruster_params['wn']  # Natural frequency
		self.zeta = thruster_params['zeta']  # Damping ratio

		# States: force and velocity for each thruster
		self.force = np.zeros(6)
		self.thrust = np.zeros(self.n_thrusters)
		self.rpm = np.zeros(self.n_thrusters)


	def update(self, dt, u_cmd):
		# Compute desired thruster forces using pseudo-inverse control allocation
		thruster_cmd = self.T_inv @ u_cmd

		# Clip desired thruster commands to actuator physical limits
		thruster_cmd = np.clip(thruster_cmd, self.min_force, self.max_force)

		# Update thruster forces using second-order propeller dynamics
		# Discretize using explicit Euler (or better with RK4 if you want more accuracy)
		accel = self.wn**2 * (thruster_cmd - self.thrust) - 2 * self.zeta * self.wn * self.rpm
		self.rpm += accel * dt
		self.thrust += self.rpm * dt

		# Clip again to be sure the physical force doesn't exceed limits
		self.force = self.T @ np.clip(self.thrust, self.min_force, self.max_force)

