import numpy as np

# Sliding Mode Controller class for 3D
class SlidingModeController:
	def __init__(self, robot, controller_params):
		self.k_smc = controller_params['k_smc']
		self.lambda_smc = controller_params['lambda_smc']
		self.phi = controller_params['phi']
		self.robot = robot

		self.last_J = None
		self.mu = 1

		self.s = np.zeros(6)
		self.u = np.zeros(6)
		self.cmd = np.zeros(6)

	def compute_J1(self, eta_err):
		"""
		Rotation matrix from body frame to inertial (NED) frame.
		"""
		sphi = np.sin(eta_err[3])
		cphi = np.cos(eta_err[3])
		stheta = np.sin(eta_err[4])
		ctheta = np.cos(eta_err[4])
		spsi = np.sin(eta_err[5])
		cpsi = np.cos(eta_err[5])

		return np.array([
			[ctheta * cpsi,
			sphi * stheta * cpsi - cphi * spsi,
			cphi * stheta * cpsi + sphi * spsi],

			[ctheta * spsi,
			sphi * stheta * spsi + cphi * cpsi,
			cphi * stheta * spsi - sphi * cpsi],

			[-stheta,
			sphi * ctheta,
			cphi * ctheta]
		])

	def compute_J2(self, eta_err):
		"""
		J2 matrix from Fossen formalism (Euler angle kinematics).
		"""
		sphi = np.sin(eta_err[3])
		cphi = np.cos(eta_err[3])
		ttheta = np.tan(eta_err[4])
		ctheta = np.cos(eta_err[4])

		return np.array([
			[1.0,      sphi * ttheta,      cphi * ttheta],
			[0.0,      cphi,               -sphi],
			[0.0,      sphi / ctheta,      cphi / ctheta]
		])
	
	def compute_J(self, eta_err):
		J1 = self.compute_J1(eta_err)
		J2 = self.compute_J2(eta_err)
		J = np.zeros((6, 6))
		J[0:3, 0:3] = J1
		J[3:6, 3:6] = J2
		return J
	
	def update(self, dt, eta_err, nu_err):
		# J = self.compute_J(eta_err)
		# if self.last_J is None:
		# 	self.last_J = J
		# 	return
		
		# A_tilde = self.robot.Minv @ (self.robot.C + self.robot.D)
		# B_tilde = self.robot.Minv @ self.robot.G
		# T_tilde = self.robot.Minv @ self.robot.T

		# T_tilde_inv = np.zeros(6)
		# for i, t in enumerate(T_tilde):
		# 	if t == 0:
		# 		T_tilde_inv[i] = 10000
		# 		continue
		# 	T_tilde_inv[i] = 1 / t
		
		# J_point = 1 / dt * (J - self.last_J)
		# J_inv = np.linalg.inv(J)
		# u_eq = T_tilde_inv * (A_tilde @ (- nu_err) + B_tilde) - J_inv @ T_tilde_inv * (J_point + self.mu * J) @ (- nu_err)

		# self.s = J @ (- nu_err) + self.lambda_smc * (- eta_err)
		# self.u = u_eq - np.diag(self.k_smc) * J_inv @ T_tilde_inv * np.array([np.tanh(si / self.phi[i]) for i, si in enumerate(self.s)])
		
		self.cmd = self.u
	