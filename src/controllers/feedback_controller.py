import numpy as np


# Sliding Mode Controller class for 3D
class FeedbackController:
	def __init__(self, robot, controller_params):
		self.kp = np.array(controller_params['kp'])
		self.kd = np.array(controller_params['kd'])
		self.noises = controller_params['noises']

		self.robot = robot
		self.u = np.zeros(6)
		self.cmd = np.zeros(6)

	def add_noise(self, M, noise):
		"""
		Add Gaussian noise to the value M.

		The parameter `noise` must be ≥ 0 and represents the *relative noise level*:
		it defines the standard deviation of the Gaussian noise as a percentage of |M|.
		In other words, the added noise has: sigma = |noise * M|.

		Returns:
			An array with Gaussian-perturbed values of the same shape as M.
		"""
		return np.random.normal(M, np.abs(noise*M), size=M.shape)

	def update(self, dt, eta_err, nu_err):
		
		if self.robot.M_eta_inv is None:
			return
		
		noise_matrix_D = self.add_noise(self.robot.D, self.noises['D'])
		noise_matrix_C = self.add_noise(self.robot.C, self.noises['C'])
		noise_matrix_G = self.add_noise(self.robot.G, self.noises['G'])
		noise_matrix_M = self.add_noise(self.robot.M, self.noises['M'])
		noise_matrix_M_inv = np.linalg.inv(noise_matrix_M)

		self.hydro_forces = - noise_matrix_D @ self.robot.nu_rel
		self.coriolis_centripetal_forces = - noise_matrix_C @ self.robot.nu
		self.ext_forces = noise_matrix_G


		A = noise_matrix_M_inv @ (self.hydro_forces + self.coriolis_centripetal_forces + self.ext_forces)
		B_inv = noise_matrix_M
		V = self.kp * eta_err + self.kd * nu_err

		self.u = - B_inv @ A + B_inv @ V

		self.cmd = self.u
	