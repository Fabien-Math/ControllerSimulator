import numpy as np


# Sliding Mode Controller class for 3D
class FeedbackController:
	def __init__(self, robot, controller_params):
		self.k = np.array(controller_params['k'])
		self.robot = robot
		self.u = np.zeros(6)
		self.cmd = np.zeros(6)

	def update(self, dt, eta_err, nu_err):
		
		if self.robot.M_eta_inv is None:
			return

		self.hydro_forces = - np.matmul(self.robot.D, self.robot.nu_rel)
		self.coriolis_centripetal_forces = - np.matmul(self.robot.C, self.robot.nu)
		self.ext_forces = self.robot.G

		noise_matrix_D = np.random.normal(self.hydro_forces, np.abs(0.2*self.hydro_forces), size=self.hydro_forces.shape)
		noise_matrix_C = np.random.normal(self.coriolis_centripetal_forces, np.abs(0.2*self.coriolis_centripetal_forces), size=self.coriolis_centripetal_forces.shape)
		noise_matrix_G = np.random.normal(self.ext_forces, np.abs(0.2*self.ext_forces), size=self.ext_forces.shape)
		noise_matrix_M = np.random.normal(self.robot.M, np.abs(0.2*self.robot.M), size=self.robot.M.shape)
		noise_matrix_M_inv = np.random.normal(self.robot.M_inv, np.abs(0.2*self.robot.M_inv), size=self.robot.M_inv.shape)


		A = noise_matrix_M_inv @ (noise_matrix_D + noise_matrix_C + noise_matrix_G)
		B_inv = noise_matrix_M
		V = self.k * eta_err + nu_err

		self.u = - B_inv @ A + B_inv @ V

		self.cmd = self.u
	