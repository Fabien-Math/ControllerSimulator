import numpy as np
from scipy.signal import place_poles 


# Sliding Mode Controller class for 3D
class PolePlacementController:
	def __init__(self, robot, controller_params):
		self.p = np.array(controller_params['p'], dtype=float)
		self.robot = robot
		self.u = np.zeros(6)
		self.cmd = np.zeros(6)

	def update(self, dt, eta_err, nu_err):
		print('\n')

		print("##########   ERROR   ##########\n")
		print("  POLE PLACEMENT NOT WORKING!\n")
		print("##########   ERROR   ##########\n")
		exit()

		if self.robot.M_eta_inv is None:
			return

		self.hydro_forces = - np.matmul(self.robot.D, self.robot.nu_rel)
		self.coriolis_centripetal_forces = - np.matmul(self.robot.C, self.robot.nu)
		self.ext_forces = self.robot.G

		noise_matrix_D = np.random.normal(self.hydro_forces, np.abs(0*self.hydro_forces), size=self.hydro_forces.shape)
		noise_matrix_C = np.random.normal(self.coriolis_centripetal_forces, np.abs(0*self.coriolis_centripetal_forces), size=self.coriolis_centripetal_forces.shape)
		noise_matrix_G = np.random.normal(self.ext_forces, np.abs(0*self.ext_forces), size=self.ext_forces.shape)
		noise_matrix_M = np.random.normal(self.robot.M, np.abs(0*self.robot.M), size=self.robot.M.shape)
		noise_matrix_M_inv = np.random.normal(self.robot.M_inv, np.abs(0*self.robot.M_inv), size=self.robot.M_inv.shape)

		M = np.ones((12, 12))
		M[:6, :6] = noise_matrix_M @ np.diag(noise_matrix_D + noise_matrix_C)

		CD = np.zeros((12, 12))
		CD[:6, :6] = noise_matrix_C + noise_matrix_D
		CD[6:, 6:] = self.robot.J

		B = np.linalg.inv(M)
		A = B @ CD

		K = place_poles(A[:6, :6], B[:6, :6], self.p)
		np.set_printoptions(threshold=np.inf, linewidth=np.inf)
		print(A - B @ K.gain_matrix[:6, :6])

		V = - K.gain_matrix[:6, :6] @ nu_err - K.gain_matrix[6:, 6:] @ eta_err
		self.u = V
		self.cmd = self.u
	