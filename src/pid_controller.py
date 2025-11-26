import numpy as np

# Sliding Mode Controller class for 3D
class PIDController:
	def __init__(self, robot, controller_params):
		self.k_p = controller_params['kp']
		self.k_i = controller_params['ki']
		self.k_d = controller_params['kd']

		self.int_eta_err = np.zeros(6)
		self.der_eta_err = np.zeros(6)
		self.last_eta_err = np.zeros(6)
		self.cmd = np.zeros(6)

	def update(self, dt, eta_err, nu_err):
		self.int_eta_err += eta_err * dt
		self.int_eta_err = np.clip(self.int_eta_err, -5, 5)
  
		if dt:
			self.der_eta_err = (eta_err - self.last_eta_err ) / dt
			self.last_eta_err = eta_err
   
		self.cmd = self.k_p * eta_err + self.k_i * self.int_eta_err + self.k_d * self.der_eta_err

	