import numpy as np
from scipy.spatial.transform import Rotation as R

from sliding_mode_controller import SlidingModeController
from pid_controller import PIDController
from thruster_system import ThrusterSystem

class ControllerManager:
	def __init__(self, robot, controller_params: dict, thruster_system: ThrusterSystem):
		self.desired_tfs = []
		self.desired_tf = None
		self.last_desired_tf = None
		self.mission_finished = False

		self.etas_err_world = np.zeros(6)
		self.etas_err = np.zeros(6)
		self.eta_tol = controller_params['eta_tol']
		self.nus_err = np.zeros(6)
		self.nu_tol = controller_params['nu_tol']
		self.cmd_offset = controller_params['cmd_offset']

		self.robot = robot
		self.T = thruster_system.T
		self.thrusters = thruster_system
		self.f_thrust = np.zeros(6)

		if controller_params['type'] == 'SMC':
			self.controller = SlidingModeController(robot, controller_params)
		elif controller_params['type'] == 'PID':
			self.controller = PIDController(robot, controller_params)


	def add_waypoint(self, wps, overwrite = False):
		if overwrite:
			if len(np.shape(wps) > 1):
				self.desired_tfs = wps
			else:
				self.desired_tfs = [wps]
		else:
			if len(np.shape(wps)) > 1:
				for wp in wps:
					self.desired_tfs.append(wp)
			else:
				self.desired_tfs.append(wps)
	

	def manage_waypoint(self, eta, nu):
		if self.desired_tf is not None:
			if np.all(np.abs(self.etas_err) < self.eta_tol) and np.all(np.abs(self.nus_err) < self.nu_tol) :
				self.last_desired_tf = self.desired_tf
				# print('Arrived')
				self.desired_tf = None
			else:
				return
		
		if self.desired_tf is None:
			if len(self.desired_tfs):
				self.desired_tf = self.desired_tfs.pop(0)
			else:
				self.mission_finished = True
				if self.last_desired_tf is not None:
					self.desired_tf = self.last_desired_tf
				else:
					self.desired_tf = eta
	

	def compute_error(self, eta, nu):
		self.etas_err_world = self.desired_tf - eta
		eta_err = self.desired_tf - eta
		
		J1 = R.from_euler('xyz', eta[3:]).as_matrix()
		R_d = R.from_euler('xyz', self.desired_tf[3:]).as_matrix()
		S = 0.5 * (R_d.T @ J1 - J1.T @ R_d).T   # skew-symmetric matrix

		eta_err[:3] = J1.T @ eta_err[:3]

		# eta_err[3:] = np.array([
		# 	S[2, 1],
		# 	S[0, 2],
		# 	S[1, 0]
    	# ])


		# if np.linalg.norm(eta_err[:3]) > 1.0:
		# 	eta_err[:3] /= np.linalg.norm(eta_err[:3])
		# if np.linalg.norm(eta_err[3:]) > 0.5:
		# 	eta_err[3:] /= np.linalg.norm(eta_err[3:])

		self.etas_err = eta_err
		self.nus_err = nu
	

	def update(self, dt, eta, nu):
		self.manage_waypoint(eta, nu)

		self.compute_error(eta, nu)

		self.controller.update(dt, self.etas_err, self.nus_err)
		self.thrusters.update(dt, self.controller.cmd + self.cmd_offset)  # Apply thruster dynamics
		