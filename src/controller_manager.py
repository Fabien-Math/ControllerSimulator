import numpy as np
from scipy.spatial.transform import Rotation as R

from sliding_mode_controller import SlidingModeController
from feedback_controller import FeedbackController
from pid_controller import PIDController
from thruster_system import ThrusterSystem

class ControllerManager:
	def __init__(self, robot, controller_params: dict, thruster_system: ThrusterSystem):
		self.desired_etas = []
		self.desired_eta = None
		self.last_desired_eta = None
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
		elif controller_params['type'] == 'FDB':
			self.controller = FeedbackController(robot, controller_params)
		elif controller_params['type'] == 'PID':
			self.controller = PIDController(robot, controller_params)


	def add_waypoint(self, wps, overwrite = False):
		if overwrite:
			if len(np.shape(wps) > 1):
				self.desired_etas = wps
			else:
				self.desired_etas = [wps]
		else:
			if len(np.shape(wps)) > 1:
				for wp in wps:
					self.desired_etas.append(wp)
			else:
				self.desired_etas.append(wps)
	

	def manage_waypoint(self, eta, nu):
		if self.desired_eta is not None:
			if np.all(np.abs(self.etas_err) < self.eta_tol) and np.all(np.abs(self.nus_err) < self.nu_tol) :
				self.last_desired_eta = self.desired_eta
				# print('Arrived')
				self.desired_eta = None
			else:
				return
		
		if self.desired_eta is None:
			if len(self.desired_etas):
				self.desired_eta = self.desired_etas.pop(0)
			else:
				self.mission_finished = True
				if self.last_desired_eta is not None:
					self.desired_eta = self.last_desired_eta
				else:
					self.desired_eta = eta
	

	def compute_error(self, eta, nu):
		self.etas_err_world = self.desired_eta - eta
		eta_err = np.zeros(6)
		eta_err = self.robot.J.T @ (self.desired_eta - eta)

		self.etas_err = eta_err
		self.nus_err = - nu
	

	def update(self, dt, eta, nu):
		self.manage_waypoint(eta, nu)

		self.compute_error(eta, nu)

		self.controller.update(dt, self.etas_err, self.nus_err)
		self.thrusters.update(dt, self.controller.cmd + self.cmd_offset)  # Apply thruster dynamics
		