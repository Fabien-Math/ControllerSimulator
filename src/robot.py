import numpy as np

from controller import ControllerManager
from thruster_system import ThrusterSystem
from logging_system import LoggingSystem


def S(v):
	return np.array([[ 0.00, -v[2],  v[1]],
				     [ v[2],  0.00, -v[0]],
					 [-v[1],  v[0],  0.00]])


class Robot:
	def __init__(self, robot_params):
		# Position and orientation
		self.eta = robot_params["initial_conditions"]["eta"]
		self.eta_prev = self.eta
		self.eta1, self.eta2 = self.eta[0:3], self.eta[3:6]

		# Velocity and angular velocity
		self.nu = robot_params["initial_conditions"]["nu"]
		self.nu_rel = self.nu
		self.nu1, self.nu2 = self.nu[0:3], self.nu[3:6]

		# Acceleration and angular acceleration
		self.gamma = np.zeros(6)
		self.forces = np.zeros(6)
		self.hydro_forces = np.zeros(6)

		# MASS PROPERTIES
		mass_prop = robot_params["mass_properties"]
		self.m = mass_prop["m"]
		self.rg = np.array(mass_prop["rg"])
		self.I0 = np.array(mass_prop["I0"])
		self.Mrb = np.zeros((6, 6))
		self.Mrb[0:3, 0:3] =  self.m * np.identity(3)	# kg
		self.Mrb[0:3, 3:6] = -self.m * S(self.rg)				# kg.m
		self.Mrb[3:6, 0:3] =  self.m * S(self.rg)				# kg.m
		self.Mrb[3:6, 3:6] =  self.I0					# kg.m2
		self.Ma = np.array(mass_prop["Ma"])
		self.M = self.Mrb + self.Ma
		self.Minv = np.linalg.inv(self.M)

		# DAMPING MATRICES
		damp_prop = robot_params["damping"]
		self.Dl = np.array(damp_prop["Dl"])
		self.Dq = np.array(damp_prop["Dq"])

		# CORIOLIS MATRICES
		self.Crb = np.zeros((6, 6))
		self.Ca = np.zeros((6, 6))
		self.C = np.zeros((6, 6))

		self.thrusters = ThrusterSystem(robot_params["thruster"])
		self.controller = ControllerManager(robot_params["controller"], self.thrusters)
		self.controller.desired_tfs = list(robot_params['mission'])

		self.time = 0
		self.log = True
		self.logger = LoggingSystem(self)


	def compute_Crb(self):
		self.Crb[0:3, 3:6] = -self.m * S(self.nu1) - self.m * S(self.nu2) * S(self.rg)
		self.Crb[3:6, 0:3] = -self.m * S(self.nu1) + self.m * S(self.rg) * S(self.nu2)
		self.Crb[3:6, 3:6] = -np.matmul(self.I0, self.nu2)


	def compute_Ca(self):
		Ma11 = self.Ma[0:3, 0:3]
		Ma12 = self.Ma[0:3, 3:6]
		Ma21 = self.Ma[3:6, 0:3]
		Ma22 = self.Ma[3:6, 3:6]
		self.Ca[0:3, 3:6] = - S(np.matmul(Ma11, self.nu1) + np.matmul(Ma12, self.nu2))
		self.Ca[3:6, 0:3] = - S(np.matmul(Ma11, self.nu1) + np.matmul(Ma12, self.nu2))
		self.Ca[3:6, 3:6] = - S(np.matmul(Ma21, self.nu1) + np.matmul(Ma22, self.nu2))


	def compute_C(self):
		self.compute_Crb()
		self.compute_Ca()
		self.C = self.Crb + self.Ca


	def compute_D(self):
		self.D = self.Dl + np.diag(np.diag(self.Dq) * np.abs(self.nu_rel))


	def compute_T(self, dt):
		self.T = np.zeros(6)
		self.T[2] = -2	# N (Restitution force: gravity + buoyancy)

		self.controller.update(dt, self.eta, self.nu)
		self.T += self.thrusters.force


	def compute_gamma(self):
		self.hydro_forces = - np.matmul(self.D, self.nu_rel)
		self.forces = - np.matmul(self.C, self.nu) + self.hydro_forces + self.T
		# Update acceleration
		self.gamma = np.matmul(self.Minv, self.forces)


	def update(self, dt, env):
		# Update relative fluid velocity
		self.nu_rel = self.nu - env.compute_fluid_vel(self.eta)
		# DAMPING
		self.compute_D()
		# CORIOLIS AND CENTRIPETAL
		self.compute_C()
		# EXTERNAL FORCES
		self.compute_T(dt)

		# Acceleration calculation
		self.compute_gamma()

		# Velocity calculation
		self.compute_nu(dt)
		# Position calculation
		self.compute_eta(dt)

		self.time += dt
		if self.log:
			self.logger.log_state(self.time)
	
	def compute_nu(self, dt):
		self.nu = (self.eta - self.eta_prev) / dt
	
	def compute_eta(self, dt):
		# Verlet integration for position
		new_eta = 2*self.eta - self.eta_prev + dt**2 * self.gamma
		self.eta_prev = self.eta
		self.eta = new_eta