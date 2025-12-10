import numpy as np
import scipy.optimize as spo

from robot_ghost import RobotGhost

# Sliding Mode Controller class for 3D
class MPController:
	def __init__(self, robot, controller_params):
		self.k = np.array(controller_params['k'])
		self.robot = robot
		self.u = np.zeros(6)
		self.cmd = np.zeros(6)

		self.N_horizon = 20
		self.dt = 0.05

		# self.u_lowerbound = np.repeat(-self.k, self.N_horizon).flatten()
		# self.u_upperbound = np.repeat(self.k, self.N_horizon).flatten()

		self.u_lowerbound = np.repeat(-1 + np.zeros(6), self.N_horizon).flatten()
		self.u_upperbound = np.repeat(1 + np.zeros(6), self.N_horizon).flatten()


		self.pose_weight_matrix = np.eye(6)
		self.u_weight_matrix = np.eye(6)
		self.du_weight_matrix = np.eye(6)
		self.final_weight = 1

		self.result_shape = (self.N_horizon, 6)

		self.raw_result = spo.OptimizeResult(x=np.zeros(6 * self.N_horizon))
		self.bounds = spo.Bounds(lb=self.u_lowerbound, ub=self.u_upperbound)
		self.tolerance = 1e-4
		self.max_iter = 1000
		self.verbose = 2
		self.constraints = ()

		self.U_last = np.reshape(self.raw_result.x, self.result_shape)
		self.ghost = None






	def update(self, dt, eta_err, nu_err):
		# Robot not initialized
		if self.robot.M_eta_inv is None:
			return

		self.Yd = self.robot.controller.desired_eta

		self.raw_result = spo.minimize(
			fun=self.cost_function,
			x0=self.raw_result.x.flatten(),
			tol=self.tolerance,
			bounds=self.bounds,
			constraints=self.constraints,
			# method='Nelder-Mead',
			options={
					'maxiter': self.max_iter, 'disp': self.verbose
			}
		)
		
		# If the minimisation succeed 
		if self.raw_result.success:
			print("Optimisation Succeed with cost:", self.raw_result.fun)
			self.U = np.reshape(self.raw_result.x, self.result_shape)
			print("Succeeded U:", self.U, '\n')
		else:
			print("Optimisation failed with cost:", self.raw_result.fun)

		self.u = self.U[0]
		self.cmd = self.u

	def cost_function(self, U):
		
		self.U = np.reshape(U, self.result_shape)

		# Initialize a ghost according to the current robot state
		self.ghost = RobotGhost(self.robot)
		# Simulate the system for one timestep
		self.ghost.simulate(self.dt, self.U, self.N_horizon)

		self.Y = self.ghost.logger.etas

		print("U:\n", self.U)
		print("Y:\n", self.Y)
		print()

		if np.isnan(self.Y.any()):
			return 1e9
		
		# Computing associated cost
		error = self.Yd - self.Y
		print("Error:")
		print(error)
		u = self.U
		du = (self.U - self.U_last)/self.dt

		# Update last U
		self.U_last = self.U

		cost = 0
		# Pose error cost
		cost += np.sum(error @ self.pose_weight_matrix @ error.T)
		print("Error cost: ", cost)
		# Control cost
		cost += np.sum(u @ self.u_weight_matrix @ u.T)
		print("Control cost: ", cost)
		# Control derivative cost
		cost += np.sum(du @ self.du_weight_matrix @ du.T)
		print("Control derivative cost: ", cost)

		# Normalization on the number of horizon simulated
		cost /= self.N_horizon

		# Final pose error cost
		cost += self.final_weight * np.sum(error[-1])
		print("Final cost: ", cost)

		return cost



def S(v):
	return np.array([[ 0.00, -v[2],  v[1]],
				     [ v[2],  0.00, -v[0]],
					 [-v[1],  v[0],  0.00]])


class Ghost:
	def __init__(self, robot):
		
		# Position and orientation
		self.eta = robot.eta.copy()
		self.eta1, self.eta2 = self.eta[0:3], self.eta[3:6]
		self.J1 = np.zeros((3, 3))
		self.J2 = np.zeros((3, 3))
		self.J = np.zeros((6, 6))

		# Velocity and angular velocity
		self.nu = robot.nu.copy()
		self.nu1, self.nu2 = self.nu[0:3], self.nu[3:6]

		# MASS PROPERTIES
		self.m = robot.m
		self.rg = robot.rg.copy()
		self.I0 = robot.I0.copy()
		# set noise on Ma
		self.Ma = np.random.normal(robot.Ma.copy(), np.abs(0.01*robot.Ma.copy()), size=robot.Ma.copy().shape)
		self.M = robot.Mrb.copy() + self.Ma
		self.M_inv = np.linalg.inv(self.M)


		# DAMPING MATRICES
		# set noise on Dl and Dq
		self.Dl = np.random.normal(robot.Dl.copy(), np.abs(0.01*robot.Dl.copy()), size=robot.Dl.copy().shape)
		self.Dq = np.random.normal(robot.Dq.copy(), np.abs(0.01*robot.Dq.copy()), size=robot.Dq.copy().shape)


		# CORIOLIS MATRICES
		self.Crb = np.zeros((6, 6))
		self.Ca = np.zeros((6, 6))
		self.C = np.zeros((6, 6))

		# VOLUMETRIC FORCES
		self.G = np.zeros(6)

	
	def compute_J1(self):
		"""
		Rotation matrix from body frame to inertial (NED) frame.
		"""
		sphi = np.sin(self.eta2[0])
		cphi = np.cos(self.eta2[0])
		stheta = np.sin(self.eta2[1])
		ctheta = np.cos(self.eta2[1])
		spsi = np.sin(self.eta2[2])
		cpsi = np.cos(self.eta2[2])

		self.J1 = np.array([
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

	def compute_J2(self):
		"""
		J2 matrix from Fossen formalism (Euler angle kinematics).
		"""
		sphi = np.sin(self.eta2[0])
		cphi = np.cos(self.eta2[0])
		ttheta = np.tan(self.eta2[1])
		ctheta = np.cos(self.eta2[1])

		self.J2 =  np.array([
			[1.0,      sphi * ttheta,      cphi * ttheta],
			[0.0,      cphi,               -sphi],
			[0.0,      sphi / ctheta,      cphi / ctheta]
		])
	
	def compute_J(self):
		self.compute_J1()
		self.compute_J2()
		self.J[0:3, 0:3] = self.J1
		self.J[3:6, 3:6] = self.J2


	def compute_Crb(self):	# Page 8
		self.Crb[0:3, 3:6] = -self.m * S(self.nu1) - self.m * S(S(self.nu2) @ self.rg)
		self.Crb[3:6, 0:3] = -self.m * S(self.nu1) - self.m * S(S(self.nu2) @ self.rg)
		self.Crb[3:6, 3:6] = -self.m * S(S(self.nu1) @ self.rg) - S(self.I0 @ self.nu2)


	def compute_Ca(self):	# Page 9
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
		self.D = self.Dl + np.diag(np.diag(self.Dq) * np.abs(self.nu))


	def compute_G(self):
		self.G = np.zeros(6)
		self.G[2] = -2	# N (Restitution force: gravity + buoyancy)

	def compute_gamma(self, dt, U):
		hydro_forces = - np.matmul(self.D, self.nu)
		forces = - np.matmul(self.C, self.nu) + hydro_forces + self.G + U
		# Update acceleration
		self.gamma = np.matmul(self.M_inv, forces)
	
	def compute_nu(self, dt):
		# Explicit Euler integration for velocity
		self.nu += self.gamma * dt
		self.nu1, self.nu2 = self.nu[0:3], self.nu[3:6]

	def compute_eta(self, dt):
		# Explicit Euler integration for position
		self.eta += (self.J @ self.nu) * dt
		self.eta1, self.eta2 = self.eta[0:3], self.eta[3:6]

	def simulate(self, dt, U, N_horizon):
		
		Y = np.zeros((N_horizon, 6))

		for i in range(N_horizon):
			self.compute_J()

			# DAMPING
			self.compute_D()
			# CORIOLIS AND CENTRIPETAL
			self.compute_C()
			# EXTERNAL FORCES
			self.compute_G()

			# Acceleration calculation
			self.compute_gamma(dt, U[i])

			# Velocity calculation
			self.compute_nu(dt)
			# Position calculation
			self.compute_eta(dt)

			Y[i] = self.eta

		return Y
