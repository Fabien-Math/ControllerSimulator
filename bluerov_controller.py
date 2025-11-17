import numpy as np
import tqdm
from bluerov_sim import Simulator
from scipy.spatial.transform import Rotation
import bluerov_plot as bplt
from display3D import run_viewer

class BlueROV:
	def __init__(self, k_smc, lambda_smc, phi):
		self.dt = None
		self.eta = np.zeros(6)
		self.eta_prev = np.zeros(6)
		self.nu = np.zeros(6)

		# MASS
		self.m = 11.5								# kg
		self.rg = np.array([0.0, 0.0, 0.02])			# m
		self.I0 = np.array([[0.16, 0.00, 0.00],
					[0.00, 0.16, 0.00],
					[0.00, 0.00, 0.16]])		# k.m2
		self.Ma = np.diag([5.5, 12.7, 14.57, 0.12, 0.12, 0.12])
		
		# DAMPING
		self.Dl = np.diag([4.03, 6.22, 5.18, 0.07, 0.07, 0.07])
		self.Dq = np.diag([18.18, 21.66, 36.99, 1.55, 1.55, 1.55])


		self.controller = Controller(k_smc, lambda_smc, phi)
		self.control_force = np.zeros(6)
	
	def update(self, dt, gamma):
		self.dt = dt
		self.controller.update(self.dt, self.eta, self.nu)
		self.control_force = self.controller.f_thrust
		self.gamma = gamma
		# Velocity calculation
		self.compute_nu()
		# Position calculation
		self.compute_eta()
	
	def compute_nu(self):
		self.nu = (self.eta - self.eta_prev) / self.dt
	
	def compute_eta(self):
		# Verlet integration for position
		new_eta = 2*self.eta - self.eta_prev + self.dt**2 * self.gamma
		self.eta_prev = self.eta
		self.eta = new_eta


		


# Define thruster system
class ThrusterSystem:
	def __init__(self, T, n_thrusters, thruster_limits=(-20, 20), wn=30, zeta=0.7):
		self.T = T
		self.Tp = np.matmul(np.linalg.inv(np.matmul(T.T, T)), T.T)
		self.n_thrusters = n_thrusters
		self.min_force, self.max_force = thruster_limits

		# 2nd-order actuator model for each thruster: x'' + 2ζωx' + ω²x = ω²u
		self.wn = wn  # Natural frequency
		self.zeta = zeta  # Damping ratio

		# States: force and velocity for each thruster
		self.force = np.zeros(n_thrusters)
		self.rpm = np.zeros(n_thrusters)

	def update(self, dt, u_cmd):
		# Compute desired thruster forces using pseudo-inverse control allocation
		thruster_cmd = self.T.T @ u_cmd

		# Clip desired thruster commands to actuator physical limits
		thruster_cmd = np.clip(thruster_cmd, self.min_force, self.max_force)

		# Update thruster forces using second-order propeller dynamics
		# Discretize using explicit Euler (or better with RK4 if you want more accuracy)
		accel = self.wn**2 * (thruster_cmd - self.force) - 2 * self.zeta * self.wn * self.rpm
		self.rpm += accel * dt
		self.force += self.rpm * dt

		# Clip again to be sure the physical force doesn't exceed limits
		self.force = np.clip(self.force, self.min_force, self.max_force)



# Sliding Mode Controller class for 3D
class SlidingModeController3D:
	def __init__(self, k_smc, lambda_smc, phi):
		self.k_smc = k_smc
		self.lambda_smc = lambda_smc
		self.phi = phi
		
		self.s = np.zeros(6)
		self.u = np.zeros(6)

	def compute(self, eta_err, nu_err):
		self.s = self.lambda_smc * (- eta_err) + nu_err
		self.u = - self.k_smc * np.array([np.tanh(si / self.phi) for si in self.s])

	
class Controller:
	def __init__(self, k_smc, lambda_smc, phi):
		self.desired_tfs = []
		self.desired_tf = None
		self.last_desired_tf = None
		self.mission_finished = False

		self.eta_err_world = np.zeros(6)
		self.eta_err = np.zeros(6)
		self.eta_tol = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
		self.nu_err = np.zeros(6)
		self.nu_tol = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01])

		self.T = np.array([
			[  0.866,   0.866,  -0.839,  -0.839,   0.000,   0.000,   0.000,   0.000],
			[ -0.500,   0.500,  -0.545,   0.545,   0.000,   0.000,   0.000,   0.000],
			[  0.000,   0.000,   0.000,   0.000,   1.000,   1.000,   1.000,   1.000],
			[ -0.033,   0.033,   0.0359, -0.0359,  0.122,   0.122,  -0.117,  -0.117],
			[  0.0654,  0.0654,  0.0653,  0.0653,  0.218,  -0.218,   0.218,  -0.218],
			[ -0.158,   0.158,   0.166,  -0.166,   0.000,   0.000,   0.000,   0.000]
		])
		self.thrusters = ThrusterSystem(T=self.T, n_thrusters=8, thruster_limits=(-35, 45), wn=8, zeta=0.9)
		self.f_thrust = np.zeros(6)

		self.SMC = SlidingModeController3D(k_smc=k_smc, lambda_smc=lambda_smc, phi=phi)

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
			if np.all(np.abs(self.eta_err) < self.eta_tol) and np.all(np.abs(self.nu_err) < self.nu_tol) :
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
		self.eta_err_world = self.desired_tf - eta
		eta_err = self.desired_tf - eta
		if np.linalg.norm(eta_err[:3]) > 1.0:
			eta_err[:3] /= np.linalg.norm(eta_err[:3])

		if np.linalg.norm(eta_err[3:]) > 0.5:
			eta_err[3:] /= np.linalg.norm(eta_err[3:])

		self.eta_err = eta_err
		self.nu_err = nu
		
	def update(self, dt, eta, nu):
		self.manage_waypoint(eta, nu)

		self.compute_error(eta, nu)


		self.SMC.compute(self.eta_err, self.nu_err)
		self.thrusters.update(dt, self.SMC.u)  # Apply thruster dynamics
		self.f_thrust = self.T @ self.thrusters.force


def build_T(R, t):
	T = np.zeros((4, 4), dtype=float)
	T[0:3, 0:3] = R
	T[0:3, 3] = t
	T[3, 3] = 1
	return T


def compute_total_distance(etas):
	d = 0
	for i in range(len(etas) - 1):
		d += np.linalg.norm(etas[i+1] - etas[i])
	print(f"Total distance: {d:.3f} m")
	return d



def simulate(k_smc, lambda_smc, phi, current_params, train=False):
	dt = 0.02
	
	bl = BlueROV(k_smc, lambda_smc, phi)
	# bl.eta = np.array([0, 0, 0, 0, 0, 0])
	bl.controller.add_waypoint(np.array([[0.5, 5.0, 0, 0, 0, 0], [-4.0, 0.0, -3.0, 0, 0, 0], [0, 0, 10, 0, 0, 0]]))
	sim = Simulator(dt, bl.eta, bl.nu, bl.m, bl.rg, bl.I0, bl.Ma, bl.Dl, bl.Dq, current_params)

	N = 5000

	# LOGGING
	gammas = np.zeros((N, 6))
	time = np.zeros(N)
	nus = np.zeros((N, 6))
	etas = np.zeros((N, 6))
	quats = np.zeros((N, 4))
	eta_err_worlds = np.zeros((N, 6))
	eta_errs = np.zeros((N, 6))
	cmds = np.zeros((N, 6))
	forces = np.zeros((N, 8))
	forces_6 = np.zeros((N, 6))

	for i in range(N):
		sim.update_states(bl.eta, bl.nu)
		sim.update(bl.control_force)
		bl.update(dt, sim.gamma)


		gammas[i] = bl.gamma
		eta_err_worlds[i] = bl.controller.eta_err_world
		eta_errs[i] = bl.controller.eta_err
		nus[i] = bl.nu
		etas[i] = bl.eta
		cmds[i] = bl.controller.SMC.u
		forces[i] = bl.controller.thrusters.force
		forces_6[i] = bl.controller.f_thrust
		time[i] = (i+1)*dt
		quats[i] = Rotation.from_euler('zyx', bl.eta[3:]).as_quat()

		if bl.controller.mission_finished:
			print(f"Total simulation time: {i*dt:.3f} s")
			if train:
				return i * dt
			break
	

	if not train:
		compute_total_distance(etas)
		# bplt.plot(time[:i], eta_err_worlds[:i], 'abs eta errors')
		# bplt.plot(time[:i], eta_errs[:i], 'eta errors')
		# bplt.plot(time[:i], etas[:i], 'eta')
		# bplt.plot(time[:i], cmds[:i], 'U cmd')
		# bplt.plot(time[:i], nus[:i], 'nu')
		# bplt.plot(time[:i], forces_6[:i], 'forces_6')
		# bplt.plot_nm(time[:i], forces[:i], 4, 2, 'C force cmd')

		# bplt.show()

		# bplt.plot_3D(etas[:i])
		run_viewer(etas[:i:5])

def main(most_probable_k_values = None, most_probable_l_values = None):
	current_params = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
	if most_probable_k_values is not None and most_probable_l_values is not None:
		k_smc 		= most_probable_k_values
		lambda_smc 	= most_probable_l_values
	else:
		k_smc 		= np.array([4, 4, 5, 2, 2, 2])
		lambda_smc 	= np.array([2, 2, 2, 2, 2, 2])
	phi = 0.8
	t = simulate(k_smc, lambda_smc, phi, current_params)


def train():
	current_params = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
	# current_params = [(0.5, -0.5, 0), (0.05, 0.05, 0)]
	refining_step = 10
	nb_pool = 5
	pool_size = 5

	phi = 0.8
	# most_probable_k_values = [80.28652322, 63.13938178, 31.62434937, 39.31356554, 17.15406625, 82.97375505]
	most_probable_k_values = np.array([15.467418184576106, 16.914037455647865, 12.894820929807382, 0.1054479995697465, 0.719519180074537, 9.936626577721984])
	most_probable_l_values = np.array([0.8572380182629944, 0.704922768864575, 2.5543558503257913, 1.8157235640885756, 1.5516382871745191, 2.0466708522707298])
	k_disparity = 10
	l_disparity = 3

	min_t = 1e9
	for i in tqdm.tqdm(range(refining_step)):
		params = {'k':None, 'l':None, 't':None}
		for _ in tqdm.tqdm(range(nb_pool)):
			pool_most_probable_k_values = np.abs(np.random.normal(most_probable_k_values, k_disparity))
			pool_most_probable_l_values = np.abs(np.random.normal(most_probable_l_values, l_disparity))
			pool_k_disparity = 1
			pool_l_disparity = 0.2

			pool_params = {'k':None, 'l':None, 't':None}
			pool_min_t = 1e9
			for _ in tqdm.tqdm(range(pool_size)):
				k_smc = np.abs(np.random.normal(pool_most_probable_k_values, pool_k_disparity))
				lambda_smc = np.abs(np.random.normal(pool_most_probable_l_values, pool_l_disparity))
				
				t = simulate(k_smc, lambda_smc, phi, current_params, True)
				if t is None:
					continue

				if t < pool_min_t:
					pool_params['k'] = k_smc
					pool_params['l'] = lambda_smc
					pool_params['t'] = t
					pool_min_t = t
					print(f"Min time: {t:.3f}")
			
			if pool_params['t'] is None:
				continue
			
			if pool_params['t'] < min_t:
				params['k'] = pool_params['k']
				params['l'] = pool_params['l']
				params['t'] = pool_params['t']
				min_t = pool_params['t']

		if params['t'] is not None:
			most_probable_k_values = params['k']
			most_probable_l_values = params['l']
			print(f"\n#####   #####\n\nMin time at iteration {i}: {params['t']:.3f}s\n\n#####   #####")
	
	print("k_smc:", list(most_probable_k_values))
	print("lambda_smc:", list(most_probable_l_values))
	print("min time:", min_t)

	if min_t < 1e8:
		return most_probable_k_values, most_probable_l_values
	return None, None
	# bplt.scatter1V(xs, times)
	# bplt.show()


if __name__ == '__main__':
	most_probable_k_values, most_probable_l_values = None, None
	most_probable_k_values = np.array([15.467418184576106, 16.914037455647865, 12.894820929807382, 5.1054479995697465, 5.719519180074537, 9.936626577721984])
	most_probable_l_values = np.array([0.8572380182629944, 0.704922768864575, 2.5543558503257913, 1.8157235640885756, 1.5516382871745191, 2.0466708522707298])
	# most_probable_k_values, most_probable_l_values = train()
	main(most_probable_k_values, most_probable_l_values)

### No current
# k_smc: [5.40700645, 38.68051198, 20.20015466, 15.35008216, 17.04343491, 17.28390245]
# lambda_smc: [3.05515945, 5.30145031, 3.45391764, 4.60171323, 1.15856585, 4.67305537]
# min time: 29.56

# k_smc: [18.22584036, 51.80568287, 20.38428097, 18.37189206, 10.40452132, 13.19087835]
# lambda_smc: [1.70350738, 1.50472984, 7.6521665, 3.06896862, 4.75386163, 1.71341387]
# min time: 31.62

# k_smc = [11.633583113769175, 44.727143482250575, 21.75386047445603, 30.64154971360749, 9.17427840251083, 18.396265025582213]
# lambda_smc = [3.2953302921902368, 1.8096671647533182, 6.526687792359065, 6.186242550394451, 3.272071934664858, 1.7517318598337623]
# min time: 28.12

# k_smc: [28.737069424207913, 23.516074242820004, 30.72879137491225, 12.296151863669806, 1.5484619940566118, 10.34478325124432]
# lambda_smc: [1.3920025174626218, 1.4357256668152414, 0.9425134604584097, 1.0241314741991898, 2.2218472153223305, 1.9270292249620224]
# min time: 45.57


# k_smc: [31.034326784012247, 19.80204922156703, 37.641414808222855, 17.021009903243492, 6.019285410604078, 15.582081789447745]
# lambda_smc: [0.4809934812706951, 0.8091172870045026, 1.2026095137376307, 2.330303520374732, 1.7487693992756814, 0.691156074389597]
# min time: 35.67


# k_smc: [24.923345363680642, 27.787589010717248, 35.464239888581744, 14.653490919725549, 5.01017593122984, 9.193287213097445]
# lambda_smc: [0.8922695346347758, 1.4406285168852726, 1.417835171548203, 0.5426852426710681, 4.295472667577729, 3.140243390945363]
# min time: 36.51

# k_smc: [15.467418184576106, 16.914037455647865, 12.894820929807382, 7.7054479995697465, 3.719519180074537, 9.936626577721984]
# lambda_smc: [0.8572380182629944, 0.704922768864575, 2.5543558503257913, 1.8157235640885756, 1.5516382871745191, 2.0466708522707298]
# min time: 43.35