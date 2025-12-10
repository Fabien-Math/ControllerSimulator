from ocean import Ocean
from robot import Robot
from viewer.viewer import Viewer
import tqdm
import numpy as np
import sys

class GraphicalSimulationManager:
	def __init__(self, simulation_params, robot_params, environment_params):
		self.dt = simulation_params['timestep']
		self.end_time = simulation_params['end_time']

		self.robot = Robot(robot_params)
		self.robot.log = True
		self.ocean = Ocean(environment_params)

		self.viewer = Viewer(self.robot, self.dt)


	def simulate(self):
		
		failed = False

		for i in tqdm.tqdm(range(int(self.end_time/self.dt))):
			self.robot.update(self.dt, self.ocean)

			if np.isnan(np.sum(self.robot.eta)) or np.linalg.norm(self.robot.nu1) > 4  or np.linalg.norm(self.robot.nu2) > 2 * np.pi:
				failed = True
				break

			if self.robot.controller.mission_finished:
				break

		if failed:
			sys.stderr.write("Mission FAILED!\nRobot fucked up!\n")
			if i <= 1:
				exit()

		if self.robot.controller.mission_finished:
			print(f"Mission accomplished!\nTotal simulation time: {i*self.dt:.3f} s")
		else:
			print(f"Mission NOT finished!\nTotal simulation time: {i*self.dt:.3f} s")

		self.robot.logger.np_format()

	def run_viewer(self):
		self.viewer.run()