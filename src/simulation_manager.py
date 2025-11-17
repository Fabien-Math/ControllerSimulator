from ocean import Ocean
from robot import Robot
import tqdm

class SimulationManager:
	def __init__(self, simulation_params, robot_params, environment_params):
		self.dt = simulation_params['timestep']
		self.end_time = simulation_params['end_time']

		self.robot = Robot(robot_params)
		self.ocean = Ocean(environment_params)

	def simulate(self):
		for i in tqdm.tqdm(range(int(self.end_time/self.dt))):
			self.robot.update(self.dt, self.ocean)

			if self.robot.controller.mission_finished:
				print(f"Mission accomplished!\nTotal simulation time: {i*self.dt:.3f} s")
				break