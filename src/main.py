from simulation_parser import load_config
from simulation_manager import SimulationManager
from graphical_simulation_manager import GraphicalSimulationManager
from graph_system import GraphSystem
import time

def main():
	# filename = "config/bluerov_config.yaml"
	filename = "config/bluerov_config_round.yaml"
	# filename = "config/nautile_config.yaml"
	
	launch_time = time.strftime("%Y-%m-%d_%H-%M-%S")
 
	scenario_params = load_config(filename=filename)
	
	simulation_params, robot_params, environment_params = scenario_params

	sim_manager = None
	if simulation_params['graphical']:
		sim_manager = GraphicalSimulationManager(simulation_params, robot_params, environment_params)
	else:
		sim_manager = SimulationManager(simulation_params, robot_params, environment_params)

	if sim_manager is not None:
		sim_manager.simulate()

	if simulation_params['save_csv']:
		sim_manager.robot.logger.save_to_csv(simulation_params['save_output'], launch_time)

	GraphSystem(sim_manager.robot.logger, simulation_params['show_graphs'], simulation_params['save_graphs'], simulation_params['save_output'], launch_time)

	if simulation_params['graphical']:
		sim_manager.run_viewer()


if __name__ == '__main__':
	main()