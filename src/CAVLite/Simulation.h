#pragma once
#include "Agent.h"
#include <map>
#include "Network.h"

class Simulation
{
public:
	Simulation()
	{
		number_of_agents = 0;
	}

	enum SimulationState {
		SIMSTATE_LOADING,
		SIMSTATE_RUNNING,
	};

	int number_of_agents;
	std::vector<Agent> agent_vector;

	std::map<int, int> internal_agent_seq_no_dict;
	std::vector<Agent> active_agent_queue;

	float simulation_start_time, simulation_end_time, simulation_duration;
	int start_simu_interval_no, end_simu_interval_no, simulation_intervals;

	float simulation_step;
	int total_assignment_iteration;
	//float time_headway;
	int demand_source;

	int number_of_simu_interval_per_min;
	//int safe_headway_in_simu_interval;
	std::vector<int> active_agent_vector;
	std::vector<int> agent_index;
	int current_active_agent_index;

	Network *net;
	std::string cflc_model;

	int *node_predecessor;
	float *node_label_cost;
	int *link_predecessor;

	std::string output_filename;

	void SimulationInitialization();
	int MacroShortestPath(int origin_node_id, int destination_node_id);
	void findPathForAgents(int iteration_no);
	void TrafficAssignment();
	void TrafficSimulation();
	void loadVehicles(int t);

	void exportSimulationResults();

};