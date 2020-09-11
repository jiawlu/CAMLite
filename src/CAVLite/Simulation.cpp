#include "Simulation.h"
#include <algorithm>
#include <queue>
#include "config.h"
#include <iostream>
#include <fstream>
#include "CACF.h"
#include "StopProgram.h"

void Simulation::SimulationInitialization()
{
	simulation_duration = simulation_end_time - simulation_start_time;
	
	start_simu_interval_no = round(simulation_start_time * 60 / simulation_step); //use relative time to save memory
	end_simu_interval_no = round(simulation_end_time * 60 / simulation_step);
	number_of_simu_interval_per_min = round(60.0 / simulation_step);

	for (int i = 0; i < number_of_agents; i++)
		agent_index.push_back(i);

	std::sort(agent_index.begin(), agent_index.end(), [&](const int& a, const int& b) {
		return (agent_vector[a].departure_time_in_simu_interval < agent_vector[b].departure_time_in_simu_interval); });

	current_active_agent_index = 0;

	//GetWhiteList(working_directory);			// update


	// ----------------- micro origin for each agent---------------------//
	MesoNode *start_node;
	MesoLink *start_link;
	Agent *p_agent;
	int seq_no;

	for (int i = 0; i < number_of_agents; i++)
	{
		p_agent = &agent_vector[i];
		start_node = &net->meso_node_vector[p_agent->meso_origin_node_seq_no];
		start_link = &net->meso_link_vector[net->meso_link_id_to_seq_no_dict[start_node->m_outgoing_link_vector[0]]];		// update: collector
		seq_no = (rand() % start_link->micro_incoming_node_id_vector.size());
		p_agent->micro_origin_node_id = start_link->micro_incoming_node_id_vector[seq_no];
		p_agent->micro_origin_node_seq_no = net->micro_node_id_to_seq_no_dict[p_agent->micro_origin_node_id];
	}
}


int Simulation::MacroShortestPath(int origin_node_id, int destination_node_id)
{
	int origin_node_seq_no = net->meso_node_id_to_seq_no_dict[origin_node_id];
	int destination_node_seq_no = net->meso_node_id_to_seq_no_dict[destination_node_id];

	if (net->meso_node_vector[origin_node_seq_no].m_outgoing_link_vector.size() == 0)
		return 0;

	node_predecessor = new int[net->number_of_meso_nodes];
	node_label_cost = new float[net->number_of_meso_nodes];
	link_predecessor = new int[net->number_of_meso_nodes];

	for (int i = 0; i < net->number_of_meso_nodes; i++)
	{
		node_predecessor[i] = -1;
		node_label_cost[i] = _MAX_COST_TREE;
		link_predecessor[i] = -1;
	}

	node_label_cost[origin_node_seq_no] = 0;

	std::queue<int> SEList;
	SEList.push(origin_node_seq_no);

	MesoNode *from_node, *to_node;
	MesoLink *outgoing_link;
	while (!SEList.empty())
	{
		int from_node_seq_no = SEList.front();
		SEList.pop();
		from_node = &net->meso_node_vector[from_node_seq_no];

		for (int i = 0; i < from_node->m_outgoing_link_vector.size(); i++)
		{
			outgoing_link = &net->meso_link_vector[net->meso_link_id_to_seq_no_dict[from_node->m_outgoing_link_vector[i]]];
			to_node = &net->meso_node_vector[net->meso_node_id_to_seq_no_dict[outgoing_link->to_node_id]];
			float new_to_node_cost = node_label_cost[from_node->node_seq_no] + outgoing_link->cost;

			if (new_to_node_cost < node_label_cost[to_node->node_seq_no])
			{
				node_label_cost[to_node->node_seq_no] = new_to_node_cost;
				node_predecessor[to_node->node_seq_no] = from_node->node_seq_no;
				link_predecessor[to_node->node_seq_no] = outgoing_link->link_seq_no;
				SEList.push(to_node->node_seq_no);
			}
		}
	}

	if (node_label_cost[destination_node_seq_no] < _MAX_COST_TREE)
		return 1;
	else
		return -1;

}


void Simulation::findPathForAgents(int iteration_no)
{
	for (int i = 0; i < number_of_agents; i++)
	{
		Agent *p_agent = &agent_vector[i];

		//if (p_agent->agent_id == 130)
		//{
		//	std::cout << "pause";
		//}

		if (p_agent->m_no_physical_path_flag)
			continue;

		if (p_agent->fixed_path_flag)
		{
			int number_of_links_in_path = p_agent->meso_path_link_seq_no_vector.size();
			for (int j = 0; j < number_of_links_in_path; j++)
				net->meso_link_vector[p_agent->meso_path_link_seq_no_vector[j]].flow_volume += 1;
			continue;
		}

		if (i % (iteration_no + 1) != 0)
		{
			int number_of_links_in_path = p_agent->meso_path_link_seq_no_vector.size();
			for (int j = 0; j < number_of_links_in_path; j++)
				net->meso_link_vector[p_agent->meso_path_link_seq_no_vector[j]].flow_volume += 1;
			continue;
		}

		p_agent->meso_path_node_seq_no_vector.clear();
		p_agent->meso_path_link_seq_no_vector.clear();

		int SP_flag = MacroShortestPath(p_agent->meso_origin_node_id, p_agent->meso_destination_node_id);

		if (SP_flag == -1)
		{
			std::cout << "no physical path for agent " << p_agent->agent_id << " from node " << p_agent->meso_origin_node_id << " to node " << p_agent->meso_destination_node_id << "\n";
			p_agent->m_no_physical_path_flag = true;
			continue;
		}

		int current_node_seq_no = net->meso_node_id_to_seq_no_dict[p_agent->meso_destination_node_id];
		p_agent->path_cost = node_label_cost[current_node_seq_no];
		int current_link_seq_no;

		std::vector<int> meso_path_node_seq_no_vector_r;
		std::vector<int> meso_path_link_seq_no_vector_r;

		while (current_node_seq_no >= 0)
		{
			if (current_node_seq_no >= 0)
			{
				current_link_seq_no = link_predecessor[current_node_seq_no];

				if (current_link_seq_no >= 0)
					meso_path_link_seq_no_vector_r.push_back(current_link_seq_no);

				meso_path_node_seq_no_vector_r.push_back(current_node_seq_no);
			}

			current_node_seq_no = node_predecessor[current_node_seq_no];
		}


		for (int j = meso_path_node_seq_no_vector_r.size() - 1; j >= 0; j--)
			p_agent->meso_path_node_seq_no_vector.push_back(meso_path_node_seq_no_vector_r[j]);
		for (int j = meso_path_link_seq_no_vector_r.size() - 1; j >= 0; j--)
			p_agent->meso_path_link_seq_no_vector.push_back(meso_path_link_seq_no_vector_r[j]);

		int number_of_links_in_path = p_agent->meso_path_link_seq_no_vector.size();
		for (int j = 0; j < number_of_links_in_path; j++)
			net->meso_link_vector[p_agent->meso_path_link_seq_no_vector[j]].flow_volume += 1;

	}

}


void Simulation::TrafficAssignment()
{
	std::cout << "Traffic Assignment...";
	for (int j = 0; j < net->number_of_meso_links; j++)
	{
		net->meso_link_vector[j].Initialization();
	}

	for (int i = 0; i < total_assignment_iteration; i++)
	{
		for (int j = 0; j < net->number_of_meso_links; j++)
		{
			net->meso_link_vector[j].CalculateBPRFunctionAndCost();
			net->meso_link_vector[j].flow_volume = 0;
		}
		findPathForAgents(i);
	}
	std::cout << "Done\n";
}


void Simulation::TrafficSimulation()
{
	if (cflc_model == "CACF")
		VehControllerCA::init(net, simulation_step);

	int cumulative_count = 0;
	int print_frequency = round(120.0 / simulation_step);
	float simulation_min;

	int agent_idd = -1;

	for (int t = start_simu_interval_no; t <= end_simu_interval_no; t++)
	{
		loadVehicles(t);
		std::vector<int> agent_remove_vector;

		for (int i = 0; i < active_agent_vector.size(); i++)
		{
			Agent *p_agent = &agent_vector[active_agent_vector[i]];
			if (p_agent->m_Veh_LinkDepartureTime_in_simu_interval.back() == t)
			{
				agent_idd = p_agent->agent_id;

				if (cflc_model == "CACF")
					VehControllerCA::moveVeh(p_agent, t);
				if (p_agent->remove_flag)
					agent_remove_vector.push_back(p_agent->agent_id);
			}

		}

		for (int j = 0; j < agent_remove_vector.size(); j++){
			for (std::vector<int>::iterator it = active_agent_vector.begin(); it != active_agent_vector.end(); it++){
				if (*it == agent_remove_vector[j]){
					active_agent_vector.erase(it);
					break;
				}
			}
		}

		if (cumulative_count%print_frequency == 0)
		{
			simulation_min = t * simulation_step / 60.0;
			printf("\rTraffic Simulation...%.1f|%.1f(current time|end time)",simulation_min,simulation_end_time);
		}
		cumulative_count++;

	}
	std::cout << " Done\n";
}


void Simulation::loadVehicles(int t)
{
	if ((t - start_simu_interval_no) % number_of_simu_interval_per_min == 0)
	{
		while (current_active_agent_index < number_of_agents &&
			agent_vector[agent_index[current_active_agent_index]].departure_time_in_simu_interval >= t &&
			agent_vector[agent_index[current_active_agent_index]].departure_time_in_simu_interval < t + number_of_simu_interval_per_min)
		{
			Agent *p_agent = &agent_vector[agent_index[current_active_agent_index]];
			if (p_agent->m_no_physical_path_flag)
			{
				current_active_agent_index++;
				continue;
			}

			p_agent->m_bGenereated = true;
			p_agent->current_macro_path_seq_no = 0;
			p_agent->micro_path_node_seq_no_vector.push_back(p_agent->micro_origin_node_seq_no);
			p_agent->micro_path_node_id_vector.push_back(p_agent->micro_origin_node_id);
			p_agent->m_Veh_LinkArrivalTime_in_simu_interval.push_back(t);
			p_agent->m_Veh_LinkDepartureTime_in_simu_interval.push_back(p_agent->departure_time_in_simu_interval);

			p_agent->current_macro_link_seq_no = p_agent->meso_path_link_seq_no_vector[p_agent->current_macro_path_seq_no];
			if (p_agent->current_macro_path_seq_no < p_agent->meso_path_link_seq_no_vector.size() - 1)
				p_agent->next_macro_link_seq_no = p_agent->meso_path_link_seq_no_vector[p_agent->current_macro_path_seq_no + 1];
			else
				p_agent->next_macro_link_seq_no = -1;

			active_agent_vector.push_back(p_agent->agent_id);
			current_active_agent_index++;
		}
	}
}


void Simulation::exportSimulationResults()
{
	std::cout << "Outputing Simulation Results...";

	std::ofstream fileOutputAgent(output_filename);
	if (!fileOutputAgent)
	{
		std::cout << "\nError: Cannot open file " << output_filename << std::endl;
		stopProgram();
	}

	fileOutputAgent << "agent_id,o_zone_id,d_zone_id,path_id,o_node_id,d_node_id,agent_type,demand_period,time_period,"\
		"volume,cost,departure_time_in_min,arrival_time_in_min,travel_time,complete_flag,distance,opti_cost,oc_diff,relative_diff,node_sequence,link_sequence,time_sequence,time_decimal_sequence\n";


	for (int i = 0; i < number_of_agents; i++)
	{
		Agent *p_agent = &agent_vector[i];

		if (!p_agent->m_bGenereated) continue;

		int number_of_nodes = p_agent->micro_path_node_id_vector.size();
		std::string complete_flag = p_agent->m_bCompleteTrip ? "c" : "n";

		std::string path_node_sequence = std::to_string(p_agent->micro_path_node_id_vector[0]);
		std::string path_time_sequence = std::to_string(p_agent->m_Veh_LinkDepartureTime_in_simu_interval[0] * simulation_step / 60);
		for (int j = 1; j < number_of_nodes; j++)
		{
			path_node_sequence += (";" + std::to_string(p_agent->micro_path_node_id_vector[j]));
			path_time_sequence += (";" + std::to_string(p_agent->m_Veh_LinkDepartureTime_in_simu_interval[j] * simulation_step / 60));
		}

		std::string path_link_sequence = std::to_string(p_agent->micro_path_link_id_vector[0]);
		for (int j = 1; j < number_of_nodes - 1; j++)
		{
			path_link_sequence += (";" + std::to_string(p_agent->micro_path_link_id_vector[j]));
		}

		fileOutputAgent << p_agent->agent_id << "," << p_agent->origin_zone_id << "," << p_agent->destination_zone_id << ",," << p_agent->micro_origin_node_id << "," <<
			p_agent->micro_destination_node_id << ",,,,1,," << p_agent->departure_time_in_min << "," << p_agent->arrival_time_in_min << "," <<
			p_agent->travel_time_in_min << "," << complete_flag << ",,,,," << path_node_sequence << "," << path_link_sequence << ",," <<
			path_time_sequence << "\n";
	}
	fileOutputAgent.close();
	std::cout << "Done\n";

	std::cout << "Finished. Press Enter key to exit.\n";
	getchar();
	exit(0);
}
