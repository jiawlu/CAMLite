#include "CACF.h"
#include "stdafx.h"
#include "CSVParser.h"
#include "config.h"
#include <iostream>


float VehControllerCA::default_time_headway;
int VehControllerCA::default_safe_headway_in_simu_interval;
std::map<int, float> VehControllerCA::time_headway_dict;
std::map<int, int> VehControllerCA::safe_headway_in_simu_interval_dict;
float VehControllerCA::simulation_step;
Network *VehControllerCA::net;

void VehControllerCA::init(Network *net_, float simulation_step_)
{
	cout << "Initializing CA car following model.\n";
	simulation_step = simulation_step_;
	default_safe_headway_in_simu_interval = round(default_time_headway / simulation_step);
	net = net_;

	CCSVParser parser_hw;
	if (parser_hw.OpenCSVFile("CACF_time_headway.csv", true))
	{
		cout << "  CACF_time_headway.csv detected, loading data.\n";
		int micro_node_id;
		float time_headway;

		while (parser_hw.ReadRecord())
		{
			parser_hw.GetValueByFieldName("micro_node_id", micro_node_id);
			parser_hw.GetValueByFieldName("headway", time_headway);

			time_headway_dict[micro_node_id] = time_headway;
			safe_headway_in_simu_interval_dict[micro_node_id] = round(time_headway / simulation_step);
		}

		cout << "  " << time_headway_dict.size() << " nodes loaded, default value (" << default_time_headway << " seconds) will apply to other nodes.\n";
	}

}

void VehControllerCA::moveVeh(Agent *p_agent, int t)
{
	MicroNode *current_node = &net->micro_node_vector[p_agent->micro_path_node_seq_no_vector.back()];
	int current_node_idd = current_node->node_id;

	int node_time_headway_in_simu_interval;
	map<int, int>::iterator iter_hw;
	iter_hw = safe_headway_in_simu_interval_dict.find(current_node->node_id);
	if (iter_hw != safe_headway_in_simu_interval_dict.end())
		node_time_headway_in_simu_interval = iter_hw->second;
	else
		node_time_headway_in_simu_interval = default_safe_headway_in_simu_interval;


	std::vector<int>::iterator ret;
	ret = std::find(net->meso_link_vector[p_agent->current_macro_link_seq_no].micro_outgoing_node_id_vector.begin(),
		net->meso_link_vector[p_agent->current_macro_link_seq_no].micro_outgoing_node_id_vector.end(), current_node->node_id);

	if (ret != net->meso_link_vector[p_agent->current_macro_link_seq_no].micro_outgoing_node_id_vector.end())
	{
		if (p_agent->current_macro_path_seq_no == p_agent->meso_path_link_seq_no_vector.size() - 1)
		{
			//agent_remove_vector.push_back(p_agent->agent_id);
			p_agent->remove_flag = true;
			p_agent->arrival_time_in_simu_interval = t;
			p_agent->arrival_time_in_min = t * simulation_step / 60;
			p_agent->micro_destination_node_id = current_node->node_id;
			p_agent->m_bCompleteTrip = true;
			p_agent->travel_time_in_min = p_agent->arrival_time_in_min - p_agent->departure_time_in_min;
			current_node->available_sim_interval = t;
			return; // continue;
		}
		else
		{
			p_agent->current_macro_path_seq_no++;
			p_agent->current_macro_link_seq_no = p_agent->meso_path_link_seq_no_vector[p_agent->current_macro_path_seq_no];
			if (p_agent->current_macro_path_seq_no < p_agent->meso_path_link_seq_no_vector.size() - 1)
				p_agent->next_macro_link_seq_no = p_agent->meso_path_link_seq_no_vector[p_agent->current_macro_path_seq_no + 1];
			else
				p_agent->next_macro_link_seq_no = -1;
		}
	}

	MicroLink *outgoing_link_keeping = NULL;
	MicroNode *to_node_keeping = NULL;
	float cost_keeping;

	MicroLink *outgoing_link_changing = NULL;
	MicroNode *to_node_changing = NULL;
	float cost_changing;

	std::vector<MicroLink*> outgoing_link_changing_vector;
	std::vector<MicroNode*> to_node_changing_vector;
	std::vector<float> cost_changing_vector;

	float tree_cost;

	bool whitelist_flag = false;
	std::vector<int> whitelist_vector;
	std::map<int, std::map<int, std::vector<int>>>::iterator iter1 = net->meso_link_vector[p_agent->current_macro_link_seq_no].white_list_dict.find(p_agent->next_macro_link_seq_no);
	if (iter1 != net->meso_link_vector[p_agent->current_macro_link_seq_no].white_list_dict.end())
	{
		std::map<int, std::vector<int>>::iterator iter2 = net->meso_link_vector[p_agent->current_macro_link_seq_no].white_list_dict[p_agent->next_macro_link_seq_no].find(current_node->node_id);
		if (iter2 != net->meso_link_vector[p_agent->current_macro_link_seq_no].white_list_dict[p_agent->next_macro_link_seq_no].end())
		{
			whitelist_flag = true;
			whitelist_vector = net->meso_link_vector[p_agent->current_macro_link_seq_no].white_list_dict[p_agent->next_macro_link_seq_no][current_node->node_id];
		}
	}


	for (int j = 0; j < current_node->m_outgoing_link_vector.size(); j++)
	{
		MicroLink *outgoing_link = &net->micro_link_vector[net->micro_link_id_to_seq_no_dict[current_node->m_outgoing_link_vector[j]]];

		if (net->meso_link_id_to_seq_no_dict[outgoing_link->macro_link_id] != p_agent->current_macro_link_seq_no)
			continue;

		bool valid = true;
		if (whitelist_flag)
		{
			std::vector<int>::iterator iter1;
			iter1 = std::find(whitelist_vector.begin(), whitelist_vector.end(), outgoing_link->to_node_id);
			if (iter1 == whitelist_vector.end())
				valid = false;
		}


		if (outgoing_link->cell_type == 1)		//travelling cell
		{
			outgoing_link_keeping = outgoing_link;
			to_node_keeping = &net->micro_node_vector[net->micro_node_id_to_seq_no_dict[outgoing_link->to_node_id]];

			if (!valid)
			{
				cost_keeping = _MAX_COST_TREE;
				continue;
			}

			tree_cost = net->meso_link_vector[p_agent->current_macro_link_seq_no].estimated_cost_tree_for_each_movement[p_agent->next_macro_link_seq_no][to_node_keeping->node_seq_no];

			if (tree_cost == _MAX_COST_TREE)
			{
				cost_keeping = _MAX_COST_TREE;
			}
			else if (to_node_keeping->available_sim_interval > t + outgoing_link->free_flow_travel_time_in_simu_interval)
			{
				cost_keeping = _MAX_TIME_INTERVAL;
			}
			else
			{
				cost_keeping = outgoing_link->free_flow_travel_time_in_simu_interval + outgoing_link->additional_cost + tree_cost;
			}

		}
		else
		{
			//lane changing cell
			outgoing_link_changing_vector.push_back(outgoing_link);
			to_node_changing = &net->micro_node_vector[net->micro_node_id_to_seq_no_dict[outgoing_link->to_node_id]];

			if (!valid)
			{
				cost_changing = _MAX_COST_TREE;
				to_node_changing_vector.push_back(to_node_changing);
				cost_changing_vector.push_back(cost_changing);
				continue;
			}

			tree_cost = net->meso_link_vector[p_agent->current_macro_link_seq_no].estimated_cost_tree_for_each_movement[p_agent->next_macro_link_seq_no][to_node_changing->node_seq_no];

			if (tree_cost == _MAX_COST_TREE)
			{
				cost_changing = _MAX_COST_TREE;
			}
			else if (to_node_changing->available_sim_interval > t + outgoing_link->free_flow_travel_time_in_simu_interval)
			{
				//cost_changing = to_node_changing->available_sim_interval + outgoing_link->additional_cost +
				//	g_macro_link_vector[p_agent->current_macro_link_seq_no].estimated_cost_tree_for_each_movement[p_agent->next_macro_link_seq_no][to_node_changing->node_seq_no];
				cost_changing = _MAX_TIME_INTERVAL;
			}
			else
			{
				cost_changing = outgoing_link->free_flow_travel_time_in_simu_interval + outgoing_link->additional_cost + tree_cost;
			}

			to_node_changing_vector.push_back(to_node_changing);
			cost_changing_vector.push_back(cost_changing);
		}
	}

	if (to_node_keeping == NULL)
	{
		p_agent->m_Veh_LinkDepartureTime_in_simu_interval.back()++;
		return; // continue;
	}

	if (cost_changing_vector.empty())
	{
		if (cost_keeping == _MAX_COST_TREE)
		{
			//remove
			//agent_remove_vector.push_back(p_agent->agent_id);
			p_agent->remove_flag = true;
			p_agent->m_bCompleteTrip = false;
			current_node->available_sim_interval = t;

			std::cout << "Warning: vehicle " << p_agent->agent_id << " was removed from micronode " << current_node->node_id << " at time " << \
				t * simulation_step / 60 << " due to accessibility\n";
		}
		else if (cost_keeping < _MAX_TIME_INTERVAL)
		{
			to_node_keeping->available_sim_interval = _MAX_TIME_INTERVAL;
			current_node->available_sim_interval = t + node_time_headway_in_simu_interval;
			p_agent->micro_path_node_seq_no_vector.push_back(to_node_keeping->node_seq_no);
			p_agent->micro_path_link_seq_no_vector.push_back(outgoing_link_keeping->link_seq_no);
			p_agent->micro_path_node_id_vector.push_back(to_node_keeping->node_id);
			p_agent->micro_path_link_id_vector.push_back(outgoing_link_keeping->link_id);
			p_agent->m_Veh_LinkArrivalTime_in_simu_interval.push_back(t);
			p_agent->m_Veh_LinkDepartureTime_in_simu_interval.push_back(t + outgoing_link_keeping->free_flow_travel_time_in_simu_interval);
		}
		else
		{
			p_agent->m_Veh_LinkDepartureTime_in_simu_interval.back()++;
			return; // continue;
		}
	}
	else
	{
		std::vector<float>::iterator smallest = std::min_element(std::begin(cost_changing_vector), std::end(cost_changing_vector));
		int smallest_position = std::distance(std::begin(cost_changing_vector), smallest);
		cost_changing = cost_changing_vector[smallest_position];
		to_node_changing = to_node_changing_vector[smallest_position];
		outgoing_link_changing = outgoing_link_changing_vector[smallest_position];

		if (cost_keeping == _MAX_COST_TREE && cost_changing == _MAX_COST_TREE)
		{
			//remove
			//agent_remove_vector.push_back(p_agent->agent_id);
			p_agent->remove_flag = true;
			p_agent->m_bCompleteTrip = false;
			current_node->available_sim_interval = t;

			std::cout << "Warning: vehicle " << p_agent->agent_id << " was removed from micronode " << current_node->node_id << " at time " << \
				t * simulation_step / 60 << " due to accessibility\n";

			return; // continue;
		}

		if (cost_keeping < _MAX_TIME_INTERVAL && cost_keeping < cost_changing)
		{
			to_node_keeping->available_sim_interval = _MAX_TIME_INTERVAL;
			current_node->available_sim_interval = t + node_time_headway_in_simu_interval;
			p_agent->micro_path_node_seq_no_vector.push_back(to_node_keeping->node_seq_no);
			p_agent->micro_path_link_seq_no_vector.push_back(outgoing_link_keeping->link_seq_no);
			p_agent->micro_path_node_id_vector.push_back(to_node_keeping->node_id);
			p_agent->micro_path_link_id_vector.push_back(outgoing_link_keeping->link_id);
			p_agent->m_Veh_LinkArrivalTime_in_simu_interval.push_back(t);
			p_agent->m_Veh_LinkDepartureTime_in_simu_interval.push_back(t + outgoing_link_keeping->free_flow_travel_time_in_simu_interval);
		}
		else if (cost_changing < _MAX_TIME_INTERVAL && cost_changing < cost_keeping)
		{
			to_node_changing->available_sim_interval = _MAX_TIME_INTERVAL;
			current_node->available_sim_interval = t + node_time_headway_in_simu_interval;
			p_agent->micro_path_node_seq_no_vector.push_back(to_node_changing->node_seq_no);
			p_agent->micro_path_link_seq_no_vector.push_back(outgoing_link_changing->link_seq_no);
			p_agent->micro_path_node_id_vector.push_back(to_node_changing->node_id);
			p_agent->micro_path_link_id_vector.push_back(outgoing_link_changing->link_id);
			p_agent->m_Veh_LinkArrivalTime_in_simu_interval.push_back(t);
			p_agent->m_Veh_LinkDepartureTime_in_simu_interval.push_back(t + outgoing_link_changing->free_flow_travel_time_in_simu_interval);
		}
		else if (cost_keeping < _MAX_TIME_INTERVAL && abs(cost_keeping - cost_changing) < 0.1)
		{

			if (rand() / double(RAND_MAX) < 0.06)
			{
				to_node_changing->available_sim_interval = _MAX_TIME_INTERVAL;
				current_node->available_sim_interval = t + node_time_headway_in_simu_interval;
				p_agent->micro_path_node_seq_no_vector.push_back(to_node_changing->node_seq_no);
				p_agent->micro_path_link_seq_no_vector.push_back(outgoing_link_changing->link_seq_no);
				p_agent->micro_path_node_id_vector.push_back(to_node_changing->node_id);
				p_agent->micro_path_link_id_vector.push_back(outgoing_link_changing->link_id);
				p_agent->m_Veh_LinkArrivalTime_in_simu_interval.push_back(t);
				p_agent->m_Veh_LinkDepartureTime_in_simu_interval.push_back(t + outgoing_link_changing->free_flow_travel_time_in_simu_interval);
			}
			else
			{
				to_node_keeping->available_sim_interval = _MAX_TIME_INTERVAL;
				current_node->available_sim_interval = t + node_time_headway_in_simu_interval;
				p_agent->micro_path_node_seq_no_vector.push_back(to_node_keeping->node_seq_no);
				p_agent->micro_path_link_seq_no_vector.push_back(outgoing_link_keeping->link_seq_no);
				p_agent->micro_path_node_id_vector.push_back(to_node_keeping->node_id);
				p_agent->micro_path_link_id_vector.push_back(outgoing_link_keeping->link_id);
				p_agent->m_Veh_LinkArrivalTime_in_simu_interval.push_back(t);
				p_agent->m_Veh_LinkDepartureTime_in_simu_interval.push_back(t + outgoing_link_keeping->free_flow_travel_time_in_simu_interval);
			}
		}
		else
		{
			p_agent->m_Veh_LinkDepartureTime_in_simu_interval.back()++;
			//continue;
		}

	}
}
