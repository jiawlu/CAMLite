#include "Network.h"
#include <set>
#include "config.h"
#include <queue>
#include <iostream>


void Network::init()
{
	std::cout << "Network initializing...";
	getMicroInOutNodesOfMesolink();
	generateCostTree();
	std::cout << "Done\n";
}


void Network::getMicroInOutNodesOfMesolink()
{
	// --------------micro incoming and outgoing nodes of each meso link------------- //
	MesoLink* p_mesolink;
	MicroLink* p_microlink;
	std::vector<int>::iterator retEndPos;

	for (int i = 0; i < number_of_meso_links; i++)
	{
		p_mesolink = &meso_link_vector[i];

		std::set<int> potential_micro_incoming_node_id_set;
		std::set<int> potential_micro_outgoing_node_id_set;

		for (int j = 0; j < p_mesolink->micro_link_set.size(); j++)
		{
			p_microlink = &micro_link_vector[micro_link_id_to_seq_no_dict[p_mesolink->micro_link_set[j]]];

			potential_micro_incoming_node_id_set.insert(p_microlink->from_node_id);
			potential_micro_outgoing_node_id_set.insert(p_microlink->to_node_id);
		}

		p_mesolink->micro_incoming_node_id_vector.resize(potential_micro_incoming_node_id_set.size());
		p_mesolink->micro_outgoing_node_id_vector.resize(potential_micro_outgoing_node_id_set.size());

		retEndPos = set_difference(potential_micro_incoming_node_id_set.begin(),
			potential_micro_incoming_node_id_set.end(), potential_micro_outgoing_node_id_set.begin(),
			potential_micro_outgoing_node_id_set.end(), p_mesolink->micro_incoming_node_id_vector.begin());
		p_mesolink->micro_incoming_node_id_vector.resize(retEndPos - p_mesolink->micro_incoming_node_id_vector.begin());

		retEndPos = set_difference(potential_micro_outgoing_node_id_set.begin(),
			potential_micro_outgoing_node_id_set.end(), potential_micro_incoming_node_id_set.begin(),
			potential_micro_incoming_node_id_set.end(), p_mesolink->micro_outgoing_node_id_vector.begin());
		p_mesolink->micro_outgoing_node_id_vector.resize(retEndPos - p_mesolink->micro_outgoing_node_id_vector.begin());
	}
}


void Network::generateCostTree()
{
	MesoLink *from_link, *to_link;
	MesoNode *current_node;
	std::vector<int>::iterator ret;

	for (int i = 0; i < number_of_meso_links; i++)
	{
		from_link = &meso_link_vector[i];
		current_node = &meso_node_vector[meso_node_id_to_seq_no_dict[from_link->to_node_id]];

		for (int j = 0; j < from_link->micro_outgoing_node_id_vector.size(); j++)
		{
			int micro_node_id = from_link->micro_outgoing_node_id_vector[j];
			for (int k = 0; k < current_node->m_outgoing_link_vector.size(); k++)
			{
				to_link = &meso_link_vector[meso_link_id_to_seq_no_dict[current_node->m_outgoing_link_vector[k]]];

				ret = std::find(to_link->micro_incoming_node_id_vector.begin(), to_link->micro_incoming_node_id_vector.end(), micro_node_id);
				if (ret != to_link->micro_incoming_node_id_vector.end())
				{
					int micro_node_seq_no = micro_node_id_to_seq_no_dict[micro_node_id];
					if (from_link->turning_node_seq_no_dict.count(to_link->link_seq_no) == 0)
					{
						std::vector<int> node_seq_no_vector;
						node_seq_no_vector.push_back(micro_node_seq_no);
						from_link->turning_node_seq_no_dict[to_link->link_seq_no] = node_seq_no_vector;
					}
					else
					{
						from_link->turning_node_seq_no_dict[to_link->link_seq_no].push_back(micro_node_seq_no);
					}
				}
			}
		}
	}


	// #pragma omp parallel for
	MesoLink *p_link;
	
	for (int i = 0; i < number_of_meso_links; i++)
	{
		p_link = &meso_link_vector[i];
		if (p_link->turning_node_seq_no_dict.empty())
		{
			//no outgoing link
			std::map<int, float>::iterator iter;
			
			for (int p = 0; p < p_link->micro_node_set.size(); p++)
			{
				for (int q = 0; q < p_link->micro_node_set[p].size(); q++)
				{
					p_link->estimated_cost_tree_for_each_movement[-1][micro_node_id_to_seq_no_dict[p_link->micro_node_set[p][q]]] = _MAX_COST_TREE;
				}
			}

			if (p_link->isconnector)
			{
				for (int q = 0; q < p_link->micro_incoming_node_id_vector.size(); q++)
				{
					p_link->estimated_cost_tree_for_each_movement[-1][micro_node_id_to_seq_no_dict[p_link->micro_incoming_node_id_vector[q]]] = _MAX_COST_TREE;
				}

				for (int q = 0; q < p_link->micro_outgoing_node_id_vector.size(); q++)
				{
					p_link->estimated_cost_tree_for_each_movement[-1][micro_node_id_to_seq_no_dict[p_link->micro_outgoing_node_id_vector[q]]] = _MAX_COST_TREE;
				}
			}

			for (int j = 0; j < p_link->micro_outgoing_node_id_vector.size(); j++)
			{
				float *micro_node_label_cost = MicroShortestPath(micro_node_id_to_seq_no_dict[p_link->micro_outgoing_node_id_vector[j]], p_link);

				iter = p_link->estimated_cost_tree_for_each_movement[-1].begin();
				while (iter != p_link->estimated_cost_tree_for_each_movement[-1].end())
				{
					if (micro_node_label_cost[iter->first] < iter->second)
						iter->second = micro_node_label_cost[iter->first];
					iter++;
				}
			}
		}
		else
		{
			//with outgoing link
			std::map<int, std::vector<int>>::iterator iter;
			std::map<int, float>::iterator iter1;
			iter = p_link->turning_node_seq_no_dict.begin();
			while (iter != p_link->turning_node_seq_no_dict.end())
			{
				for (int p = 0; p < p_link->micro_node_set.size(); p++)
				{
					for (int q = 0; q < p_link->micro_node_set[p].size(); q++)
					{
						p_link->estimated_cost_tree_for_each_movement[iter->first][micro_node_id_to_seq_no_dict[p_link->micro_node_set[p][q]]] = _MAX_COST_TREE;
					}
				}

				if (p_link->isconnector)
				{
					for (int q = 0; q < p_link->micro_incoming_node_id_vector.size(); q++)
					{
						p_link->estimated_cost_tree_for_each_movement[iter->first][micro_node_id_to_seq_no_dict[p_link->micro_incoming_node_id_vector[q]]] = _MAX_COST_TREE;
					}

					for (int q = 0; q < p_link->micro_outgoing_node_id_vector.size(); q++)
					{
						p_link->estimated_cost_tree_for_each_movement[iter->first][micro_node_id_to_seq_no_dict[p_link->micro_outgoing_node_id_vector[q]]] = _MAX_COST_TREE;
					}
				}


				for (int j = 0; j < iter->second.size(); j++)
				{
					float *micro_node_label_cost = MicroShortestPath(iter->second[j], p_link);

					iter1 = p_link->estimated_cost_tree_for_each_movement[iter->first].begin();
					while (iter1 != p_link->estimated_cost_tree_for_each_movement[iter->first].end())
					{
						if (micro_node_label_cost[iter1->first] < iter1->second)
							iter1->second = micro_node_label_cost[iter1->first];
						iter1++;
					}

					delete micro_node_label_cost;
				}
				iter++;
			}

		}

	}
}


float * Network::MicroShortestPath(int destination_node_seq_no, MesoLink * mesolink)
{
	if (micro_node_vector[destination_node_seq_no].m_incoming_link_vector.size() == 0)
		return NULL;

	int *micro_node_predecessor = new int[number_of_micro_nodes];
	float *micro_node_label_cost = new float[number_of_micro_nodes];
	int *micro_link_predecessor = new int[number_of_micro_nodes];

	for (int i = 0; i < number_of_micro_nodes; i++)
	{
		micro_node_predecessor[i] = -1;
		micro_node_label_cost[i] = _MAX_COST_TREE;
		micro_link_predecessor[i] = -1;
	}

	micro_node_label_cost[destination_node_seq_no] = 0;

	std::queue<int> SEList;
	SEList.push(destination_node_seq_no);

	while (!SEList.empty())
	{
		int to_node_seq_no = SEList.front();
		SEList.pop();
		MicroNode *to_node = &micro_node_vector[to_node_seq_no];

		for (int i = 0; i < to_node->m_incoming_link_vector.size(); i++)
		{
			MicroLink *incoming_link = &micro_link_vector[micro_link_id_to_seq_no_dict[to_node->m_incoming_link_vector[i]]];
			MicroNode *from_node = &micro_node_vector[micro_node_id_to_seq_no_dict[incoming_link->from_node_id]];
			if (from_node->meso_link_id != mesolink->link_id)
				continue;

			float new_from_node_cost = micro_node_label_cost[to_node->node_seq_no] + incoming_link->free_flow_travel_time_in_simu_interval + incoming_link->additional_cost;

			if (new_from_node_cost < micro_node_label_cost[from_node->node_seq_no])
			{
				micro_node_label_cost[from_node->node_seq_no] = new_from_node_cost;
				micro_node_predecessor[from_node->node_seq_no] = to_node->node_seq_no;
				micro_link_predecessor[from_node->node_seq_no] = incoming_link->link_seq_no;
				SEList.push(from_node->node_seq_no);

			}
		}
	}

	delete micro_node_predecessor;
	delete micro_link_predecessor;
	return micro_node_label_cost;
}
