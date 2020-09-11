#pragma once
#include <vector>
#include <string>



class Agent
{
public:
	Agent()
	{
		m_bCompleteTrip = false;
		m_no_physical_path_flag = false;
		PCE_factor = 1.0;
		fixed_path_flag = false;
		remove_flag = false;
	}

	int agent_id;
	int origin_zone_id;
	int destination_zone_id;

	int meso_origin_node_id;
	int meso_destination_node_id;
	int meso_origin_node_seq_no;
	int meso_destination_node_seq_no;
	int micro_origin_node_id;
	int micro_destination_node_id;
	int micro_origin_node_seq_no;
	int micro_destination_node_seq_no;
	float departure_time_in_min;
	int departure_time_in_simu_interval;
	float arrival_time_in_min;
	int arrival_time_in_simu_interval;
	float travel_time_in_min;

	std::vector<int> meso_path_node_id_vector;
	std::vector<int> meso_path_node_seq_no_vector;
	std::vector<float> meso_path_time_vector;
	std::vector<int> meso_path_link_seq_no_vector;
	std::string path_node_seq_str;
	std::string path_time_seq_str;
	std::vector<int> path_node_id_vector;
	int number_of_nodes;
	int m_bMoveable = 1;
	bool m_bGenereated;
	int origin_zone_seq_no;
	int destination_zone_seq_no;

	bool fixed_path_flag;
	bool remove_flag;

	float PCE_factor;
	float path_cost;
	int m_path_link_seq_no_list_size;

	int m_current_link_seq_no;
	float latest_arrival_time;

	std::vector<int> micro_path_node_seq_no_vector;
	std::vector<int> micro_path_link_seq_no_vector;
	std::vector<int> micro_path_node_id_vector;
	std::vector<int> micro_path_link_id_vector;

	std::vector<int> m_Veh_LinkArrivalTime_in_simu_interval;
	std::vector<int> m_Veh_LinkDepartureTime_in_simu_interval;
	std::vector<int> path_timestamp_list;

	int current_macro_path_seq_no;
	int current_macro_link_seq_no;
	int next_macro_link_seq_no;
	bool m_bCompleteTrip;

	bool m_no_physical_path_flag;

	float original_x;
	float original_y;
};