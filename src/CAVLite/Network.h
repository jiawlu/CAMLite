#pragma once
#include "Agent.h"
#include <queue>
#include <map>


class MesoNode
{
public:
	MesoNode()
	{
		valid = true;
		iszone = false;
	}

	std::string name;
	int node_id;
	int node_seq_no;
	bool iszone;
	int control_type;
	std::string control_type_name;
	int cycle_length_in_second;
	float x_original;		//used to update location when dragging in GUI
	float y_original;
	float x;
	float y;
	float lon;
	float lat;
	std::string geometry_str;
	std::vector<int> m_outgoing_link_vector;
	std::vector<int> m_incoming_link_vector;
	bool valid;



};

class MesoLink
{
public:
	MesoLink()
	{
		valid = true;
		isconnector = false;
		BPR_alpha = 0.15;
		BPR_beta = 4.0;
		flow_volume = 0;
		link_id_of_original_link = -1;
	}

	std::string name;
	std::string link_key;
	int link_id;
	int link_seq_no;
	int from_node_id;
	int to_node_id;
	float length;
	int number_of_lanes;
	int speed_limit;
	int lane_cap;
	int flow_volume;
	float BPR_alpha;
	float BPR_beta;
	int link_capacity;
	float travel_time;
	float cost;
	int link_id_of_original_link;		//link id of link from external data

	std::map<int, std::map<int, std::vector<int>>> white_list_dict;

	bool valid;
	bool isconnector;


	std::vector<float*> geometry_lonlat_vector;
	std::vector<float*> geometry_vector;

	std::vector<std::vector<int>> micro_node_set;			//micro node id, lane by lane;
	std::vector<int> micro_link_set;			//micro link id;

	float free_flow_travel_time_in_min;
	int free_flow_travel_time_in_sim_interval;

	std::queue<Agent*> m_active_agent_queue;

	std::vector<int> micro_incoming_node_id_vector;
	std::vector<int> micro_outgoing_node_id_vector;

	std::map<int, std::vector<int>> turning_node_seq_no_dict;             //macro_link_seq_no:micro_node_seq_no
	std::map<int, std::map<int, float>> estimated_cost_tree_for_each_movement;     //macro_link_seq_no : node_seq_no: cost

	void CalculateBPRFunctionAndCost()
	{
		travel_time = free_flow_travel_time_in_min * (1 + BPR_alpha * pow(flow_volume / link_capacity, BPR_beta));
		cost = travel_time;
	}

	void Initialization()
	{
		link_capacity = number_of_lanes * lane_cap;
		free_flow_travel_time_in_min = length / speed_limit * 0.06;
	}

};

class MicroNode
{
public:
	MicroNode()
	{
		valid = true;
		available_sim_interval = 0;
	}
	int node_id;
	int node_seq_no;
	float x;
	float y;
	int meso_link_id;
	int lane_no;
	bool valid;
	std::vector<int> m_outgoing_link_vector;
	std::vector<int> m_incoming_link_vector;
	int available_sim_interval;

};


class MicroLink
{
public:
	MicroLink()
	{
		valid = true;
	}

	int link_id;
	int link_seq_no;
	int from_node_id;
	int to_node_id;
	int macro_link_id;
	int lane_no;
	float length;
	int speed_limit;
	float free_flow_travel_time_in_min;
	int free_flow_travel_time_in_simu_interval;
	float additional_cost;
	bool valid;
	int cell_type;	//1:traveling; 2:changing

	float from_x, from_y, to_x, to_y;

	std::queue<Agent*> m_active_agent_queue;




};

class Network
{
public:
	std::string coordinate_type;

	std::vector<MicroNode> micro_node_vector;
	std::vector<MicroLink> micro_link_vector;
	std::vector<MesoNode> meso_node_vector;
	std::vector<MesoLink> meso_link_vector;

	std::map<int, int> meso_node_id_to_seq_no_dict;
	std::map<std::string, int> meso_link_key_to_seq_no_dict;
	std::map<int, int> meso_link_id_to_seq_no_dict;

	std::map<int, int> micro_node_id_to_seq_no_dict;
	std::map<int, int> micro_link_id_to_seq_no_dict;

	std::vector<int> micro_destination_seq_no_vector;

	int number_of_meso_nodes = 0;
	int number_of_meso_links = 0;

	int meso_node_id = 0;
	int meso_link_id = 0;

	int number_of_micro_nodes = 0;
	int number_of_micro_links = 0;

	int micro_node_id = 0;
	int micro_link_id = 0;

	float network_center[2];

	void init();
	void getMicroInOutNodesOfMesolink();
	void generateCostTree();
	
	float* MicroShortestPath(int destination_node_seq_no, MesoLink *mesolink);

};