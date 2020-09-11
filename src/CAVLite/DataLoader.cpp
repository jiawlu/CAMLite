#include "stdafx.h"
#include "DataLoader.h"
#include "CSVParser.h"
#include "StopProgram.h"
#include <random>
#include <ctime>
#include <math.h>
#include "config.h"
#include "CACF.h"



void DataLoader::loadData(Simulation *simulator, Network *net)
{
	DataLoader::simulator = simulator;
	DataLoader::net = net;

	try
	{
		readSettings();
		readNetwork();
		readSignal();
		
		if (!agent_filepath.empty())
		{
			simulator->demand_source = 0;
			readAgent();
		}
		else if (!flow_filepath.empty())
		{
			simulator->demand_source = 1;
			readFlow();
		}
		else
		{
			string msg = "No demand file has been specified.\n";
			throw msg;
		}
	}
	catch (string msg)
	{
		cout << msg;
		stopProgram();
	}

}


static bool readValueFromConfigFile(const string & key, string & value)
{
	fstream cfgFile;
	cfgFile.open("settings.txt");
	if (!cfgFile.is_open())
	{
		string msg = "Cannot open settings.txt\n";
		throw msg;
	}
	char tmp[1000];
	while (!cfgFile.eof())
	{
		cfgFile.getline(tmp, 1000);
		string line(tmp);
		size_t pos = line.find('=');
		if (pos == string::npos) continue;
		string tmpKey = line.substr(0, pos);
		if (key == tmpKey)
		{
			value = line.substr(pos + 1);
			return true;
		}
	}
	return false;
}


static void SplitString(const string & s, vector<string>& v, const string & c)
{
	string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while (string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2 - pos1));

		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length())
		v.push_back(s.substr(pos1));
}


void DataLoader::readSettings()
{
	std::string value, start_time, end_time;
	int hour_int, min_int;

	readValueFromConfigFile("micronode_filepath", micro_node_filepath);
	readValueFromConfigFile("microlink_filepath", micro_link_filepath);
	readValueFromConfigFile("mesonode_filepath", meso_node_filepath);
	readValueFromConfigFile("mesolink_filepath", meso_link_filepath);
	readValueFromConfigFile("coordinate_type", net->coordinate_type);
	
	readValueFromConfigFile("agent_filepath", agent_filepath);
	readValueFromConfigFile("flow_filepath", flow_filepath);

	readValueFromConfigFile("total_assignment_iterations", value);
	simulator->total_assignment_iteration = std::stoi(value);
	readValueFromConfigFile("simulation_start_time", start_time);
	hour_int = stoi(start_time.substr(0, 2));
	min_int = stoi(start_time.substr(2, 2));
	simulator->simulation_start_time = hour_int * 60 + min_int;
	readValueFromConfigFile("simulation_end_time", end_time);
	hour_int = stoi(end_time.substr(0, 2));
	min_int = stoi(end_time.substr(2, 2));
	simulator->simulation_end_time = hour_int * 60 + min_int;
	readValueFromConfigFile("simulation_step", value);
	simulator->simulation_step = std::stof(value);

	readValueFromConfigFile("output_filename", simulator->output_filename);

	readValueFromConfigFile("cflc_model", value);
	vector<string> value_vector;
	SplitString(value, value_vector, ";");
	if (value_vector[0] == "CACF")
	{
		simulator->cflc_model = value_vector[0];
		VehControllerCA::default_time_headway = stof(value_vector[1]);
	}
	else
	{
		cout<< "select cflc from CACF\n";
		stopProgram();
	}


	cout << "SETTINGS:" << endl;
	cout << "[NETWORK]" << endl;
	cout << "micronode_filepath: " << micro_node_filepath << endl;
	cout << "microlink_filepath: " << micro_link_filepath << endl;
	cout << "mesonode_filepath: " << meso_node_filepath << endl;
	cout << "mesolink_filepath: " << meso_link_filepath << endl;
	cout << "coordinate_type: " << net->coordinate_type << endl;

	cout << "[SIMULATION]" << endl;
	cout << "agent_filepath: " << agent_filepath << endl;
	cout << "flow_filepath: " << flow_filepath << endl;
	cout << "total_assignment_iterations: " << simulator->total_assignment_iteration << endl;
	cout << "simulation_start_time: " << start_time << " (" << simulator->simulation_start_time << " min)" << endl;
	cout << "simulation_end_time: " << end_time << " (" << simulator->simulation_end_time << " min)" << endl;
	cout << "simulation_step: " << simulator->simulation_step << " second" << endl;
	//cout << "time_headway: " << simulator->time_headway << " second" << endl;
	cout << endl;
}


static float LonLat2m(float lon1, float lat1, float lon2, float lat2)
{
	float lonrad1 = lon1 * pi / 180;
	float latrad1 = lat1 * pi / 180;
	float lonrad2 = lon2 * pi / 180;
	float latrad2 = lat2 * pi / 180;

	float a = latrad1 - latrad2;
	float b = lonrad1 - lonrad2;
	float cal = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(latrad1) * cos(latrad2) * pow(sin(b / 2), 2))) * 6378.137;
	return cal * 1000;
}


static float * LonLatToXY(float lon, float lat, float lon_central, float lat_central)
{
	static float xy[2];
	float delta_x, delta_y;

	delta_x = LonLat2m(lon_central, lat_central, lon, lat_central);
	delta_y = LonLat2m(lon_central, lat_central, lon_central, lat);
	if (lon < lon_central)
		xy[0] = -1 * delta_x;
	else
		xy[0] = delta_x;

	if (lat < lat_central)
		xy[1] = -1 * delta_y;
	else
		xy[1] = delta_y;

	return xy;
}




static vector<float*> StringCoordinateToVector(const string & s)
{

	string::size_type idx1, idx2;
	idx1 = s.find("<LineString><coordinates>");
	if (idx1 != string::npos)
	{
		// <LineString><coordinates> format
		vector<string> s_str;
		vector<string> ss_str;
		string s_data(s, 25, s.length() - 52);
		SplitString(s_data, s_str, " ");

		vector<float*> coordinate_vector;

		for (int i = 0; i < s_str.size(); i++)
		{
			ss_str.clear();
			SplitString(s_str[i], ss_str, ",");
			float* coordinate = new float[3]{ float(atof(ss_str[0].c_str())), float(atof(ss_str[1].c_str())), float(atof(ss_str[2].c_str())) };
			coordinate_vector.push_back(coordinate);
		}
		return coordinate_vector;
	}
	else
	{
		idx2 = s.find("LINESTRING(");
		if (idx2 != string::npos)
		{
			// LINESTRING format
			vector<string> s_str;
			vector<string> ss_str;
			string s_data(s, 11, s.length() - 12);
			SplitString(s_data, s_str, ",");

			vector<float*> coordinate_vector;

			for (int i = 0; i < s_str.size(); i++)
			{
				ss_str.clear();
				SplitString(s_str[i], ss_str, " ");
				float* coordinate = new float[3]{ float(atof(ss_str[0].c_str())), float(atof(ss_str[1].c_str())), 0.0 };
				coordinate_vector.push_back(coordinate);
			}
			return coordinate_vector;
		}
		else
		{
			// invalid format
		}
	}

}

void DataLoader::readNetwork()
{
	cout << "Loading Network\n";
	float lon_min = 180, lon_max = -180, lat_min = 90, lat_max = -90, lon_central = 0, lat_central = 0;
	float* xy;

	CCSVParser parser_meso_node;
	if (parser_meso_node.OpenCSVFile(meso_node_filepath, true))   // read agent as demand input 
	{
		float x, y;

		while (parser_meso_node.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			MesoNode mesonode;  // create an agent object 

			parser_meso_node.GetValueByFieldName("node_id", mesonode.node_id);
			parser_meso_node.GetValueByFieldName("x_coord", x);
			parser_meso_node.GetValueByFieldName("y_coord", y);

			if (net->coordinate_type == "lonlat")
			{
				mesonode.lon = x;
				mesonode.lat = y;
				if (x < lon_min)
					lon_min = x;
				if (x > lon_max)
					lon_max = x;
				if (y < lat_min)
					lat_min = y;
				if (y > lat_max)
					lat_max = y;
			}
			else if (net->coordinate_type == "localmeter")
			{
				mesonode.x = x;
				mesonode.y = y;
			}
			else if (net->coordinate_type == "localfeet")
			{
				mesonode.x = 0.3048 * x;
				mesonode.y = 0.3048 * y;
			}
			else
			{
				string msg = "select coordinate_type from lonlat, localmeter, localfeet\n";
				throw msg;
			}

			mesonode.node_seq_no = net->number_of_meso_nodes;
			net->meso_node_id_to_seq_no_dict[mesonode.node_id] = mesonode.node_seq_no;

			net->meso_node_vector.push_back(mesonode);
			net->number_of_meso_nodes++;
		}
	}
	else
	{
		string msg = "Cannot open Meso Node file\n";
		throw msg;
	}


	if (net->coordinate_type == "lonlat")
	{
		lon_central = 0.5 * (lon_min + lon_max);
		lat_central = 0.5 * (lat_min + lat_max);

		MesoNode *p_node;
		for (int i = 0; i < net->number_of_meso_nodes; i++)
		{
			p_node = &net->meso_node_vector[i];
			xy = LonLatToXY(p_node->lon, p_node->lat, lon_central, lat_central);
			p_node->x = xy[0];
			p_node->y = xy[1];
		}
	}


	CCSVParser parser_meso_link;
	if (parser_meso_link.OpenCSVFile(meso_link_filepath, true))
	{
		string geometry_str;

		while (parser_meso_link.ReadRecord())
		{
			MesoLink mesolink;

			parser_meso_link.GetValueByFieldName("name", mesolink.name);
			parser_meso_link.GetValueByFieldName("road_link_id", mesolink.link_id);
			parser_meso_link.GetValueByFieldName("from_node_id", mesolink.from_node_id);
			parser_meso_link.GetValueByFieldName("to_node_id", mesolink.to_node_id);
			parser_meso_link.GetValueByFieldName("length", mesolink.length);
			parser_meso_link.GetValueByFieldName("lanes", mesolink.number_of_lanes);
			parser_meso_link.GetValueByFieldName("free_speed", mesolink.speed_limit);
			parser_meso_link.GetValueByFieldName("capacity", mesolink.lane_cap);
			parser_meso_link.GetValueByFieldName("geometry", geometry_str);

			if (mesolink.lane_cap <= 0)
				mesolink.lane_cap = 1200;

			if (net->coordinate_type == "lonlat")
			{
				mesolink.geometry_lonlat_vector = StringCoordinateToVector(geometry_str);

				for (int j = 0; j < mesolink.geometry_lonlat_vector.size(); j++)
				{
					xy = LonLatToXY(mesolink.geometry_lonlat_vector[j][0],
						mesolink.geometry_lonlat_vector[j][1], lon_central, lat_central);
					float* xyz = new float[3]{ xy[0], xy[1], 0 };
					mesolink.geometry_vector.push_back(xyz);
				}
			}
			else if (net->coordinate_type == "localmeter")
			{
				mesolink.geometry_vector = StringCoordinateToVector(geometry_str);
			}
			else if (net->coordinate_type == "localfeet")
			{
				string msg = "localfeet coordinate is not supported\n";
				throw msg;
			}

			mesolink.link_key = to_string(mesolink.from_node_id) + " " + to_string(mesolink.to_node_id);
			mesolink.link_seq_no = net->number_of_meso_links;
			net->meso_link_key_to_seq_no_dict[mesolink.link_key] = mesolink.link_seq_no;

			for (int j = 0; j < mesolink.number_of_lanes; j++)
			{
				vector<int> p_micro_node_set;
				mesolink.micro_node_set.push_back(p_micro_node_set);
			}

			net->meso_link_vector.push_back(mesolink);

			net->meso_node_vector[net->meso_node_id_to_seq_no_dict[mesolink.from_node_id]].m_outgoing_link_vector.push_back(mesolink.link_id);
			net->meso_node_vector[net->meso_node_id_to_seq_no_dict[mesolink.to_node_id]].m_incoming_link_vector.push_back(mesolink.link_id);
			net->meso_link_id_to_seq_no_dict[mesolink.link_id] = mesolink.link_seq_no;

			net->number_of_meso_links++;

		}
	}
	else
	{
		string msg = "Cannot open Meso Link file\n";
		throw msg;
	}


	CCSVParser parser_micro_node;
	if (parser_micro_node.OpenCSVFile(micro_node_filepath, true))   // read agent as demand input 
	{
		float x, y;
		MesoLink *p_mesolink;

		while (parser_micro_node.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			MicroNode micronode;  // create an agent object 

			parser_micro_node.GetValueByFieldName("node_id", micronode.node_id);
			parser_micro_node.GetValueByFieldName("x_coord", x);
			parser_micro_node.GetValueByFieldName("y_coord", y);
			parser_micro_node.GetValueByFieldName("meso_link_id", micronode.meso_link_id);
			parser_micro_node.GetValueByFieldName("lane_no", micronode.lane_no);

			if (net->coordinate_type == "lonlat")
			{
				xy = LonLatToXY(x, y, lon_central, lat_central);
				micronode.x = xy[0];
				micronode.y = xy[1];
			}
			else if (net->coordinate_type == "localmeter")
			{
				micronode.x = x;
				micronode.y = y;
			}
			else if (net->coordinate_type == "localfeet")
			{
				micronode.x = 0.3048 * x;
				micronode.y = 0.3048 * y;
			}
			else
			{
				string msg = "select coordinate_type from lonlat, localmeter, localfeet\n";
				throw msg;
			}

			p_mesolink = &net->meso_link_vector[net->meso_link_id_to_seq_no_dict[micronode.meso_link_id]];
			if (micronode.lane_no < 1 || micronode.lane_no > p_mesolink->number_of_lanes)
			{
				ostringstream ostr;
				ostr << "lane no of micro node " << micronode.node_id << " is not consistent with the number of lanes of its macro link " << micronode.meso_link_id << ".\n";
				string msg = ostr.str();
				throw msg;
			}
			p_mesolink->micro_node_set[micronode.lane_no - 1].push_back(micronode.node_id);

			micronode.node_seq_no = net->number_of_micro_nodes;
			net->micro_node_id_to_seq_no_dict[micronode.node_id] = micronode.node_seq_no;

			net->micro_node_vector.push_back(micronode);
			net->number_of_micro_nodes++;
		}
	}
	else
	{
		string msg = "Cannot open Micro Node file\n";
		throw msg;
	}


	CCSVParser parser_micro_link;
	if (parser_micro_link.OpenCSVFile(micro_link_filepath, true))
	{
		while (parser_micro_link.ReadRecord())
		{
			MicroLink microlink;

			parser_micro_link.GetValueByFieldName("road_link_id", microlink.link_id);
			parser_micro_link.GetValueByFieldName("from_node_id", microlink.from_node_id);
			parser_micro_link.GetValueByFieldName("to_node_id", microlink.to_node_id);
			parser_micro_link.GetValueByFieldName("meso_link_id", microlink.macro_link_id);
			parser_micro_link.GetValueByFieldName("cell_type", microlink.cell_type);
			parser_micro_link.GetValueByFieldName("length", microlink.length);
			parser_micro_link.GetValueByFieldName("free_speed", microlink.speed_limit);
			parser_micro_link.GetValueByFieldName("additional_cost", microlink.additional_cost);

			microlink.link_seq_no = net->number_of_micro_links;
			microlink.from_x = net->micro_node_vector[net->micro_node_id_to_seq_no_dict[microlink.from_node_id]].x;
			microlink.from_y = net->micro_node_vector[net->micro_node_id_to_seq_no_dict[microlink.from_node_id]].y;
			microlink.to_x = net->micro_node_vector[net->micro_node_id_to_seq_no_dict[microlink.to_node_id]].x;
			microlink.to_y = net->micro_node_vector[net->micro_node_id_to_seq_no_dict[microlink.to_node_id]].y;

			if (microlink.cell_type == 1)
			{
				microlink.additional_cost = 0;
			}
			else
			{
				microlink.additional_cost = 2;
			}

			microlink.free_flow_travel_time_in_min = microlink.length / microlink.speed_limit * 0.06;
			microlink.free_flow_travel_time_in_simu_interval = max(round(microlink.free_flow_travel_time_in_min * 60 / simulator->simulation_step), 1);

			net->micro_link_vector.push_back(microlink);

			net->meso_link_vector[net->meso_link_id_to_seq_no_dict[microlink.macro_link_id]].micro_link_set.push_back(microlink.link_id);
			net->micro_node_vector[net->micro_node_id_to_seq_no_dict[microlink.from_node_id]].m_outgoing_link_vector.push_back(microlink.link_id);
			net->micro_node_vector[net->micro_node_id_to_seq_no_dict[microlink.to_node_id]].m_incoming_link_vector.push_back(microlink.link_id);
			net->micro_link_id_to_seq_no_dict[microlink.link_id] = microlink.link_seq_no;

			net->number_of_micro_links++;
		}
	}
	else
	{
		string msg = "Cannot open Micro Link file\n";
		throw msg;
	}

	cout << "Network Loaded: ";
	cout << net->number_of_meso_nodes << " meso nodes, " << net->number_of_meso_links << " meso links, " << net->number_of_micro_nodes 
		<< " micro nodes, " << net->number_of_micro_links << " micro links\n";
}

void DataLoader::readSignal()
{

}


void time2timedecimal(string &time_str, float &time_decimal)
{
	int hour_int, min_int, sec_int;

	hour_int = stoi(time_str.substr(0, 2));
	min_int = stoi(time_str.substr(2, 2));

	if (time_str.length() == 4)
	{
		sec_int = 0;
	}
	else if (time_str.length() == 6)
	{
		sec_int = stoi(time_str.substr(4));
	}

	time_decimal = hour_int * 60.0 + min_int + sec_int / 60.0;
}


void DataLoader::readAgent()
{
	cout << "Loading Agents\n";
	CCSVParser parser_agent;
	if (parser_agent.OpenCSVFile(agent_filepath, true))
	{
		int agent_id;
		float volume, volume_fraction;
		int volume_int;
		int o_node_id, d_node_id, o_node_no, d_node_no, o_zone_id, d_zone_id;
		string time_period, node_sequence, link_sequence;
		string demand_start_time, demand_end_time;
		float demand_start_time_decimal, demand_end_time_decimal, demand_duration;

		int node_id_int;
		string link_key;

		default_random_engine e;
		e.seed((unsigned)time(NULL));
		uniform_real_distribution<float> u(0.0, 1.0);
		float random_num;

		map<int, int>::iterator iter1;
		map<string, int>::iterator iter2;
		bool fixed_path_flag, path_valid_flag;

		MesoNode *start_node;

		while (parser_agent.ReadRecord())
		{
			parser_agent.GetValueByFieldName("agent_id", agent_id);
			parser_agent.GetValueByFieldName("o_node_id", o_node_id);
			parser_agent.GetValueByFieldName("d_node_id", d_node_id);
			parser_agent.GetValueByFieldName("o_node_id", o_zone_id);
			parser_agent.GetValueByFieldName("d_node_id", d_zone_id);
			parser_agent.GetValueByFieldName("time_period", time_period);
			parser_agent.GetValueByFieldName("volume", volume);
			fixed_path_flag = parser_agent.GetValueByFieldName("node_sequence", node_sequence);

			iter1 = net->meso_node_id_to_seq_no_dict.find(o_node_id);
			if (iter1 == net->meso_node_id_to_seq_no_dict.end()){
				cout << "warning: from_origin_node_id " << o_node_id << "does not exist, corresponding demand will be discarded. (agent id " << agent_id << ")\n";
				continue;
			}
			o_node_no = iter1->second;
			start_node = &net->meso_node_vector[o_node_no];
			if (start_node->m_outgoing_link_vector.size() == 0){
				cout << "warning: cannot find an outgoing link from origin node " << o_node_id << ", corresponding demand will be discarded. (agent id " << agent_id << ")\n";
				continue;
			}

			iter1 = net->meso_node_id_to_seq_no_dict.find(d_node_id);
			if (iter1 == net->meso_node_id_to_seq_no_dict.end()){
				cout << "warning: to_destination_node_id " << d_node_id << "does not exist, corresponding demand will be discarded. (agent id " << agent_id << ")\n";
				continue;
			}
			d_node_no = iter1->second;

			path_valid_flag = true;
			vector<int> meso_path_node_no_vector, meso_path_link_seq_no_vector;
			if (fixed_path_flag)
			{
				vector<string> node_sequence_str;
				SplitString(node_sequence, node_sequence_str, ";");
				
				for (string id_str : node_sequence_str)
				{
					node_id_int = stoi(id_str);
					iter1 = net->meso_node_id_to_seq_no_dict.find(node_id_int);
					if (iter1 == net->meso_node_id_to_seq_no_dict.end()){
						cout << "warning: node_id " << node_id_int << "in the node_sequence does not exist, corresponding demand will be discarded. (agent id " << agent_id << ")\n";
						path_valid_flag = false;
						break;
					}
					meso_path_node_no_vector.push_back(iter1->second);
				}
				for (int i = 1; i < node_sequence_str.size(); i++)
				{
					link_key = node_sequence_str[i - 1] + " " + node_sequence_str[i];
					iter2 = net->meso_link_key_to_seq_no_dict.find(link_key);
					if (iter2 == net->meso_link_key_to_seq_no_dict.end()) {
						cout << "warning: node pair " << node_sequence_str[i - 1] << "-" << node_sequence_str[i] << " in the node_sequence does not exist, corresponding demand will be discarded. (agent id " << agent_id << ")\n";
						path_valid_flag = false;
						break;
					}
					meso_path_link_seq_no_vector.push_back(iter2->second);
				}
				
				if (!path_valid_flag)
					continue;
			}


			random_num = u(e);
			volume_fraction = volume - (int)volume;
			if (volume_fraction >= random_num)
				volume_int = ceil(volume);
			else
				volume_int = floor(volume);
			
			vector<string> time_period_str;
			SplitString(time_period, time_period_str, "_");
			demand_start_time = time_period_str[0];
			demand_end_time = time_period_str[1];
			time2timedecimal(demand_start_time, demand_start_time_decimal);
			time2timedecimal(demand_end_time, demand_end_time_decimal);
			demand_duration = demand_end_time_decimal - demand_start_time_decimal;


			for (int i = 0; i < volume_int; i++)
			{
				Agent agent;
				agent.agent_id = simulator->number_of_agents;
				agent.meso_origin_node_id = o_node_id;
				agent.meso_destination_node_id = d_node_id;
				agent.meso_origin_node_seq_no = o_node_no;
				agent.meso_destination_node_seq_no = d_node_no;
				agent.origin_zone_id = o_zone_id;
				agent.destination_zone_id = d_zone_id;

				random_num = u(e);
				agent.departure_time_in_min = demand_start_time_decimal + demand_duration * random_num;
				agent.departure_time_in_simu_interval = round(agent.departure_time_in_min * 60 / simulator->simulation_step);

				//start_link = &net->meso_link_vector[net->meso_link_id_to_seq_no_dict[start_node->m_outgoing_link_vector[0]]];		// update: collector
				//seq_no = (rand() % start_link->micro_incoming_node_id_vector.size());
				//agent.micro_origin_node_id = start_link->micro_incoming_node_id_vector[seq_no];
				//agent.micro_origin_node_seq_no = net->micro_node_id_to_seq_no_dict[agent.micro_origin_node_id];

				if (fixed_path_flag) {
					agent.fixed_path_flag = true;
					agent.meso_path_node_seq_no_vector = meso_path_node_no_vector;
					agent.meso_path_link_seq_no_vector = meso_path_link_seq_no_vector;
				}

				simulator->agent_vector.push_back(agent);
				simulator->number_of_agents++;
			}

		}
	}
	else
	{
		string msg = "Cannot open Agent file\n";
		throw msg;
	}

	cout << "Agents Loaded: ";
	cout << simulator->number_of_agents << " agents\n\n";
}


void DataLoader::readFlow()
{
	string msg = "flow demand is not supported now.\n";
	throw msg;
}
