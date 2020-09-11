#pragma once

#include <string>
#include "Simulation.h"
#include "Network.h"

class DataLoader
{
public:
	std::string micro_node_filepath;
	std::string micro_link_filepath;
	std::string meso_node_filepath;
	std::string meso_link_filepath;

	std::string agent_filepath;
	std::string flow_filepath;

	Simulation *simulator;
	Network *net;

	void loadData(Simulation *simulator, Network *net);

	void readSettings();
	void readNetwork();
	void readSignal();
	void readAgent();
	void readFlow();
};