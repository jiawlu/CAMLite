#pragma once
#include "Network.h"
#include "Agent.h"

class VehControllerCA
{
public:
	static float default_time_headway;
	static int default_safe_headway_in_simu_interval;
	static std::map<int, float> time_headway_dict;
	static std::map<int, int> safe_headway_in_simu_interval_dict;
	static float simulation_step;
	static Network *net;


	static void init(Network *net_, float simulation_step_);
	static void moveVeh(Agent *p_agent, int t);
};