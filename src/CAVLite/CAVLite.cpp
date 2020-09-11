// CAVLite.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "DataLoader.h"


int main()
{
	DataLoader loader;
	Simulation simulator;
	Network net;
	loader.loadData(&simulator, &net);
	net.init();
	simulator.net = &net;
	simulator.SimulationInitialization();
	simulator.TrafficAssignment();
	simulator.TrafficSimulation();
	simulator.exportSimulationResults();
}
