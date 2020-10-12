# -*- coding:utf-8 -*-
# @author       Jiawei Lu (jiaweil9@asu.edu)
#               Xuesong Zhou (xzhou74@asu.edu)
# @time         2020/10/11 19:29
# @desc         [script description]


import csv
import time
import numpy as np
from collections import deque
import pandas as pd
import os
import networkx as nx
import sys
import argparse


g_working_directory = ''
g_number_of_seconds_per_interval = 0.0
g_Simulation_StartTimeInMin = 0.0
g_Simulation_EndTimeInMin = 0.0
g_total_assignment_iteration = 0
g_time_headway = 0.0
output_filename = ''

g_micro_node_list = []
g_micro_link_list = []
g_meso_node_list = []
g_meso_link_list = []
g_agent_list = []
g_meso_node_id_to_seq_no_dict = {}
g_micro_node_id_to_seq_no_dict = {}
g_meso_link_id_to_seq_no_dict = {}
g_micro_link_id_to_seq_no_dict = {}

g_meso_link_key_to_seq_no_dict = {}

g_number_of_micro_nodes = 0
g_number_of_micro_links = 0
g_number_of_meso_nodes = 0
g_number_of_meso_links = 0
g_number_of_agents = 0
g_start_simu_interval_no = 0
g_end_simu_interval_no = 0

_MAX_COST_TREE = 99999999
_MAX_TIME_INTERVAL = 9999999


def load_args():
    global g_working_directory
    global g_number_of_seconds_per_interval
    global g_Simulation_StartTimeInMin
    global g_Simulation_EndTimeInMin
    global g_total_assignment_iteration
    global g_time_headway
    global output_filename

    print('Loading Simulation Settings...')

    parser = argparse.ArgumentParser(description='ParameterOptimizer')
    # --- Working directory ---
    parser.add_argument('--cwd', default=r'datasets\small_net', type=str,
                        help='Working directory of simulation')

    # --- Simulation settings ---
    parser.add_argument('--sim_step', default=0.1, type=float,
                        help='Simulation step.')
    parser.add_argument('--sim_start_time', default=0, type=float,
                        help='Simulation start time in minutes.')
    parser.add_argument('--sim_end_time', default=60, type=float,
                        help='Simulation end time in minutes.')
    parser.add_argument('--assignment_iters', default=10, type=int,
                        help='Number of MSA assignment iterations.')
    parser.add_argument('--headway', default=1.4, type=float,
                        help='Minimum time headway between vehicles.')

    # --- Outputs ---
    parser.add_argument('--output', default='output_agent.csv', type=str,
                        help='Filename of outputs.')
    parsed_arguments = vars(parser.parse_args())

    g_working_directory = parsed_arguments['cwd']
    g_number_of_seconds_per_interval = parsed_arguments['sim_step']
    g_Simulation_StartTimeInMin = parsed_arguments['sim_start_time']
    g_Simulation_EndTimeInMin = parsed_arguments['sim_end_time']
    g_total_assignment_iteration = parsed_arguments['assignment_iters']
    g_time_headway = parsed_arguments['headway']
    output_filename = parsed_arguments['output']

    print(f'  cwd: {g_working_directory}')
    print(f'  sim_step: {g_number_of_seconds_per_interval}')
    print(f'  sim_start_time: {g_Simulation_StartTimeInMin}')
    print(f'  sim_end_time: {g_Simulation_EndTimeInMin}')
    print(f'  assignment_iters: {g_total_assignment_iteration}')
    print(f'  headway: {g_time_headway}')
    print(f'  output: {output_filename}')


def printProgress(current_iter, total_iters, width=30):
    percent = current_iter / total_iters
    show_str = ('[%%-%ds]' % width) % (int(width * percent) * "#")
    print('\r%s %d%%' % (show_str, percent*100), end='')


class MesoNode:
    def __init__(self):
        self.name = ''
        self.node_id = 0
        self.node_seq_no = 0
        self.iszone = False
        self.control_type = 0
        self.control_type_name = ''
        self.cycle_length_in_second = 0
        self.x = 0.0
        self.y = 0.0
        self.m_outgoing_link_list = []
        self.m_incoming_link_list = []


class MesoLink:
    def __init__(self):
        self.name = ''
        self.link_id = 0
        self.link_seq_no = 0
        self.from_node_id = 0
        self.to_node_id = 0
        self.length = 0.0
        self.number_of_lanes = 0
        self.speed_limit = 0.0
        self.lane_cap = 0.0
        self.flow_volume = 0.0
        self.BPR_alpha = 0.15
        self.BPR_beta = 4.0
        self.link_capacity = 0.0
        self.travel_time = 0.0
        self.cost = 0.0

        self.link_key = 0
        self.isconnector = False
        self.geometry_list = []  # vector<float*> geometry_vector;

        self.micro_node_list = []  # vector<vector<int>> micro_node_set;			//micro node id, lane by lane;
        self.micro_link_list = []  # vector<int> micro_link_set;			//micro link id;

        self.free_flow_travel_time_in_min = 0.0
        self.free_flow_travel_time_in_sim_interval = 0
        self.m_active_agent_queue = deque()

        self.graph = nx.DiGraph()
        self.micro_incoming_node_id_list = None
        self.micro_outgoing_node_id_list = None
        self.turning_node_seq_no_dict = {}  # std::map<int, vector<int>> turning_node_seq_no_dict;             //meso_link_seq_no:micro_node_seq_no
        self.estimated_cost_tree_for_each_movement = {}  # std::map<int, map<int,float>> estimated_cost_tree_for_each_movement;     //meso_link_seq_no : node_seq_no: cost

    def CalculateBPRFunctionAndCost(self):
        self.travel_time = self.free_flow_travel_time_in_min * (
                    1 + self.BPR_alpha * pow(self.flow_volume / self.link_capacity, self.BPR_beta))
        self.cost = self.travel_time

    def Initialization(self):
        self.link_capacity = self.number_of_lanes * self.lane_cap
        self.free_flow_travel_time_in_min = self.length / self.speed_limit * 0.06
        self.link_key = self.from_node_id * 100000 + self.to_node_id
        for i in range(self.number_of_lanes): self.micro_node_list.append([])


class MicroNode:
    def __init__(self):
        self.node_id = 0
        self.node_seq_no = 0
        self.x = 0.0
        self.y = 0.0
        self.meso_link_id = 0
        self.lane_no = 0
        self.m_outgoing_link_list = []
        self.m_incoming_link_list = []
        self.available_sim_interval = 0


class MicroLink:
    def __init__(self):
        self.link_id = 0
        self.link_seq_no = 0
        self.from_node_id = 0
        self.to_node_id = 0
        self.meso_link_id = 0
        self.lane_no = 0
        self.length = 0.0
        self.speed_limit = 0.0
        self.free_flow_travel_time_in_min = 0.0
        self.free_flow_travel_time_in_simu_interval = 0
        self.additional_cost = 0.0
        self.cost = 0.0
        self.link_type = 1  # //1:traveling; 2:changing

    def Initialization(self):
        self.additional_cost = 0 if self.link_type == 1 else 2
        self.free_flow_travel_time_in_min = self.length / self.speed_limit * 0.06
        self.free_flow_travel_time_in_simu_interval = max(
            round(self.free_flow_travel_time_in_min * 60 / g_number_of_seconds_per_interval), 1)
        self.cost = self.free_flow_travel_time_in_simu_interval + self.additional_cost


class Agent:
    def __init__(self):
        self.agent_id = 0
        self.agent_seq_no = 0
        self.origin_zone_id = 0
        self.destination_zone_id = 0

        self.meso_origin_node_id = 0
        self.meso_destination_node_id = 0
        self.meso_origin_node_seq_no = 0
        self.meso_destination_node_seq_no = 0
        self.micro_origin_node_id = 0
        self.micro_destination_node_id = 0
        self.micro_origin_node_seq_no = 0
        self.micro_destination_node_seq_no = 0
        self.departure_time_in_min = 0.0
        self.departure_time_in_simu_interval = 0
        self.arrival_time_in_min = 0.0
        self.arrival_time_in_simu_interval = 0
        self.travel_time_in_min = 0.0

        self.meso_path_node_seq_no_list = []
        self.meso_path_link_seq_no_list = []
        self.micro_path_node_str = ''
        self.micro_path_time_str = ''
        self.path_node_id_list = []
        self.number_of_nodes = 0
        self.m_bMoveable = 1
        self.m_bGenereated = False
        self.origin_zone_seq_no = 0
        self.destination_zone_seq_no = 0

        self.PCE_factor = 1.0
        self.path_cost = 0.0
        self.m_path_link_seq_no_list_size = 0

        self.m_current_link_seq_no = 0
        self.latest_arrival_time = 0.0

        self.micro_path_node_seq_no_list = []
        self.micro_path_link_seq_no_list = []
        self.micro_path_node_id_list = []
        self.micro_path_link_id_list = []

        self.m_Veh_LinkArrivalTime_in_simu_interval = []  # vector<int> m_Veh_LinkArrivalTime_in_simu_interval;
        self.m_Veh_LinkDepartureTime_in_simu_interval = []  # vector<int> m_Veh_LinkDepartureTime_in_simu_interval;
        self.path_timestamp_list = []

        self.current_meso_path_seq_no = 0
        self.current_meso_link_seq_no = 0
        self.next_meso_link_seq_no = 0
        self.m_bCompleteTrip = False

        self.m_no_physical_path_flag = False

    def outputDataGeneration(self):
        self.micro_path_node_str = str(self.micro_path_node_id_list[0])
        for node_id in self.micro_path_node_id_list[1:]:
            self.micro_path_node_str += (';' + str(node_id))

        self.micro_path_time_str = str(
            round(self.m_Veh_LinkArrivalTime_in_simu_interval[0] * 0.2 / 60 + g_Simulation_StartTimeInMin, 3))
        for arrival_time_interval in self.m_Veh_LinkArrivalTime_in_simu_interval[1:]:
            self.micro_path_time_str += (';' + str(round(arrival_time_interval * 0.2 / 60 + g_Simulation_StartTimeInMin, 3)))

        self.number_of_nodes = len(self.micro_path_node_id_list)


def g_ReadInputData():
    global g_number_of_micro_nodes
    global g_number_of_micro_links
    global g_number_of_meso_nodes
    global g_number_of_meso_links
    global g_number_of_agents
    global g_start_simu_interval_no
    global g_end_simu_interval_no

    print('Reading input data...')
    # ------------------Meso Node--------------------#
    print('  loading mesoscopic nodes')
    meso_node_data = pd.read_csv(os.path.join(g_working_directory, r'mesonet\node.csv'))
    g_number_of_meso_nodes = len(meso_node_data)

    for i in range(g_number_of_meso_nodes):
        mesonode = MesoNode()

        try:
            mesonode.node_id = meso_node_data.loc[i, 'node_id']
            mesonode.x = meso_node_data.loc[i, 'x_coord']
            mesonode.y = meso_node_data.loc[i, 'y_coord']
        except KeyError as e:
            print(f'Cannot find {e.args[0]} in the meso node.csv. Press Enter key to exit')
            g_program_stop()

        mesonode.node_seq_no = i

        g_meso_node_list.append(mesonode)
        g_meso_node_id_to_seq_no_dict[mesonode.node_id] = mesonode.node_seq_no

    # ------------------Meso Link--------------------#
    print('  loading mesoscopic links')
    meso_link_data = pd.read_csv(os.path.join(g_working_directory, r'mesonet\link.csv'))
    g_number_of_meso_links = len(meso_link_data)

    for i in range(g_number_of_meso_links):
        mesolink = MesoLink()

        try:
            mesolink.link_id = meso_link_data.loc[i, 'link_id']
            mesolink.from_node_id = meso_link_data.loc[i, 'from_node_id']
            mesolink.to_node_id = meso_link_data.loc[i, 'to_node_id']
            mesolink.length = meso_link_data.loc[i, 'length']
            mesolink.number_of_lanes = meso_link_data.loc[i, 'lanes']
            mesolink.speed_limit = meso_link_data.loc[i, 'free_speed']
            lane_cap = meso_link_data.loc[i, 'capacity']
            mesolink.lane_cap = lane_cap if lane_cap > 0 else 1200
            mesolink.geometry = meso_link_data.loc[i, 'geometry']

        except KeyError as e:
            print(f'Cannot find {e.args[0]} in the meso link.csv. Press Enter key to exit')
            g_program_stop()

        mesolink.link_seq_no = i
        from_node = g_meso_node_list[g_meso_node_id_to_seq_no_dict[mesolink.from_node_id]]
        from_node.m_outgoing_link_list.append(mesolink.link_id)
        to_node = g_meso_node_list[g_meso_node_id_to_seq_no_dict[mesolink.to_node_id]]
        to_node.m_incoming_link_list.append(mesolink.link_id)

        mesolink.Initialization()

        g_meso_link_list.append(mesolink)
        g_meso_link_id_to_seq_no_dict[mesolink.link_id] = mesolink.link_seq_no
        g_meso_link_key_to_seq_no_dict[mesolink.link_key] = mesolink.link_seq_no

    # ------------------Micro Node--------------------#
    print('  loading microscopic nodes')
    micro_node_data = pd.read_csv(os.path.join(g_working_directory, r'micronet\node.csv'))
    g_number_of_micro_nodes = len(micro_node_data)

    for i in range(g_number_of_micro_nodes):
        micronode = MicroNode()

        try:
            micronode.node_id = micro_node_data.loc[i, 'node_id']
            micronode.x = micro_node_data.loc[i, 'x_coord']
            micronode.y = micro_node_data.loc[i, 'y_coord']
            micronode.meso_link_id = micro_node_data.loc[i, 'meso_link_id']
            micronode.lane_no = micro_node_data.loc[i, 'lane_no']
        except KeyError as e:
            print(f'Cannot find {e.args[0]} in the micro node.csv. Press Enter key to exit')
            g_program_stop()

        micronode.node_seq_no = i
        g_micro_node_list.append(micronode)

        g_micro_node_id_to_seq_no_dict[micronode.node_id] = micronode.node_seq_no
        meso_link = g_meso_link_list[g_meso_link_id_to_seq_no_dict[micronode.meso_link_id]]
        meso_link.micro_node_list[micronode.lane_no - 1].append(micronode.node_id)

    # ------------------Micro Link--------------------#
    print('  loading microscopic links')
    micro_link_data = pd.read_csv(os.path.join(g_working_directory, r'micronet\link.csv'))
    g_number_of_micro_links = len(micro_link_data)

    for i in range(g_number_of_micro_links):
        microlink = MicroLink()

        try:
            microlink.link_id = micro_link_data.loc[i, 'link_id']
            microlink.from_node_id = micro_link_data.loc[i, 'from_node_id']
            microlink.to_node_id = micro_link_data.loc[i, 'to_node_id']
            microlink.meso_link_id = micro_link_data.loc[i, 'meso_link_id']
            microlink.link_type = micro_link_data.loc[i, 'link_type']
            microlink.length = micro_link_data.loc[i, 'length']
            microlink.speed_limit = micro_link_data.loc[i, 'free_speed']
            microlink.additional_cost = micro_link_data.loc[i, 'additional_cost']
        except KeyError as e:
            print(f'Cannot find {e.args[0]} in the micro link.csv. Press Enter key to exit')
            g_program_stop()

        microlink.link_seq_no = i
        microlink.Initialization()

        g_micro_link_list.append(microlink)

        from_node = g_micro_node_list[g_micro_node_id_to_seq_no_dict[microlink.from_node_id]]
        from_node.m_outgoing_link_list.append(microlink.link_id)
        to_node = g_micro_node_list[g_micro_node_id_to_seq_no_dict[microlink.to_node_id]]
        to_node.m_incoming_link_list.append(microlink.link_id)

        mesolink = g_meso_link_list[g_meso_link_id_to_seq_no_dict[microlink.meso_link_id]]
        mesolink.micro_link_list.append(microlink.link_id)
        mesolink.graph.add_edge(microlink.to_node_id, microlink.from_node_id,
                                weight=microlink.cost)  # reverse link direction to reduce shortest path searching time
        g_micro_link_id_to_seq_no_dict[microlink.link_id] = microlink.link_seq_no

    print(f'  net: {g_number_of_meso_nodes} meso nodes, {g_number_of_meso_links} meso links, '
          f'{g_number_of_micro_nodes} micro nodes, {g_number_of_micro_links} micro links')

    # ----------------outgoing node identification----------------------#
    for i in range(g_number_of_meso_links):
        mesolink = g_meso_link_list[i]

        potential_micro_incoming_node_id_set = set()
        potential_micro_outgoing_node_id_set = set()

        for micro_link_id in mesolink.micro_link_list:
            micro_link = g_micro_link_list[g_micro_link_id_to_seq_no_dict[micro_link_id]]
            potential_micro_incoming_node_id_set.add(micro_link.from_node_id)
            potential_micro_outgoing_node_id_set.add(micro_link.to_node_id)

        mesolink.micro_incoming_node_id_list = list(
            potential_micro_incoming_node_id_set - potential_micro_outgoing_node_id_set)
        mesolink.micro_outgoing_node_id_list = list(
            potential_micro_outgoing_node_id_set - potential_micro_incoming_node_id_set)

    # ------------------Demand--------------------#
    print('  loading demands')
    demand_data = pd.read_csv(os.path.join(g_working_directory, 'input_demand.csv'))
    number_of_demand_records = len(demand_data)
    from_origin_node_id = -1
    to_destination_node_id = -1
    number_of_trips = -1
    time_start = -1
    time_end = -1

    for i in range(number_of_demand_records):
        try:
            from_origin_node_id = demand_data.loc[i, 'o_node_id']
            to_destination_node_id = demand_data.loc[i, 'd_node_id']
            number_of_trips = demand_data.loc[i, 'volume']
            time_start = demand_data.loc[i, 'time_start']
            time_end = demand_data.loc[i, 'time_end']
        except KeyError as e:
            print(f'Cannot find {e.args[0]} in the input_demand.csv. Press Enter key to exit')
            g_program_stop()

        start_node = g_meso_node_list[g_meso_node_id_to_seq_no_dict[from_origin_node_id]]
        start_link = g_meso_link_list[g_meso_link_id_to_seq_no_dict[start_node.m_outgoing_link_list[0]]]

        for j in range(number_of_trips):
            agent = Agent()
            agent.agent_id = g_number_of_agents
            agent.agent_seq_no = g_number_of_agents
            agent.meso_origin_node_id = from_origin_node_id
            agent.meso_destination_node_id = to_destination_node_id
            agent.departure_time_in_min = np.random.random() * (time_end - time_start) + time_start
            agent.meso_origin_node_seq_no = g_meso_node_id_to_seq_no_dict[from_origin_node_id]
            agent.meso_destination_node_seq_no = g_meso_node_id_to_seq_no_dict[to_destination_node_id]

            seq_no = np.random.randint(len(start_link.micro_incoming_node_id_list))
            agent.micro_origin_node_id = start_link.micro_incoming_node_id_list[seq_no]
            agent.micro_origin_node_seq_no = g_micro_node_id_to_seq_no_dict[agent.micro_origin_node_id]

            agent.departure_time_in_simu_interval = round(
                (agent.departure_time_in_min - g_Simulation_StartTimeInMin) * 60 / g_number_of_seconds_per_interval)

            g_agent_list.append(agent)
            g_number_of_agents += 1

    print(f'  demand: {g_number_of_agents} agents')


class Network:
    def __init__(self):
        self.graph = nx.DiGraph()

    def findPathForAgents(self, iteration_no):

        for agent in g_agent_list:
            residual = agent.agent_seq_no % (iteration_no + 1)

            if residual != 0:
                for meso_link_seq_no in agent.meso_path_link_seq_no_list:
                    g_meso_link_list[meso_link_seq_no].flow_volume += agent.PCE_factor
                continue

            try:
                path = nx.dijkstra_path(self.graph, source=agent.meso_origin_node_id, target=agent.meso_destination_node_id)
            except:
                print('cannot find a feasible meso path for agent {}, meso_origin_node_id {}, meso_destination_node_id {}'.format(
                        agent.agent_id, agent.meso_origin_node_id, agent.meso_destination_node_id))
                continue

            agent.meso_path_node_seq_no_list = []
            agent.meso_path_link_seq_no_list = []

            for node_id in path: agent.meso_path_node_seq_no_list.append(g_meso_node_id_to_seq_no_dict[node_id])
            number_of_nodes_in_path = len(path)
            for i in range(1, number_of_nodes_in_path): agent.meso_path_link_seq_no_list.append(
                g_meso_link_key_to_seq_no_dict[path[i - 1] * 100000 + path[i]])

            number_of_links_in_path = number_of_nodes_in_path - 1
            for i in range(number_of_links_in_path):
                g_meso_link_list[agent.meso_path_link_seq_no_list[i]].flow_volume += agent.PCE_factor


def g_program_stop():
    input()
    sys.exit()


def g_TrafficAssignment():
    print('Conducting Traffic Assignment...')
    time_start = time.time()
    network = Network()

    for mesolink in g_meso_link_list:
        mesolink.flow_volume = 0
        mesolink.CalculateBPRFunctionAndCost()
        network.graph.add_edge(mesolink.from_node_id, mesolink.to_node_id, weight=mesolink.cost)

    printProgress(1, g_total_assignment_iteration, width=30)
    network.findPathForAgents(0)

    for i in range(1, g_total_assignment_iteration):
        for mesolink in g_meso_link_list:
            mesolink.CalculateBPRFunctionAndCost()
            network.graph[mesolink.from_node_id][mesolink.to_node_id]['weight'] = mesolink.cost
            mesolink.flow_volume = 0

        printProgress(i+1, g_total_assignment_iteration, width=30)
        network.findPathForAgents(i)
    print()
    time_end = time.time()
    print(f'time used for traffic assignment: {round(time_end-time_start,3)} seconds')


def g_CostTreeCalculation():
    for mesolink in g_meso_link_list:

        current_node = g_meso_node_list[g_meso_node_id_to_seq_no_dict[mesolink.to_node_id]]

        for micro_node_id in mesolink.micro_outgoing_node_id_list:

            for downstream_mesolink_id in current_node.m_outgoing_link_list:

                downstream_mesolink = g_meso_link_list[g_meso_link_id_to_seq_no_dict[downstream_mesolink_id]]

                if micro_node_id in downstream_mesolink.micro_incoming_node_id_list:

                    micro_node_seq_no = g_micro_node_id_to_seq_no_dict[micro_node_id]

                    if downstream_mesolink.link_seq_no in mesolink.turning_node_seq_no_dict.keys():
                        mesolink.turning_node_seq_no_dict[downstream_mesolink.link_seq_no].append(micro_node_seq_no)
                    else:
                        mesolink.turning_node_seq_no_dict[downstream_mesolink.link_seq_no] = [micro_node_seq_no]

    for mesolink in g_meso_link_list:

        if mesolink.turning_node_seq_no_dict:

            for downstream_mesolink_seq_no in mesolink.turning_node_seq_no_dict.keys():

                mesolink.estimated_cost_tree_for_each_movement[downstream_mesolink_seq_no] = {}

                for lane_node_set in mesolink.micro_node_list:
                    for node_id in lane_node_set:
                        mesolink.estimated_cost_tree_for_each_movement[downstream_mesolink_seq_no][
                            g_micro_node_id_to_seq_no_dict[node_id]] = _MAX_COST_TREE

                if True:  # mesolink.isconnector
                    for node_id in mesolink.micro_incoming_node_id_list:
                        mesolink.estimated_cost_tree_for_each_movement[downstream_mesolink_seq_no][
                            g_micro_node_id_to_seq_no_dict[node_id]] = _MAX_COST_TREE

                    for node_id in mesolink.micro_outgoing_node_id_list:
                        mesolink.estimated_cost_tree_for_each_movement[downstream_mesolink_seq_no][
                            g_micro_node_id_to_seq_no_dict[node_id]] = _MAX_COST_TREE

                for outgoing_node_seq_no in mesolink.turning_node_seq_no_dict[downstream_mesolink_seq_no]:

                    outgoing_node_id = g_micro_node_list[outgoing_node_seq_no].node_id
                    path_length = nx.single_source_dijkstra_path_length(mesolink.graph, outgoing_node_id)

                    for node_seq_no in mesolink.estimated_cost_tree_for_each_movement[downstream_mesolink_seq_no].keys():
                        node_id = g_micro_node_list[node_seq_no].node_id
                        if node_id in path_length.keys():
                            if path_length[node_id] < \
                                    mesolink.estimated_cost_tree_for_each_movement[downstream_mesolink_seq_no][
                                        node_seq_no]:
                                mesolink.estimated_cost_tree_for_each_movement[downstream_mesolink_seq_no][
                                    node_seq_no] = path_length[node_id]

        else:

            mesolink.estimated_cost_tree_for_each_movement[-1] = {}

            for lane_node_set in mesolink.micro_node_list:
                for node_id in lane_node_set:
                    mesolink.estimated_cost_tree_for_each_movement[-1][
                        g_micro_node_id_to_seq_no_dict[node_id]] = _MAX_COST_TREE

            if True:  # mesolink.isconnector
                for node_id in mesolink.micro_incoming_node_id_list:
                    mesolink.estimated_cost_tree_for_each_movement[-1][
                        g_micro_node_id_to_seq_no_dict[node_id]] = _MAX_COST_TREE

                for node_id in mesolink.micro_outgoing_node_id_list:
                    mesolink.estimated_cost_tree_for_each_movement[-1][
                        g_micro_node_id_to_seq_no_dict[node_id]] = _MAX_COST_TREE

            for outgoing_node_id in mesolink.micro_outgoing_node_id_list:

                path_length = nx.single_source_dijkstra_path_length(mesolink.graph, outgoing_node_id)

                for node_seq_no in mesolink.estimated_cost_tree_for_each_movement[-1].keys():
                    node_id = g_micro_node_list[node_seq_no].node_id
                    if node_id in path_length.keys():
                        if path_length[node_id] < mesolink.estimated_cost_tree_for_each_movement[-1][node_seq_no]:
                            mesolink.estimated_cost_tree_for_each_movement[-1][node_seq_no] = path_length[node_id]


def g_TrafficSimulation():
    number_of_simu_intervals = int(np.ceil((g_Simulation_EndTimeInMin - g_Simulation_StartTimeInMin) * 60 / g_number_of_seconds_per_interval))
    active_agent_list = []
    number_of_simu_interval_per_min = 60 / g_number_of_seconds_per_interval
    current_active_agent_index = 0
    safe_headway_in_simu_interval = np.round(g_time_headway / g_number_of_seconds_per_interval)

    departure_time_list = [agent.departure_time_in_simu_interval for agent in g_agent_list]
    agent_index = np.argsort(departure_time_list)

    print('Doing Simulation...')
    time_start = time.time()

    for t in range(number_of_simu_intervals):

        if t % number_of_simu_interval_per_min == 0:

            printProgress(t * g_number_of_seconds_per_interval / 60 + g_Simulation_StartTimeInMin, g_Simulation_EndTimeInMin, width=30)

            while current_active_agent_index < g_number_of_agents:

                if g_agent_list[agent_index[current_active_agent_index]].departure_time_in_simu_interval < t:
                    current_active_agent_index += 1
                    continue
                if g_agent_list[agent_index[
                    current_active_agent_index]].departure_time_in_simu_interval >= t + number_of_simu_interval_per_min: break

                agent = g_agent_list[agent_index[current_active_agent_index]]

                if agent.m_no_physical_path_flag:
                    current_active_agent_index += 1
                    continue

                agent.m_bGenereated = True
                agent.current_meso_path_seq_no = 0
                agent.micro_path_node_seq_no_list.append(agent.micro_origin_node_seq_no)
                agent.micro_path_node_id_list.append(agent.micro_origin_node_id)
                agent.m_Veh_LinkArrivalTime_in_simu_interval.append(t)
                agent.m_Veh_LinkDepartureTime_in_simu_interval.append(agent.departure_time_in_simu_interval)

                agent.current_meso_link_seq_no = agent.meso_path_link_seq_no_list[agent.current_meso_path_seq_no]
                if agent.current_meso_path_seq_no < len(agent.meso_path_link_seq_no_list) - 1:
                    agent.next_meso_link_seq_no = agent.meso_path_link_seq_no_list[agent.current_meso_path_seq_no + 1]
                else:
                    agent.next_meso_link_seq_no = -1

                active_agent_list.append(agent.agent_seq_no)
                current_active_agent_index += 1

        agent_remove_list = []

        for agent_no in active_agent_list:

            agent = g_agent_list[agent_no]

            if agent.m_Veh_LinkDepartureTime_in_simu_interval[-1] == t:

                current_node = g_micro_node_list[agent.micro_path_node_seq_no_list[-1]]

                if current_node.node_id in g_meso_link_list[agent.current_meso_link_seq_no].micro_outgoing_node_id_list:
                    if agent.current_meso_path_seq_no == len(agent.meso_path_link_seq_no_list) - 1:
                        agent_remove_list.append(agent.agent_id)
                        agent.arrival_time_in_simu_interval = t
                        agent.arrival_time_in_min = t * g_number_of_seconds_per_interval / 60 + g_Simulation_StartTimeInMin
                        agent.micro_destination_node_id = current_node.node_id
                        agent.m_bCompleteTrip = True
                        agent.travel_time_in_min = agent.arrival_time_in_min - agent.departure_time_in_min
                        current_node.available_sim_interval = t
                        continue
                    else:
                        agent.current_meso_path_seq_no += 1
                        agent.current_meso_link_seq_no = agent.meso_path_link_seq_no_list[
                            agent.current_meso_path_seq_no]
                        if agent.current_meso_path_seq_no < len(agent.meso_path_link_seq_no_list) - 1:
                            agent.next_meso_link_seq_no = agent.meso_path_link_seq_no_list[
                                agent.current_meso_path_seq_no + 1]
                        else:
                            agent.next_meso_link_seq_no = -1

                outgoing_link_keeping = None
                to_node_keeping = None
                cost_keeping = 0.0

                # outgoing_link_changing = None
                # to_node_changing = None
                # cost_changing = 0.0

                outgoing_link_changing_list = []
                to_node_changing_list = []
                cost_changing_list = []

                # tree_cost = 0.0

                for outgoing_link_id in current_node.m_outgoing_link_list:

                    outgoing_link = g_micro_link_list[g_micro_link_id_to_seq_no_dict[outgoing_link_id]]

                    if g_meso_link_id_to_seq_no_dict[outgoing_link.meso_link_id] != agent.current_meso_link_seq_no:
                        continue

                    if outgoing_link.link_type == 1:
                        outgoing_link_keeping = outgoing_link
                        to_node_keeping = g_micro_node_list[g_micro_node_id_to_seq_no_dict[outgoing_link.to_node_id]]

                        tree_cost = g_meso_link_list[agent.current_meso_link_seq_no].estimated_cost_tree_for_each_movement[agent.next_meso_link_seq_no][to_node_keeping.node_seq_no]

                        if tree_cost == _MAX_COST_TREE:
                            cost_keeping = _MAX_COST_TREE
                        elif to_node_keeping.available_sim_interval > t + outgoing_link.free_flow_travel_time_in_simu_interval:
                            cost_keeping = _MAX_TIME_INTERVAL
                        else:
                            cost_keeping = outgoing_link.free_flow_travel_time_in_simu_interval + outgoing_link.additional_cost + tree_cost

                    else:
                        outgoing_link_changing_list.append(outgoing_link)
                        to_node_changing = g_micro_node_list[g_micro_node_id_to_seq_no_dict[outgoing_link.to_node_id]]

                        tree_cost = g_meso_link_list[agent.current_meso_link_seq_no].estimated_cost_tree_for_each_movement[agent.next_meso_link_seq_no][to_node_changing.node_seq_no]

                        if tree_cost == _MAX_COST_TREE:
                            cost_changing = _MAX_COST_TREE
                        elif to_node_changing.available_sim_interval > t + outgoing_link.free_flow_travel_time_in_simu_interval:
                            cost_changing = _MAX_TIME_INTERVAL
                        else:
                            cost_changing = outgoing_link.free_flow_travel_time_in_simu_interval + outgoing_link.additional_cost + tree_cost

                        to_node_changing_list.append(to_node_changing)
                        cost_changing_list.append(cost_changing)

                if to_node_keeping is None:
                    agent.m_Veh_LinkDepartureTime_in_simu_interval[-1] += 1
                    continue

                if len(cost_changing_list) == 0:

                    if cost_keeping == _MAX_COST_TREE:
                        agent_remove_list.append(agent.agent_id)
                        agent.m_bCompleteTrip = False
                        current_node.available_sim_interval = t

                    elif cost_keeping < _MAX_TIME_INTERVAL:
                        to_node_keeping.available_sim_interval = _MAX_TIME_INTERVAL
                        current_node.available_sim_interval = t + safe_headway_in_simu_interval
                        agent.micro_path_node_seq_no_list.append(to_node_keeping.node_seq_no)
                        agent.micro_path_link_seq_no_list.append(outgoing_link_keeping.link_seq_no)
                        agent.micro_path_node_id_list.append(to_node_keeping.node_id)
                        agent.micro_path_link_id_list.append(outgoing_link_keeping.link_id)
                        agent.m_Veh_LinkArrivalTime_in_simu_interval.append(t)
                        agent.m_Veh_LinkDepartureTime_in_simu_interval.append(
                            t + outgoing_link_keeping.free_flow_travel_time_in_simu_interval)
                    else:
                        agent.m_Veh_LinkDepartureTime_in_simu_interval[-1] += 1
                        continue

                else:
                    cost_changing = min(cost_changing_list)
                    smallest_position = cost_changing_list.index(cost_changing)
                    to_node_changing = to_node_changing_list[smallest_position]
                    outgoing_link_changing = outgoing_link_changing_list[smallest_position]

                    if (cost_keeping == _MAX_COST_TREE) & (cost_changing == _MAX_COST_TREE):
                        agent_remove_list.append(agent.agent_id)
                        agent.m_bCompleteTrip = False
                        current_node.available_sim_interval = t
                        continue

                    if (cost_keeping < _MAX_TIME_INTERVAL) & (cost_keeping < cost_changing):
                        to_node_keeping.available_sim_interval = _MAX_TIME_INTERVAL
                        current_node.available_sim_interval = t + safe_headway_in_simu_interval
                        agent.micro_path_node_seq_no_list.append(to_node_keeping.node_seq_no)
                        agent.micro_path_link_seq_no_list.append(outgoing_link_keeping.link_seq_no)
                        agent.micro_path_node_id_list.append(to_node_keeping.node_id)
                        agent.micro_path_link_id_list.append(outgoing_link_keeping.link_id)
                        agent.m_Veh_LinkArrivalTime_in_simu_interval.append(t)
                        agent.m_Veh_LinkDepartureTime_in_simu_interval.append(
                            t + outgoing_link_keeping.free_flow_travel_time_in_simu_interval)

                    elif (cost_changing < _MAX_TIME_INTERVAL) & (cost_changing < cost_keeping):
                        to_node_changing.available_sim_interval = _MAX_TIME_INTERVAL
                        current_node.available_sim_interval = t + safe_headway_in_simu_interval
                        agent.micro_path_node_seq_no_list.append(to_node_changing.node_seq_no)
                        agent.micro_path_link_seq_no_list.append(outgoing_link_changing.link_seq_no)
                        agent.micro_path_node_id_list.append(to_node_changing.node_id)
                        agent.micro_path_link_id_list.append(outgoing_link_changing.link_id)
                        agent.m_Veh_LinkArrivalTime_in_simu_interval.append(t)
                        agent.m_Veh_LinkDepartureTime_in_simu_interval.append(
                            t + outgoing_link_changing.free_flow_travel_time_in_simu_interval)

                    elif (cost_keeping < _MAX_TIME_INTERVAL) & (abs(cost_keeping - cost_changing) < 0.1):
                        if np.random.random() < 0.06:
                            to_node_changing.available_sim_interval = _MAX_TIME_INTERVAL
                            current_node.available_sim_interval = t + safe_headway_in_simu_interval
                            agent.micro_path_node_seq_no_list.append(to_node_changing.node_seq_no)
                            agent.micro_path_link_seq_no_list.append(outgoing_link_changing.link_seq_no)
                            agent.micro_path_node_id_list.append(to_node_changing.node_id)
                            agent.micro_path_link_id_list.append(outgoing_link_changing.link_id)
                            agent.m_Veh_LinkArrivalTime_in_simu_interval.append(t)
                            agent.m_Veh_LinkDepartureTime_in_simu_interval.append(
                                t + outgoing_link_changing.free_flow_travel_time_in_simu_interval)
                        else:
                            to_node_keeping.available_sim_interval = _MAX_TIME_INTERVAL
                            current_node.available_sim_interval = t + safe_headway_in_simu_interval
                            agent.micro_path_node_seq_no_list.append(to_node_keeping.node_seq_no)
                            agent.micro_path_link_seq_no_list.append(outgoing_link_keeping.link_seq_no)
                            agent.micro_path_node_id_list.append(to_node_keeping.node_id)
                            agent.micro_path_link_id_list.append(outgoing_link_keeping.link_id)
                            agent.m_Veh_LinkArrivalTime_in_simu_interval.append(t)
                            agent.m_Veh_LinkDepartureTime_in_simu_interval.append(
                                t + outgoing_link_keeping.free_flow_travel_time_in_simu_interval)

                    else:
                        agent.m_Veh_LinkDepartureTime_in_simu_interval[-1] += 1

        for agent_no in agent_remove_list: active_agent_list.remove(agent_no)

    printProgress(g_Simulation_EndTimeInMin, g_Simulation_EndTimeInMin,width=30)
    print()
    time_end = time.time()
    print(f'time used for traffic simulation: {round(time_end - time_start, 3)} seconds')


def outputResults():
    print('Outputing Simulation Results...')
    with open(os.path.join(g_working_directory, output_filename), 'w', newline='') as outfile:
        writer = csv.writer(outfile)
        writer.writerow(
            ['agent_id', 'tourid', 'dependency_agent_id', 'duration_in_min', 'following_agent_id', 'from_zone_id',
             'to_zone_id', 'from_origin_node_id',
             'to_destination_node_id', 'departure_time_in_min', 'arrival_time_in_min', 'complete_flag',
             'travel_time_in_min', 'demand_type',
             'vehicle_type', 'PCE', 'information_type', 'value_of_time', 'toll_cost', 'distance', 'TotalEnergy_(KJ)',
             'CO2_(g)', 'NOX_(g)', 'CO_(g)', 'HC_(g)',
             'PM_(g)', 'PM_2.5(g)', 'vehicle_age', 'number_of_nodes', 'path_node_sequence', 'org_path_node_sequence',
             'path_time_sequence', 'origin_node_x',
             'origin_node_y', 'destination_node_x', 'destination_node_y'])
        for agent in g_agent_list:
            agent.outputDataGeneration()
            line = [agent.agent_id,
                    '', '', '', '', '', '',
                    agent.meso_origin_node_id,
                    agent.meso_destination_node_id,
                    agent.departure_time_in_min,
                    agent.arrival_time_in_min,
                    'c' if agent.m_bCompleteTrip else 'n',
                    agent.travel_time_in_min,
                    '', '',
                    agent.PCE_factor,
                    '', '', '', '', '', '', '', '', '', '', '', '',
                    agent.number_of_nodes,
                    agent.micro_path_node_str,
                    '',
                    agent.micro_path_time_str,
                    '', '', '', '']
            writer.writerow(line)
    print('Done')


if "__main__" == __name__:
    load_args()
    g_ReadInputData()
    g_TrafficAssignment()
    g_CostTreeCalculation()
    g_TrafficSimulation()
    outputResults()
