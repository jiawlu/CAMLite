# @author       Jiawei Lu (jiaweil9@asu.edu)
#               Xuesong Zhou (xzhou74@asu.edu)
# @time         2020/12/21 13:52
# @desc         [script description]

from util import printProgress
import networkx as nx
import time
import random
import numpy as np


_MAX_COST_TREE = 99999999
_MAX_TIME_INTERVAL = 9999999

class Simulation:
    def __init__(self, net, demands, signal_manager, args):
        self.number_of_seconds_per_interval = args['sim_step']
        self.Simulation_StartTimeInMin = args['sim_start_time']
        self.Simulation_EndTimeInMin = args['sim_end_time']
        self.time_headway = args['headway']

        self.network = net
        self.demands = demands
        self.signal_manager = signal_manager

        self.generateCostTree()


    def generateCostTree(self):
        print('Initializing Simulator (Backward Tree Generation)')
        net = self.network
        for mesolink in net.meso_link_list:
            current_node = net.meso_node_list[net.meso_node_id_to_seq_no_dict[mesolink.to_node_id]]
            for micro_node_id in mesolink.micro_outgoing_node_id_list:
                for downstream_mesolink_id in current_node.m_outgoing_link_list:
                    downstream_mesolink = net.meso_link_list[net.meso_link_id_to_seq_no_dict[downstream_mesolink_id]]
                    if micro_node_id in downstream_mesolink.micro_incoming_node_id_list:
                        micro_node_seq_no = net.micro_node_id_to_seq_no_dict[micro_node_id]
                        if downstream_mesolink.link_seq_no in mesolink.turning_node_seq_no_dict.keys():
                            mesolink.turning_node_seq_no_dict[downstream_mesolink.link_seq_no].append(micro_node_seq_no)
                        else:
                            mesolink.turning_node_seq_no_dict[downstream_mesolink.link_seq_no] = [micro_node_seq_no]

        for mesolink in net.meso_link_list:
            for downstream_mesolink_seq_no in mesolink.turning_node_seq_no_dict.keys():
                mesolink.estimated_cost_tree_for_each_movement[downstream_mesolink_seq_no] = {}
                for lane_node_set in mesolink.micro_node_list:
                    for node_id in lane_node_set:
                        mesolink.estimated_cost_tree_for_each_movement[downstream_mesolink_seq_no][net.micro_node_id_to_seq_no_dict[node_id]] = _MAX_COST_TREE

                if True:  # mesolink.isconnector
                    for node_id in mesolink.micro_incoming_node_id_list:
                        mesolink.estimated_cost_tree_for_each_movement[downstream_mesolink_seq_no][net.micro_node_id_to_seq_no_dict[node_id]] = _MAX_COST_TREE
                    for node_id in mesolink.micro_outgoing_node_id_list:
                        mesolink.estimated_cost_tree_for_each_movement[downstream_mesolink_seq_no][net.micro_node_id_to_seq_no_dict[node_id]] = _MAX_COST_TREE

                for outgoing_node_seq_no in mesolink.turning_node_seq_no_dict[downstream_mesolink_seq_no]:
                    outgoing_node_id = net.micro_node_list[outgoing_node_seq_no].node_id
                    path_length = nx.single_source_dijkstra_path_length(mesolink.graph, outgoing_node_id)
                    for node_seq_no in mesolink.estimated_cost_tree_for_each_movement[downstream_mesolink_seq_no].keys():
                        node_id = net.micro_node_list[node_seq_no].node_id
                        if node_id in path_length.keys():
                            if path_length[node_id] < mesolink.estimated_cost_tree_for_each_movement[downstream_mesolink_seq_no][node_seq_no]:
                                mesolink.estimated_cost_tree_for_each_movement[downstream_mesolink_seq_no][
                                    node_seq_no] = path_length[node_id]

            mesolink.estimated_cost_tree_for_each_movement[-1] = {}
            for lane_node_set in mesolink.micro_node_list:
                for node_id in lane_node_set:
                    mesolink.estimated_cost_tree_for_each_movement[-1][net.micro_node_id_to_seq_no_dict[node_id]] = _MAX_COST_TREE

            if True:  # mesolink.isconnector
                for node_id in mesolink.micro_incoming_node_id_list:
                    mesolink.estimated_cost_tree_for_each_movement[-1][net.micro_node_id_to_seq_no_dict[node_id]] = _MAX_COST_TREE
                for node_id in mesolink.micro_outgoing_node_id_list:
                    mesolink.estimated_cost_tree_for_each_movement[-1][net.micro_node_id_to_seq_no_dict[node_id]] = _MAX_COST_TREE

            for outgoing_node_id in mesolink.micro_outgoing_node_id_list:
                path_length = nx.single_source_dijkstra_path_length(mesolink.graph, outgoing_node_id)
                for node_seq_no in mesolink.estimated_cost_tree_for_each_movement[-1].keys():
                    node_id = net.micro_node_list[node_seq_no].node_id
                    if node_id in path_length.keys():
                        if path_length[node_id] < mesolink.estimated_cost_tree_for_each_movement[-1][node_seq_no]:
                            mesolink.estimated_cost_tree_for_each_movement[-1][node_seq_no] = path_length[node_id]

    def startSim(self):
        net = self.network
        agent_list = self.demands.agent_list
        number_of_agents = self.demands.number_of_agents
        signal_manager = self.signal_manager

        number_of_simu_intervals = int(np.ceil((self.Simulation_EndTimeInMin - self.Simulation_StartTimeInMin) * 60 / self.number_of_seconds_per_interval))
        active_agent_list = []
        number_of_simu_interval_per_min = 60 / self.number_of_seconds_per_interval
        current_active_agent_index = 0
        safe_headway_in_simu_interval = np.round(self.time_headway / self.number_of_seconds_per_interval)

        departure_time_list = [agent.departure_time_in_simu_interval for agent in agent_list]
        agent_index = np.argsort(departure_time_list)

        print('Performing Simulation...')
        time_start = time.time()
        number_of_agents_finished_trip = 0

        for t in range(number_of_simu_intervals):

            simu_time_in_min = t * self.number_of_seconds_per_interval / 60 + self.Simulation_StartTimeInMin

            if t % number_of_simu_interval_per_min == 0:

                printProgress(t * self.number_of_seconds_per_interval / 60, self.Simulation_EndTimeInMin - self.Simulation_StartTimeInMin, width=30)

                while current_active_agent_index < number_of_agents:
                    if agent_list[agent_index[current_active_agent_index]].departure_time_in_simu_interval < t:
                        current_active_agent_index += 1
                        continue
                    if agent_list[agent_index[
                        current_active_agent_index]].departure_time_in_simu_interval >= t + number_of_simu_interval_per_min: break

                    agent = agent_list[agent_index[current_active_agent_index]]
                    if agent.m_no_physical_path_flag:
                        current_active_agent_index += 1
                        continue

                    agent.m_bGenereated = True
                    agent.current_meso_path_seq_no = 0
                    agent.micro_path_node_seq_no_list.append(agent.micro_origin_node_seq_no)
                    agent.micro_path_node_id_list.append(agent.micro_origin_node_id)
                    agent.m_Veh_LinkArrivalTime_in_simu_interval.append(agent.departure_time_in_simu_interval)
                    agent.m_Veh_LinkDepartureTime_in_simu_interval.append(agent.departure_time_in_simu_interval)
                    agent.meso_path_node_time_in_simu_interval.append(agent.departure_time_in_simu_interval)

                    agent.current_meso_link_seq_no = agent.meso_path_link_seq_no_list[agent.current_meso_path_seq_no]
                    if agent.current_meso_path_seq_no < len(agent.meso_path_link_seq_no_list) - 1:
                        agent.next_meso_link_seq_no = agent.meso_path_link_seq_no_list[agent.current_meso_path_seq_no + 1]
                    else:
                        agent.next_meso_link_seq_no = -1

                    active_agent_list.append(agent.agent_seq_no)
                    current_active_agent_index += 1

            random.shuffle(active_agent_list)
            agent_remove_list = []
            inactive_microlinks = signal_manager.getStatus()

            for agent_no in active_agent_list:
                agent = agent_list[agent_no]
                if agent.m_Veh_LinkDepartureTime_in_simu_interval[-1] == t:
                    current_node = net.micro_node_list[agent.micro_path_node_seq_no_list[-1]]
                    if current_node.node_id in net.meso_link_list[agent.current_meso_link_seq_no].micro_outgoing_node_id_list:
                        agent.meso_path_node_time_in_simu_interval.append(t)
                        if agent.current_meso_path_seq_no == len(agent.meso_path_link_seq_no_list) - 1:
                            agent.micro_destination_node_id, agent.micro_destination_node_seq_no = current_node.node_id, current_node.node_seq_no
                            agent_remove_list.append(agent.agent_seq_no)
                            number_of_agents_finished_trip += 1
                            current_node.available_sim_interval = t
                            continue
                        else:
                            agent.current_meso_path_seq_no += 1
                            agent.current_meso_link_seq_no = agent.meso_path_link_seq_no_list[agent.current_meso_path_seq_no]
                            if agent.current_meso_path_seq_no < len(agent.meso_path_link_seq_no_list) - 1:
                                agent.next_meso_link_seq_no = agent.meso_path_link_seq_no_list[agent.current_meso_path_seq_no + 1]
                            else:
                                agent.next_meso_link_seq_no = -1

                    outgoing_link_keeping = None
                    outgoing_link_keeping_td_travel_time_in_simu_interval = 0
                    to_node_keeping = None
                    cost_keeping = 0.0

                    outgoing_link_changing_list = []
                    to_node_changing_list = []
                    cost_changing_list = []

                    for outgoing_link_id in current_node.m_outgoing_link_list:
                        outgoing_link = net.micro_link_list[net.micro_link_id_to_seq_no_dict[outgoing_link_id]]
                        outgoing_link_td_travel_time_in_simu_interval = max(round(outgoing_link.length / outgoing_link.td_travel_speed(simu_time_in_min) * 3.6 / self.number_of_seconds_per_interval), 1)

                        if net.meso_link_id_to_seq_no_dict[outgoing_link.meso_link.link_id] != agent.current_meso_link_seq_no:
                            continue
                        if outgoing_link in inactive_microlinks:
                            continue

                        if outgoing_link.cell_type == 1:
                            outgoing_link_keeping = outgoing_link
                            outgoing_link_keeping_td_travel_time_in_simu_interval = outgoing_link_td_travel_time_in_simu_interval
                            to_node_keeping = net.micro_node_list[net.micro_node_id_to_seq_no_dict[outgoing_link.to_node_id]]

                            tree_cost = net.meso_link_list[agent.current_meso_link_seq_no].estimated_cost_tree_for_each_movement[agent.next_meso_link_seq_no][to_node_keeping.node_seq_no]
                            if tree_cost == _MAX_COST_TREE:
                                cost_keeping = _MAX_COST_TREE
                            elif to_node_keeping.available_sim_interval > t + outgoing_link_td_travel_time_in_simu_interval:
                                cost_keeping = _MAX_TIME_INTERVAL
                            else:
                                cost_keeping = outgoing_link_td_travel_time_in_simu_interval + outgoing_link.additional_cost + tree_cost

                        else:
                            outgoing_link_changing_list.append(outgoing_link)
                            to_node_changing = net.micro_node_list[net.micro_node_id_to_seq_no_dict[outgoing_link.to_node_id]]

                            tree_cost = net.meso_link_list[agent.current_meso_link_seq_no].estimated_cost_tree_for_each_movement[agent.next_meso_link_seq_no][to_node_changing.node_seq_no]
                            if tree_cost == _MAX_COST_TREE:
                                cost_changing = _MAX_COST_TREE
                            elif to_node_changing.available_sim_interval > t + outgoing_link_td_travel_time_in_simu_interval:
                                cost_changing = _MAX_TIME_INTERVAL
                            else:
                                cost_changing = outgoing_link_td_travel_time_in_simu_interval + outgoing_link.additional_cost + tree_cost

                            to_node_changing_list.append(to_node_changing)
                            cost_changing_list.append(cost_changing)

                    if to_node_keeping is None:
                        agent.m_Veh_LinkDepartureTime_in_simu_interval[-1] += 1
                        continue

                    if len(cost_changing_list) == 0:

                        if cost_keeping == _MAX_COST_TREE:
                            usable_incoming_nodes = []
                            for incoming_node_id in net.meso_link_list[agent.current_meso_link_seq_no].micro_incoming_node_id_list:
                                incoming_node = net.micro_node_list[net.micro_node_id_to_seq_no_dict[incoming_node_id]]
                                tree_cost = net.meso_link_list[agent.current_meso_link_seq_no].estimated_cost_tree_for_each_movement[agent.next_meso_link_seq_no][incoming_node.node_seq_no]
                                if tree_cost < _MAX_COST_TREE:
                                    usable_incoming_nodes.append(incoming_node)
                            
                            if usable_incoming_nodes:
                                node_to_switch = random.choice(usable_incoming_nodes)
                                if node_to_switch.available_sim_interval <= t:
                                    node_to_switch.available_sim_interval = _MAX_TIME_INTERVAL
                                    current_node.available_sim_interval = t + safe_headway_in_simu_interval
                                    agent.micro_path_node_seq_no_list.append(node_to_switch.node_seq_no)
                                    agent.micro_path_link_seq_no_list.append(-1)
                                    agent.micro_path_node_id_list.append(node_to_switch.node_id)
                                    agent.micro_path_link_id_list.append(-1)
                                    agent.m_Veh_LinkArrivalTime_in_simu_interval.append(t + 1)
                                    agent.m_Veh_LinkDepartureTime_in_simu_interval.append(t + 1)
                                    agent.note = f'moved to micro node {current_node.node_id} to {node_to_switch.node_id}'
                                else:
                                    agent.m_Veh_LinkDepartureTime_in_simu_interval[-1] += 1
                            else:
                                agent_remove_list.append(agent.agent_seq_no)
                                current_node.available_sim_interval = t
                                agent.note = f'removed from simulation on micro node {current_node.node_id}'

                        elif cost_keeping < _MAX_TIME_INTERVAL:
                            to_node_keeping.available_sim_interval = _MAX_TIME_INTERVAL
                            current_node.available_sim_interval = t + safe_headway_in_simu_interval
                            agent.micro_path_node_seq_no_list.append(to_node_keeping.node_seq_no)
                            agent.micro_path_link_seq_no_list.append(outgoing_link_keeping.link_seq_no)
                            agent.micro_path_node_id_list.append(to_node_keeping.node_id)
                            agent.micro_path_link_id_list.append(outgoing_link_keeping.link_id)
                            agent.m_Veh_LinkArrivalTime_in_simu_interval.append(t + outgoing_link_keeping_td_travel_time_in_simu_interval)
                            agent.m_Veh_LinkDepartureTime_in_simu_interval.append(t + outgoing_link_keeping_td_travel_time_in_simu_interval)
                        else:
                            agent.m_Veh_LinkDepartureTime_in_simu_interval[-1] += 1

                    else:
                        cost_changing = min(cost_changing_list)
                        smallest_position = cost_changing_list.index(cost_changing)
                        to_node_changing = to_node_changing_list[smallest_position]
                        outgoing_link_changing = outgoing_link_changing_list[smallest_position]
                        outgoing_link_td_travel_time_in_simu_interval = max(round(outgoing_link_changing.length / outgoing_link_changing.td_travel_speed(simu_time_in_min) * 3.6 / self.number_of_seconds_per_interval), 1)

                        if (cost_keeping == _MAX_COST_TREE) & (cost_changing == _MAX_COST_TREE):
                            usable_incoming_nodes = []
                            for incoming_node_id in net.meso_link_list[agent.current_meso_link_seq_no].micro_incoming_node_id_list:
                                incoming_node = net.micro_node_list[net.micro_node_id_to_seq_no_dict[incoming_node_id]]
                                tree_cost = net.meso_link_list[agent.current_meso_link_seq_no].estimated_cost_tree_for_each_movement[agent.next_meso_link_seq_no][incoming_node.node_seq_no]
                                if tree_cost < _MAX_COST_TREE:
                                    usable_incoming_nodes.append(incoming_node)
                            
                            if usable_incoming_nodes:
                                node_to_switch = random.choice(usable_incoming_nodes)
                                if node_to_switch.available_sim_interval <= t:
                                    node_to_switch.available_sim_interval = _MAX_TIME_INTERVAL
                                    current_node.available_sim_interval = t + safe_headway_in_simu_interval
                                    agent.micro_path_node_seq_no_list.append(node_to_switch.node_seq_no)
                                    agent.micro_path_link_seq_no_list.append(-1)
                                    agent.micro_path_node_id_list.append(node_to_switch.node_id)
                                    agent.micro_path_link_id_list.append(-1)
                                    agent.m_Veh_LinkArrivalTime_in_simu_interval.append(t + 1)
                                    agent.m_Veh_LinkDepartureTime_in_simu_interval.append(t + 1)
                                    agent.note = f'moved to micro node {current_node.node_id} to {node_to_switch.node_id}'
                                else:
                                    agent.m_Veh_LinkDepartureTime_in_simu_interval[-1] += 1
                            else:
                                agent_remove_list.append(agent.agent_seq_no)
                                current_node.available_sim_interval = t
                                agent.note = f'removed from simulation on micro node {current_node.node_id}'

                        elif (cost_keeping < _MAX_TIME_INTERVAL) & (cost_keeping < cost_changing):
                            to_node_keeping.available_sim_interval = _MAX_TIME_INTERVAL
                            current_node.available_sim_interval = t + safe_headway_in_simu_interval
                            agent.micro_path_node_seq_no_list.append(to_node_keeping.node_seq_no)
                            agent.micro_path_link_seq_no_list.append(outgoing_link_keeping.link_seq_no)
                            agent.micro_path_node_id_list.append(to_node_keeping.node_id)
                            agent.micro_path_link_id_list.append(outgoing_link_keeping.link_id)
                            agent.m_Veh_LinkArrivalTime_in_simu_interval.append(t + outgoing_link_keeping_td_travel_time_in_simu_interval)
                            agent.m_Veh_LinkDepartureTime_in_simu_interval.append(t + outgoing_link_keeping_td_travel_time_in_simu_interval)

                        elif (cost_changing < _MAX_TIME_INTERVAL) & (cost_changing < cost_keeping):
                            to_node_changing.available_sim_interval = _MAX_TIME_INTERVAL
                            current_node.available_sim_interval = t + safe_headway_in_simu_interval
                            agent.micro_path_node_seq_no_list.append(to_node_changing.node_seq_no)
                            agent.micro_path_link_seq_no_list.append(outgoing_link_changing.link_seq_no)
                            agent.micro_path_node_id_list.append(to_node_changing.node_id)
                            agent.micro_path_link_id_list.append(outgoing_link_changing.link_id)
                            agent.m_Veh_LinkArrivalTime_in_simu_interval.append(t + outgoing_link_td_travel_time_in_simu_interval)
                            agent.m_Veh_LinkDepartureTime_in_simu_interval.append(t + outgoing_link_td_travel_time_in_simu_interval)

                        elif (cost_keeping < _MAX_TIME_INTERVAL) & (abs(cost_keeping - cost_changing) < 0.1):
                            if random.random() < 0.15:
                                to_node_changing.available_sim_interval = _MAX_TIME_INTERVAL
                                current_node.available_sim_interval = t + safe_headway_in_simu_interval
                                agent.micro_path_node_seq_no_list.append(to_node_changing.node_seq_no)
                                agent.micro_path_link_seq_no_list.append(outgoing_link_changing.link_seq_no)
                                agent.micro_path_node_id_list.append(to_node_changing.node_id)
                                agent.micro_path_link_id_list.append(outgoing_link_changing.link_id)
                                agent.m_Veh_LinkArrivalTime_in_simu_interval.append(t + outgoing_link_td_travel_time_in_simu_interval)
                                agent.m_Veh_LinkDepartureTime_in_simu_interval.append(t + outgoing_link_td_travel_time_in_simu_interval)
                            else:
                                to_node_keeping.available_sim_interval = _MAX_TIME_INTERVAL
                                current_node.available_sim_interval = t + safe_headway_in_simu_interval
                                agent.micro_path_node_seq_no_list.append(to_node_keeping.node_seq_no)
                                agent.micro_path_link_seq_no_list.append(outgoing_link_keeping.link_seq_no)
                                agent.micro_path_node_id_list.append(to_node_keeping.node_id)
                                agent.micro_path_link_id_list.append(outgoing_link_keeping.link_id)
                                agent.m_Veh_LinkArrivalTime_in_simu_interval.append(t + outgoing_link_keeping_td_travel_time_in_simu_interval)
                                agent.m_Veh_LinkDepartureTime_in_simu_interval.append(t + outgoing_link_keeping_td_travel_time_in_simu_interval)

                        else:
                            agent.m_Veh_LinkDepartureTime_in_simu_interval[-1] += 1

            for agent_no in agent_remove_list: active_agent_list.remove(agent_no)

        printProgress(self.Simulation_EndTimeInMin, self.Simulation_EndTimeInMin, width=30)
        print()
        time_end = time.time()
        print(f'time used for traffic simulation: {round(time_end - time_start, 3)} seconds')
        print(f'{number_of_agents_finished_trip}/{number_of_agents} agents finished trip')
