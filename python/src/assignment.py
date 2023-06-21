# @author       Jiawei Lu (jiaweil9@asu.edu)
#               Xuesong Zhou (xzhou74@asu.edu)
# @time         2020/12/21 13:53
# @desc         [script description]

import time
from util import printProgress
import networkx as nx


def _findPathForAgents(demands, net, netgraph, iteration_no):
    for agent in demands.agent_list:
        residual = agent.agent_seq_no % (iteration_no + 1)

        if agent.is_fixed_route:
            continue

        if residual != 0:
            for meso_link_seq_no in agent.meso_path_link_seq_no_list:
                net.meso_link_list[meso_link_seq_no].flow_volume += agent.PCE_factor
            continue

        try:
            path = nx.dijkstra_path(netgraph, source=agent.meso_origin_node_id, target=agent.meso_destination_node_id)
        except:
            print(
                'cannot find a feasible meso path for agent {}, meso_origin_node_id {}, meso_destination_node_id {}'.format(
                    agent.agent_id, agent.meso_origin_node_id, agent.meso_destination_node_id))
            continue

        agent.meso_path_node_id_list = []
        agent.meso_path_node_seq_no_list = []
        agent.meso_path_link_seq_no_list = []
        agent.meso_path_link_list = []

        for node_id in path:
            agent.meso_path_node_id_list.append(node_id)
            agent.meso_path_node_seq_no_list.append(net.meso_node_id_to_seq_no_dict[node_id])
        number_of_nodes_in_path = len(path)
        for i in range(1, number_of_nodes_in_path):
            agent.meso_path_link_seq_no_list.append(net.meso_link_key_to_seq_no_dict[path[i - 1] * 100000 + path[i]])
            agent.meso_path_link_list.append(net.meso_link_list[agent.meso_path_link_seq_no_list[-1]])

        number_of_links_in_path = number_of_nodes_in_path - 1
        for i in range(number_of_links_in_path):
            net.meso_link_list[agent.meso_path_link_seq_no_list[i]].flow_volume += agent.PCE_factor



def trafficAssignment(net,demands,args):
    print('Conducting Traffic Assignment...')
    time_start = time.time()
    netgraph = nx.DiGraph()
    total_assignment_iteration = args['assignment_iters']

    for mesolink in net.meso_link_list:
        mesolink.flow_volume = mesolink.base_volume
        mesolink.CalculateBPRFunctionAndCost()
        netgraph.add_edge(mesolink.from_node_id, mesolink.to_node_id, weight=mesolink.cost)

    printProgress(1, total_assignment_iteration, width=30)
    _findPathForAgents(demands, net, netgraph, 0)

    for i in range(1, total_assignment_iteration):
        for mesolink in net.meso_link_list:
            mesolink.CalculateBPRFunctionAndCost()
            netgraph[mesolink.from_node_id][mesolink.to_node_id]['weight'] = mesolink.cost
            mesolink.flow_volume = mesolink.base_volume

        printProgress(i+1, total_assignment_iteration, width=30)
        _findPathForAgents(demands, net, netgraph, i)
    print()
    time_end = time.time()
    print(f'time used for traffic assignment: {round(time_end-time_start,3)} seconds')