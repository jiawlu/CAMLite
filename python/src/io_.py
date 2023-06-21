# @author       Jiawei Lu (jiaweil9@asu.edu)
#               Xuesong Zhou (xzhou74@asu.edu)
# @time         2020/12/21 13:57
# @desc         [script description]

import argparse
import csv
import os
import sys

from network import *
from demand import Demand
from traffic_signal import *
import re


_working_directory = ''


def load_args():
    global _working_directory
    print('Loading Simulation Settings...')

    parser = argparse.ArgumentParser(description='ParameterOptimizer')
    # --- Working directory ---
    parser.add_argument('--cwd', default=r'', type=str,
                        help='Working directory of simulation')

    # --- Simulation settings ---
    parser.add_argument('--sim_step', default=0.1, type=float,
                        help='Simulation step. Default: 0.1s')
    parser.add_argument('--sim_start_time', default=0, type=float,
                        help='Simulation start time in minutes. Default: 0min')
    parser.add_argument('--sim_end_time', default=60, type=float,
                        help='Simulation end time in minutes. Default: 60min')
    parser.add_argument('--assignment_iters', default=10, type=int,
                        help='Number of MSA assignment iterations. Default: 10')
    parser.add_argument('--headway', default=1.4, type=float,
                        help='Minimum time headway between vehicles. Default: 1.4s')

    args = vars(parser.parse_args())

    print(f'  cwd: {args["cwd"]}')
    print(f'  sim_step: {args["sim_step"]}')
    print(f'  sim_start_time: {args["sim_start_time"]}')
    print(f'  sim_end_time: {args["sim_end_time"]}')
    print(f'  assignment_iters: {args["assignment_iters"]}')
    print(f'  headway: {args["headway"]}')

    _working_directory = args["cwd"]
    return args


def readInputData():
    net = Network()
    demands = Demand()
    signal_manager = SignalManager()

    print('Reading input data...')
    # ------------------Meso Node--------------------#
    print('  loading mesoscopic nodes')
    mesonode_filepath = os.path.join(_working_directory, r'mesonet/node.csv')
    with open(mesonode_filepath, 'r') as fp:
        reader = csv.DictReader(fp)
        for i, line in enumerate(reader):
            mesonode = MesoNode()
            mesonode.node_id = int(line['node_id'])
            mesonode.zone_id = int(line['zone_id']) if line['zone_id'] else None
            mesonode.x = float(line['x_coord'])
            mesonode.y = float(line['y_coord'])
            mesonode.node_seq_no = i

            net.meso_node_list.append(mesonode)
            net.meso_node_id_to_seq_no_dict[mesonode.node_id] = mesonode.node_seq_no
            if mesonode.zone_id is not None:
                if mesonode.zone_id not in net.zone_dict.keys():
                    zone = Zone(mesonode.zone_id)
                    zone.meso_node_list.append(mesonode)
                    net.zone_dict[zone.zone_id] = zone
                else:
                    zone = net.zone_dict[mesonode.zone_id]
                    zone.meso_node_list.append(mesonode)

    net.number_of_meso_nodes = len(net.meso_node_list)

    # ------------------Meso Link--------------------#
    print('  loading mesoscopic links')
    mesolink_filepath = os.path.join(_working_directory, r'mesonet/link.csv')
    with open(mesolink_filepath, 'r') as fp:
        reader = csv.DictReader(fp)
        for i,line in enumerate(reader):
            mesolink = MesoLink()
            mesolink.link_id = line['link_id']
            mesolink.from_node_id = int(line['from_node_id'])
            mesolink.to_node_id = int(line['to_node_id'])
            mesolink.length = float(line['length'])
            mesolink.number_of_lanes = int(line['lanes'])
            mesolink.speed_limit = float(line['free_speed']) * 0.83
            lane_cap = line['capacity']
            mesolink.lane_cap = float(lane_cap) if lane_cap else 1200
            mesolink.geometry = line['geometry']

            mesolink.link_seq_no = i
            from_node = net.meso_node_list[net.meso_node_id_to_seq_no_dict[mesolink.from_node_id]]
            from_node.m_outgoing_link_list.append(mesolink.link_id)
            to_node = net.meso_node_list[net.meso_node_id_to_seq_no_dict[mesolink.to_node_id]]
            to_node.m_incoming_link_list.append(mesolink.link_id)

            mesolink.setLinkKey()
            mesolink.initializeMicroNodeList()
            net.meso_link_list.append(mesolink)
            net.meso_link_id_to_seq_no_dict[mesolink.link_id] = mesolink.link_seq_no
            net.meso_link_key_to_seq_no_dict[mesolink.link_key] = mesolink.link_seq_no

    net.number_of_meso_links = len(net.meso_link_list)

    # ------------------Micro Node--------------------#
    print('  loading microscopic nodes')
    micronode_filepath = os.path.join(_working_directory, r'micronet/node.csv')
    with open(micronode_filepath, 'r') as fp:
        reader = csv.DictReader(fp)
        for i,line in enumerate(reader):
            micronode = MicroNode()
            micronode.node_id = int(line['node_id'])
            micronode.x = float(line['x_coord'])
            micronode.y = float(line['y_coord'])
            micronode.meso_link_id = line['meso_link_id']
            micronode.lane_no = int(line['lane_no'])

            micronode.node_seq_no = i
            net.micro_node_list.append(micronode)

            net.micro_node_id_to_seq_no_dict[micronode.node_id] = micronode.node_seq_no
            meso_link = net.meso_link_list[net.meso_link_id_to_seq_no_dict[micronode.meso_link_id]]
            meso_link.micro_node_list[micronode.lane_no - 1].append(micronode.node_id)

    net.number_of_micro_nodes = len(net.micro_node_list)

    # ------------------Micro Link--------------------#
    print('  loading microscopic links')
    microlink_filepath = os.path.join(_working_directory, r'micronet/link.csv')
    with open(microlink_filepath, 'r') as fp:
        reader = csv.DictReader(fp)
        for i,line in enumerate(reader):
            microlink = MicroLink()
            microlink.link_id = line['link_id']
            microlink.from_node_id = int(line['from_node_id'])
            microlink.to_node_id = int(line['to_node_id'])
            microlink.meso_link = net.meso_link_list[net.meso_link_id_to_seq_no_dict[line['meso_link_id']]]
            microlink.cell_type = int(line['cell_type'])
            microlink.length = float(line['length'])
            microlink.speed_limit = float(line['free_speed']) * 0.83
            microlink.additional_cost = float(line['additional_cost'])

            microlink.link_seq_no = i
            # microlink.Initialization()

            net.micro_link_list.append(microlink)

            from_node = net.micro_node_list[net.micro_node_id_to_seq_no_dict[microlink.from_node_id]]
            from_node.m_outgoing_link_list.append(microlink.link_id)
            to_node = net.micro_node_list[net.micro_node_id_to_seq_no_dict[microlink.to_node_id]]
            to_node.m_incoming_link_list.append(microlink.link_id)

            mesolink = microlink.meso_link
            mesolink.micro_link_list.append(microlink.link_id)
            mesolink.graph.add_edge(microlink.to_node_id, microlink.from_node_id, weight=microlink.cost)  # reverse link direction to reduce shortest path searching time
            net.micro_link_id_to_seq_no_dict[microlink.link_id] = microlink.link_seq_no

    net.number_of_micro_links = len(net.micro_link_list)

    print(f'  net: {net.number_of_meso_nodes} meso nodes, {net.number_of_meso_links} meso links, '
          f'{net.number_of_micro_nodes} micro nodes, {net.number_of_micro_links} micro links')

    # ------------------Demand--------------------#
    print('  loading demands')
    agent_filepath = os.path.join(_working_directory, r'input_agent.csv')
    demand_filepath = os.path.join(_working_directory, r'input_demand.csv')

    if os.path.isfile(agent_filepath):
        agent_record_list = demands.agent_record_list
        with open(agent_filepath, 'r') as fp:
            reader = csv.DictReader(fp)
            for line in reader:
                agent_record = {'agent_id': line['agent_id'],
                                'o_zone_id': line['o_zone_id'],
                                'd_zone_id': line['d_zone_id'],
                                'o_node_id': line['o_node_id'],
                                'd_node_id': line['d_node_id'],
                                'departure_time': float(line['departure_time']),
                                'route': line['route']}
                agent_record_list.append(agent_record)
        print(f'  demand: {len(agent_record_list)} agents')

    elif os.path.isfile(demand_filepath):
        demand_record_list = demands.demand_record_list
        total_demands = 0
        with open(demand_filepath, 'r') as fp:
            reader = csv.DictReader(fp)
            for line in reader:
                demand_record = {'o_zone_id': line['o_zone_id'],
                                 'd_zone_id': line['d_zone_id'],
                                 'o_node_id': line['o_node_id'],
                                 'd_node_id': line['d_node_id'],
                                 'volume': int(float(line['volume'])),
                                 'time_start': float(line['time_start']),
                                 'time_end': float(line['time_end']),
                                 'route': line['route']}
                demand_record_list.append(demand_record)
                total_demands += int(line['volume'])

        print(f'  demand: {total_demands} agents')

    else:
        print('ERROR: No input_agent.csv or input_demand detected in the working directory')
        sys.exit(-1)

    # ------------------Signal--------------------#
    signal_filepath = os.path.join(_working_directory, r'input_signal.csv')
    if os.path.isfile(signal_filepath):
        print('  loading signal')
        controller_data_dict = signal_manager.controller_data_dict

        with open(signal_filepath, 'r') as fp:
            reader = csv.DictReader(fp)
            for line in reader:
                controller_id = line['controller_id']
                phase_id = int(line['phase_id'])
                movements = line['movement_links']
                duration = int(line['duration'])

                if controller_id not in controller_data_dict.keys(): controller_data_dict[controller_id] = []
                controller = controller_data_dict[controller_id]
                controller.append({'phase_id':phase_id,'movements':movements,'duration':duration})

        print(f'  signal: {len(controller_data_dict)} controllers')

    # ------------------Link Performance--------------------#
    lkpf_filepath = os.path.join(_working_directory, r'link_performance.csv')
    if os.path.isfile(lkpf_filepath):
        print('  loading link_performance')

        with open(lkpf_filepath, 'r') as fp:
            reader = csv.DictReader(fp)

            speed_fieldname_dict = {}
            speed_fieldname_pattern = re.compile(r'v\d{2}:\d{2}')
            for fieldname in reader.fieldnames:
                res = speed_fieldname_pattern.findall(fieldname)
                if res:
                    speed_fieldname = res[0]
                    speed_time_list = speed_fieldname[1:].split(':')
                    speed_time_minute = int(speed_time_list[0]) * 60 + int(speed_time_list[1])
                    speed_fieldname_dict[speed_fieldname] = speed_time_minute

            for line in reader:
                link_id = line['link_id']
                link = net.meso_link_list[net.meso_link_id_to_seq_no_dict[link_id]]
                td_speed_list = [{'time':speed_time_minute, 'value':float(line[speed_fieldname])} for speed_fieldname, speed_time_minute in speed_fieldname_dict.items()]
                td_speed_list.sort(key=lambda x: x['time'])
                for td_speed_info in td_speed_list:
                    link.td_speed_time_list.append(td_speed_info['time'])
                    link.td_speed_value_list.append(td_speed_info['value'])

    return net, demands, signal_manager


def outputResults(demands):
    print('Outputing Simulation Results...')
    with open(os.path.join(_working_directory, 'agent.csv'), 'w', newline='') as outfile:
        writer = csv.writer(outfile)
        writer.writerow(
            ['agent_id', 'o_zone_id', 'd_zone_id',
             'from_origin_node_id', 'to_destination_node_id', 'from_origin_node_id_meso', 'to_destination_node_id_meso',
             'departure_time_in_min', 'arrival_time_in_min', 'fixed_route', 'complete_flag',
             'travel_time_in_min', 'demand_type', 'PCE', 'information_type', 'value_of_time', 'distance',
             'number_of_nodes', 'node_sequence', 'time_sequence','time_sequence_hhmm:ss', 'meso_node_sequence', 'meso_time_sequence',
             'emission_CO2', 'note'])
        for agent in demands.agent_list:
            if not agent.m_bGenereated: continue
            agent.outputDataGeneration()
            line = [agent.agent_id, agent.origin_zone_id, agent.destination_zone_id,
                    agent.micro_origin_node_id, agent.micro_destination_node_id,
                    agent.meso_origin_node_id, agent.meso_destination_node_id,
                    round(agent.departure_time_in_min,3),
                    round(agent.arrival_time_in_min,3) if agent.arrival_time_in_min is not None else None,
                    1 if agent.is_fixed_route else 0,
                    1 if agent.m_bCompleteTrip else 0,
                    round(agent.travel_time_in_min,3) if agent.travel_time_in_min is not None else None,
                    '', 
                    agent.PCE_factor,
                    '', '', agent.distance,
                    agent.number_of_nodes,
                    agent.micro_path_node_str,
                    agent.micro_path_time_str, agent.micro_path_time_formatted_str,
                    agent.meso_node_sequence_str, agent.meso_time_sequence_str,
                    round(agent.emission_CO2,3),
                    agent.note
                    ]
            writer.writerow(line)
    print('Done')