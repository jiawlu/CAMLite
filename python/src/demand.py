# @author       Jiawei Lu (jiaweil9@asu.edu)
#               Xuesong Zhou (xzhou74@asu.edu)
# @time         2020/12/21 14:01
# @desc         [script description]

import random
from util import getFormattedTime

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
        self.micro_path_time_formatted_str = ''
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
        self.number_of_seconds_per_interval = 0
        self.Simulation_StartTimeInMin = 0.0

    def outputDataGeneration(self):
        micro_path_node_id_list_str = list(map(str, self.micro_path_node_id_list))
        self.micro_path_node_str = ';'.join(micro_path_node_id_list_str)

        micro_path_time_list_str = list(map(lambda x: str(round(x * self.number_of_seconds_per_interval / 60 + self.Simulation_StartTimeInMin, 3)),
                                            self.m_Veh_LinkArrivalTime_in_simu_interval))
        self.micro_path_time_str = ';'.join(micro_path_time_list_str)

        formatted_time_list = []
        for arrival_time_interval in self.m_Veh_LinkArrivalTime_in_simu_interval:
            time_decimal = arrival_time_interval * self.number_of_seconds_per_interval / 60 + self.Simulation_StartTimeInMin
            time_formatted = getFormattedTime(time_decimal)
            formatted_time_list.append(time_formatted)
        self.micro_path_time_formatted_str = ';'.join(formatted_time_list)

        self.number_of_nodes = len(self.micro_path_node_id_list)


class Demand:
    def __init__(self):
        self.number_of_seconds_per_interval = 0
        self.Simulation_StartTimeInMin = 0.0

        self.demand_record_list = []
        self.number_of_agents = 0
        self.agent_list = []

    def initialization(self, net, args_):
        print('  initializing demand')
        self.number_of_seconds_per_interval = args_['sim_step']
        self.Simulation_StartTimeInMin = args_['sim_start_time']
        self.generateAgents(net)

    def generateAgents(self, net):
        for demand_record in self.demand_record_list:
            from_origin_node_id = demand_record['o_node_id']
            to_destination_node_id = demand_record['d_node_id']
            number_of_trips = demand_record['volume']
            time_start = demand_record['time_start']
            time_end = demand_record['time_end']

            start_node = net.meso_node_list[net.meso_node_id_to_seq_no_dict[from_origin_node_id]]
            start_link = net.meso_link_list[net.meso_link_id_to_seq_no_dict[start_node.m_outgoing_link_list[0]]]

            for j in range(number_of_trips):
                agent = Agent()
                agent.agent_id = self.number_of_agents
                agent.agent_seq_no = self.number_of_agents
                agent.meso_origin_node_id = from_origin_node_id
                agent.meso_destination_node_id = to_destination_node_id
                agent.departure_time_in_min = random.random() * (time_end - time_start) + time_start
                agent.meso_origin_node_seq_no = net.meso_node_id_to_seq_no_dict[from_origin_node_id]
                agent.meso_destination_node_seq_no = net.meso_node_id_to_seq_no_dict[to_destination_node_id]

                seq_no = random.randint(0,len(start_link.micro_incoming_node_id_list)-1)
                agent.micro_origin_node_id = start_link.micro_incoming_node_id_list[seq_no]
                agent.micro_origin_node_seq_no = net.micro_node_id_to_seq_no_dict[agent.micro_origin_node_id]

                agent.departure_time_in_simu_interval = round((agent.departure_time_in_min - self.Simulation_StartTimeInMin) * 60 / self.number_of_seconds_per_interval)
                agent.number_of_seconds_per_interval = self.number_of_seconds_per_interval
                agent.Simulation_StartTimeInMin = self.Simulation_StartTimeInMin

                self.agent_list.append(agent)
                self.number_of_agents += 1
