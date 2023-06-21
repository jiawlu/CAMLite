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
        self.m_agent_seq_no = 0
        self.origin_zone_id = None
        self.destination_zone_id = None

        self.departure_time_in_min = 0.0
        self.departure_time_in_simu_interval = 0

        self.meso_origin_node_id = 0
        self.meso_destination_node_id = 0
        self.meso_origin_node_seq_no = 0
        self.meso_destination_node_seq_no = 0

        self.is_fixed_route = False
        self.meso_path_node_seq_no_list = []
        self.meso_path_link_seq_no_list = []
        self.meso_path_link_list = []
        self.meso_path_node_id_list = []
        self.meso_path_node_time_in_simu_interval = []

        self.micro_origin_node_id = None
        self.micro_destination_node_id = None
        self.micro_origin_node_seq_no = None
        self.micro_destination_node_seq_no = None

        self.micro_path_node_str = ''
        self.micro_path_time_str = ''
        self.micro_path_time_formatted_str = ''

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

        self.m_Veh_LinkArrivalTime_in_simu_interval = []
        self.m_Veh_LinkDepartureTime_in_simu_interval = []

        self.current_meso_path_seq_no = 0
        self.current_meso_link_seq_no = 0
        self.next_meso_link_seq_no = 0

        self.m_no_physical_path_flag = False
        self.number_of_seconds_per_interval = 0
        self.Simulation_StartTimeInMin = 0.0

        self.note = ''

    @property
    def m_bCompleteTrip(self):
        return True if self.micro_destination_node_id is not None else False
    @property
    def arrival_time_in_simu_interval(self):
        return self.m_Veh_LinkArrivalTime_in_simu_interval[-1] if self.m_bCompleteTrip else None
    @property
    def arrival_time_in_min(self):
        return self.arrival_time_in_simu_interval * self.number_of_seconds_per_interval / 60 + self.Simulation_StartTimeInMin if self.m_bCompleteTrip else None
    @property
    def travel_time_in_min(self):
        return self.arrival_time_in_min - self.departure_time_in_min if self.m_bCompleteTrip else None
    @property
    def number_of_nodes(self):
        return len(self.micro_path_node_id_list)
    @property
    def meso_node_sequence_str(self):
        return ';'.join(list(map(str, self.meso_path_node_id_list)))
    @property
    def meso_time_sequence_str(self):
        return ';'.join(list(map(lambda x: str(round(x * self.number_of_seconds_per_interval / 60 + self.Simulation_StartTimeInMin, 3)), self.meso_path_node_time_in_simu_interval)))
    @property
    def emission_CO2(self):
        path_emission = 0
        for i in range(1,len(self.meso_path_node_time_in_simu_interval)):
            link = self.meso_path_link_list[i-1]
            travel_time = (self.meso_path_node_time_in_simu_interval[i] - self.meso_path_node_time_in_simu_interval[i-1]) * self.number_of_seconds_per_interval / 3600
            speed = link.length / 1000 / travel_time
            emission_rate = link.mean_emission_list[min(range(len(link.mean_speed_list)), key = lambda k: abs(link.mean_speed_list[k]-speed))]
            link_emission = travel_time * emission_rate
            path_emission += link_emission
        return path_emission
    @property
    def distance(self):
        path_length = 0
        for link in self.meso_path_link_list:
            path_length += link.length
        return round(path_length,2)


    def outputDataGeneration(self):
        micro_path_node_id_list_str = list(map(str, self.micro_path_node_id_list))
        self.micro_path_node_str = ';'.join(micro_path_node_id_list_str)

        micro_path_time_list_str = list(map(lambda x: str(round(x * self.number_of_seconds_per_interval / 60 + self.Simulation_StartTimeInMin, 3)), self.m_Veh_LinkArrivalTime_in_simu_interval))
        self.micro_path_time_str = ';'.join(micro_path_time_list_str)

        formatted_time_list = []
        for arrival_time_interval in self.m_Veh_LinkArrivalTime_in_simu_interval:
            time_decimal = arrival_time_interval * self.number_of_seconds_per_interval / 60 + self.Simulation_StartTimeInMin
            time_formatted = getFormattedTime(time_decimal)
            formatted_time_list.append(time_formatted)
        self.micro_path_time_formatted_str = ';'.join(formatted_time_list)


class Demand:
    def __init__(self):
        self.number_of_seconds_per_interval = 0
        self.Simulation_StartTimeInMin = 0.0

        self.agent_record_list = []
        self.demand_record_list = []
        self.number_of_agents = 0
        self.agent_list = []

    def initialization(self, net, args_):
        print('  initializing demand')
        self.number_of_seconds_per_interval = args_['sim_step']
        self.Simulation_StartTimeInMin = args_['sim_start_time']
        self.generateAgents(net)


    def generateAgents(self, net):
        if self.agent_record_list:
            self._generateAgentsFromAgent(net)
        elif self.demand_record_list:
            self._generateAgentsFromDemand(net)

    def _generateAgentsFromAgent(self, net):
        for agent_record in self.agent_record_list:
            if agent_record['route']:
                is_fixed_route = True
                fixed_meso_path_node_id_list = list(map(int, agent_record['route'].split(';')))
            else:
                is_fixed_route = False

            if agent_record['o_zone_id'] and agent_record['d_zone_id']:
                o_zone_id, d_zone_id = int(agent_record['o_zone_id']), int(agent_record['d_zone_id'])
                if is_fixed_route:
                    from_origin_node_id_list, to_destination_node_id_list = [fixed_meso_path_node_id_list[0]], [fixed_meso_path_node_id_list[-1]]
                else:
                    from_origin_node_id_list = [node.node_id for node in net.zone_dict[o_zone_id].outgoing_meso_node_list]
                    to_destination_node_id_list = [node.node_id for node in net.zone_dict[d_zone_id].incoming_meso_node_list]
            elif agent_record['o_node_id'] and agent_record['d_node_id']:
                o_zone_id, d_zone_id = None, None
                if is_fixed_route:
                    from_origin_node_id_list, to_destination_node_id_list = [fixed_meso_path_node_id_list[0]], [fixed_meso_path_node_id_list[-1]]
                else:
                    from_origin_node_id_list = [int(agent_record['o_node_id'])]
                    to_destination_node_id_list = [int(agent_record['d_node_id'])]
            else:
                continue

            if is_fixed_route:
                meso_path_node_id_list = fixed_meso_path_node_id_list
                meso_path_node_seq_no_list = list(map(lambda x: net.meso_node_id_to_seq_no_dict[x], meso_path_node_id_list))
                meso_path_link_seq_no_list = [net.meso_link_key_to_seq_no_dict[meso_path_node_id_list[i - 1] * 100000 + meso_path_node_id_list[i]] for i in range(1, len(meso_path_node_id_list))]
                meso_path_link_list = list(map(lambda x: net.meso_link_list[x], meso_path_link_seq_no_list))

            from_origin_node_id = random.choice(from_origin_node_id_list)
            to_destination_node_id = random.choice(to_destination_node_id_list)
            start_node = net.meso_node_list[net.meso_node_id_to_seq_no_dict[from_origin_node_id]]
            start_link = net.meso_link_list[net.meso_link_id_to_seq_no_dict[start_node.m_outgoing_link_list[0]]]

            agent = Agent()
            agent.agent_id = agent_record['agent_id']
            agent.agent_seq_no = self.number_of_agents
            agent.origin_zone_id, agent.destination_zone_id = o_zone_id, d_zone_id
            agent.meso_origin_node_id = from_origin_node_id
            agent.meso_destination_node_id = to_destination_node_id
            agent.departure_time_in_min = agent_record['departure_time']
            agent.meso_origin_node_seq_no = net.meso_node_id_to_seq_no_dict[from_origin_node_id]
            agent.meso_destination_node_seq_no = net.meso_node_id_to_seq_no_dict[to_destination_node_id]

            if is_fixed_route:
                agent.is_fixed_route = True
                agent.meso_path_node_id_list = meso_path_node_id_list.copy()
                agent.meso_path_node_seq_no_list = meso_path_node_seq_no_list.copy()
                agent.meso_path_link_seq_no_list = meso_path_link_seq_no_list.copy()
                agent.meso_path_link_list = meso_path_link_list

            seq_no = random.randint(0, len(start_link.micro_incoming_node_id_list) - 1)
            agent.micro_origin_node_id = start_link.micro_incoming_node_id_list[seq_no]
            agent.micro_origin_node_seq_no = net.micro_node_id_to_seq_no_dict[agent.micro_origin_node_id]

            agent.departure_time_in_simu_interval = round((agent.departure_time_in_min - self.Simulation_StartTimeInMin) * 60 / self.number_of_seconds_per_interval)
            agent.number_of_seconds_per_interval = self.number_of_seconds_per_interval
            agent.Simulation_StartTimeInMin = self.Simulation_StartTimeInMin

            self.agent_list.append(agent)
            self.number_of_agents += 1

            if is_fixed_route:
                from_node_id = fixed_meso_path_node_id_list[0]
                for to_node_id in fixed_meso_path_node_id_list[1:]:
                    link_no = net.meso_link_key_to_seq_no_dict[from_node_id * 100000 + to_node_id]
                    net.meso_link_list[link_no].base_volume += 1
                    from_node_id = to_node_id


    def _generateAgentsFromDemand(self, net):
        for demand_record in self.demand_record_list:
            if demand_record['route']:
                is_fixed_route = True
                fixed_meso_path_node_id_list = list(map(int, demand_record['route'].split(';')))
            else:
                is_fixed_route = False

            if demand_record['o_zone_id'] and demand_record['d_zone_id']:
                o_zone_id, d_zone_id = int(demand_record['o_zone_id']), int(demand_record['d_zone_id'])
                if is_fixed_route:
                    from_origin_node_id_list, to_destination_node_id_list = [fixed_meso_path_node_id_list[0]], [fixed_meso_path_node_id_list[-1]]
                else:
                    from_origin_node_id_list = [node.node_id for node in net.zone_dict[o_zone_id].outgoing_meso_node_list]
                    to_destination_node_id_list = [node.node_id for node in net.zone_dict[d_zone_id].incoming_meso_node_list]
            elif demand_record['o_node_id'] and demand_record['d_node_id']:
                o_zone_id, d_zone_id = None, None
                if is_fixed_route:
                    from_origin_node_id_list, to_destination_node_id_list = [fixed_meso_path_node_id_list[0]], [fixed_meso_path_node_id_list[-1]]
                else:
                    from_origin_node_id_list = [int(demand_record['o_node_id'])]
                    to_destination_node_id_list = [int(demand_record['d_node_id'])]
            else:
                continue

            if is_fixed_route:
                meso_path_node_id_list = fixed_meso_path_node_id_list
                meso_path_node_seq_no_list = list(map(lambda x: net.meso_node_id_to_seq_no_dict[x], meso_path_node_id_list))
                meso_path_link_seq_no_list = [net.meso_link_key_to_seq_no_dict[meso_path_node_id_list[i-1]*100000 + meso_path_node_id_list[i]] for i in range(1,len(meso_path_node_id_list))]
                meso_path_link_list = list(map(lambda x: net.meso_link_list[x], meso_path_link_seq_no_list))

            number_of_trips = demand_record['volume']
            time_start = demand_record['time_start']
            time_end = demand_record['time_end']

            for _ in range(number_of_trips):
                from_origin_node_id = random.choice(from_origin_node_id_list)
                to_destination_node_id = random.choice(to_destination_node_id_list)
                start_node = net.meso_node_list[net.meso_node_id_to_seq_no_dict[from_origin_node_id]]
                start_link = net.meso_link_list[net.meso_link_id_to_seq_no_dict[start_node.m_outgoing_link_list[0]]]

                agent = Agent()
                agent.agent_id = self.number_of_agents
                agent.agent_seq_no = self.number_of_agents
                agent.origin_zone_id, agent.destination_zone_id = o_zone_id, d_zone_id
                agent.meso_origin_node_id = from_origin_node_id
                agent.meso_destination_node_id = to_destination_node_id
                agent.departure_time_in_min = random.random() * (time_end - time_start) + time_start
                agent.meso_origin_node_seq_no = net.meso_node_id_to_seq_no_dict[from_origin_node_id]
                agent.meso_destination_node_seq_no = net.meso_node_id_to_seq_no_dict[to_destination_node_id]

                if is_fixed_route:
                    agent.is_fixed_route = True
                    agent.meso_path_node_id_list = meso_path_node_id_list.copy()
                    agent.meso_path_node_seq_no_list = meso_path_node_seq_no_list.copy()
                    agent.meso_path_link_seq_no_list = meso_path_link_seq_no_list.copy()
                    agent.meso_path_link_list = meso_path_link_list

                seq_no = random.randint(0,len(start_link.micro_incoming_node_id_list)-1)
                agent.micro_origin_node_id = start_link.micro_incoming_node_id_list[seq_no]
                agent.micro_origin_node_seq_no = net.micro_node_id_to_seq_no_dict[agent.micro_origin_node_id]

                agent.departure_time_in_simu_interval = round((agent.departure_time_in_min - self.Simulation_StartTimeInMin) * 60 / self.number_of_seconds_per_interval)
                agent.number_of_seconds_per_interval = self.number_of_seconds_per_interval
                agent.Simulation_StartTimeInMin = self.Simulation_StartTimeInMin

                self.agent_list.append(agent)
                self.number_of_agents += 1

            if is_fixed_route:
                from_node_id = fixed_meso_path_node_id_list[0]
                for to_node_id in fixed_meso_path_node_id_list[1:]:
                    link_no = net.meso_link_key_to_seq_no_dict[from_node_id * 100000 + to_node_id]
                    net.meso_link_list[link_no].base_volume += number_of_trips
                    from_node_id = to_node_id

