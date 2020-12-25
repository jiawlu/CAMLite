# @author       Jiawei Lu (jiaweil9@asu.edu)
#               Xuesong Zhou (xzhou74@asu.edu)
# @time         2020/12/21 13:53
# @desc         [script description]

class SignalManager:
    def __init__(self):
        self.number_of_seconds_per_interval = 0
        self.controller_data_dict = {}          # raw data
        self.controller_dict = {}               # processed     controller_id: [phase...]
        self.controller_state_dict = {}         # state info    controller_id: [phase...]      current_phase_no, time_last
        self.microlink_state_default = {}       # all False

    def initialization(self, net, args):
        print('  initializing signal')
        self.number_of_seconds_per_interval = args['sim_step']
        control_microlinks_all = []

        for controller_id, signal_plan in self.controller_data_dict.items():
            micro_signal_plan = []
            for phase_no, phase in enumerate(signal_plan):
                phase_id = phase['phase_id']
                movements = phase['movements']
                duration = phase['duration']

                movement_id_list = movements.split(';')
                micro_links_list = []
                for movement_id in movement_id_list:
                    try:
                        movement_link = net.meso_link_list[net.meso_link_id_to_seq_no_dict[movement_id]]
                    except KeyError:
                        print(f'    warning: movement {movement_id} in controller {controller_id} phase {phase_id} does not exist')
                        continue
                    micro_links_list += movement_link.control_micro_link_list
                control_microlinks_all += micro_links_list

                next_phase_no = phase_no + 1 if phase_no != len(signal_plan)-1 else 0
                phase_micro = {'phase_id': phase_id, 'microlinks': micro_links_list, 'duration': duration, 'next_phase_no': next_phase_no}
                micro_signal_plan.append(phase_micro)

            self.controller_dict[controller_id] = micro_signal_plan

        for microlink in control_microlinks_all: self.microlink_state_default[microlink] = False
        for controller_id in self.controller_dict.keys():
            self.controller_state_dict[controller_id] = {'current_phase_no':0, 'last':0}


    def getStatus(self):
        microlink_states = self.microlink_state_default.copy()

        for controller_id, controller_state in self.controller_state_dict.items():
            current_phase_info = self.controller_dict[controller_id][controller_state['current_phase_no']]
            last = controller_state['last']

            if current_phase_info['duration'] - last < 0.5 * self.number_of_seconds_per_interval:
                # switch to the next phase
                controller_state['current_phase_no'] = current_phase_info['next_phase_no']
                current_phase_info = self.controller_dict[controller_id][controller_state['current_phase_no']]
                controller_state['last'] = self.number_of_seconds_per_interval
            else:
                # keep the current phase
                controller_state['last'] += self.number_of_seconds_per_interval

            for microlink in current_phase_info['microlinks']:
                microlink_states[microlink] = True

        inactive_microlinks = {microlink for microlink, state in microlink_states.items() if not state}
        return inactive_microlinks
