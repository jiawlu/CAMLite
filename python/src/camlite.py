# @author       Jiawei Lu (jiaweil9@asu.edu)
#               Xuesong Zhou (xzhou74@asu.edu)
# @time         2020/12/21 14:15
# @desc         [script description]

from io_ import *
from simulation import *
from assignment import trafficAssignment


def initializeEvn(net_, demands_, signal_manager_, args_):
    print('Initializing environment...')
    net_.initialization(args_)
    demands_.initialization(net_,args_)
    signal_manager_.initialization(net_, args_)


if __name__ == '__main__':
    args = load_args()

    net, demands, signal_manager = readInputData()
    initializeEvn(net, demands, signal_manager, args)
    trafficAssignment(net,demands,args)
    simulator = Simulation(net, demands, signal_manager, args)
    simulator.startSim()

    outputResults(demands)

