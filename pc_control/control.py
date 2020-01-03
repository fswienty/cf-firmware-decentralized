import sys
import time
import os
import numpy as np

import helperFunctions as helpFun

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
# from cflib.crazyflie.console import Console


def init_swarm(swarm):
    available_drones = []
    for uri in swarm._cfs.keys():
        available_drones.append(int(uri[-1]))
    available_drones.sort()
    print(f"Available drones: {available_drones}")

    args_dict = {}
    for uri in swarm._cfs.keys():
        amount = len(uris)
        droneId = int(uri[-1])

        # prevIdIndex = available_drones.index(droneId)
        # if prevIdIndex == 0:
        #     prevIdIndex = len(available_drones) - 1
        # else:
        #     prevIdIndex -= 1
        # prevId = available_drones[prevIdIndex]

        # nextIdIndex = available_drones.index(droneId)
        # if nextIdIndex == len(available_drones) - 1:
        #     nextIdIndex = 0
        # else:
        #     nextIdIndex += 1
        # nextId = available_drones[nextIdIndex]

        # ctr = available_drones.index(droneId)

        args_dict[uri] = [amount, droneId]
    swarm.parallel_safe(init_drone, args_dict=args_dict)


def init_drone(scf, amount, droneId):
    helpFun.set_param(scf, 'drone.amount', amount)
    time.sleep(0.2)
    helpFun.set_param(scf, 'drone.id', droneId)
    # time.sleep(0.2)
    # helpFun.set_param(scf, 'drone.prevId', prevId)
    # time.sleep(0.2)
    # helpFun.set_param(scf, 'drone.nextId', nextId)
    # time.sleep(0.2)
    # helpFun.set_param(scf, 'drone.ctr', ctr)
    time.sleep(0.2)
    helpFun.set_param(scf, 'drone.cmd', 100)
    time.sleep(0.2)
    helpFun.get_param(scf, 'dbg.chr')
    # print(f"Initialized drone nr {droneId}, prevId: {prevId}, nextId: {nextId}")
    print(f"Initialized drone nr {droneId}")


def set_formation(scf, formation):
    helpFun.set_param(scf, 'drone.targetX', x)
    time.sleep(0.2)
    helpFun.set_param(scf, 'drone.targetY', y)
    time.sleep(0.2)
    helpFun.set_param(scf, 'drone.targetZ', z)


def set_target(scf, x, y, z):
    helpFun.set_param(scf, 'drone.targetX', x)
    time.sleep(0.2)
    helpFun.set_param(scf, 'drone.targetY', y)
    time.sleep(0.2)
    helpFun.set_param(scf, 'drone.targetZ', z)


def start(scf):
    helpFun.set_param(scf, 'drone.cmd', 1)


def land(scf):
    helpFun.set_param(scf, 'drone.cmd', 2)


def comm(scf):
    helpFun.set_param(scf, 'drone.cmd', 3)


def idle(scf):
    helpFun.set_param(scf, 'drone.cmd', 4)


def trigger(scf):
    helpFun.set_param(scf, 'drone.cmd', 5)


def debug(scf):
    helpFun.set_param(scf, 'drone.cmd', 10)


uris = {
    'radio://0/80/2M/E7E7E7E7E4',
    'radio://0/80/2M/E7E7E7E7E9',
    # 'radio://0/80/2M/E7E7E7E7E1',
    # 'radio://0/80/2M/E7E7E7E7E0',
}


if __name__ == '__main__':
    print("###################################")
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(uris, factory=factory) as swarm:
        # swarm.parallel(helpFun.reset_estimator)
        print('Waiting for parameters to be downloaded...')
        swarm.parallel(helpFun.wait_for_param_download)
        init_swarm(swarm)
        print("###################################")

        while True:
            inp = input().split()

            if inp[0] == "quit":
                swarm.close_links()
                time.sleep(0.2)
                print("Program quit")
                sys.exit(0)
            elif inp[0] == "formation" and len(inp) == 2:  # usage: [crazyflie] formation [formation name] (please only use with [crazyflie] == "all")
                # TODO check formation size vs available drones

                formation_name = inp[1]
                # load formation
                path = os.path.dirname(os.path.abspath(__file__))
                path = os.path.join(path, "formations")
                path = os.path.join(path, f"{formation_name}.csv")
                try:
                    formation = np.loadtxt(path, delimiter=",")
                except:
                    print(f"Formation {formation_name} not found")
                    continue
                formation

                # construct args_dict
                args_dict = {}
                available_uris = list(swarm._cfs)
                print(available_uris)
                for i in range(0, len(available_uris)):
                    args_dict[available_uris[i]] = [formation[i][0], formation[i][1], formation[i][2]]
                print(args_dict)
                print(f"Setting formation {formation_name}")
                swarm.parallel_safe(set_target, args_dict=args_dict)
                continue

            # the scf object of the crazyflie that should execute some action, or "all"
            crazyflie = None
            if inp[0] == "all":
                crazyflie = "all"
            else:
                for uri in swarm._cfs.keys():
                    if uri[-1] is inp[0]:
                        crazyflie = swarm._cfs[uri]
            if crazyflie is None:
                print(f"Crazyflie number {inp[0]} is not connected. Use a valid number (0-9) or \"all\" to address all crazyflies.")
                continue

            # the action that the crazyflie(s) should execute
            action = ""
            try:
                action = inp[1]
            except:
                print("Arguments missing")
                continue

            # the args_dict contains all remaining arguments, keyed with the appropriate uri(s)
            # args is a list with all remaining arguments
            args_dict = {}
            args = inp[2:]
            if crazyflie == "all":
                for uri in swarm._cfs.keys():
                    args_dict[uri] = args

            # setting desired function
            function = None
            if action == "set" and len(args) == 2:  # usage: [crazyflie] set [group.name] [value]
                function = helpFun.set_param
            elif action == "get" and len(args) == 1:  # usage: [crazyflie] get [group.name]
                function = helpFun.get_param
            elif action == "start" and len(args) == 0:  # usage: [crazyflie] start
                function = start
            elif action == "land" and len(args) == 0:  # usage: [crazyflie] land
                function = land
            elif action == "comm" and len(args) == 0:  # usage: [crazyflie] comm
                function = comm
            elif action == "idle" and len(args) == 0:  # usage: [crazyflie] idle
                function = idle
            elif action == "trigger" and len(args) == 0:  # usage: [crazyflie] trigger
                function = trigger
            elif action == "debug" and len(args) == 0:  # usage: [crazyflie] debug
                function = debug
            elif action == "target" and len(args) == 3:  # usage: [crazyflie] target [x] [y] [z]
                function = set_target
            else:
                print("Invalid command")
                continue

            # execution of commands
            try:
                if crazyflie == "all":
                    swarm.parallel_safe(function, args_dict=args_dict)
                else:
                    function(crazyflie, *args)
            except:
                print("Exeption occured")
        else:
            print("While loop exited due to some unexpected reason")



# from cflib.drivers.crazyradio import Crazyradio

# cr = Crazyradio(devid=0)
# cr.set_channel(80)
# cr.set_data_rate(cr.DR_2MPS)

# cr.set_address((0xe7, 0xe7, 0xe7, 0xe7, 0xe4))
# cr.set_ack_enable(False)
# cr.send_packet((0xff, 0x80, 0x63, 0x01))
# print('send')
