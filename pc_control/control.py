import sys
import time

import helperFunctions as func

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm


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

        ctr = available_drones.index(droneId)

        args_dict[uri] = [amount, droneId, ctr]
    swarm.parallel_safe(init_drone, args_dict=args_dict)


def init_drone(scf, amount, droneId, ctr):
    time.sleep(0.2)
    func.set_param(scf, 'drone.amount', amount)
    time.sleep(0.2)
    func.set_param(scf, 'drone.id', droneId)
    # time.sleep(0.2)
    # func.set_param(scf, 'drone.prevId', prevId)
    # time.sleep(0.2)
    # func.set_param(scf, 'drone.nextId', nextId)
    # time.sleep(0.2)
    # func.set_param(scf, 'drone.ctr', ctr)
    time.sleep(0.2)
    func.set_param(scf, 'drone.cmd', 100)
    time.sleep(0.2)
    func.get_param(scf, 'dbg.chr')
    # print(f"Initialized drone nr {droneId}, prevId: {prevId}, nextId: {nextId}")
    print(f"Initialized drone nr {droneId}")


def comm(scf):
    func.set_param(scf, 'drone.cmd', 3)


def idle(scf):
    func.set_param(scf, 'drone.cmd', 4)


def trigger(scf):
    func.set_param(scf, 'drone.cmd', 5)


def debug(scf):
    func.set_param(scf, 'drone.cmd', 10)


uris = {
    'radio://0/80/2M/E7E7E7E7E4',
    'radio://0/80/2M/E7E7E7E7E9',
}


if __name__ == '__main__':
    print("###################################")
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(uris, factory=factory) as swarm:
        # swarm.parallel(func.reset_estimator)
        print('Waiting for parameters to be downloaded...')
        swarm.parallel(func.wait_for_param_download)
        init_swarm(swarm)
        print("###################################")

        while True:
            inp = input()
            args = inp.split()

            if args[0] == "quit":
                swarm.close_links()
                time.sleep(0.2)
                print("Program quit")
                sys.exit(0)

            # the scf object of the crazyflie that should execute some action, or "all"
            crazyflie = None
            if args[0] == "all":
                crazyflie = "all"
            else:
                for uri in swarm._cfs.keys():
                    if uri[-1] is args[0]:
                        crazyflie = swarm._cfs[uri]
            if crazyflie is None:
                print(f"Crazyflie number {args[0]} is not connected. Use a valid number (0-9) or \"all\" to address all crazyflies.")
                continue

            # the action that the crazyflie(s) should execute
            action = ""
            try:
                action = args[1]
            except:
                print("Arguments missing")
                continue

            # the args_dict contains all remaining arguments, keyed with the appropriate uri(s)
            args_dict = {}
            if crazyflie == "all":
                for uri in swarm._cfs.keys():
                    args_dict[uri] = args[2:]

            # Execution of commands
            try:
                if action == "set" and len(args) == 4:  # usage: [crazyflie] set [group.name] [value]
                    if crazyflie == "all":
                        swarm.parallel_safe(func.set_param, args_dict=args_dict)
                    else:
                        func.set_param(crazyflie, args[2], args[3])

                elif action == "get" and len(args) == 3:  # usage: [crazyflie] get [group.name]
                    if crazyflie == "all":
                        swarm.parallel_safe(func.get_param, args_dict=args_dict)
                    else:
                        func.get_param(crazyflie, args[2])

                elif action == "comm" and len(args) == 2:  # usage: [crazyflie] comm
                    if crazyflie == "all":
                        swarm.parallel_safe(comm, args_dict=args_dict)
                    else:
                        comm(crazyflie)

                elif action == "idle" and len(args) == 2:  # usage: [crazyflie] idle
                    if crazyflie == "all":
                        swarm.parallel_safe(idle, args_dict=args_dict)
                    else:
                        idle(crazyflie)

                elif action == "trigger" and len(args) == 2:  # usage: [crazyflie] trigger
                    if crazyflie == "all":
                        swarm.parallel_safe(trigger, args_dict=args_dict)
                    else:
                        trigger(crazyflie)

                elif action == "debug" and len(args) == 2:  # usage: [crazyflie] debug
                    if crazyflie == "all":
                        swarm.parallel_safe(debug, args_dict=args_dict)
                    else:
                        debug(crazyflie)

                else:
                    print("Invalid command")
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
