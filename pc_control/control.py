import sys
import time

import helperFunctions as func

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

from cflib.crazyflie.console import Console


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
        triggerIdIndex = available_drones.index(droneId)
        if triggerIdIndex == 0:
            triggerIdIndex = len(available_drones) - 1
        else:
            triggerIdIndex -= 1
        triggerId = available_drones[triggerIdIndex]

        args_dict[uri] = [amount, droneId, triggerId]
    swarm.parallel_safe(init_drone, args_dict=args_dict)


def init_drone(scf, amount, droneId, triggerId):
    time.sleep(0.2)
    func.set_param(scf, 'drone.amount', amount)
    time.sleep(0.2)
    func.set_param(scf, 'drone.id', droneId)
    time.sleep(0.2)
    func.set_param(scf, 'drone.triggerId', triggerId)
    time.sleep(0.2)
    func.set_param(scf, 'drone.cmd', 100)
    time.sleep(0.2)
    func.get_param(scf, 'dbg.chr')
    print(f"Initialized drone nr {droneId}, prev drone: {triggerId}")


uris = {
    'radio://0/80/2M/E7E7E7E7E4'
    ,'radio://0/80/2M/E7E7E7E7E9'
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
        con = Console(swarm._cfs[0])
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
