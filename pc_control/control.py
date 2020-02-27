import sys
import time
import os
import numpy as np

import helperFunctions as helpFun

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
# from cflib.crazyflie.console import Console


uris = [
    'radio://0/80/2M/E7E7E7E7E4',  # seem to be fine
    'radio://0/80/2M/E7E7E7E7E3',
    'radio://0/80/2M/E7E7E7E7E6',
    'radio://0/80/2M/E7E7E7E7E7',
    # 'radio://0/80/2M/E7E7E7E7E5',  # maybe fine
    # 'radio://0/80/2M/E7E7E7E7E9',
    # 'radio://0/80/2M/E7E7E7E7E1',  # bad drones from here
    # 'radio://0/80/2M/E7E7E7E7E2',
    # 'radio://0/80/2M/E7E7E7E7E0',
]

initData = {
    'mode': 0,
    'forceFalloff': 1.0,
    'targetForce': 0.3,
    'avoidRange': 0.9,
    'avoidForce': 0.8,
    'maxLength': 0.2,
    'accBudget': 1.0,
}


if __name__ == '__main__':
    print("###################################")
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(uris, factory=factory) as swarm:
        print('Resetting estimators...')
        swarm.parallel(helpFun.reset_estimator)
        print('Waiting for parameters to be downloaded...')
        swarm.parallel(helpFun.wait_for_param_download)
        print('Initializing...')
        helpFun.init_swarm(swarm, initData)
        print('Resetting timers...')
        swarm.parallel(helpFun.reset_timer)
        print("###################################")

        while True:
            inp = input().split()

            if inp == []:
                print("No input given.")
                continue

            if inp[0] == "quit":
                swarm.close_links()
                time.sleep(0.2)
                print("Program quit")
                sys.exit(0)
            elif inp[0] == "formation" and len(inp) == 2:  # usage: formation [formation name (without the ".csv")]
                helpFun.set_formation(swarm, inp[1])
                continue

            # the scf object of the crazyflie that should execute some action, or "all"
            crazyflie = None
            if inp[0] == "all":
                crazyflie = "all"
            else:
                for uri in list(swarm._cfs):
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

            # creates the args_dict containing all remaining arguments, keyed with the appropriate uri(s)
            args_dict = {}
            args = inp[2:]  # list with all remaining arguments
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
                function = helpFun.start
            elif action == "land" and len(args) == 0:  # usage: [crazyflie] land
                function = helpFun.land
            elif action == "debug1" and len(args) == 0:  # usage: [crazyflie] debug1
                function = helpFun.debug1
            elif action == "debug2" and len(args) == 0:  # usage: [crazyflie] debug2
                function = helpFun.debug2
            elif action == "off" and len(args) == 0:  # usage: [crazyflie] idle
                function = helpFun.off
            elif action == "reset" and len(args) == 0:  # usage: [crazyflie] reset
                function = helpFun.reset_timer
            elif action == "info" and len(args) == 0:  # usage: [crazyflie] debug
                function = helpFun.info
            elif action == "target" and len(args) == 3:  # usage: [crazyflie] target [x] [y] [z]
                function = helpFun.set_target
            else:
                print("Invalid command")
                continue

            # execution of commands
            try:
                if crazyflie == "all":  # execute the command for all crazyflies
                    swarm.parallel_safe(function, args_dict=args_dict)
                else:  # execute the command for a single crazyfly
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


# list(swarm._cfs): ['radio://0/80/2M/E7E7E7E7E4', 'radio://0/80/2M/E7E7E7E7E9']
# swarm._cfs.keys(): dict_keys(['radio://0/80/2M/E7E7E7E7E4', 'radio://0/80/2M/E7E7E7E7E9'])
# swarm._cfs: {'radio://0/80/2M/E7E7E7E7E9': <cflib.crazyflie.syncCrazyflie.SyncCrazyflie object at 0x000002C674794908>}
