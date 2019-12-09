import sys
import time

import helperFunctions as func

# from cflib.drivers.crazyradio import Crazyradio
import cflib.crtp
# from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
# from cflib.crazyflie.syncLogger import SyncLogger
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


def init_drone(scf, amount):
    droneID = (scf.cf.link_uri[-1])
    func.set_param(scf, 'drone.id', droneID)
    time.sleep(0.2)
    func.set_param(scf, 'drone.amount', amount)
    time.sleep(0.2)
    func.set_param(scf, 'drone.cmd', 42)
    time.sleep(0.2)
    func.get_param(scf, 'dbg.chr')
    print(f"Initialized drone nr {droneID}")


uris = {
    'radio://0/80/2M/E7E7E7E7E4'
    #,'radio://0/80/2M/E7E7E7E7E9'
}


if __name__ == '__main__':
    print("###################################")
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel(func.reset_estimator)

        print('Waiting for parameters to be downloaded...')
        swarm.parallel(func.wait_for_param_download)
        # print(f"Connected crazyflies: {swarm._cfs}")
        args_dict = {}
        for uri in swarm._cfs.keys():
            args_dict[uri] = [len(uris)]
        swarm.parallel_safe(init_drone, args_dict=args_dict)
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
