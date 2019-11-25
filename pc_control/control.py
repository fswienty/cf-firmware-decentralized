import sys
import time

from helperFunctions import HelperFunctions

# from cflib.drivers.crazyradio import Crazyradio
import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


def send(scf, value):
    cf = scf.cf
    cf.param.set_value('p2p.send', value)
    time.sleep(0.5)
    cf.param.set_value('cmd.cmd', '100')


uris = {
    'radio://0/80/2M/E7E7E7E7E4',
    'radio://0/80/2M/E7E7E7E7E9'
}


if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    helpers = HelperFunctions()

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        # If the copters are started in their correct positions this is
        # probably not needed. The Kalman filter will have time to converge
        # any way since it takes a while to start them all up and connect. We
        # keep the code here to illustrate how to do it.
        # swarm.parallel(helpers.reset_estimator)

        # The current values of all parameters are downloaded as a part of the
        # connections sequence. Since we have 10 copters this is clogging up
        # communication and we have to wait for it to finish before we start
        # flying.
        print('Waiting for parameters to be downloaded...')
        swarm.parallel(helpers.wait_for_param_download)

        print(f"Connected crazyflies: {swarm._cfs}")
        print("###################################")

        # premium validity checking
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
                print(f"Crazyflie number {args[0]} is not connected. Use a valid number or \"all\" to address all crazyflies.")
                continue

            # the action that the crazyflie(s) should execute
            action = args[1]

            # the args_dict contains all remaining arguments, keyed with the appropriate uri(s)
            args_dict = {}
            if crazyflie == "all":
                for uri in swarm._cfs.keys():
                    args_dict[uri] = args[2:]

            # print(f"args_dict: {args_dict}")
            # print(f"swarm._cfs: {swarm._cfs}")
            
            try:
                if action == "send" and len(args) == 3: # usage: [crazyflie] send [value]
                    if crazyflie == "all":
                        swarm.parallel_safe(send, args_dict=args_dict)
                    else:
                        send(crazyflie, args[2])

                elif action == "set" and len(args) == 4: # usage: [crazyflie] set [group.name] [value]
                    if crazyflie == "all":
                        swarm.parallel_safe(helpers.set_param, args_dict=args_dict)
                    else:
                        helpers.set_param(crazyflie, args[2], args[3])
                        
                elif action == "get" and len(args) == 3: # usage: [crazyflie] get [group.name]
                    if crazyflie == "all":
                        swarm.parallel_safe(helpers.get_param, args_dict=args_dict)
                    else:
                        helpers.get_param(crazyflie, args[2])

                else:
                    print("Invalid command")
            except:
                print("Exeption occured")
        else:
            print("While loop was exited due to some unexpected reason")





# cr = Crazyradio(devid=0)  # maybe 1, idk
# cr.set_channel(80)
# cr.set_data_rate(cr.DR_2MPS)
# cr.set_address((0xe7, 0xe7, 0xe7, 0xe7, 0xe4))
# cr.set_ack_enable(True)


