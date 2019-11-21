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

        print('Waiting for parameters to be downloaded...')
        swarm.parallel(helpers.wait_for_param_download)

        print(f"Connected crazyflies: {swarm._cfs}")
        print("###################################")

        # premium validity checking
        while True:
            inp = input()

            args_dict = {
                'radio://0/80/2M/E7E7E7E7E4': ['cmd.cmd', inp],
                'radio://0/80/2M/E7E7E7E7E9': ['cmd.cmd', inp]
            }

            print(args_dict)

            try:
                swarm.parallel_safe(helpers.set_param, args_dict=args_dict)
            except:
                print("Exeption occured")
        else:
            print("Program exited due to some unexpected reason")