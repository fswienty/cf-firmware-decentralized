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


def send_packet(scf):
    cf = scf.cf
    cf.param.set_value('command.command', '2')
    print("start")


uris = {
    'radio://0/80/2M/E7E7E7E7E4'
    #,'radio://0/80/2M/E7E7E7E7E9'
}


if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    helpers = HelperFunctions()
    uri = 'radio://0/80/2M/E7E7E7E7E4'
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # reset_estimator(scf)
        print('Waiting for parameters to be downloaded...')
        #wait_for_param_download(scf)s
        helpers.wait_for_param_download(scf)

        # premium validity checking
        while True:
            inp = input()
            inp = inp.split()
            try:
                if inp[0] == "send":
                    send_packet(scf)
                elif inp[0] == "quit":
                    break
                elif inp[0] == "set" and len(inp) == 3: # usage: set [group.name] [value]
                    helpers.set_param(scf, inp[1], inp[2])
                elif inp[0] == "get" and len(inp) == 2: # usage: set [group.name]
                    value = helpers.get_param(scf, inp[1])
                    print(value)
                else:
                    print("invalid command")
            except:
                print("exeption occured")


    # factory = CachedCfFactory(rw_cache='./cache')
    # with Swarm(uris, factory=factory) as swarm:
    #     # If the copters are started in their correct positions this is
    #     # probably not needed. The Kalman filter will have time to converge
    #     # any way since it takes a while to start them all up and connect. We
    #     # keep the code here to illustrate how to do it.
    #     # swarm.parallel(reset_estimator)

    #     # The current values of all parameters are downloaded as a part of the
    #     # connections sequence. Since we have 10 copters this is clogging up
    #     # communication and we have to wait for it to finish before we start
    #     # flying.
    #     print('Waiting for parameters to be downloaded...')
    #     swarm.parallel(helpers.wait_for_param_download)
    #     print(swarm._cfs)



# cr = Crazyradio(devid=0)  # maybe 1, idk
# cr.set_channel(80)
# cr.set_data_rate(cr.DR_2MPS)
# cr.set_address((0xe7, 0xe7, 0xe7, 0xe7, 0xe4))
# cr.set_ack_enable(True)


