import time

# from cflib.drivers.crazyradio import Crazyradio
import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


def start(scf):
    start_myNumber_printing(scf)
    print("start")


def set_command_neg_one(scf):
    cf = scf.cf
    cf.param.set_value('command.command', '-1')


def set_command_one(scf):
    # cr.send_packet((0xff, 0x80, 0x63, 0x01))
    cf = scf.cf
    cf.param.set_value('command.command', '1')


def myNumber_callback(timestamp, data, logconf):
    num = data['test.myNumber']
    print('myNumber = {}'.format(num))


def start_myNumber_printing(scf):
    log_conf = LogConfig(name='myNumber', period_in_ms=75)
    log_conf.add_variable('test.myNumber', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(myNumber_callback)
    log_conf.start()
    time.sleep(0.1)
    log_conf.stop()


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(cf)


def wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print('Parameters downloaded for', scf.cf.link_uri)


uris = {
    'radio://0/80/2M/E7E7E7E7E4'
    #,'radio://0/80/2M/E7E7E7E7E9'
}


if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    uri = 'radio://0/80/2M/E7E7E7E7E4'
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # reset_estimator(scf)
        print('Waiting for parameters to be downloaded...')
        wait_for_param_download(scf)

        while True:
            inp = input()
            if inp == "start":
                start(scf)
            elif inp == "1":
                set_command_one(scf)
            elif inp == "-1":
                set_command_neg_one(scf)
            elif inp == "quit":
                break
            else:
                print("invalid command")

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
    #     swarm.parallel(wait_for_param_download)
    #     print(swarm._cfs)

    #     while True:
    #         inp = input()
    #         if inp == "start":
    #             swarm.parallel(start)
    #         elif inp == "land":
    #             swarm.parallel(land)
    #         elif inp == "send":
    #             swarm.parallel(send)
    #         elif inp == "quit":
    #             break
    #         else:
    #             print("invalid command")


# cr = Crazyradio(devid=0)  # maybe 1, idk
# cr.set_channel(80)
# cr.set_data_rate(cr.DR_2MPS)
# cr.set_address((0xe7, 0xe7, 0xe7, 0xe7, 0xe4))
# cr.set_ack_enable(True)


