import os
import time
import numpy as np

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger


##### SIMPLE FUNCTIONS FOR CONVENIENCE #####
def start(scf):
    set_param(scf, 'drone.cmd', 1)


def land(scf):
    set_param(scf, 'drone.cmd', 2)


def debug1(scf):
    set_param(scf, 'drone.cmd', 3)


def debug2(scf):
    set_param(scf, 'drone.cmd', 4)


def off(scf):
    set_param(scf, 'drone.cmd', 5)


def reset_timer(scf):
    set_param(scf, 'drone.cmd', 6)


def info(scf):
    set_param(scf, 'drone.cmd', 10)


def set_target(scf, x, y, z):
    set_param(scf, 'drone.targetX', x)
    time.sleep(0.2)
    set_param(scf, 'drone.targetY', y)
    time.sleep(0.2)
    set_param(scf, 'drone.targetZ', z)


##### DRONE INITIALIZATION #####
def init_swarm(swarm, initData):
    available_drones = []
    for uri in list(swarm._cfs):
        available_drones.append(int(uri[-1]))
    available_drones.sort()
    print(f"Available drones: {available_drones}")

    args_dict = {}
    available_uris = list(swarm._cfs)
    for uri in available_uris:
        amount = len(available_uris)
        droneId = int(uri[-1])
        args_dict[uri] = [amount, droneId, initData]

    swarm.parallel_safe(init_drone, args_dict=args_dict)


def init_drone(scf, amount, droneId, initData):
    set_param(scf, 'drone.amount', amount)
    time.sleep(0.2)
    set_param(scf, 'drone.id', droneId)
    time.sleep(0.2)
    set_param(scf, 'drone.forceFalloff', initData['forceFalloff'])
    time.sleep(0.2)
    set_param(scf, 'drone.targetForce', initData['targetForce'])
    time.sleep(0.2)
    set_param(scf, 'drone.avoidRange', initData['avoidRange'])
    time.sleep(0.2)
    set_param(scf, 'drone.avoidForce', initData['avoidForce'])
    time.sleep(0.2)
    set_param(scf, 'drone.maxLength', initData['maxLength'])
    time.sleep(0.2)
    set_param(scf, 'drone.cmd', 100)


##### FORMATION SETTING #####
def set_formation(swarm, formation_name):
    # load formation
    path = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(path, "formations")
    path = os.path.join(path, f"{formation_name}.csv")
    # print(path)
    try:
        formation = np.loadtxt(path, delimiter=",")
    except:
        print(f"Formation {formation_name} not found.")
        return

    # construct args_dict
    available_uris = list(swarm._cfs)
    formation_size = np.size(formation, 0)
    available_drones = len(available_uris)
    max_iterator = formation_size  # Normally, formation_size and available_drones has the same value, so just use one of them here.
    if formation_size < available_drones:
        print(f"Formation {formation_name} requires {formation_size} drones, but there are {available_drones} available. Some drones will remain stationary.")
        max_iterator = formation_size
    elif formation_size > available_drones:
        print(f"Formation {formation_name} requires {formation_size} drones, but there are only {available_drones} available. Some places in the formation will not be occupied.")
        max_iterator = available_drones
    args_dict = {}
    for i in range(0, max_iterator):
        args_dict[available_uris[i]] = [formation[i][0], formation[i][1], formation[i][2]]
    # print(args_dict)

    # set formation
    print(f"Setting formation {formation_name}")
    swarm.parallel_safe(set_target, args_dict=args_dict)


##### PARAMETER GET AND SET METHODS #####
def set_param(scf, param_name, value):
    # print(f"param_name: {type(param_name)} {param_name} | value: {type(value)} {value}")
    cf = scf.cf
    cf.param.set_value(str(param_name), str(value))


def get_param(scf, param_name):
    getter = ParamGetter()
    getter.get_param(scf, str(param_name))


class ParamGetter:
    param_name = ""
    uri = ""

    def get_param(self, scf, param_name):
        self.param_name = param_name
        self.uri = scf._link_uri
        log_conf = LogConfig(name=self.param_name, period_in_ms=75)
        log_conf.add_variable(self.param_name)
        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.get_param_callback)
        log_conf.start()
        time.sleep(0.1)
        log_conf.stop()

    def get_param_callback(self, timestamp, data, logconf):
        # value = data[self.param_name]
        print(f"{self.uri}: {data}")


##### ESTIMATOR RESET AND STARTUP STUFF #####
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





### old code from init_swarm
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
