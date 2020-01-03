import sys
import time
import numpy as np
import os

import helperFunctions as func

# from cflib.drivers.crazyradio import Crazyradio
import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


def testFun(arg1):
    print(f"{arg1}")


if __name__ == '__main__':
    print("#################")
    name = "2_circle"
    path = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(path, "formations")
    path = os.path.join(path, f"{name}.csv")
    print(path)
    formation = np.loadtxt(path, delimiter=",")
    print(formation[1])
    # formation = np.loadtxt(path, delimiter=",")
