import sys
import time

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
    dic = {
        1: "benis",
        2: "lul",
        3: "ayy",
    }
    print(type(dic))
    print(dic)
    print(dic.keys())
    print(dic.values())
