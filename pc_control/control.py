from cflib.drivers.crazyradio import Crazyradio
import time

cr = Crazyradio(devid=0)  # maybe 1, idk


def start():
    print("start")


def land():
    print("land")



while True:
    inp = input()
    if inp == "start":
        start()
    elif inp == "land":
        land()
    elif inp == "quit":
        break
    else:
        print("invalid command")
