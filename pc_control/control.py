from cflib.drivers.crazyradio import Crazyradio
import time


def start():
    print("start")


def land():
    print("land")


def send():
    cr.send_packet((0xff, 0x80, 0x63, 0x01))
    print('send')



cr = Crazyradio(devid=0)  # maybe 1, idk
cr.set_channel(80)
cr.set_data_rate(cr.DR_2MPS)
cr.set_address((0xe7, 0xe7, 0xe7, 0xe7, 0xe4))
cr.set_ack_enable(True)

while True:
    inp = input()
    if inp == "start":
        start()
    elif inp == "land":
        land()
    elif inp == "send":
        send()
    elif inp == "quit":
        break
    else:
        print("invalid command")
