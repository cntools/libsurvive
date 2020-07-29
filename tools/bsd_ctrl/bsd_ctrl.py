#! /usr/bin/python3

from bluepy import btle
import sys
import os
import binascii
import argparse
from collections import defaultdict
import time

def get_service(dev, svc):
    for s in dev.services:
        if s.uuid == svc:
            return s
    return None

def get_characteristic(s, ch):
    chars = s.getCharacteristics()
    for i, c in enumerate(chars):
        props = c.propertiesToString()        
        if c.uuid == ch:
            return c
    return None

def get_characteristic_val(s, ch):
    c = get_characteristic(s, ch)
    if c is None:
        return None
    return int.from_bytes(c.read(), byteorder='big', signed=False)

SERVICE_UUID="00001523-1212-efde-1523-785feabcd124"
MODE_UUID="00001524-1212-EFDE-1523-785FEABCD124"
IDENTIFY_UUID="00008421-1212-EFDE-1523-785FEABCD124"
POWER_UUID="00001525-1212-EFDE-1523-785FEABCD124"

def get_scan_data(d, find_desc):
    for (sdid, desc, val) in d.getScanData():
        if desc == find_desc:
            return val
    return None
            
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--hci', action='store', type=int, default=0,
                        help='Interface number for scan')
    parser.add_argument('-t', '--timeout', action='store', type=int, default=4,
                        help='Scan delay, 0 for continuous')
    parser.add_argument('-m', '--mode', action='store', type=int, default=0,
                        help='Set mode 1-16')
    parser.add_argument('-s', '--sleep', action='store_true',
                        help='Sleep found lighthouses')
    parser.add_argument('-w', '--wake', action='store_true',
                        help='Wake up lighthouses')
    parser.add_argument('-b', '--blink', action='store_true',
                        help='Blink lighthouses')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Verbose output')
    parser.add_argument('-d', '--devices', action='append')

    arg = parser.parse_args(sys.argv[1:])

    scanner = btle.Scanner(arg.hci)
    devices = scanner.scan(arg.timeout)

    mode_lists = defaultdict(list)

    if arg.devices is None and arg.mode != 0:
        print("WARNING: Setting identical mode on multiple devices")

    devs = []
    free_modes = list(range(1,17))
    for d in devices:
        if arg.devices is not None and d.addr not in arg.devices:
            continue

        name = get_scan_data(d, "Complete Local Name")
        if arg.verbose:
            print("Found ", name)
        mfg = get_scan_data(d, "Manufacturer")
        if mfg is not None and mfg.startswith("5d05"):
            dev = btle.Peripheral(d)
            s = get_service(dev, SERVICE_UUID)
            if s is None:
                continue

            if arg.sleep:
                ch = get_characteristic(s, POWER_UUID)
                ch.write(bytes([1]))
                ch.write(bytes([0]))
            elif arg.wake:
                ch = get_characteristic(s, POWER_UUID)
                ch.write(bytes([1]))

            if arg.blink:
                ch = get_characteristic(s, IDENTIFY_UUID)
                ch.write(bytes([1]))

            mode = get_characteristic_val(s, MODE_UUID)
            power = get_characteristic_val(s, POWER_UUID)

            if arg.mode:
                ch = get_characteristic(s, MODE_UUID)
                ch.write(bytes([arg.mode]))

            if mode in free_modes:
                free_modes.remove(mode)

            mode_lists[mode].append(dev)
            print(dev.addr, name,
                  "MODE {0}\tPOWER SETTING {1}".format(mode, power))

            devs.append(dev)

    if not arg.mode:
        for i in range(1, 17):
            if len(mode_lists[i]) > 1:
                for dev in mode_lists[i][1:]:
                    s = get_service(dev, SERVICE_UUID)
                    ch = get_characteristic(s, MODE_UUID)
                    if ch is None:
                        continue

                    new_mode = free_modes[0]
                    free_modes.remove(new_mode)
                    print("Assigning mode {1} to {0}".format(dev.addr, new_mode))
                    ch.write(bytes([new_mode]))

    if arg.blink:
        time.sleep(1)

    for dev in devs:
        if arg.blink:
            s = get_service(dev, SERVICE_UUID)
            ch = get_characteristic(s, IDENTIFY_UUID)
            ch.write(bytes([0]))

        dev.disconnect()
    
    return 0

if __name__ == "__main__":
    main()
