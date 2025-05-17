#!/usr/bin/env python3

"""
Open a shell over MAVLink.

@author: Beat Kueng (beat-kueng@gmx.net)
"""


from __future__ import print_function
import sys, select
import termios
from timeit import default_timer as timer
from argparse import ArgumentParser
import os

try:
    from pymavlink import mavutil
except ImportError as e:
    print("Failed to import pymavlink: " + str(e))
    print("")
    print("You may need to install it with:")
    print("    pip3 install --user pymavlink")
    print("")
    sys.exit(1)

try:
    import serial
except ImportError as e:
    print("Failed to import pyserial: " + str(e))
    print("")
    print("You may need to install it with:")
    print("    pip3 install --user pyserial")
    print("")
    sys.exit(1)


class MavlinkSerialPort():
    '''an object that looks like a serial port, but
    transmits using mavlink SERIAL_CONTROL packets'''
    def __init__(self, portname, baudrate, devnum=0, debug=0):
        self.baudrate = 0
        self._debug = debug
        self.buf = ''
        self.port = devnum
        self.debug("Connecting with MAVLink to %s ..." % portname)
        self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)
        self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        self.mav.wait_heartbeat()
        self.debug("HEARTBEAT OK\n")
        self.debug("Locked serial device\n")

    def debug(self, s, level=1):
        '''write some debug text'''
        if self._debug >= level:
            print(s)

    def write(self, b):
        '''write some bytes'''
        self.debug("sending '%s' (0x%02x) of len %u\n" % (b, ord(b[0]), len(b)), 2)
        while len(b) > 0:
            n = len(b)
            if n > 70:
                n = 70
            buf = [ord(x) for x in b[:n]]
            buf.extend([0]*(70-len(buf)))
            self.mav.mav.serial_control_send(self.port,
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                                             0,
                                             0,
                                             n,
                                             buf)
            b = b[n:]

    def close(self):
        self.mav.mav.serial_control_send(self.port, 0, 0, 0, 0, [0]*70)

    def _recv(self):
        '''read some bytes into self.buf'''
        m = self.mav.recv_match(condition='SERIAL_CONTROL.count!=0',
                                type='SERIAL_CONTROL', blocking=True,
                                timeout=0.03)
        if m is not None:
            if self._debug > 2:
                print(m)
            data = m.data[:m.count]
            self.buf += ''.join(str(chr(x)) for x in data)

    def read(self, n):
        '''read some bytes'''
        if len(self.buf) == 0:
            self._recv()
        if len(self.buf) > 0:
            if n > len(self.buf):
                n = len(self.buf)
            ret = self.buf[:n]
            self.buf = self.buf[n:]
            if self._debug >= 2:
                for b in ret:
                    self.debug("read 0x%x" % ord(b), 2)
            return ret
        return ''


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('port', metavar='PORT', nargs='?', default = None,
            help='Mavlink port name: serial: DEVICE[,BAUD], udp: IP:PORT, tcp: tcp:IP:PORT. Eg: \
    /dev/ttyUSB0 or 0.0.0.0:14550. Auto-detect serial if not given.')
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int,
                      help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()


    if args.port == None:
        args.port = "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00"


    print("Connecting to MAVLINK...")
    mav_serialport = MavlinkSerialPort(args.port, args.baudrate, devnum=10)

    # Send the command
    command = "uxrce_dds_client start -t udp -h 192.168.1.125 -p 8888\n"
    mav_serialport.write(command)

    # Wait for a short time to receive the response
    import time
    time.sleep(2)  # Adjust as needed

    # Read and print any available output
    data = mav_serialport.read(4096)
    if data and len(data) > 0:
        sys.stdout.write(data)
        sys.stdout.flush()

    # Close the connection and exit
    mav_serialport.close()

if __name__ == '__main__':
    main()
