#!/usr/bin/env python3
###### TeraRanger Evo Example Code STD #######
#                                            #
# All rights reserved Terabee France (c) 2018#
#                                            #
############ www.terabee.com #################

import sys

import crcmod.predefined  # To install: pip install crcmod
import serial
import serial.tools.list_ports


def find_arduino_port():
    # Find Live Ports, return port name if found, NULL if not
    print("Scanning all live ports on this PC")
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # print p # This causes each port's information to be printed out.
        if "AQ015E9Y" in p[2]:
            print("Arduino found on port " + p[0])
            return p[0]
    return "NULL"


class ContactArduino:
    def __init__(self):
        self.port = find_arduino_port()
        self.serial_port = None
        self.left_offset = 0
        self.right_offset = 0
        self.left_threshold = 40000
        self.right_threshold = -270000
        if self.port == "NULL":
            print("No Arduino was found")
        else:
            self.open_contact_arduino(self.port)

    def open_contact_arduino(self, port_name):
        print("Attempting to open port...")
        # Open the Evo and catch any exceptions thrown by the OS
        print(port_name)
        self.serial_port = serial.Serial(port_name, baudrate=115200, timeout=2)

        print("Serial port opened")

    def get_contact_data(self):
        # crc8_fn = crcmod.predefined.mkPredefinedCrcFun("crc-8")
        # Read one byte
        data = self.serial_port.readline()
        data = data.decode("utf-8").split(":")
        if data[0] == "read":
            left = float(data[2]) - self.left_offset
            right = float(data[4]) - self.right_offset

            return left, right
            # return left > self.left_threshold, right < self.right_threshold
        else:
            self.serial_port.flush()
            return 0.0, 0.0

    def close(self):
        self.serial_port.close()


if __name__ == "__main__":

    print("Starting contact data streaming")
    # Get the port the evo has been connected to
    contact = ContactArduino()

    if contact.port == "NULL":
        print("Sorry couldn't find the contact's Arduino. Exiting.")
        sys.exit()

    while True:
        try:
            print(contact.get_contact_data())
        except serial.serialutil.SerialException:
            print("Device disconnected (or multiple access on port). Exiting...")
            break

    contact.close()
    sys.exit()
