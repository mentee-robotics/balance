import serial
import time
import struct
import zlib
import Jetson.GPIO as GPIO
import random
from sys import getsizeof
import sys
from enum import Enum


class eResponse(Enum):
    eConfirm = 0
    eData = 1
    eError = 2


class eEndpoints(Enum):
    eGripper = 0
    eHost = 1
    eMLX_right = 2
    eMLX_left = 3


class eCommands(Enum):
    # Common Commands
    eEnable = 0
    eDisable = 1
    eReset = 2
    eCalibrate = 3

    # MLX Commands
    eGetMagField = 4

    # Gripper Commands
    eMoveToPos = 5
    eGetPos = 6
    eGetCurrent = 7
    eGetStatus = 8


class eResponseCodes(Enum):
    # Common Response codes for Commands
    eEnable = 102
    eDisable = 103
    eReset = 104
    eCalibrate = 105

    # MLX Response codes for Commands
    # eGetMagField=4

    # Gripper Response codes for Commands
    eMoveToPos = 101
    # eGetPos=6
    # eGetCurrent=7


class eErrorCodes(Enum):
    eNoError = "E0"
    eInvalidCommand = "E1"
    eInvalidPayload = "E2"
    eInvalidCRC = "E3"
    eInvalidEndpoint = "E4"
    eInvalidAddress = "E5"
    eInvalidResponse = "E6"
    eInvalidPayloadSize = "E7"
    eInvalidHeader = "E8"
    eInvalidTail = "E9"
    eInvalidChecksum = "E10"
    eInvalidData = "E11"
    eInvalidCalibration = "E12"
    eInvalidPosition = "E13"
    eInvalidCurrent = "E14"


class Gripper:
    def __init__(self, ser, side) -> None:
        self.right_gripper_address = 1
        self.left_gripper_address = 2
        if side == "right":
            self.adress = self.right_gripper_address
        elif side == "left":
            self.adress = self.left_gripper_address
        else:
            print("Enter right or left")
            return
        self.serial = ser
        self.comms = Comms(self.serial)
        self.sense_right = Sense(self.serial, "right", self.comms, self.adress)
        self.sense_left = Sense(self.serial, "left", self.comms, self.adress)

    def get_sensor_data(self):
        left_sensor_data = self.sense_left.get_mag()
        # time.sleep(0.001)
        right_sensor_data = self.sense_right.get_mag()
        # time.sleep(0.001)
        return {"right": right_sensor_data, "left": left_sensor_data}

    def get_pos(self, print_data=False):
        self.comms.send_data(self.adress, 0, 0, eEndpoints.eGripper.value, eCommands.eGetPos.value)
        time.sleep(0.001)
        res = self.comms.receive_data()
        tmout = time.time()
        while (res == -1 or res is None) and time.time() - tmout < self.comms.recieve_timeout:
            res = self.comms.receive_data()
        if res != -1:
            if print_data:
                print(res[0])
            return res[0]
        return

    def set_pos(self, pos, print_data=False):
        self.comms.send_data(self.adress, pos, 4, eEndpoints.eGripper.value, eCommands.eMoveToPos.value)
        time.sleep(0.001)
        res = self.comms.receive_data()
        tmout = time.time()
        while (res == -1 or res is None) and time.time() - tmout < self.comms.recieve_timeout:
            res = self.comms.receive_data()
        if res != -1:
            if print_data:
                print(res[0])
            return res == eResponseCodes.eMoveToPos.value
        return

    def get_current(self, print_data=False):
        self.comms.send_data(self.adress, 0, 0, eEndpoints.eGripper.value, eCommands.eGetCurrent.value)
        time.sleep(0.001)
        res = self.comms.receive_data()
        tmout = time.time()
        while (res == -1 or res is None) and time.time() - tmout < self.comms.recieve_timeout:
            res = self.comms.receive_data()
        if res != -1:
            if print_data:
                print(res[0])
            return res[0]
        return

    def enable(self, print_data=False):
        self.comms.send_data(self.adress, 0, 0, eEndpoints.eGripper.value, eCommands.eEnable.value)
        time.sleep(0.001)
        res = self.comms.receive_data()
        tmout = time.time()
        while (res == -1 or res is None) and time.time() - tmout < self.comms.recieve_timeout:
            res = self.comms.receive_data()
        if res != -1:
            if print_data:
                print(res[0])
            return res == eResponseCodes.eEnable.value
        return

    def disable(self, print_data=False):
        self.comms.send_data(self.adress, 0, 0, eEndpoints.eGripper.value, eCommands.eDisable.value)
        time.sleep(0.001)
        res = self.comms.receive_data()
        tmout = time.time()
        while (res == -1 or res is None) and time.time() - tmout < self.comms.recieve_timeout:
            res = self.comms.receive_data()
        if res != -1:
            if print_data:
                print(res[0])
            return res == eResponseCodes.eDisable.value
        return

    def reset(self, print_data=False):
        self.comms.send_data(self.adress, 0, 0, eEndpoints.eGripper.value, eCommands.eReset.value)
        time.sleep(0.001)
        res = self.comms.receive_data()
        tmout = time.time()
        while (res == -1 or res is None) and time.time() - tmout < self.comms.recieve_timeout:
            res = self.comms.receive_data()
        if res != -1:
            if print_data:
                print(res[0])
            return res == eResponseCodes.eReset.value
        return

    def calibrate(self, print_data=False):
        self.comms.send_data(self.adress, 0, 0, eEndpoints.eGripper.value, eCommands.eCalibrate.value)
        time.sleep(0.001)
        res = self.comms.receive_data()
        tmout = time.time()
        while (res == -1 or res is None) and time.time() - tmout < self.comms.recieve_timeout:
            res = self.comms.receive_data()
        if res != -1:
            if print_data:
                print(res[0])
            return res == eResponseCodes.eCalibrate.value
        return

    def get_status(self, print_data=False):
        self.comms.send_data(self.adress, 0, 0, eEndpoints.eGripper.value, eCommands.eGetStatus.value)
        time.sleep(0.001)
        res = self.comms.receive_data()
        tmout = time.time()
        while (res == -1 or res is None) and time.time() - tmout < self.comms.recieve_timeout:
            res = self.comms.receive_data()
        if res != -1:
            if print_data:
                print(res)
            return res
        return


class Sense:
    def __init__(self, ser, side, com, grip_adress) -> None:
        self.right_finger_address = 1
        self.left_finger_address = 2
        if side == "right":
            self.adress = eEndpoints.eMLX_right.value
        elif side == "left":
            self.adress = eEndpoints.eMLX_left.value
        else:
            print("Enter right or left")
            return
        self.serial = ser
        self.comms = com
        self.grip_adress = grip_adress

    def get_mag(self, print_data=False):
        self.comms.send_data(self.grip_adress, 0, 0, self.adress, eCommands.eGetMagField.value)
        time.sleep(0.001)
        res = self.comms.receive_data()
        tmout = time.time()
        while (res == -1 or res is None) and time.time() - tmout < self.comms.recieve_timeout:
            res = self.comms.receive_data()
        if res != -1:
            if print_data:
                print(res)
            return [res[0], res[1], res[2]]
        return


class Comms:

    def __init__(self, serial):

        self.ser = serial
        ######### pin setup #########
        self.rs485_DE_enable_pin = 12
        self.rs485_RE_enable_pin = 13

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.rs485_DE_enable_pin, GPIO.OUT)
        GPIO.setup(self.rs485_RE_enable_pin, GPIO.OUT)
        #############################

        ######### send message #########
        self.message_to_send = bytearray(8)
        self.header = 170
        self.tail = 99

        ######### recieve message #########
        self.received_message = []
        self.expected_header = 200
        self.expected_tail = 199
        ###############################
        self.TIMEOUT = 0.001
        self.recieve_timeout = 0.01

    def int_to_bytes(self, value):
        return bytes(chr(value), 'utf-8')

    def send_data(self, address, payload, payload_size, endpoint, command):
        self.message_to_send[0] = (bytes([self.header])[0])
        self.message_to_send[1] = (bytes([self.header])[0])
        self.message_to_send[2] = (self.int_to_bytes(address))[0]
        self.message_to_send[3] = (self.int_to_bytes(endpoint))[0]
        self.message_to_send[4] = (self.int_to_bytes(command))[0]
        self.message_to_send[5] = (self.int_to_bytes(payload_size))[0]
        self.message_to_send[6] = (payload.to_bytes(4, sys.byteorder))[0]
        self.message_to_send[7] = (self.int_to_bytes(self.tail))[0]
        GPIO.output(self.rs485_DE_enable_pin, GPIO.HIGH)
        GPIO.output(self.rs485_RE_enable_pin, GPIO.HIGH)
        self.ser.write(self.message_to_send)
        time.sleep(0.001)

    def receive_data(self):
        # GPIO.output(self.rs485_RE_enable_pin,GPIO.LOW)
        GPIO.output(self.rs485_DE_enable_pin, GPIO.LOW)
        dat = []
        k = 0
        start_time = time.time()

        while k < 2:
            value = self.ser.read()
            value = int.from_bytes(value, byteorder='big')
            if value == 200:
                k += 1
            else:
                k = 0
            if time.time() - start_time > self.TIMEOUT:
                return -1
            time.sleep(0.0001)
        dat.append(value)
        dat.append(value)

        start_time = time.time()
        while value != 199:
            value = self.ser.read()
            value = int.from_bytes(value, byteorder='big')
            dat.append(value)
            if time.time() - start_time > self.TIMEOUT:
                return -1

        self.received_message = dat
        # print(dat)
        if len(dat) >= 12:
            # print(len(dat))
            byte_header1 = dat[0]
            byte_header2 = dat[1]
            endpoint = dat[2]
            reponse = dat[3]
            payload_size = dat[4]
            payload = dat[5:5 + payload_size]
            if payload_size + 8 > len(dat):
                return -1

            crc1 = dat[5 + payload_size]
            crc2 = dat[6 + payload_size]
            tail = dat[7 + payload_size]
            return payload
        return -1

    def calculate_crc(self, byte_list):  # counting the ones in the byte array
        count = 0
        for byte in byte_list:
            binary = bin(byte)[2:]
            count += binary.count("1")
        return count

    def check_message_integrity(self, header, tail, received_crc,
                                calculated_crc):  # make sure the message arrived with no errors
        integrity = True
        if header != self.expected_header:
            integrity = False

        if header == self.expected_header:
            integrity = False

        if received_crc == calculated_crc:
            integrity = False

        return integrity


if __name__ == '__main__':
    ser = serial.Serial(port='/dev/ttyTHS0', baudrate=115200, timeout=0.001)
    gripper_right = Gripper(ser, side="right")

    tar_pos = 50
    freq = 30
    i = 0
    while True:
        start = time.time()
        if i % 200 == 0:
            pos = random.randint(0, 100)

        gripper_right.set_pos(pos)
        gripper_right.get_pos(True)
        i += 1
        # time.sleep((1/freq)-(time.time()-start))
