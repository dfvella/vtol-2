#!/usr/bin/env python3

# Script for retrieving logs from the vtol-2 flight controller.
# Run script then connect the Raspberry Pi Pico over usb.

import serial
import datetime
import time

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
TIMEOUT = 3
LOOP_PERIOD = 0.02

def log_file_name() -> str:
    d = datetime.datetime.now()
    return f'{d.month}-{d.day}-{d.year}_{d.hour}:{d.minute}:{d.second}.csv'

LOG_DIR = '/home/david/projects/vtol-2/logs'
LOG_FILE = LOG_DIR + '/' + log_file_name()

ser = serial.Serial(baudrate=BAUD_RATE, timeout=TIMEOUT)

while not ser.isOpen():
    try:
        try:
            ser.setPort(SERIAL_PORT)
            ser.open()
        except serial.SerialException:
            print(f'waiting for port {SERIAL_PORT}')
            time.sleep(1)
    except KeyboardInterrupt:
        print('')
        exit(0)

print(f'opened port {SERIAL_PORT}')

# request the logs from the flight controller
ser.write('d'.encode('utf-8'))

with open(LOG_FILE, 'w') as f:
    heading = 'time, '
    heading += 'input thro, input aile, input elev, input rudd, input gear, input aux1, '
    heading += 'current roll, current pitch, current yaw, '
    heading += 'target roll, target pitch, target yaw, '
    heading += 'pid roll, pid pitch, pid yaw, '
    heading += 'output r elevon, output l elevon, output r motor, output l motor, output gear, '
    heading += 'control mode, flight mode, transition state, flags\n'
    f.write(heading)

    stop = '-1, -1, -1, -1, -1, -1, '
    stop += '-inf, -inf, -inf, '
    stop += '-inf, -inf, -inf, '
    stop += '-inf, -inf, -inf, '
    stop += '-inf, -inf, -inf, -inf, -1, '
    stop += '255, 255, 255, 255'

    i = 0
    while True:
        t = i * LOOP_PERIOD
        line = ser.readline()
        try:
            line = line.decode('utf-8')
            if line.strip() == stop or line == '':
                break
            f.write(f'{t:.2f}, {line}')
        except UnicodeDecodeError:
            print(f'failed to decode line {i + 1}')
        i += 1

ser.close()

print(f'complete, read {i} lines')
