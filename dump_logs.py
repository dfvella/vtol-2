#!/usr/bin/env python3

# Script for retrieving logs from the vtol-2 flight controller.
# Run script then connect the Raspberry Pi Pico over usb.

from serial import Serial, SerialException
from time import sleep
from datetime import datetime

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

FLASH_SIZE = 2 * 1024 * 1024
FLASH_LOG_START = 128 * 1024
LOG_DATA_SIZE = 64
LOG_LINES = (FLASH_SIZE - FLASH_LOG_START) // LOG_DATA_SIZE

TIMEOUT = 3
LOOP_PERIOD = 0.02

COMMAND_DUMP_LOGS = 'd'.encode('utf-8')

LOG_DIR = '/home/david/projects/vtol-2/logs'

LOG_HEADING = ' '.join([
    'time, ',
    'input thro, input aile, input elev, input rudd, input gear, input aux1,',
    'current roll, current pitch, current yaw,',
    'target roll, target pitch, target yaw,',
    'pid roll, pid pitch, pid yaw,',
    'output r elevon, output l elevon, output r motor, output l motor, output gear,',
    'control mode, flight mode, transition state, flags\n'
])

def log_file_name() -> str:
    d = datetime.now()
    return f'{d.month}-{d.day}-{d.year}_{d.hour}:{d.minute}:{d.second}.csv'

def request_logs(ser: Serial) -> None:
    ser.write(COMMAND_DUMP_LOGS)
    print('requested logs from flight controller')

def open_port(ser: Serial, port: str) -> None:
    ser.setPort(port)
    while not ser.isOpen():
        try:
            ser.open()
        except SerialException:
            print(f'waiting for port {port}')
            sleep(1)
    print(f'opened port {port}')

def read_port(ser: Serial) -> None:
    log_file = LOG_DIR + '/' + log_file_name()
    with open(log_file, 'w') as f:
        f.write(LOG_HEADING)
        for i in range(LOG_LINES):
            p = int(1 + ((i / LOG_LINES) * 100))
            t = i * LOOP_PERIOD
            try:
                line = ser.readline().decode('utf-8')
                if line == '':
                    break
                f.write(f'{t:.2f}, {line}')
            except UnicodeDecodeError:
                f.write('failed to decode line\n')
            print(f'\r{p}%', end='', flush=True)
        print('\nfinished')

def close_port(ser: Serial) -> None:
    if ser.isOpen():
        ser.close()
        print(f'closed port {ser.port}')

def main() -> None:
    ser = Serial(baudrate=BAUD_RATE, timeout=TIMEOUT)

    try:
        open_port(ser, SERIAL_PORT)
        request_logs(ser)
        read_port(ser)
    except KeyboardInterrupt:
        print('')
    finally:
        close_port(ser)

if __name__ =='__main__':
    main()
