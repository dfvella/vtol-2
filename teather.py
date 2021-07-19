#!/usr/bin/env python3

# A script for reading/formating usb serial data from the vtol-2 flight controller

from serial import Serial, SerialException
from time import sleep

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

WIDTH_D = 2
WIDTH_F = 6

DATA_NEEDLE = 'log: '
DATA_LABELS = ' '.join([
    '  thro', '  aile', '  elev', '  rudd', '  gear', '  aux1',
    'roll_c', 'ptch_c', ' yaw_c',
    'roll_t', 'ptch_t', ' yaw_t',
    'roll_p', 'ptch_p', ' yaw_p',
    'r_elev', 'l_elev', ' r_mtr', ' l_mtr', 'ln_leg',
    'fm',
    'cm',
    'ts',
    'fl'
])

def format_d(val: str) -> str:
    ret = [ ' ' for _ in range(WIDTH_D - len(val)) ]
    ret.append(val)
    return ''.join(ret)

def format_f(val: str) -> str:
    ret = [ ' ' for _ in range(4 - val.index('.')) ]
    ret.append(val)
    return ''.join(ret)[:WIDTH_F]

def is_data_line(line: str) -> bool:
    return line.find(DATA_NEEDLE) >= 0

def format_line(line: str) -> str:
    if is_data_line(line):
        line = line.replace(DATA_NEEDLE, '')
        data = []
        for val in line.split():
            if '.' in val:
                s = format_f(val)
            else:
                s = format_d(val)
            data.append(s)
        data = ' '.join(data)
        return f'\r{data}\n{DATA_LABELS}\r'
    else:
        return f'{line}\n'

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
    line = ''
    while ser.isOpen():
        try:
            line = ser.readline().decode('utf-8').strip()
        except UnicodeDecodeError:
            line = 'error: failed to decode line\n'
        except SerialException:
            break
        else:
            line = format_line(line)
        finally:
            print(line, end='')

def close_port(ser: Serial) -> None:
    if ser.isOpen():
        ser.close()
        print(f'closed port {ser.port}')

def main() -> None:
    ser = Serial(baudrate=BAUD_RATE)

    try:
        open_port(ser, SERIAL_PORT)
        read_port(ser)
    except KeyboardInterrupt:
        print('')
    finally:
        close_port(ser)

if __name__ == '__main__':
    main()
