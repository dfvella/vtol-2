#!/usr/bin/env python3

# A script for forcing the vtol-2 flight controller into bootsel mode over usb

from serial import Serial, SerialException

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

COMMAND_BOOTSEL = 'b'.encode('utf-8')

def main() -> None:
    try:
        ser = Serial(port=SERIAL_PORT, baudrate=BAUD_RATE)
        print(f'opened serial port {SERIAL_PORT}')

        ser.write(COMMAND_BOOTSEL)
        print('sent bootsel command')

        ser.close()
        print(f'closed serial port {SERIAL_PORT}')

        exit(0)

    except KeyboardInterrupt:
        print('')
        exit(1)

    except SerialException:
        print('error: failed to put flight controller into bootsel mode')
        exit (1)

if __name__ == '__main__':
    main()
