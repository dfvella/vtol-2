#!/usr/bin/env python3

# A script for checking the simulation output for incorrect behavior

from io import FileIO
from sys import argv

USAGE = f'usage: {argv[0]} <sim output file>'

thro = 0
aile = 1
elev = 2
rudd = 3
gear = 4
aux1 = 5
roll_c = 6
pitch_c = 7
yaw_c = 8
roll_t = 9
pitch_t = 10
yaw_t = 11
roll_p = 12
pitch_p = 13
yaw_p = 14
r_elev = 15
l_elev = 16
r_mtr = 17
l_mtr = 18
ln_leg = 19
f_mode = 20
c_mode = 21
tstate = 22
flags = 23

MIN_VAL = -100
CEN_VAL = 0
MAX_VAL = 100

FC_RX_FAILED = 1
FC_IMU_FAILED = 2
FC_BMP_FAILED = 4
FC_WD_REBOOT = 8
FC_OVERRUN = 16
FC_WAITING = 32

FC_DEAD_STICK = 5

def test_eq(a, b) -> bool:
    try:
        assert(a == b)
        return True
    except AssertionError:
        print('FAIL')
        print(f'{a} is not equal to {b}')
        raise Exception

def test_feq(a: float, b: float) -> bool:
    try:
        assert(abs(a - b) < 0.001)
        return True
    except AssertionError:
        print('FAIL')
        print(f'{a} is not almost equal to {b}')
        raise Exception

def test_manual_controls(f: FileIO) -> None:
    pass

def test_motor_safety(f: FileIO) -> None:
    '''
    make sure motors do not turn on unless the following conditions are met:
    '''
    for line in f:
        line = [ float(val.strip()) for val in line.split(',') ]
        assert(len(line) == flags + 1)
        if (int(line[flags]) & FC_WAITING) or \
           (int(line[flags]) & FC_RX_FAILED) or \
           (line[thro] < MIN_VAL + FC_DEAD_STICK):
            test_feq(line[r_mtr], MIN_VAL)
            test_feq(line[l_mtr], MIN_VAL)

def test_loop_timing(f: FileIO) -> None:
    for _ in range(3):
        next(f)
    for line in f:
        line = [ float(val.strip()) for val in line.split(',') ]
        assert(len(line) == flags + 1)
        test_eq(int(line[flags]) & FC_OVERRUN, False)

def test_wait_state(f: FileIO) -> None:
    pass

tests = (
    # test_manual_controls,
    test_motor_safety,
    test_loop_timing,
    # test_wait_state
)

def main() -> None:
    if len(argv) != 2:
        exit(USAGE)

    passes = 0

    with open(argv[1], 'r') as f:
        for t in tests:
            print(f'running {t.__name__} ... ', end='')

            f.seek(0)
            next(f)

            try:
                t(f)
                print('PASS')
                passes += 1
            except Exception:
                pass

    print('finished')
    print(f'{passes}/{len(tests)} tests passed')

if __name__ == '__main__':
    main()
