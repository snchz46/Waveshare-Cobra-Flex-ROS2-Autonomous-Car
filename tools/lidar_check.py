#!/usr/bin/env python3
"""Talk to an RPLIDAR A2 (A2M8 / A2M12) over serial without ROS.

Asks the LiDAR for its device info and health, sends the motor command the
same way the SLAMTEC SDK does, then runs a short standard SCAN and reports
samples/s and rotations/s. Use it to tell hardware problems (power, cables,
adapter) from ROS/launch problems.

The motor step and the scan step are separated on purpose: watch whether the
motor already spins after the motor command, or only once the scan starts.

Motor control depends on the unit, as in the SDK (sl_lidar_driver.cpp):
  * accessory board with motor control (e.g. A2M12): SET_MOTOR_PWM command
  * otherwise (e.g. older A2M8): DTR line of the USB adapter
GET_INFO and GET_HEALTH do not need the motor, so they are asked first.

Usage:
    python3 lidar_check.py [port] [baudrate]
    python3 lidar_check.py                      # /dev/ttyUSB0, tries 256000 and 115200
    python3 lidar_check.py /dev/rplidar 256000  # A2M12
    python3 lidar_check.py /dev/rplidar 115200  # A2M8

Needs pyserial: sudo apt install python3-serial
"""
import os
import struct
import sys
import time

import serial

SYNC = b'\xa5\x5a'
CMD_SCAN = 0x20
CMD_STOP = 0x25
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52
CMD_MOTOR_RPM = 0xA8          # built-in motor control (model major id >= 6)
CMD_MOTOR_PWM = 0xF0          # accessory board motor control (A2/A3)
CMD_ACC_BOARD_FLAG = 0xFF
MOTOR_SPEED = 600             # same fallback value the SDK uses
SCAN_SECONDS = 4.0
HEALTH = {0: 'OK', 1: 'WARNING', 2: 'ERROR'}


def frame(cmd, payload=b''):
    """Build a command frame: A5 cmd [size payload checksum]."""
    if not payload:
        return bytes([0xA5, cmd])
    body = bytes([0xA5, cmd, len(payload)]) + payload
    checksum = 0
    for byte in body:
        checksum ^= byte
    return body + bytes([checksum])


def request(ser, cmd, length, payload=b''):
    """Send a command and return (payload, raw bytes received)."""
    ser.reset_input_buffer()
    ser.write(frame(cmd, payload))
    descriptor = ser.read(7)
    if len(descriptor) == 7 and descriptor[:2] == SYNC:
        data = ser.read(length)
        if len(data) == length:
            return data, descriptor + data
        return None, descriptor + data
    return None, descriptor + ser.read(32)


def port_users(port):
    """Processes (of this user) that already have the serial device open."""
    target = os.path.realpath(port)
    users = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        fd_dir = f'/proc/{pid}/fd'
        try:
            if any(os.path.realpath(os.path.join(fd_dir, fd)) == target
                   for fd in os.listdir(fd_dir)):
                with open(f'/proc/{pid}/comm') as comm:
                    users.append(f'{pid} ({comm.read().strip()})')
        except OSError:
            continue
    return users


def set_dtr(ser, asserted):
    try:
        ser.dtr = asserted
    except OSError as err:
        print(f'  cannot set DTR on this port: {err}')


def set_motor(ser, method, speed):
    if method == 'rpm':
        ser.write(frame(CMD_MOTOR_RPM, struct.pack('<H', speed)))
    elif method == 'pwm':
        ser.write(frame(CMD_MOTOR_PWM, struct.pack('<H', speed)))
    else:
        set_dtr(ser, speed == 0)  # SDK: setDTR() stops, clearDTR() spins
    time.sleep(0.01)


def motor_method(ser, model):
    """Pick the motor control method the SDK would use for this model."""
    major = model >> 4
    if major >= 6:
        return 'rpm'
    if major >= 2:
        flag, raw = request(ser, CMD_ACC_BOARD_FLAG, 4, payload=b'\x00\x00\x00\x00')
        if flag is None:
            print(f'  accessory board flag: no valid answer ({raw[:16].hex(" ") or "0 bytes"})')
        else:
            value = struct.unpack('<I', flag)[0]
            print(f'  accessory board flag: 0x{value:08x}')
            if value & 0x1:
                return 'pwm'
    return 'dtr'


def check(port, baud):
    print(f'--- {port} @ {baud} baud')
    with serial.Serial(port, baud, timeout=1) as ser:
        # The SDK clears DTR right after opening the port, before any command
        # (seen with strace on sllidar_node). pyserial asserts DTR on open, so
        # undo that first; the accessory board query below depends on it.
        set_dtr(ser, False)
        time.sleep(0.05)
        ser.write(frame(CMD_STOP))   # leave any previous scan state
        time.sleep(0.1)

        info, raw = request(ser, CMD_GET_INFO, 20)
        if info is None:
            if raw:
                print(f'  no valid answer, received {len(raw)} bytes: {raw[:16].hex(" ")}')
                print('  (bytes but no A5 5A header: usually wrong baud rate)')
            else:
                print('  no answer, 0 bytes received')
            return False

        print(f'  model 0x{info[0]:02x}  firmware {info[2]}.{info[1]:02d}  '
              f'hardware rev {info[3]}')
        print(f'  serial number {info[4:].hex().upper()}')

        health, _ = request(ser, CMD_GET_HEALTH, 3)
        if health is None:
            print('  health: no answer')
        else:
            status = HEALTH.get(health[0], f'unknown ({health[0]})')
            code = int.from_bytes(health[1:3], 'little')
            print(f'  health {status}, error code {code}')

        method = motor_method(ser, info[0])
        print(f'  motor control: {method}')

        # Same order as sllidar_node: motor command first, then the scan.
        set_motor(ser, method, MOTOR_SPEED)
        print('  [1/2] motor command sent, no scan yet (2 s) -> does it spin?')
        time.sleep(2)
        print(f'  [2/2] scanning for {SCAN_SECONDS:.0f} s -> does it spin now?')
        result = scan(ser, SCAN_SECONDS)
        if result is None:
            print('  scan: no answer to the SCAN command')
        else:
            samples, rotations = result
            print(f'  scan: {samples / SCAN_SECONDS:.0f} samples/s, '
                  f'~{rotations / SCAN_SECONDS:.1f} rotations/s')
            if samples == 0:
                print('  (scan started but no samples: motor not reaching speed?)')

        set_motor(ser, method, 0)
        ser.write(frame(CMD_STOP))
        return True


def scan(ser, seconds):
    """Run a standard SCAN and count valid samples and new-rotation flags."""
    ser.reset_input_buffer()
    ser.write(frame(CMD_SCAN))
    descriptor = ser.read(7)
    if len(descriptor) < 7 or descriptor[:2] != SYNC:
        ser.write(frame(CMD_STOP))
        return None

    data = bytearray()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        data += ser.read(ser.in_waiting or 1)
    ser.write(frame(CMD_STOP))
    time.sleep(0.05)
    ser.reset_input_buffer()

    # 5-byte samples: byte0 bit0 = S (new rotation), bit1 = !S; byte1 bit0 = check bit (1)
    samples = rotations = 0
    i = 0
    while i + 5 <= len(data):
        start, inv_start, check_bit = data[i] & 1, (data[i] >> 1) & 1, data[i + 1] & 1
        if start != inv_start and check_bit:
            samples += 1
            rotations += start
            i += 5
        else:
            i += 1  # resync on the next byte
    return samples, rotations


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    bauds = [int(sys.argv[2])] if len(sys.argv) > 2 else [256000, 115200]

    if not os.path.exists(port):
        print(f'{port} does not exist: check the USB data cable (lsusb, ls /dev/ttyUSB*).')
        return 2

    users = port_users(port)
    if users:
        print(f'{port} is already open by: {", ".join(users)}')
        print('Stop those processes first (e.g. the ROS launch); two readers on one '
              'serial port steal each other\'s bytes.')
        return 2

    for baud in bauds:
        try:
            if check(port, baud):
                print(f'\nLiDAR answers at {baud} baud: hardware OK.'
                      '\nIf ROS still fails, check serial_port / serial_baudrate in the launch.')
                return 0
        except serial.SerialException as err:
            print(f'  cannot open {port}: {err}')
            return 2

    print('\nNo answer at any baud rate: the LiDAR core is not talking.'
          '\nCheck power (aux 5 V), the flat cable to the adapter and the connectors.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
