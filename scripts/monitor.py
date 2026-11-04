"""J280 binary telemetry reader; --file supports offline replay without pyserial."""
import argparse
import csv
from datetime import datetime
from functools import reduce
from operator import xor
from pathlib import Path
import struct

LENGTH = 29
LENGTHS = {1: 29, 2: 45, 3: 53, 4: 65}
BASE_FIELDS = ['version', 'sequence', 'state', 'fault', 'sensor_status', 'flags',
          'adc', 'angle_mdeg', 'position_counts', 'target_counts',
          'command_permille', 'upright_adc', 'down_adc']
EXTRA_FIELDS = ['angle_delta_mdeg_per_ms', 'velocity_q8', 'travel_origin_counts',
                'capture_ticks', 'limit_mask', 'keys']
DEBUG_FIELDS = ['integral_permille', 'base_trim_mdeg', 'return_bias_mdeg', 'return_damping_mdeg']
RELEASE_FIELDS = ['firmware_tag', 'retry_count', 'recovery_status', 'final_target_counts',
                  'reference_velocity_q8', 'excursion_counts']
IO_FIELDS = ['diagnostic_schema','io_raw_keys','io_filtered_keys','io_raw_seen','io_filtered_seen',
             'io_s0_presses','io_s1_presses','io_launches','io_last_direction',
             'encoder_a_level','encoder_b_level','encoder_a_edges','encoder_b_edges',
             'encoder_illegal_transitions','motor_pwma_level','motor_ain1_level','motor_ain2_level',
             'motor_enabled','motor_pwm_pulses']
SWING_FIELDS = ['swing_filtered_rate_mdeg_ms','swing_phase','swing_direction_hold_ms','swing_rejected_ticks']
CAL_FIELDS = ['pendulum_reverse','down_cal_pending','down_cal_wait_release','start_inhibited']
FIELDS = BASE_FIELDS + EXTRA_FIELDS + DEBUG_FIELDS + RELEASE_FIELDS + IO_FIELDS + SWING_FIELDS + CAL_FIELDS
STATES = {0: 'IDLE', 1: 'SWING', 2: 'CAPTURE', 3: 'BALANCE', 4: 'FAULT'}


def decode_io_fields(record):
    """A409 diagnostic-only payload; old/normal frames retain their meaning."""
    def n(key):return int(record.get(key) or 0)
    if n('firmware_tag') not in (0xA409,0xA412,0xA413) or not n('flags')&32:return {}
    bits=n('integral_permille')&0xffff
    presses=n('angle_mdeg')&0xffffffff
    seen=n('recovery_status')&0xff
    direction=(bits>>14)&3
    return dict(diagnostic_schema=9,io_raw_keys=bits&15,io_filtered_keys=(bits>>4)&15,
        io_raw_seen=seen&15,io_filtered_seen=(seen>>4)&15,
        io_s0_presses=presses&0xffff,io_s1_presses=presses>>16,
        io_launches=n('base_trim_mdeg')&0xffff,io_last_direction={1:1,2:-1}.get(direction,0),
        encoder_a_level=(bits>>8)&1,encoder_b_level=(bits>>9)&1,
        encoder_a_edges=n('return_bias_mdeg')&0xffff,encoder_b_edges=n('return_damping_mdeg')&0xffff,
        encoder_illegal_transitions=n('angle_delta_mdeg_per_ms')&0xffffffff,
        motor_pwma_level=(bits>>12)&1,motor_ain1_level=(bits>>10)&1,motor_ain2_level=(bits>>11)&1,
        motor_enabled=(bits>>13)&1,motor_pwm_pulses=n('reference_velocity_q8')&0xffff)


def decode_swing_fields(record):
    """A412/A413 while SWING. Balance and IO diagnostic payloads are unchanged."""
    def n(key):return int(record.get(key) or 0)
    if n('firmware_tag') not in (0xA412,0xA413) or n('state')!=1 or n('flags')&32:return {}
    return dict(swing_filtered_rate_mdeg_ms=n('integral_permille'),
        swing_phase=n('base_trim_mdeg')&0xffff,
        swing_direction_hold_ms=n('return_bias_mdeg')&0xffff,
        swing_rejected_ticks=n('return_damping_mdeg')&0xffff)


def decode_cal_fields(record):
    """A413 normal image: upper sensor-status bits describe calibration/start."""
    if int(record.get('firmware_tag') or 0)!=0xA413 or int(record.get('flags') or 0)&32:return {}
    bits=int(record.get('sensor_status') or 0)
    return dict(pendulum_reverse=(bits>>4)&1,down_cal_pending=(bits>>5)&1,
        down_cal_wait_release=(bits>>6)&1,start_inhibited=(bits>>7)&1)


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0

    def feed(self, data):
        self.buffer.extend(data)
        records = []
        while True:
            index = self.buffer.find(b'\xa5\x5a')
            if index < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b'\xa5' else b''
                break
            del self.buffer[:index]
            if len(self.buffer) < 3:
                break
            length = LENGTHS.get(self.buffer[2])
            if length is None:
                self.bad_frames += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < length:
                break
            frame = self.buffer[:length]
            if reduce(xor, frame[2:-1], 0) != frame[-1]:
                self.bad_frames += 1
                del self.buffer[0]
                continue
            record = dict(zip(BASE_FIELDS, struct.unpack('<6BHiiihHH', frame[2:28])))
            if frame[2] in (2, 3, 4):
                record.update(zip(EXTRA_FIELDS, struct.unpack('<iiiHBB', frame[28:44])))
            if frame[2] in (3, 4):
                record.update(zip(DEBUG_FIELDS, struct.unpack('<hhhh', frame[44:52])))
            if frame[2] == 4:
                record.update(zip(RELEASE_FIELDS, struct.unpack('<HBBihH', frame[52:64])))
                record.update(decode_io_fields(record))
                record.update(decode_swing_fields(record))
                record.update(decode_cal_fields(record))
            records.append(record)
            del self.buffer[:length]
        return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--port', help='e.g. COM5 or /dev/ttyUSB0; 115200 8N1')
    group.add_argument('--file', type=Path, help='raw binary UART capture')
    parser.add_argument('--csv', type=Path, default=Path('j280_log.csv'))
    parser.add_argument('--telemetry-hz', type=int, default=100, help='Actual build telemetry rate; ACG720 r54r7=100')
    args = parser.parse_args()
    if args.telemetry_hz <= 0: parser.error('--telemetry-hz must be positive')
    if args.port:
        try:
            import serial
        except ImportError:
            parser.error('Serial mode needs pyserial: python -m pip install pyserial')
        source = serial.Serial(args.port, 115200, timeout=0.1)
    else:
        source = args.file.open('rb')
    decoder = Parser()
    try:
        with source, args.csv.open('w', newline='', encoding='utf-8-sig') as log:
            writer = csv.DictWriter(log, fieldnames=['received_at', 'telemetry_hz', *FIELDS])
            writer.writeheader()
            while True:
                data = source.read(256)
                if not data and args.file:
                    break
                for record in decoder.feed(data):
                    record['telemetry_hz'] = args.telemetry_hz
                    record['received_at'] = datetime.now().isoformat(timespec='milliseconds')
                    writer.writerow(record)
                    extra = (f" rate={record['angle_delta_mdeg_per_ms']:5d} "
                             f"velQ8={record['velocity_q8']:6d} origin={record['travel_origin_counts']:6d} "
                             f"settle={record['capture_ticks']:4d} limits={record['limit_mask']:02X} "
                             f"keys={record['keys']:02X}") if record['version'] in (2, 3, 4) else ''
                    if record['version'] in (3, 4):
                        extra += (f" I={record['integral_permille']:4d} trim={record['base_trim_mdeg']:5d}"
                                  f" bias={record['return_bias_mdeg']:4d} damp={record['return_damping_mdeg']:4d}")
                    if record.get('diagnostic_schema')==9:
                        print(f"{record['sequence']:03d} IO {record['firmware_tag']:04X} pos={record['position_counts']:+d} "
                            f"rawKeys={record['io_raw_keys']:04b} keys={record['io_filtered_keys']:04b} "
                            f"S0={record['io_s0_presses']} S1={record['io_s1_presses']} launches={record['io_launches']} "
                            f"AB={record['encoder_a_level']}{record['encoder_b_level']} "
                            f"edges={record['encoder_a_edges']}/{record['encoder_b_edges']} "
                            f"PWMpulses={record['motor_pwm_pulses']} fault={record['fault']:02X}")
                    else:print(f"{record['sequence']:03d} {STATES.get(record['state'], '?'):7s} "
                          f"ADC={record['adc']:4d} theta={record['angle_mdeg']/1000:8.3f}deg "
                          f"pos={record['position_counts']:6d} goal={record['target_counts']:5d} "
                          f"PWM={record['command_permille']:5d} fault={record['fault']:02X} "
                          f"sensor={record['sensor_status']:02X} flags={record['flags']:02X}{extra}")
                log.flush()
    except KeyboardInterrupt:
        pass
    print(f'Checksum/version rejections: {decoder.bad_frames}. CSV: {args.csv}')


if __name__ == '__main__':
    main()
