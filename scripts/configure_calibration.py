"""Set measured startup constants in the source; rebuild/reflash afterwards.

No serial device is accessed and no EEPROM/flash is written by this script.
"""
import argparse
from pathlib import Path
import re


def configure(root, up=None, down=None, *, confirmed=False, disable=False):
    if disable:
        enabled, up, down = 0, 0, 0
    else:
        if not confirmed:
            raise ValueError('Confirm mechanical upright/down positions with --confirm-measured')
        if up is None or down is None or not (1 <= up <= 1022 and 1 <= down <= 1022):
            raise ValueError('Both ADC values must be in 1..1022')
        if abs(down-up) < 64:
            raise ValueError('Calibration span must be at least 64 ADC counts')
        enabled = 1
    path = Path(root)/'rtl/acg720_top.v'
    original = path.read_text(encoding='utf-8-sig')
    updated, count = re.subn(
        r'parameter integer FIXED_CAL_ENABLE=\d+, FIXED_UP=\d+, FIXED_DOWN=\d+,',
        f'parameter integer FIXED_CAL_ENABLE={enabled}, FIXED_UP={up}, FIXED_DOWN={down},',
        original)
    if count != 1:
        raise ValueError('Expected exactly one calibration parameter block; source unchanged')
    path.write_text(updated, encoding='utf-8')
    return enabled, up, down


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--up', type=int, help='Verified mechanical upright ADC value')
    parser.add_argument('--down', type=int, help='Verified down ADC value')
    parser.add_argument('--confirm-measured', action='store_true')
    parser.add_argument('--disable', action='store_true', help='Boot uncalibrated after reflash')
    args = parser.parse_args()
    if args.disable and (args.up is not None or args.down is not None or args.confirm_measured):
        parser.error('--disable must be used alone')
    try:
        values = configure(Path(__file__).resolve().parents[1], args.up, args.down,
                           confirmed=args.confirm_measured, disable=args.disable)
    except ValueError as exc:
        parser.error(str(exc))
    print('Source updated: enabled=%d, upright=%d, down=%d' % values)
    print('Rebuild in Gowin, inspect complete timing summary, then download the new bitstream.')
    print('The original release hash of rtl/acg720_top.v will now differ by design.')


if __name__ == '__main__':
    main()
