import importlib.util
from pathlib import Path
import struct
import unittest
import csv
import subprocess
import sys
import tempfile
from functools import reduce
from operator import xor

spec = importlib.util.spec_from_file_location('monitor', Path(__file__).resolve().parents[1] / 'scripts/monitor.py')
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


class MonitorTests(unittest.TestCase):
    def frame(self):
        data = struct.pack('<6BHiiihHH', 1, 7, 3, 0, 0, 1, 512, -1234, -130, 130, -400, 400, 700)
        return b'\xa5\x5a' + data + bytes([reduce(xor, data, 0)])

    def test_fragmented_and_signed(self):
        p = monitor.Parser()
        records = []
        for byte in self.frame():
            records += p.feed(bytes([byte]))
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]['angle_mdeg'], -1234)
        self.assertEqual(records[0]['command_permille'], -400)
        self.assertEqual(records[0]['position_counts'], -130)

    def test_corruption_resynchronization(self):
        bad = bytearray(self.frame())
        bad[10] ^= 1
        p = monitor.Parser()
        records = p.feed(b'noise\xa5' + bad + self.frame() + self.frame())
        self.assertEqual(len(records), 2)
        self.assertEqual(p.bad_frames, 1)

    def extended_frame(self):
        data = struct.pack('<6BHiiihHHiiiHBB', 2, 8, 2, 0, 0, 129, 746,
                           -1200, 1122, 1122, -40, 746, 210, -83, -256, 1122, 999, 1, 3)
        return b'\xa5\x5a' + data + bytes([reduce(xor, data, 0)])

    def test_extended_fragmented_and_mixed(self):
        p = monitor.Parser()
        records = []
        for byte in self.frame() + self.extended_frame() + self.frame():
            records += p.feed(bytes([byte]))
        self.assertEqual([r['version'] for r in records], [1, 2, 1])
        r = records[1]
        self.assertEqual(r['angle_delta_mdeg_per_ms'], -83)
        self.assertEqual(r['velocity_q8'], -256)
        self.assertEqual(r['travel_origin_counts'], 1122)
        self.assertEqual(r['capture_ticks'], 999)
        self.assertEqual(r['limit_mask'], 1)
        self.assertEqual(r['keys'], 3)

    def test_extended_corruption_and_unknown_version(self):
        bad = bytearray(self.extended_frame())
        bad[38] ^= 4
        p = monitor.Parser()
        records = p.feed(b'noise\xa5\x5a\x7f' + bad + self.extended_frame())
        self.assertEqual(len(records), 1)
        self.assertEqual(p.bad_frames, 2)

    def test_csv_cadence_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / 'input.bin'
            output = Path(tmp) / 'output.csv'
            binary.write_bytes(self.extended_frame())
            for options, expected in [([], '100'), (['--telemetry-hz', '50'], '50')]:
                subprocess.run([sys.executable, str(Path(monitor.__file__)), '--file', str(binary),
                                '--csv', str(output), *options], check=True, capture_output=True)
                with output.open(encoding='utf-8-sig', newline='') as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]['telemetry_hz'], expected)
                self.assertEqual(rows[0]['command_permille'], '-40')


if __name__ == '__main__':
    unittest.main()
