from pathlib import Path
import sys,struct,unittest
from functools import reduce
from operator import xor
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from monitor import Parser,decode_cal_fields,decode_swing_fields,decode_io_fields
from live_monitor import explanation

class CalibrationStatus(unittest.TestCase):
    def row(self,**kw):
        r=dict(version=4,firmware_tag=0xA413,state=0,fault=0,flags=1,sensor_status=0x10,
               adc=258,down_adc=324,upright_adc=819,angle_mdeg=156000,angle_delta_mdeg_per_ms=0,
               integral_permille=37,base_trim_mdeg=1,return_bias_mdeg=90,return_damping_mdeg=0)
        r.update(kw);return r
    def test_pending_and_polarity(self):
        self.assertEqual(decode_cal_fields(self.row(sensor_status=0xf8)),
            dict(pendulum_reverse=1,down_cal_pending=1,down_cal_wait_release=1,start_inhibited=1))
        self.assertEqual(decode_cal_fields(self.row(sensor_status=0))['pendulum_reverse'],0)
    def test_old_versions_and_diagnostic_are_not_reinterpreted(self):
        self.assertEqual(decode_cal_fields(self.row(firmware_tag=0xA412)),{})
        self.assertEqual(decode_cal_fields(self.row(flags=32)),{})
        self.assertEqual(decode_io_fields(self.row(flags=32))['diagnostic_schema'],9)
        self.assertEqual(decode_swing_fields(self.row()),{})
        self.assertEqual(decode_swing_fields(self.row(state=1))['swing_filtered_rate_mdeg_ms'],37)
    def test_pending_instructions(self):
        self.assertIn('松开S3',explanation(self.row(flags=3,sensor_status=0x70)))
        text=explanation(self.row(flags=3,sensor_status=0x30))
        self.assertIn('稳定1秒',text);self.assertIn('请勿手扶',text)
    def test_blocked_and_retry_diagnosis(self):
        text=explanation(self.row(sensor_status=0x90))
        self.assertIn('启动未执行',text);self.assertIn('258',text);self.assertIn('324',text)
        self.assertIn('没有改写标定',text)
        text=explanation(self.row(state=4,fault=4,recovery_status=24))
        self.assertIn('重新标定',text);self.assertIn('未放宽',text)
    def test_actual_frame_layout_stays_65_bytes(self):
        b=bytearray(b'\xa5\x5a')+struct.pack('<6BHiiihHH',4,1,0,0,0x90,1,258,156000,0,0,0,819,324)
        b+=struct.pack('<iiiHBB',0,0,0,0,0,0)+struct.pack('<hhhh',0,0,0,0)
        b+=struct.pack('<HBBihH',0xA413,0,24,0,0,400)
        b+=bytes([reduce(xor,b[2:],0)])
        self.assertEqual(len(b),65)
        p=Parser();r=p.feed(b)[0]
        self.assertEqual(r['start_inhibited'],1);self.assertEqual(r['pendulum_reverse'],1)
        self.assertEqual(r['down_cal_pending'],0);self.assertEqual(p.bad_frames,0)

if __name__=='__main__':unittest.main()
