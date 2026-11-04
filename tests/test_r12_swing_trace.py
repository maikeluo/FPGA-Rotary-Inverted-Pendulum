from pathlib import Path
import sys,unittest,struct
from functools import reduce
from operator import xor
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from monitor import Parser,decode_swing_fields,decode_io_fields
from live_monitor import explanation

class SwingTrace(unittest.TestCase):
    def row(self,**kw):
        r=dict(version=4,firmware_tag=0xA412,state=1,fault=0,flags=1,
               sensor_status=8,integral_permille=-37,base_trim_mdeg=1,
               return_bias_mdeg=60,return_damping_mdeg=5)
        r.update(kw);return r
    def test_stage_rate_hold_and_unsigned_counter(self):
        self.assertEqual(decode_swing_fields(self.row()),dict(swing_filtered_rate_mdeg_ms=-37,
            swing_phase=1,swing_direction_hold_ms=60,swing_rejected_ticks=5))
        self.assertEqual(decode_swing_fields(self.row(return_damping_mdeg=-1))['swing_rejected_ticks'],65535)
    def test_keep_other_modes_and_versions(self):
        for kw in [dict(firmware_tag=0xA409),dict(state=3),dict(flags=32)]:
            self.assertEqual(decode_swing_fields(self.row(**kw)),{})
        self.assertEqual(decode_io_fields(self.row()),{})
        self.assertEqual(decode_io_fields(self.row(flags=32))['diagnostic_schema'],9)
    def test_framed_uart_semantics(self):
        b=bytearray(b'\xa5\x5a')
        b+=struct.pack('<6BHiiihHH',4,22,1,0,8,1,257,179000,0,0,320,790,257)
        b+=struct.pack('<iiiHBB',14,0,0,0,0,0)
        b+=struct.pack('<hhhh',-37,1,60,5)
        b+=struct.pack('<HBBihH',0xA412,1,88,0,0,400)
        b+=bytes([reduce(xor,b[2:],0)]);self.assertEqual(len(b),65)
        p=Parser();result=[]
        for byte in b:result+=p.feed(bytes([byte]))
        self.assertEqual(len(result),1);self.assertEqual(p.bad_frames,0)
        self.assertEqual(result[0]['swing_filtered_rate_mdeg_ms'],-37)
        self.assertEqual(result[0]['swing_phase'],1)
        b[13]^=1;self.assertEqual(p.feed(b),[]);self.assertEqual(p.bad_frames,1)
    def test_operator_message(self):
        text=explanation(self.row())
        self.assertIn('跟随摆杆运动',text);self.assertIn('历史角度跳变',text)
        self.assertIn('暂停推摆',explanation(self.row(base_trim_mdeg=2)))
        self.assertIn('初始推摆',explanation(self.row(base_trim_mdeg=0)))

if __name__=='__main__':unittest.main()
