from pathlib import Path
import sys,unittest,struct,csv,json,tempfile
from functools import reduce
from operator import xor
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from monitor import Parser,FIELDS,decode_io_fields
from live_monitor import io_readout,io_hint,analyze

class IOTraceTests(unittest.TestCase):
    def record(self,**changes):
        r=dict(version=4,firmware_tag=0xA409,flags=32,state=0,fault=0,
            integral_permille=0x5100,base_trim_mdeg=1,return_bias_mdeg=4,return_damping_mdeg=4,
            angle_mdeg=1,angle_delta_mdeg_per_ms=0,recovery_status=0x11,
            reference_velocity_q8=120,position_counts=8,travel_origin_counts=0)
        r.update(changes);return r
    def test_payload_fields(self):
        d=decode_io_fields(self.record())
        self.assertEqual((d['io_raw_keys'],d['io_raw_seen'],d['io_filtered_seen'],d['io_s0_presses'],d['io_s1_presses']),(0,1,1,1,0))
        self.assertEqual((d['encoder_a_level'],d['encoder_b_level'],d['io_last_direction'],d['motor_enabled']),(1,0,1,0))
        self.assertEqual((d['encoder_a_edges'],d['encoder_b_edges'],d['motor_pwm_pulses']),(4,4,120))
    def test_unsigned_fields_do_not_turn_negative(self):
        d=decode_io_fields(self.record(angle_mdeg=-1,integral_permille=-1,base_trim_mdeg=-1,
            return_bias_mdeg=-1,return_damping_mdeg=-1,angle_delta_mdeg_per_ms=-1,reference_velocity_q8=-1))
        self.assertEqual(d['io_s0_presses'],65535);self.assertEqual(d['io_s1_presses'],65535)
        self.assertEqual(d['encoder_a_edges'],65535);self.assertEqual(d['motor_pwm_pulses'],65535)
        self.assertEqual(d['encoder_illegal_transitions'],0xffffffff)
    def test_old_and_normal_payloads_are_not_reinterpreted(self):
        self.assertEqual(decode_io_fields(self.record(firmware_tag=0xA407)),{})
        self.assertEqual(decode_io_fields(self.record(flags=1)),{})
    def test_idle_screenshot_is_not_proof_of_motor_failure(self):
        r=self.record(base_trim_mdeg=0,recovery_status=0,return_bias_mdeg=0,return_damping_mdeg=0,reference_velocity_q8=0)
        self.assertIn('尚未收到',io_hint(r));self.assertNotIn('编码器损坏',io_hint(r))
        self.assertIn('未通过10ms',io_hint(self.record(base_trim_mdeg=0,recovery_status=1)))
        self.assertIn('尚未启动',io_hint(self.record(base_trim_mdeg=0)))
        self.assertIn('尚未记录有效PWM',io_hint(self.record(reference_velocity_q8=0)))
    def test_encoder_branches_and_retention(self):
        self.assertIn('尚未看到A/B',io_hint(self.record(return_bias_mdeg=0,return_damping_mdeg=0)))
        self.assertIn('单相',io_hint(self.record(return_bias_mdeg=0,return_damping_mdeg=7)))
        self.assertIn('非法跳变',io_hint(self.record(angle_delta_mdeg_per_ms=1)))
        self.assertIn('净位移仍小',io_hint(self.record(position_counts=1)))
        self.assertIn('S0计数增大',io_hint(self.record()))
        text=io_readout(self.record());self.assertIn('点动启动 1 次',text);self.assertIn('120',text);self.assertIn('输出已停',text)
    def test_actual_uart_fixture(self):
        p=ROOT/'build/sim/io_trace_uart.bin'
        if not p.exists():self.skipTest('Run RTL testbench tb_r9_io_uart first')
        decoder=Parser();records=[]
        for byte in p.read_bytes():records+=decoder.feed(bytes([byte]))
        self.assertEqual(decoder.bad_frames,0);self.assertGreater(len(records),0)
        d=records[-1];self.assertEqual(d['position_counts'],8)
        self.assertEqual((d['encoder_a_edges'],d['encoder_b_edges'],d['io_launches']),(4,4,1))
        self.assertGreater(d['motor_pwm_pulses'],0);self.assertEqual(d['motor_enabled'],0)
    def test_diagnostic_summary_never_reports_packed_data_as_angle(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'io.csv'
            with p.open('w',newline='',encoding='utf-8-sig') as stream:
                writer=csv.DictWriter(stream,fieldnames=['elapsed_s',*FIELDS]);writer.writeheader()
                writer.writerow(dict(self.record(),elapsed_s=0));writer.writerow(dict(self.record(),elapsed_s=2))
            target,text=analyze(p);summary=json.loads(text)
            self.assertFalse(summary['连续60秒00档平衡记录']);self.assertNotIn('摆杆角度_度',summary)
            self.assertEqual(summary['最后诊断记录']['io_launches'],1);self.assertTrue(target.exists())

if __name__=='__main__':unittest.main()
