import csv
import importlib.util
from pathlib import Path
import sys,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import live_monitor as app
from monitor import Parser

class LiveMonitorTests(unittest.TestCase):
    def test_actual_uart_v4(self):
        raw=(ROOT/'build/sim/telemetry_v4_uart.bin').read_bytes()
        parser=Parser();rows=[]
        for byte in raw:rows.extend(parser.feed(bytes([byte])))
        self.assertEqual(len(rows),5)
        self.assertEqual(parser.bad_frames,0)
        for r in rows:
            self.assertEqual((r['version'],r['firmware_tag'],r['retry_count'],r['recovery_status']), (4,0x5404,2,0x50))
            self.assertEqual((r['final_target_counts'],r['reference_velocity_q8'],r['excursion_counts']),(-401,-16,400))
    def test_fault_explanations(self):
        row={'version':4,'state':4,'fault':1,'limit_mask':0,'flags':1,'recovery_status':0x38,'angle_mdeg':179000,'angle_delta_mdeg_per_ms':0}
        self.assertIn('等待重试',app.explanation(row))
        row['recovery_status']=0x18
        self.assertIn('不允许自动重试',app.explanation(row))
        row['limit_mask']=1;row['fault']=2
        self.assertIn('行程已超限',app.explanation(row))
        row['recovery_status']=1
        self.assertIn('取消自动重起',app.explanation(row))
    def test_record_log_events_and_summary(self):
        with tempfile.TemporaryDirectory() as tmp:
            log=app.Recorder(tmp)
            log.mark('轻推一下')
            base={'sequence':0,'version':4,'state':3,'fault':0,'flags':1,'keys':0,'position_counts':11,'travel_origin_counts':10,
                'target_counts':10,'angle_mdeg':500,'command_permille':74}
            log.write([dict(base)],b'ab');base['sequence']=2;log.write([dict(base)],b'cd')
            log.close();log.close()
            self.assertEqual(log.gaps,1)
            self.assertEqual(log.csv_path.with_suffix('.bin').read_bytes(),b'abcd')
            self.assertIn('轻推一下',log.csv_path.with_suffix('.events.csv').read_text(encoding='utf-8-sig'))
            path,text=app.analyze(log.csv_path)
            self.assertTrue(path.exists())
            import json
            result=json.loads(text);self.assertFalse(result['连续60秒00档平衡记录'])
            self.assertEqual(result['水平对原点误差_计数']['均值'],1)
            self.assertEqual(result['丢帧下限'],1)
    def test_sixty_seconds_gate(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'data.csv'
            def write(state=3,keys=0):
                with path.open('w',encoding='utf-8-sig',newline='') as f:
                    w=csv.DictWriter(f,fieldnames=['elapsed_s','state','fault','keys','sequence_gaps_min','position_counts','travel_origin_counts','target_counts','angle_mdeg','command_permille'])
                    w.writeheader()
                    for i in range(6001):w.writerow(dict(elapsed_s=i/100,state=state,fault=0,keys=keys,sequence_gaps_min=0,
                        position_counts=5+(i%2),travel_origin_counts=5,target_counts=5,angle_mdeg=(-1)**i*500,command_permille=(-1)**i*74))
            import json
            write();report=json.loads(app.analyze(path)[1]);self.assertTrue(report['连续60秒00档平衡记录'])
            self.assertEqual(report['水平对原点误差_计数']['峰峰值'],1)
            self.assertEqual(report['电机命令_千分比']['峰峰值'],148)
            write(keys=0x30);self.assertFalse(json.loads(app.analyze(path)[1])['连续60秒00档平衡记录'])
            write(state=4);self.assertFalse(json.loads(app.analyze(path)[1])['连续60秒00档平衡记录'])

if __name__=='__main__':unittest.main()
