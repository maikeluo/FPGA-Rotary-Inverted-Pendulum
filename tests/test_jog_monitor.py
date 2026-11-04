import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import live_monitor as app

class JogMonitorTests(unittest.TestCase):
    def row(self,**changes):
        r=dict(flags=32,firmware_tag=0xA407,state=0,fault=0,limit_mask=64,position_counts=-123,
               travel_origin_counts=-120,capture_ticks=140,recovery_status=1,version=4)
        r.update(changes);return r
    def test_true_signed_displacement(self):
        self.assertIn('-3 计数',app.jog_readout(self.row()))
        self.assertIn('140 ms',app.jog_readout(self.row()))
        self.assertIn('+1 计数',app.jog_readout(self.row(position_counts=-119)))
        self.assertEqual(app.jog_readout(self.row(flags=1)),'')
    def test_diagnostic_needs_no_calibration_or_recovery(self):
        text=app.explanation(self.row())
        self.assertIn('无需标定',text);self.assertNotIn('尚未完成标定',text)
        self.assertNotIn('取消自动重起',text)
        self.assertIn('再次点动',text)
    def test_stop_reasons(self):
        self.assertIn('至少4计数',app.explanation(self.row(fault=32,limit_mask=4)))
        self.assertIn('64计数边界',app.explanation(self.row(fault=2,limit_mask=1)))
        self.assertIn('速度达到',app.explanation(self.row(fault=2,limit_mask=2)))
        self.assertIn('0.5秒',app.explanation(self.row(limit_mask=8)))
        self.assertIn('8%爬升至20%',app.explanation(self.row(limit_mask=32)))
        self.assertIn('编码器异常',app.explanation(self.row(flags=40,fault=1)))
    def test_legacy_jog_and_normal_are_distinct(self):
        self.assertIn('固定8%',app.explanation(self.row(firmware_tag=0xA406)))
        self.assertEqual(app.jog_readout(self.row(firmware_tag=0xA406)),'')
        self.assertIn('行程已超限',app.explanation(self.row(flags=1,state=4,fault=2,limit_mask=1)))

if __name__=='__main__':unittest.main()
