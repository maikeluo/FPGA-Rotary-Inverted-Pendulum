import importlib.util
from pathlib import Path
import tempfile
import unittest

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('calconfig', root/'scripts/configure_calibration.py')
cal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cal)


class CalibrationConfigTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root/'rtl').mkdir()
        self.path = self.root/'rtl/acg720_top.v'
        self.original = (root/'rtl/acg720_top.v').read_text(encoding='utf-8-sig')
        self.path.write_text(self.original, encoding='utf-8')

    def test_enable_disable(self):
        # Synthetic fixtures only; these are NOT approved hardware defaults.
        self.assertEqual(cal.configure(self.root, 750, 210, confirmed=True), (1,750,210))
        self.assertIn('FIXED_CAL_ENABLE=1, FIXED_UP=750, FIXED_DOWN=210', self.path.read_text(encoding='utf-8'))
        self.assertEqual(cal.configure(self.root, disable=True), (0,0,0))
        self.assertIn('FIXED_CAL_ENABLE=0, FIXED_UP=0, FIXED_DOWN=0', self.path.read_text(encoding='utf-8'))

    def test_reject_invalid_or_unconfirmed(self):
        for up, down, confirmed in [(750,210,False),(0,210,True),(750,1023,True),
                                     (500,550,True),(-1,200,True),(750,None,True)]:
            with self.subTest(up=up,down=down):
                with self.assertRaises(ValueError):
                    cal.configure(self.root,up,down,confirmed=confirmed)
                self.assertEqual(self.path.read_text(encoding='utf-8'),self.original)

    def test_wrong_source_unchanged(self):
        self.path.write_text('module different; endmodule')
        with self.assertRaises(ValueError):
            cal.configure(self.root,750,210,confirmed=True)
        self.assertEqual(self.path.read_text(encoding='utf-8'),'module different; endmodule')
