import csv,importlib.util,struct,subprocess,sys,tempfile,unittest
from pathlib import Path
from functools import reduce
from operator import xor
root=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('monitor',root/'scripts/monitor.py')
monitor=importlib.util.module_from_spec(spec);spec.loader.exec_module(monitor)

class DebugTests(unittest.TestCase):
    def packet(self,version=3):
        body=struct.pack('<6BHiiihHH',version,8,3,0,0,1,750,-333,-200,0,-400,750,210)
        if version>=2: body+=struct.pack('<iiiHBB',-100,-20,0,0,0,0)
        if version==3: body+=struct.pack('<hhhh',-87,275,-123,384)
        return b'\xa5\x5a'+body+bytes([reduce(xor,body,0)])
    def test_mixed_fragmented(self):
        p=monitor.Parser();records=[]
        for c in self.packet(1)+self.packet(3)+self.packet(2):records+=p.feed(bytes([c]))
        self.assertEqual([x['version'] for x in records],[1,3,2])
        self.assertEqual([records[1][x] for x in monitor.DEBUG_FIELDS],[-87,275,-123,384])
        self.assertNotIn('integral_permille',records[0])
    def test_bad_debug_and_resync(self):
        bad=bytearray(self.packet());bad[49]^=1
        p=monitor.Parser();records=p.feed(b'junk'+bad+self.packet())
        self.assertEqual(len(records),1);self.assertEqual(p.bad_frames,1)
    def test_real_rtl_uart_capture_and_csv(self):
        # run_tests executes RTL first. This is decoded external UART bytes,
        # not a second hand-constructed software packet.
        raw=root/'build/sim/telemetry_v3_uart.bin'
        self.assertTrue(raw.exists(),'run scripts/run_tests.py so RTL capture is generated first')
        records=monitor.Parser().feed(raw.read_bytes())
        self.assertEqual(len(records),10)
        self.assertEqual([x['sequence'] for x in records],list(range(10)))
        for row in records:
            self.assertEqual([row[x] for x in monitor.DEBUG_FIELDS],[-87,275,-123,384])
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'decoded.csv'
            subprocess.run([sys.executable,str(root/'scripts/monitor.py'),'--file',str(raw),'--csv',str(out)],check=True,capture_output=True)
            with out.open(encoding='utf-8-sig') as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual(len(rows),10)
            self.assertEqual(rows[-1]['return_damping_mdeg'],'384')
            self.assertEqual(rows[-1]['telemetry_hz'],'100')

if __name__=='__main__':unittest.main()
