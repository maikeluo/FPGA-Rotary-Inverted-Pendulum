"""Check release hashes, ACG720 target, all project paths and port constraints."""
import hashlib,json,re
from pathlib import Path
import xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[1]
manifest=json.loads((root/'SHA256SUMS.json').read_text(encoding='utf-8-sig'))
for name,expected in manifest.items():
    p=(root/name).resolve()
    assert p.is_relative_to(root) and p.is_file(),name
    assert hashlib.sha256(p.read_bytes()).hexdigest()==expected,name
for kind,top in [('basic','acg720_top'),('io_test','acg720_io_test')]:
    project=root/f'gowin/{kind}/{top}.gprj';doc=ET.parse(project)
    assert doc.find('Device').get('pn')=='GW5AT-LV138PG484C1/I0'
    paths=[]
    for item in doc.findall('.//File'):
        p=(project.parent/item.get('path')).resolve()
        assert p.is_relative_to(root) and p.is_file(),p
        paths.append(p)
    names={p.name for p in paths}
    assert {'acg720_top.v','acg720_cal_guard.v','diagnostic_jog.v','io_observer.v','adc128s102_acg720.v','balance_control.v','traj_gen.v','acg720.cst','acg720.sdc'}<=names
    assert not {'j280_top.v','adc_3pa1030.v'}&names
    cfg=json.loads((project.parent/f'impl/{top}_process_config.json').read_text())
    assert cfg['TopModule']==top and cfg['OUTPUT_BASE_NAME']==top
    assert f'set_option -top_module {top}' in (project.parent/'build.tcl').read_text()
    assert (root/f'bitstreams/{top}.fs').stat().st_size>1000
cst=(root/'constraints/acg720.cst').read_text()
pins=dict(re.findall(r'IO_LOC\s+"([^"]+)"\s+(\w+);',cst))
assert len(pins)==22 and len(set(pins.values()))==22
assert {x:pins[x] for x in ['pwma','ain1','ain2','enc1_a','enc1_b']}==dict(pwma='E17',ain1='D20',ain2='F16',enc1_a='C20',enc1_b='F18')
timings=json.loads((root/'verification/timing_summary.json').read_text(encoding='utf-8-sig'))
for t in timings.values():assert t['setup_violations']==t['hold_violations']==0
top_text=(root/'rtl/acg720_top.v').read_text(encoding='utf-8-sig')
assert "16'hA413" in top_text
assert re.search(r'PENDULUM_REVERSE\s*=\s*1\b',top_text)
assert re.search(r'ENCODER_REVERSE\s*=\s*0\b',top_text)
assert re.search(r'ADC_AVERAGE_LOG2\s*=\s*9\b',top_text)
status=json.loads((root/'RELEASE_STATUS.json').read_text(encoding='utf-8-sig'))
assert status['firmware_tag']==42003 and status['hardware_tested'] is False
assert re.search(r'JOG_MAX_DUTY\s*=\s*200\b',top_text)
assert re.search(r'JOG_MAX_TICKS\s*=\s*500\b',top_text)
assert re.search(r'JOG_NO_MOTION_TICKS\s*=\s*200\b',top_text)
assert re.search(r'JOG_TRAVEL_COUNTS\s*=\s*64\b',top_text)
print(f'PASS {len(manifest)} hashes, both ACG720 projects/bitstreams, 22 distinct pins, final timing summaries')
