"""Run the delivered Tcl script; check self-test PASS rather than exit code alone."""
from pathlib import Path
import argparse,subprocess,json
root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser()
p.add_argument('--vsim',required=True,help='Path to vsim.exe')
p.add_argument('--top',default='tb_v54_trajectory')
a=p.parse_args()
allowed={f.stem for f in (root/'tests').glob('tb_*.sv')}
if a.top not in allowed:p.error('Unknown testbench')
do=(root/'sim/modelsim/run.do').as_posix()
result=subprocess.run([a.vsim,'-c','-do',f'set J280_ROOT {{{root.as_posix()}}}; set TEST_TOP {a.top}; do {{{do}}}; quit -force'],cwd=root,
 capture_output=True,text=True,errors='replace',timeout=240)
log=result.stdout+result.stderr
out=root/'build/modelsim';out.mkdir(parents=True,exist_ok=True)
(out/(a.top+'.log')).write_text(log,encoding='utf-8')
passed=result.returncode==0 and 'PASS ' in log and 'Fatal:' not in log and '** Error' not in log
(out/(a.top+'.json')).write_text(json.dumps({'test':a.top,'passed':passed,'exit_code':result.returncode},indent=2),encoding='utf-8')
print(log[-6500:])
raise SystemExit(0 if passed else 1)
