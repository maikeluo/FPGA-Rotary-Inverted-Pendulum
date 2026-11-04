"""Run real RTL testbenches with Icarus. No third-party Python modules needed."""
import argparse
import json
import re
from pathlib import Path
import shutil
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--iverilog', default=shutil.which('iverilog'))
parser.add_argument('--vvp', default=shutil.which('vvp'))
args = parser.parse_args()
if not args.iverilog:
    parser.error('Install Icarus Verilog or pass --iverilog /path/to/iverilog')
if not args.vvp:
    candidate = Path(args.iverilog).with_name('vvp.exe' if sys.platform == 'win32' else 'vvp')
    args.vvp = str(candidate)
out = root / 'build' / 'sim'
out.mkdir(parents=True, exist_ok=True)
(out / 'results.json').unlink(missing_ok=True)
rtl = sorted((root / 'rtl').glob('*.v'))
results = []
for bench in sorted((root / 'tests').glob('tb_*.sv')):
    name = bench.stem
    binary = out / (name + '.vvp')
    commands = [[args.iverilog, '-g2012', '-Wall', '-I', str(root/'tests'), '-s', name, '-o', str(binary), *map(str, rtl), str(bench)],
                [args.vvp, str(binary)]]
    log = ''
    for command in commands:
        p = subprocess.run(command, cwd=out, text=True, capture_output=True, timeout=120)
        log += p.stdout + p.stderr
        if p.returncode:
            (out / (name+'.log')).write_text(log, encoding='utf-8')
            print(log)
            raise SystemExit(p.returncode)
    (out / (name+'.log')).write_text(log, encoding='utf-8')
    if 'PASS ' not in log:
        raise SystemExit('Missing pass assertion: ' + name)
    print(log.strip())
    results.append({'test': name, 'passed': True, 'log': name+'.log'})
for top in ['acg720_top', 'acg720_io_test']:
    command = [args.iverilog, '-g2012', '-Wall', '-s', top, '-o', str(out / (top+'.vvp')), *map(str, rtl)]
    p = subprocess.run(command, text=True, capture_output=True)
    if p.returncode:
        print(p.stderr);raise SystemExit(p.returncode)
    results.append({'test': top+'_elaboration', 'passed': True})
p = subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', str(root/'tests'), '-p', 'test_*.py'],
                   text=True, capture_output=True, timeout=60)
(out/'python_tests.log').write_text(p.stdout+p.stderr, encoding='utf-8')
print((p.stdout+p.stderr).strip())
if p.returncode:
    raise SystemExit(p.returncode)
(out/'results.json').write_text(json.dumps({'rtl_and_elaboration': results,
    'python_unittest': {'passed': True, 'test_count': int(re.search(r'Ran (\d+) tests?', p.stdout+p.stderr).group(1)), 'log': 'python_tests.log'}}, indent=2), encoding='utf-8')
print(f'{len(results)} checks passed; logs: {out}')
