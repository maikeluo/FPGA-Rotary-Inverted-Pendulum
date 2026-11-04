"""Generic structural synthesis checks only; these do not replace Gowin STA."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
choice = parser.add_mutually_exclusive_group()
choice.add_argument('--yosys', help='Yosys/yowasp-yosys executable')
choice.add_argument('--yowasp-lib', type=Path, help='Directory containing Python yowasp_yosys and dependencies')
args = parser.parse_args()
if args.yowasp_lib:
    launcher = [sys.executable, '-c',
        'import sys; sys.path.insert(0,sys.argv[1]); import yowasp_yosys; '
        'sys.exit(yowasp_yosys.run_yosys(sys.argv[2:]))', str(args.yowasp_lib.resolve())]
else:
    program = args.yosys or shutil.which('yosys') or shutil.which('yowasp-yosys')
    if not program:
        parser.error('Install Yosys or specify --yosys / --yowasp-lib')
    launcher = [program]
out = root / 'build' / 'synthesis'
out.mkdir(parents=True, exist_ok=True)
(out / 'results.json').unlink(missing_ok=True)
results = []
for top in ['acg720_top', 'acg720_io_test']:
    script = f'read_verilog -sv rtl/*.v; synth -noabc -top {top}; check -assert; stat'
    result = subprocess.run([*launcher, '-p', script], cwd=root, text=True,
                            capture_output=True, timeout=600)
    log = result.stdout + result.stderr
    (out / f'{top}.log').write_text(log, encoding='utf-8')
    if result.returncode or 'Found and reported 0 problems.' not in log:
        print(log[-6000:])
        raise SystemExit(result.returncode or 1)
    results.append({'top': top, 'passed': True, 'log': f'{top}.log'})
    print(f'PASS generic synthesis/check: {top}')
(out / 'results.json').write_text(json.dumps({
    'kind': 'Generic synthesis without ABC; not Gowin placement/routing or timing',
    'results': results}, indent=2), encoding='utf-8')
