"""Read-only telemetry audit: swing runs, first trips and encoder response evidence.

Usage: python scripts/analyze_startup.py path/to/log.csv --json report.json
This does not control a motor, infer physical force or prove plant stability.
The direction statistic is a closed-loop observation, not a calibrated jog test.
"""
import argparse
import csv
import hashlib
import json
import statistics
from pathlib import Path

def analyze(path):
    data=[]
    with path.open(encoding='utf-8-sig',newline='') as f:
        for row in csv.DictReader(f):
            data.append({k:(float(v) if k=='elapsed_s' else int(v))
                         for k,v in row.items() if k!='received_at' and v!=''})
    if not data:
        raise ValueError('No telemetry rows')
    required={'sequence','state','fault','angle_mdeg','angle_delta_mdeg_per_ms',
              'position_counts','travel_origin_counts','velocity_q8','command_permille','elapsed_s'}
    if not required.issubset(data[0]):
        raise ValueError('Need extended V2+ CSV and elapsed_s')
    runs=[];i=0;all_responses=[]
    while i<len(data):
        if data[i]['state']!=1:
            i+=1;continue
        first=i
        while i<len(data) and data[i]['state']==1:i+=1
        g=data[first:i];sign=[1 if r['command_permille']>0 else -1 for r in g]
        responses=[]
        for j in range(3,len(g)-5):
            if sign[j]==sign[j-1]:continue
            if any(x!=sign[j-1] for x in sign[j-3:j]) or any(x!=sign[j] for x in sign[j:j+4]):continue
            window=g[j-3:j+4]
            if any((b['sequence']-a['sequence'])%256!=1 for a,b in zip(window,window[1:])):continue
            if any(r['command_permille']==0 for r in window):continue
            responses.append((g[j+1]['velocity_q8']-g[j-1]['velocity_q8'])*sign[j])
        all_responses.extend(responses)
        end=data[i] if i<len(data) else None
        runs.append({
          'start_s':g[0]['elapsed_s'],'last_s':g[-1]['elapsed_s'],'samples':len(g),
          'nearest_upright_deg':min(abs(r['angle_mdeg']) for r in g)/1000,
          'first_offset':g[0]['position_counts']-g[0]['travel_origin_counts'],
          'last_offset':g[-1]['position_counts']-g[-1]['travel_origin_counts'],
          'next_state':end['state'] if end else None,'next_fault':end['fault'] if end else None,
          'trip_offset':end['position_counts']-end['travel_origin_counts'] if end else None,
          'command_sign_changes_observed_100Hz':sum(a!=b for a,b in zip(sign,sign[1:])),
          'qualified_reversals':len(responses),'opposite_acceleration':sum(v<0 for v in responses),
          'median_signed_delta_velocity_q8':statistics.median(responses) if responses else None})
    return {
      'source':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
      'rows':len(data),'elapsed_end_s':data[-1]['elapsed_s'],
      'firmware_tags':sorted({r.get('firmware_tag') for r in data if 'firmware_tag' in r}),
      'observed_states':sorted({r['state'] for r in data}),
      'sequence_gaps_lower_bound':data[-1].get('sequence_gaps_min'),
      'runs':runs,'qualified_reversals':len(all_responses),
      'opposite_acceleration':sum(v<0 for v in all_responses),
      'opposite_fraction':sum(v<0 for v in all_responses)/len(all_responses) if all_responses else None,
      'method':'100Hz observations: same command sign for 3 samples before and 4 after reversal; 7 consecutive sequence numbers; compare velocity at reversal+1 against reversal-1, times new command sign. Host timestamps may be batched. No reconstructed 1kHz data or mechanical model.',
      'limitation':'Closed-loop logs suggest motor/encoder polarity; verify with positive and negative jog on the actual wiring. Video/events do not mark a controlled direction test.'}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv',type=Path);p.add_argument('--json',type=Path)
    a=p.parse_args();result=analyze(a.csv)
    s=json.dumps(result,ensure_ascii=False,indent=2)
    if a.json:
        a.json.parent.mkdir(parents=True,exist_ok=True);a.json.write_text(s,encoding='utf-8')
    print(s)
