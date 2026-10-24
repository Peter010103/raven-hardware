"""Additional trace-derived metrics; never alter captured simulation traces."""
import argparse,json
from pathlib import Path
import numpy as np

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--suite',type=Path,required=True);args=p.parse_args();suite=args.suite.resolve();index=json.loads((suite/'summary.json').read_text())
    for entry in index['cases']:
        folder=suite/entry['case'];summary=json.loads((folder/'summary.json').read_text());rows=[json.loads(x) for x in (folder/'trace.jsonl').read_text().splitlines()];s=np.array([x['state'] for x in rows]);r=np.array([x['reference'] for x in rows]);t=np.array([x['time_s'] for x in rows]);commands=np.array([x['actuator']['motor_external_counts'] for x in rows]);limits=summary['map']['support_external_counts'];assert np.all(commands>=limits[0]-1e-7) and np.all(commands<=limits[1]+1e-7);moving=(t>=summary['reference']['hold_s'])&(t<=summary['reference']['hold_s']+summary['reference']['moving_s'])
        metrics={'moving_position_RMSE_m':float(np.sqrt(np.mean(np.sum((s[moving,:3]-r[moving,:3])**2,axis=1)))) if moving.any() and not summary['options']['hover_only'] else None,'final_speed_m_s':float(np.linalg.norm(s[-1,7:10])),'final_body_rate_rad_s':s[-1,10:13].tolist(),'all_recorded_rotor_commands_within_measured_support':True,'motor_command_range_counts':[float(commands.min()),float(commands.max())]};summary.update(metrics);entry.update(metrics);(folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    (suite/'summary.json').write_text(json.dumps(index,indent=2)+'\n');print('Assessed',len(index['cases']),'complete traces')

if __name__=='__main__':main()
