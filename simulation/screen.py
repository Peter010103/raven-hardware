"""Sequential offline hover/flip sensitivity suite; retain every result."""
import argparse,subprocess,sys,json,time
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);args=p.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    cases=[('hover',['--hover-only']),('nominal',[]),('reverse',['--direction','-1']),('three_flips',['--laps','3']),('comp100',['--compensation','100']),('rate_1p5',['--rate-scale','1.5']),('rate_2',['--rate-scale','2']),('motor_1p5',['--lag-scale','1.5']),('delay20',['--delay-ticks','1']),('delay40',['--delay-ticks','2']),('force_minus10',['--force-scale','.9']),('inertia_plus20',['--inertia-scale','1.2']),('voltage12p6',['--voltage','12.6','--cells','3']),('voltage16',['--voltage','16'])]
    results=[]
    for name,options in cases:
        command=[sys.executable,str(Path(__file__).with_name('run.py')),'--output',str(out/name),*options];result=subprocess.run(command,capture_output=True,text=True);(out/(name+'.log')).write_text(result.stdout+result.stderr)
        file=out/name/'summary.json'
        if file.exists():d=json.loads(file.read_text());row={k:d.get(k) for k in ['success','failure','ticks','pitch_turns','inversions','position_RMSE_m','final_position_error_m','terminal_attitude_error_deg','height_range_m','motor_command_support_clipped_inner_updates','mpc_time_ms']}
        else:row={'success':False,'failure':'process_or_preflight_failure','stderr':result.stderr[-3000:]}
        row.update(case=name,options=options,returncode=result.returncode);results.append(row);(out/'summary.json').write_text(json.dumps({'scope':'Conditional engineering-prior simulation screen; no Raven rate-loop identification','cases':results},indent=2)+'\n');print(name,row['success'],row['failure'],flush=True)

if __name__=='__main__':main()
