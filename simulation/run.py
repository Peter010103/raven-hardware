"""Offline measured-rotor Raven flip MPC screen. No flight or network interface."""
import argparse,json,sys,subprocess,select,time,os,hashlib,shutil
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from plant import RavenPlant,ROOT
from reference import RunInPitchReference

class Worker:
    def __init__(self,config,output):
        self.log=(output/'solver.log').open('w');env={**os.environ,'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','MKL_NUM_THREADS':'1'}
        self.process=subprocess.Popen([sys.executable,'-u',str(Path(__file__).with_name('controller.py')),'--config',str(config)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,env=env);self.info=self.read(240)

    def read(self,timeout=15):
        if not select.select([self.process.stdout],[],[],timeout)[0]:raise TimeoutError('MPC worker timeout')
        line=self.process.stdout.readline()
        if not line:raise RuntimeError('MPC worker exited; see solver.log')
        return json.loads(line)

    def call(self,**data):
        self.process.stdin.write(json.dumps(data)+'\n');self.process.stdin.flush();return self.read()

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.write('{"op":"close"}\n');self.process.stdin.flush();self.process.wait(timeout=10)
        self.log.close()

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--voltage',type=float,default=14.8);p.add_argument('--cells',type=int,choices=[3,4],default=4);p.add_argument('--compensation',type=float,default=0);p.add_argument('--laps',type=int,default=1);p.add_argument('--direction',type=int,choices=[-1,1],default=1);p.add_argument('--rate-scale',type=float,default=1.);p.add_argument('--lag-scale',type=float,default=1.);p.add_argument('--force-scale',type=float,default=1.);p.add_argument('--inertia-scale',type=float,default=1.);p.add_argument('--delay-ticks',type=int,default=0);p.add_argument('--hover-only',action='store_true');args=p.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    plant=RavenPlant(args.voltage,args.compensation,args.rate_scale,args.lag_scale,args.force_scale,args.inertia_scale,cells=args.cells);cfg=plant.controller_config();profile=json.loads(Path(__file__).with_name('reference_profile.json').read_text());ref=RunInPitchReference(cfg,laps=args.laps,direction=args.direction,profile=profile);plant.reset(ref.table[0,:3]);(out/'raven.xml').write_text(plant.xml);(out/'mpc_config.json').write_text(json.dumps(cfg,indent=2)+'\n');(out/'reference.json').write_text(json.dumps(ref.info,indent=2)+'\n')
    sources=out/'source';sources.mkdir();hashes={}
    for f in Path(__file__).parent.glob('*.py'):shutil.copy2(f,sources/f.name);hashes[f.name]=hashlib.sha256(f.read_bytes()).hexdigest()
    for f in [ROOT/'raven_actuator.py',*[ROOT/'parameters'/name for name in ['inertial_model.json','static_models.json','dynamic_models.json','reaction_torque_model.json']],*[ROOT/'simulation'/name for name in ['proposed_profile.json','firmware_reference.json']]]:
        target=out/f.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,target);hashes[str(f.relative_to(ROOT))]=hashlib.sha256(f.read_bytes()).hexdigest()
    shutil.copy2(Path(__file__).with_name('reference_profile.json'),sources/'reference_profile.json')
    worker=Worker(out/'mpc_config.json',out);rows=[];failure=None;reject=None;hover=plant.mass*9.81;queue=[np.array([hover,0.,0.,0.]) for _ in range(args.delay_ticks)];duration=5. if args.hover_only else ref.duration;started=time.monotonic()
    try:
        for tick in range(int(np.ceil(duration/.02))):
            t=tick*.02;state=plant.observe();preview=ref.sample(t+args.delay_ticks*.02+np.arange(cfg['nodes']+1)*cfg['prediction_dt_s'])
            if args.hover_only:preview=np.tile(np.r_[ref.table[0,:3],1.,0.,0.,0.,np.zeros(6),hover,hover,0.,0.,0.],(cfg['nodes']+1,1))
            segments=[(.02,c.tolist()) for c in queue];reply=worker.call(op='infer',state=state.tolist(),preview=preview.tolist(),prediction_segments=segments);command=np.array(reply['ctbr']);status=reply['mpc']
            if not status['valid']:
                failure='MPC_invalid';reject=reply;break
            queue.append(command);applied=queue.pop(0);actuator=plant.advance(applied);nextstate=plant.observe();target=ref.sample(t+.02) if not args.hover_only else preview[0]
            rows.append({'time_s':t+.02,'state':nextstate.tolist(),'reference':target.tolist(),'command':command.tolist(),'applied':applied.tolist(),'actuator':actuator,'mpc':status})
            if not np.isfinite(nextstate).all():failure='nonfinite';break
            if not .12<nextstate[2]<2.0:failure='height_envelope';break
            if np.linalg.norm(nextstate[:2])>2.2:failure='horizontal_envelope';break
        s=np.array([r['state'] for r in rows]);rr=np.array([r['reference'] for r in rows]);tt=np.array([r['time_s'] for r in rows]);rot=Rotation.from_quat(s[:,[4,5,6,3]]).as_matrix() if len(s) else None
        if len(s):
            pitch=np.unwrap(np.arctan2(rot[:,0,2],rot[:,2,2]));turns=-(pitch[-1]-pitch[0])/(2*np.pi)*args.direction;tail=tt>=duration-1;attitude=Rotation.from_quat(s[tail][:,[4,5,6,3]]).magnitude();inverted=rot[:,2,2]<-.7;inversions=int(np.sum(inverted&~np.r_[False,inverted[:-1]]));err=np.linalg.norm(s[:,:3]-rr[:,:3],axis=1);final_error=float(np.linalg.norm(s[-1,:3]-rr[-1,:3]));timings=np.array([r['mpc']['total_ms'] for r in rows]);success=bool(failure is None and (args.hover_only or abs(turns-args.laps)<.1 and inversions==args.laps) and final_error<.15 and np.degrees(attitude[-1])<10)
        else:turns=0.;inversions=0;err=np.array([np.nan]);final_error=None;timings=np.array([0]);success=False
        terminal_attitude=float(np.degrees(attitude[-1])) if len(s) else None
        if not success and failure is None:failure='terminal_recovery_criteria'
        summary={'terminal_attitude_error_deg':terminal_attitude,'scope':'OFFLINE NOMINAL/SENSITIVITY SIMULATION; not hardware validated','success':success,'failure':failure,'options':{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},'map':plant.map.info,'rate_loop':'Assumed PI rate servo with fixed hover-slope linear-command mixer; not a Betaflight PID identification','state_feedback':'Ideal native MuJoCo COM position/quaternion/velocity/body-rate and actual force; no mocap/observer errors','reference':ref.info,'model_sources':hashes,'ticks':len(rows),'pitch_turns':float(turns),'inversions':inversions,'position_RMSE_m':float(np.sqrt(np.mean(err**2))) if len(s) else None,'final_position_error_m':final_error,'height_range_m':[float(s[:,2].min()),float(s[:,2].max())] if len(s) else None,'motor_command_support_clipped_inner_updates':plant.domain_clips,'mixer_range_scaled_updates':plant.saturated_steps,'mpc_time_ms':{'p99':float(np.percentile(timings,99)),'max':float(timings.max())},'wall_s':time.monotonic()-started,'rejected_result':reject,'hardware_access':False,'controller':worker.info}
        (out/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows));(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps({k:summary[k] for k in ['success','failure','ticks','pitch_turns','inversions','position_RMSE_m','final_position_error_m','height_range_m','motor_command_support_clipped_inner_updates','mpc_time_ms']},indent=2))
    finally:worker.close()

if __name__=='__main__':main()
