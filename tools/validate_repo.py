#!/usr/bin/env python3
"""Validate packaged provenance, actuator behaviour, documentation and inertia."""
import argparse,hashlib,json,re,sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,pandas as pd
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'tools/inertia'))
from raven_actuator import steady,DynamicMotor,reaction_torque,S,D
from analyze import analyze
MANIFEST=ROOT/'data/provenance/manifest.json'

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def manifest():
 return {'schema_version':1,'hash':'SHA256','scope':'Packaged immutable evidence, canonical parameters, robot descriptions and curated media; generated build/ is excluded','files':[{ 'path':str(p.relative_to(ROOT)),'bytes':p.stat().st_size,'sha256':digest(p)} for top in ['data','parameters','media','models'] for p in sorted((ROOT/top).rglob('*')) if p.is_file() and p!=MANIFEST]}

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--refresh-manifest',action='store_true');args=ap.parse_args()
 if args.refresh_manifest:MANIFEST.write_text(json.dumps(manifest(),indent=2)+'\n');print('Updated packaged-file manifest.')
 saved=json.loads(MANIFEST.read_text());actual=manifest();assert saved==actual,'Packaged-file inventory or hash mismatch; inspect changes before refreshing manifest'
 for f in [ROOT/'README.md',*sorted((ROOT/'docs').rglob('*.md')),*sorted((ROOT/'simulation').glob('*.md')),ROOT/'data/README.md',ROOT/'LICENSES.md']:
  for target in re.findall(r'\]\(([^)]+)\)|(?:href|src)="([^"]+)"',f.read_text()):
   link=next(v for v in target if v).split('#')[0]
   if not link or '://' in link or link.startswith('mailto:'):continue
   path=(f.parent/link).resolve();assert path.is_relative_to(ROOT) and path.exists(),f'Broken local link in {f.name}: {link}'
 model=json.loads((ROOT/'data/provenance/simulation_model.json').read_text());inertial=json.loads((ROOT/'parameters/inertial_model.json').read_text())
 for key in ['native_XML','URDF','MPC_config']:assert (ROOT/model[key]).is_file(),f'Missing model artifact: {model[key]}'
 for path,sha in model['parameter_SHA256'].items():assert digest(ROOT/path)==sha
 urdf=ET.parse(ROOT/model['URDF']).getroot();mjcf=ET.parse(ROOT/model['native_XML']).getroot();base=urdf.find("link[@name='COM']/inertial");native=mjcf.find("worldbody/body[@name='raven_COM']/inertial")
 assert float(base.find('mass').get('value'))==float(native.get('mass'))==inertial['mass_kg']
 diagonal=np.diag(inertial['I_COM_kg_m2']);np.testing.assert_allclose([float(base.find('inertia').get(k)) for k in ['ixx','iyy','izz']],diagonal);np.testing.assert_allclose(np.fromstring(native.get('diaginertia'),sep=' '),diagonal)
 for i,r in enumerate(model['rotor_centres_relative_COM_m']):np.testing.assert_allclose(np.fromstring(urdf.find(f"joint[@name='rotor_fixed_{i}']/origin").get('xyz'),sep=' '),r)
 held=pd.read_csv(ROOT/'data/processed/actuator/static_heldout_predictions.csv');reference=json.loads((ROOT/'data/processed/actuator/runtime_validation.json').read_text());scores={}
 for name in ['simple','physics','surface']:
  result=steady(held.u,held.V,name);scores[name]={key:float(np.sqrt(np.mean((result[key]-held[key])**2))) for key in ['omega','T_N','Q_Nm']}
  for key in scores[name]:np.testing.assert_allclose(scores[name][key],reference['heldout_RMSE'][name][key],rtol=1e-8,atol=1e-10)
 for key in ['omega','T_N','Q_Nm']:np.testing.assert_allclose(steady(held.u,held.V)['%s'%key],held[key+'_best_prediction'],rtol=1e-10,atol=1e-10)
 assert all(float(steady(1000,14)[k])==0 for k in ['omega','T_N','Q_Nm'])
 for u,V in [(2000,16.8),(1049,14),(1500,25)]:
  try:steady(u,V)
  except ValueError:pass
  else:raise AssertionError('Unsupported command/voltage accepted')
 initial=float(steady(1300,14,'physics')['omega']);target=float(steady(1500,14,'physics')['omega']);first=next(x for x in D['models'] if x['signal']=='RPM' and x['kind']=='first');delay,tau=first['parameters']
 for dt in [.001,.01]:
  motor=DynamicMotor(omega0=initial);y=np.array([motor.step(1500,14,dt)['omega'] for _ in range(round(.5/dt))]);t=np.arange(1,len(y)+1)*dt;truth=initial+(target-initial)*(1-np.exp(-np.maximum(t-delay,0)/tau));np.testing.assert_allclose(y,truth,rtol=1e-10,atol=1e-8)
 assert reaction_torque(1000,0)>0 and reaction_torque(1000,-1e5)<0
 step_data=np.load(ROOT/'data/processed/actuator/step_curves.npz');step_rows=pd.read_csv(ROOT/'data/processed/actuator/step_metrics.csv');assert len(step_rows)==step_data['curves'].shape[0]==936;assert step_data['curves'].shape[1]==len(step_data['t'])
 inertia,_=analyze(ROOT/'build/inertia');output=ROOT/'build';output.mkdir(exist_ok=True);validation={'status':'passed','packaged_files':len(saved['files']),'checks':['SHA256 inventory','Local Markdown/HTML links','Robot paths, source hashes, matching mass/inertia and rotor frames','Original held-out actuator predictions and reported scores','STOP and joint-domain rejection','Analytic delayed speed state at1ms and10ms','Reaction-torque sign under braking','Inertia refit from original poses with contact midpoint and full quaternion correction'],'actuator_heldout_RMSE':scores,'inertia_COM_diagonal_kg_m2':inertia['I_COM_diagonal_kg_m2']};(output/'validation.json').write_text(json.dumps(validation,indent=2)+'\n');print(json.dumps(validation,indent=2))
if __name__=='__main__':main()
