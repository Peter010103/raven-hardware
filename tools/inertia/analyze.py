#!/usr/bin/env python3
"""Refit packaged mocap windows and recover the adopted pose-aware diagonal inertia."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from bifilar_core import fit_oscillation,geometry
ROOT=Path(__file__).resolve().parents[2]

def analyze(output=None):
 model=json.loads((ROOT/'parameters/inertial_model.json').read_text())
 settings=json.loads((ROOT/'parameters/measurement_settings.json').read_text())
 old=json.loads((ROOT/'data/processed/inertia/oscillation_models.json').read_text())
 rows=[];traces={};com=np.array(model['COM_body_m']);mass=model['mass_kg']
 for axis in ['yaw','pitch','roll']:
  saved=old[axis];folder=ROOT/'data/raw/bifilar'/saved['run'];receipt=json.loads((folder/'capture_receipt.json').read_text());r=list(csv.DictReader((folder/'poses.csv').open()))
  host=np.array([float(x['host_monotonic_s']) for x in r])-receipt['cue_monotonic_s'];sec=np.array([int(x['stamp_sec']) for x in r],dtype=np.int64);ns=np.array([int(x['stamp_nanosec']) for x in r],dtype=np.int64);t=(sec-sec[0]).astype(float)+(ns-ns[0])*1e-9
  if np.any(np.diff(t)<=0):raise ValueError('Nonmonotonic source timestamps')
  R=Rotation.from_quat(np.array([[float(x[k]) for k in ['qx','qy','qz','qw']] for x in r]));start,end=saved['train_window_s_after_cue'];window=(host>=start)&(host<=end);relative=(R*R[window].mean().inv()).as_quat();theta=np.unwrap(2*np.arctan2(relative[:,2],relative[:,3]));theta-=np.median(theta[window]);fit,grid,y,pred=fit_oscillation(t[window],theta[window],.05,5)
  if fit['failures']:raise ValueError((axis,fit['failures']))
  rb=R*Rotation.from_quat(settings['marker_from_body_xyzw']);n=rb[window].inv().apply([0.,0.,1.]);projection=np.mean(n*n,axis=0);p=np.array(model['suspension_axis_points_body_m'][axis]);delta=com-p;dist2=float(np.mean(delta@delta-(n@delta)**2));g=geometry(mass*1000,**{k:settings['geometry_by_axis'][axis][k] for k in ['length_cm','top_cm','bottom_cm']});omega=2*np.pi/fit['period_damped_s'];Iaxis=g['stiffness_Nm_rad']/(omega**2+fit['decay_per_s']**2)
  rows.append(dict(axis=axis,period_s=fit['period_damped_s'],decay_per_s=fit['decay_per_s'],fit_RMS_deg=fit['residual_rms_deg'],fit_window_s=[start,end],projection=projection.tolist(),I_suspension_kg_m2=Iaxis,mean_squared_COM_axis_distance_m2=dist2))
  # Same-capture temporal holdout; coefficients and reference use training only.
  X=np.c_[np.ones(len(grid)),grid-grid.mean(),np.exp(-fit['decay_per_s']*grid)*np.cos(omega*grid),np.exp(-fit['decay_per_s']*grid)*np.sin(omega*grid)];coef=np.linalg.lstsq(X,y,rcond=None)[0];tau=t-t[np.flatnonzero(window)[0]];prediction=coef[0]+coef[1]*(tau-grid.mean())+np.exp(np.clip(-fit['decay_per_s']*tau,-50,50))*(coef[2]*np.cos(omega*tau)+coef[3]*np.sin(omega*tau));forecast=(host>end)&(host<=end+3)
  traces[axis]=dict(host=host,theta=theta,prediction=prediction,window=window,forecast=forecast,mean_world_from_body=(R[window].mean()*Rotation.from_quat(settings['marker_from_body_xyzw'])).as_matrix())
 A=np.array([x['projection'] for x in rows]);b=np.array([x['I_suspension_kg_m2']-mass*x['mean_squared_COM_axis_distance_m2'] for x in rows]);moments=np.linalg.solve(A,b);assert np.all(moments>0) and 2*max(moments)<=sum(moments)
 np.testing.assert_allclose(moments,np.diag(model['I_COM_kg_m2']),rtol=1e-6,atol=1e-12)
 result={'mass_kg':mass,'COM_body_m':com.tolist(),'I_COM_diagonal_kg_m2':moments.tolist(),'projection_condition_number':float(np.linalg.cond(A)),'rows':rows,'checks':'Refit selected raw windows, use all selected quaternions, correct each distinct contact midpoint, compare adopted model.','limits':'Three scalar measurements; diagonal COM tensor assumption, no attachment subtraction, adopted CAD-based COM, inferred contact edges.'}
 if output is not None:
  output=Path(output);output.mkdir(parents=True,exist_ok=True);(output/'recomputed.json').write_text(json.dumps(result,indent=2)+'\n')
 return result,traces

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,default=ROOT/'build/inertia');args=p.parse_args();result,_=analyze(args.output);print(json.dumps(result,indent=2))
