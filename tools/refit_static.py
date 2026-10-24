#!/usr/bin/env python3
"""Static model comparison using repeat3 selection and untouched12/16V tests."""
import json,itertools
from pathlib import Path
import numpy as np,pandas as pd
from scipy.optimize import minimize
from scipy.spatial import Delaunay
ROOT=Path(__file__).resolve().parent.parent;A=ROOT/'data/processed/actuator';MODELS=ROOT/'build/refit';MODELS.mkdir(parents=True,exist_ok=True)
def powers(d):return [(i,j) for total in range(d+1) for i in range(total+1) for j in [total-i]]
def matrix(u,v,terms):
 x=(np.asarray(u)-1500)/500;z=(np.asarray(v)-13.5)/3.5
 return np.column_stack([x**i*z**j for i,j in terms])
def fit_surface(d,y,degree):
 terms=powers(degree);X=matrix(d.u,d.V,terms);target=d[y].to_numpy();target_scale=max(float(np.std(target)),1e-6);target=target/target_scale;coef=np.linalg.lstsq(X,target,rcond=None)[0]
 for _ in range(4):
  e=X@coef-target;scale=max(1e-5,1.4826*np.median(abs(e-np.median(e))));w=np.minimum(1,1.5*scale/np.maximum(abs(e),1e-9));coef=np.linalg.lstsq(X*np.sqrt(w[:,None]),target*np.sqrt(w),rcond=None)[0]
 grid=np.array(list(itertools.product(np.linspace(d.u.min(),d.u.max(),23),np.linspace(d.V.min(),d.V.max(),17))));grid=grid[Delaunay(d[['u','V']].to_numpy()).find_simplex(grid)>=0];G=matrix(grid[:,0],grid[:,1],terms);du=(matrix(grid[:,0]+.1,grid[:,1],terms)-G)/.1;dv=(matrix(grid[:,0],grid[:,1]+.001,terms)-G)/.001;constraints=np.concatenate([G,du,dv]);H=X.T@(w[:,None]*X);b=X.T@(w*target)
 sol=minimize(lambda c:.5*c@H@c-b@c,coef,jac=lambda c:H@c-b,method='SLSQP',constraints={'type':'ineq','fun':lambda c:constraints@c,'jac':lambda c:constraints},options={'maxiter':500,'ftol':1e-11})
 if sol.success:coef=sol.x
 elif np.min(constraints@coef)<-1e-4:raise RuntimeError('Constraint fit failed '+str(sol.message))
 return dict(family='polynomial_surface',degree=degree,terms=terms,coefficients=(coef*target_scale).tolist(),normalization={'x':'(u-1500)/500','z':'(V-13.5)/3.5'},grid_min_value_or_slope=float(np.min(constraints@coef)))
def evaluate(m,u,v):return matrix(u,v,m['terms'])@m['coefficients']
def metrics(y,p):
 e=np.asarray(p)-np.asarray(y);return dict(RMSE=float(np.sqrt(np.mean(e*e))),MAE=float(np.mean(abs(e))),max_abs=float(np.max(abs(e))),R2=float(1-np.sum(e*e)/np.sum((y-np.mean(y))**2)))
def main():
 df=pd.read_csv(A/'plateaus.csv');d=df[(df.kind=='static_plateau')&df.stationary_T].copy();train=d[(d.train_role=='train')];select=d[d.train_role=='selection_repeat3'];test=d[d.train_role=='voltage_test'];alltrain=d[d.source_setpoint_V.isin([10.5,11.1,12.6,13.2,14.8,16.8])];models={};scores=[]
 for y in ['T_N','Q_Nm','omega','P_gross_W','source_I_A']:
  candidates=[]
  for degree in [2,3,4]:
   m=fit_surface(train,y,degree);s=metrics(select[y],evaluate(m,select.u,select.V));candidates.append((s['RMSE'],degree));scores.append(dict(signal=y,family='surface'+str(degree),selection_RMSE=s['RMSE'],train_RMSE=metrics(train[y],evaluate(m,train.u,train.V))['RMSE']))
  degree=min(candidates)[1];m=fit_surface(alltrain,y,degree);m.update(signal=y,selection_degree=degree,voltage_test_metrics=metrics(test[y],evaluate(m,test.u,test.V)),development_metrics=metrics(alltrain[y],evaluate(m,alltrain.u,alltrain.V)));models[y]=m
 # Physical rotor laws, coefficients ordered a2,a1,a0 in physical rad/s.
 laws={};rng=np.random.default_rng(42)
 for y in ['T_N','Q_Nm']:
  laws[y]={}
  for family in ['square','quadratic']:
   def design(w):return (np.asarray(w)/2000)[:,None]**2 if family=='square' else np.column_stack([(np.asarray(w)/2000)**2,np.asarray(w)/2000,np.ones(len(w))])
   X=design(train.omega);c=np.linalg.lstsq(X,train[y],rcond=None)[0];sel=metrics(select[y],design(select.omega)@c);X=design(alltrain.omega);c=np.linalg.lstsq(X,alltrain[y],rcond=None)[0];co=np.r_[c[0]/4e6,c[1]/2000 if len(c)>1 else 0,c[2] if len(c)>1 else 0];groups=list(alltrain.groupby(['source_setpoint_V','u']).indices.values());boot=[]
   for _ in range(300):
    ix=np.concatenate([groups[j] for j in rng.integers(0,len(groups),len(groups))]);b=np.linalg.lstsq(X[ix],alltrain[y].to_numpy()[ix],rcond=None)[0];boot.append(np.r_[b[0]/4e6,b[1]/2000 if len(b)>1 else 0,b[2] if len(b)>1 else 0])
   laws[y][family]=dict(coefficients_a2_a1_a0=co.tolist(),selection_metrics=sel,voltage_test_metrics=metrics(test[y],design(test.omega)@c),bootstrap_95_interval_a2_a1_a0=np.percentile(boot,[2.5,97.5],axis=0).tolist(),bootstrap_note='Resample voltage-command groups; measurement repeatability only, not calibration or installation uncertainty.')
 # Simple cascade: equilibrium speed proportional to voltage times quadratic command.
 def speedX(d):
  x=(d.u.to_numpy()-1000)/1000;return d.V.to_numpy()[:,None]*np.column_stack([np.ones(len(x)),x,x*x])
 beta=np.linalg.lstsq(speedX(alltrain),alltrain.omega,rcond=None)[0];predw=speedX(test)@beta;kt=laws['T_N']['square']['coefficients_a2_a1_a0'][0];kq=laws['Q_Nm']['square']['coefficients_a2_a1_a0'][0]
 simple=dict(name='voltage_speed_quadratic_plus_rotor_square',omega_equilibrium_formula='omega_ss = V*(b0+b1*s+b2*s^2), s=(u-1000)/1000; STOP u<=1000 forces equilibrium0',speed_coefficients=beta.tolist(),kT_N_per_rad_s_squared=kt,kQ_Nm_per_rad_s_squared=kq,voltage_test_metrics={'omega':metrics(test.omega,predw),'T_N':metrics(test.T_N,kt*predw**2),'Q_Nm':metrics(test.Q_Nm,kq*predw**2)})
 output=dict(schema_version=1,scope='One EMAX ECOII2207 1900KV, HQ5x4.8x3V1S ACW prop, disarmed direct MSP motor4 / DShot600. No armed Acro/vbat-compensation identification.',units={'u':'MSP counts, not microsecond ESC pulse','V':'delivered FC volts','omega':'rad/s','T_N':'newtons','Q_Nm':'positive resisting-torque magnitude; recorded rig sign negated','P_gross_W':'supply electrical power incl wiring/electronics'},split={'development_voltages':[10.5,11.1,12.6,13.2,14.8,16.8],'selection':'repeat3 at development voltages; degrees selected before voltage-test evaluation','voltage_test':[12.0,16.0],'n_initial_train':len(train),'n_selection':len(select),'n_development_refit':len(alltrain),'n_voltage_test':len(test)},domain={'u':[float(d.u.min()),float(d.u.max())],'FC_V':[float(d.V.min()),float(d.V.max())],'convex_hull_points':d[['u','V']].drop_duplicates().to_numpy().tolist(),'stop_and_startup':'u<=1000 STOP. Positive-domain fit is not a startup/deadband model. Lowest commanded startup probe1010; do not infer a precise threshold below it.'},simple=simple,surfaces=models,rotor_laws=laws,quality={'static_rows':len(df[df.kind=='static_plateau']),'stationary_fit_rows':len(d),'excluded_nonstationary_phase_ids':df[(df.kind=='static_plateau')&~df.stationary_T].phase_id.tolist(),'calibration':'Thrust up to9.80665N reference; torque only +/-0.024516625Nm, larger values extrapolate gain; no additional motors/directions tested.'})
 (MODELS/'static_models.json').write_text(json.dumps(output,indent=2)+'\n');pd.DataFrame(scores).to_csv(MODELS/'static_model_selection.csv',index=False)
 pred=test[['run','phase_id','u','V','T_N','Q_Nm','omega']].copy()
 for y in ['T_N','Q_Nm','omega']:pred[y+'_best_prediction']=evaluate(models[y],test.u,test.V)
 pred.to_csv(MODELS/'static_heldout_predictions.csv',index=False);print(json.dumps({'split':output['split'],'simple':simple,'best_surfaces':{y:{'degree':m['degree'],'test':m['voltage_test_metrics']} for y,m in models.items()},'rotor_laws':laws},indent=2))
if __name__=='__main__':main()
