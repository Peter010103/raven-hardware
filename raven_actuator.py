#!/usr/bin/env python3
"""Raven ACW actuator models. NumPy/SciPy; volts at FC, omega in rad/s.

Steady models: 'simple' (voltage-speed quadratic and square rotor laws),
'physics' (4th degree speed surface + quadratic thrust / square torque),
'surface' (independent command-voltage response surfaces; best static accuracy).
Positive operating points must lie in the measured command-voltage convex hull.
STOP returns zero *equilibrium*, not instantaneous rotor arrest.

DynamicMotor models rotor speed, not the HX711 measurement path. It uses a
command delay and FO speed state. Startup, braking below the measured speed
range, loaded flight and the opposite propeller remain unvalidated. Runtime
state estimates at low speed are explicitly extrapolations of the running model.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import Delaunay
BASE=Path(__file__).resolve().parent/"parameters"
S=json.loads((BASE/'static_models.json').read_text())
D=json.loads((BASE/'dynamic_models.json').read_text())
HULL=Delaunay(S['domain']['convex_hull_points'])
def surface(signal,u,V):
 u,V=np.broadcast_arrays(np.asarray(u,float),np.asarray(V,float));x=(u.ravel()-1500)/500;z=(V.ravel()-13.5)/3.5;m=S['surfaces'][signal]
 y=sum(c*x**i*z**j for c,(i,j) in zip(m['coefficients'],m['terms']))
 return y.reshape(u.shape)
def steady(u,V,variant='surface',allow_extrapolation=False):
 u,V=np.broadcast_arrays(np.asarray(u,float),np.asarray(V,float));stop=u<=1000;inside=HULL.find_simplex(np.c_[u.ravel(),V.ravel()]).reshape(u.shape)>=0
 if not np.all(np.isfinite(u)&np.isfinite(V)&(V>0)):raise ValueError('Finite command and positive delivered volts required')
 if not allow_extrapolation and np.any(~inside&~stop):raise ValueError('Outside measured command-voltage convex hull; do not infer high-voltage full-command performance')
 if variant=='simple':
  s=(u-1000)/1000;b=S['simple']['speed_coefficients'];w=np.maximum(0,V*(b[0]+b[1]*s+b[2]*s*s));T=S['simple']['kT_N_per_rad_s_squared']*w*w;Q=S['simple']['kQ_Nm_per_rad_s_squared']*w*w
 elif variant in ['physics','surface']:
  w=np.maximum(0,surface('omega',u,V))
  if variant=='physics':T=np.maximum(0,np.polyval(S['rotor_laws']['T_N']['quadratic']['coefficients_a2_a1_a0'],w));Q=S['simple']['kQ_Nm_per_rad_s_squared']*w*w
  else:T=np.maximum(0,surface('T_N',u,V));Q=np.maximum(0,surface('Q_Nm',u,V))
 else:raise ValueError('variant must be simple, physics or surface')
 return {**{k:np.where(stop,0,v) for k,v in [('omega',w),('T_N',T),('Q_Nm',Q)]},'inside_domain':inside|stop}
class DynamicMotor:
 def __init__(self,variant='physics',voltage_direction=False,omega0=0.):
  if variant not in ['physics','simple']:raise ValueError('Dynamic variants are physics or simple')
  self.variant=variant;self.lpv=voltage_direction;self.omega=float(omega0);self.time=0.;self.history=[]
 def step(self,u,V,dt):
  if dt<=0:raise ValueError('dt must be positive')
  target=float(steady(u,V,self.variant)['omega']);m=D['selected']['RPM'] if self.lpv else next(m for m in D['models'] if m['signal']=='RPM' and m['kind']=='first');delay=m['delay_s'] if self.lpv else m['parameters'][0]
  if not self.history:self.history=[(-1.,self.omega)]
  self.history.append((self.time,target));end=self.time+dt
  boundaries=[self.time]+[t+delay for t,v in self.history if self.time<t+delay<end]+[end]
  for left,right in zip(boundaries[:-1],boundaries[1:]):
   q=self.history[0][1]
   for t,v in self.history:
    if t+delay<=left+1e-12:q=v
   if self.lpv:
    beta=np.array(m['log_tau_coefficients']);tau=float(np.exp(np.dot([1,(V-14)/3,(u-1450)/300,1 if q<self.omega else -1],beta)))
   else:tau=m['parameters'][1]
   self.omega=q+(self.omega-q)*np.exp(-(right-left)/tau)
  self.time=end
  while len(self.history)>2 and self.history[1][0]<self.time-delay:self.history.pop(0)
  w=self.omega
  T=S['simple']['kT_N_per_rad_s_squared']*w*w if self.variant=='simple' else max(0.,float(np.polyval(S['rotor_laws']['T_N']['quadratic']['coefficients_a2_a1_a0'],w)))
  if w<1e-8:T=0.
  return dict(omega=w,T_N=T,Q_Nm=S['simple']['kQ_Nm_per_rad_s_squared']*w*w,tau_s=tau)
def reaction_torque(omega,omega_dot):
 """Optional effective acceleration-reaction term; no rig/sensor filtering.

 This coefficient is conditional on the calibrated bench model, not a separate
 rotor inertia measurement. Signed output may reverse during rapid braking.
 """
 r=json.loads((BASE/'reaction_torque_model.json').read_text())
 return r['kQ']*np.asarray(omega)**2+r['J_effective_kg_m2']*np.asarray(omega_dot)

if __name__=='__main__':
 print('Example at u=1500, delivered V=14:')
 for variant in ['simple','physics','surface']: print(variant,steady(1500,14,variant))
