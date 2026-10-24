#!/usr/bin/env python3
"""Build the curated figures using only packaged data; no hardware or CAD SDK."""
import argparse,json,sys
from pathlib import Path
import numpy as np,pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import Delaunay
from scipy.spatial.transform import Rotation
from scipy.optimize import brentq
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'tools/inertia'))
from raven_actuator import steady,surface,S,D
from analyze import analyze
for p in (Path.home()/'.local/share/fonts').glob('Avenir*.ttf'):fm.fontManager.addfont(str(p))
plt.rcParams.update({'font.family':['Avenir','DejaVu Sans'],'font.size':10,'axes.titlesize':11,'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.16,'svg.fonttype':'none','pdf.fonttype':3})
INK='#173641';TEAL='#257d98';GOLD='#bf9226';VOLTS=[10.5,11.1,12,12.6,13.2,14.8,16,16.8];COL={v:plt.cm.viridis(i/8) for i,v in enumerate(VOLTS)}

def save(fig,out,name,raster=False):
 out.mkdir(parents=True,exist_ok=True);path=out/(name+('.png' if raster else '.svg'));fig.savefig(path,dpi=900,bbox_inches='tight',pad_inches=.15)
 if not raster:
  preview=ROOT/'build/figure-previews';preview.mkdir(parents=True,exist_ok=True);fig.savefig(preview/(name+'.png'),dpi=150,bbox_inches='tight')
 plt.close(fig);print(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path,flush=True)

def response(t,m):
 x=np.maximum(t-m['parameters'][0],0);p=m['parameters']
 if m['kind']=='first':return 1-np.exp(-x/p[1])
 a,b=p[1:3];return 1-(a*np.exp(-x/a)-b*np.exp(-x/b))/(a-b)

def actuator(out):
 data=ROOT/'data/processed/actuator';d=pd.read_csv(data/'static_enriched.csv');hull=Delaunay(S['domain']['convex_hull_points']);u,v=np.meshgrid(np.linspace(1050,2000,50),np.linspace(9.89,16.94,45));inside=hull.find_simplex(np.c_[u.ravel(),v.ravel()]).reshape(u.shape)>=0
 fig=plt.figure(figsize=(12,5))
 for j,(key,title,label) in enumerate([('T_N','Thrust surface','Rotor thrust [N]'),('Q_Nm','Reaction-torque surface','Resisting torque [N m]')]):
  a=fig.add_subplot(1,2,j+1,projection='3d');z=surface(key,u,v);z[~inside]=np.nan;a.plot_surface(u,v,z,cmap='viridis',alpha=.8,linewidth=0,rasterized=True);a.scatter(d.u,d.V,d[key],s=7,c='#203b48',alpha=.3,depthshade=False);a.set(xlabel='Direct MSP command [counts]',ylabel='Delivered FC voltage [V]',zlabel=label,title=title);a.view_init(25,-125)
 fig.suptitle('One ACW motor: degree-4 command–voltage models and measured plateaus');fig.subplots_adjust(wspace=.20);save(fig,out,'command-voltage-surfaces')
 fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
 for a,(key,label,scale) in zip(axes.flat,[('T_N','Thrust [N]',1),('Q_Nm','Reaction torque [N m]',1),('omega','Mechanical RPM',60/(2*np.pi)),('P_gross_W','Gross supply power [W]',1)]):
  for V,g in d.groupby('source_setpoint_V'):
   a.scatter(g.u,g[key]*scale,s=11,alpha=.20,color=COL[V]);med=g.groupby('u').V.median();uu=np.linspace(med.index.min(),med.index.max(),150);vv=np.interp(uu,med.index,med.values);a.plot(uu,surface(key,uu,vv)*scale,color=COL[V],lw=1.7,label=f'{V:g} V source')
  a.set(xlabel='Direct MSP command [counts]',ylabel=label)
 axes[0,0].legend(fontsize=8,ncol=2);fig.suptitle('Faint measurements; solid shared motor model along measured voltage paths');save(fig,out,'command-curves')
 fig,axes=plt.subplots(1,2,figsize=(11,4.3),layout='constrained');w=np.linspace(d.omega.min(),d.omega.max(),400)
 for a,key,label in zip(axes,['T_N','Q_Nm'],['Thrust [N]','Resisting torque [N m]']):
  a.scatter(d.omega,d[key],s=11,alpha=.2,c=d.source_setpoint_V,cmap='viridis');a.plot(w,np.polyval(S['rotor_laws'][key]['quadratic']['coefficients_a2_a1_a0'],w),color=INK,lw=2,label='Quadratic with linear/constant terms');a.plot(w,S['rotor_laws'][key]['square']['coefficients_a2_a1_a0'][0]*w*w,color=GOLD,lw=1.7,label='Square law');a.set(xlabel='Mechanical angular speed [rad/s]',ylabel=label);a.legend(fontsize=8)
 fig.suptitle('Speed-based rotor laws: one measured motor and ACW propeller');save(fig,out,'rotor-laws')
 fig,axes=plt.subplots(1,2,figsize=(11,4.3),layout='constrained')
 for V,g in d.groupby('source_setpoint_V'):
  axes[0].scatter(g.T_N,g.effectiveness_gf_W,alpha=.25,s=12,color=COL[V]);med=g.groupby('u')[['V']].median();uu=med.index.to_numpy();vv=med.V.to_numpy();T=surface('T_N',uu,vv);P=surface('P_gross_W',uu,vv);axes[0].plot(T,T/9.80665*1000/P,color=COL[V],lw=1.5,label=f'{V:g} V');axes[1].scatter(g.source_I_A,g.source_V-g.V,s=12,color=COL[V],alpha=.25)
 e=json.loads((data/'supplementary_summary.json').read_text())['electrical_path'];ii=np.linspace(0,d.source_I_A.max(),200);axes[1].plot(ii,e['offset_V']+e['R_effective_ohm']*ii,color=INK,lw=2,label='Apparent path fit');axes[0].set(xlabel='Rotor thrust [N]',ylabel='Thrust effectiveness [gf / gross W]');axes[0].legend(ncol=2,fontsize=8);axes[1].set(xlabel='Supply current [A]',ylabel='Source voltage − FC voltage [V]');axes[1].legend(fontsize=8);fig.suptitle('Bench electrical effectiveness and supply-to-FC voltage drop');save(fig,out,'efficiency-voltage-drop')
 steps=pd.read_csv(data/'step_metrics.csv');c=np.load(data/'step_curves.npz');t=c['t'];fig,axes=plt.subplots(2,3,figsize=(12,7),layout='constrained')
 for j,V in enumerate([11.1,14.8,16.8]):
  for i,signal in enumerate(['RPM','T_N']):
   a=axes[i,j];ix=np.flatnonzero((steps.signal==signal)&(steps.source_V==V)&(steps.center_u==1425)&(steps.step_counts==150))
   for curve in c['curves'][ix]:a.plot(t*1000,curve,color=TEAL,alpha=.17,lw=.8)
   m=next(m for m in D['models'] if m['signal']==signal and m['kind']=='first');a.plot(t*1000,response(t,m),color=INK,lw=2,label='Global first order')
   if signal=='T_N':a.plot(t*1000,response(t,D['selected']['T_N']),color=GOLD,lw=2,label='Two-pole observation model')
   a.set(xlim=(-30,250),ylim=(-.2,1.6 if signal=='RPM' else 1.25),title=f'{signal} • {V:g} V source • n={len(ix)}',xlabel='Time from FC output edge [ms]',ylabel='Normalised change')
   if j==0:a.legend(fontsize=8)
 fig.suptitle('Faint individual transitions; solid models (force includes rig/HX711 path)');save(fig,out,'step-responses')
 vb=pd.read_csv(data/'vibration.csv');fig,axes=plt.subplots(1,2,figsize=(11,4.3),layout='constrained')
 for V,g in vb.groupby('source_V'):
  axes[0].scatter(g.RPM,g.gyro_RMS_30_1800_dps,s=10,alpha=.3,color=COL[V]);axes[1].scatter(g.RPM,g.filtered_RMS_30_1800_dps,s=10,alpha=.3,color=COL[V],label=f'{V:g} V')
 for a in axes:a.set(xlabel='Mechanical RPM',ylabel='Gyro XYZ combined RMS [deg/s]')
 axes[0].set_title('Raw gyro • 30–1800 Hz');axes[1].set_title('Filtered gyro • 30–1800 Hz');axes[1].legend(ncol=2,fontsize=8);fig.suptitle('FC vibration on the fixed bench; not a free-flight survey');save(fig,out,'vibration')

def load_geometry():
 g=np.load(ROOT/'data/processed/geometry/cad_preview.npz');return [(g[f'v{i}'],g[f'f{i}'],str(group),str(g['colors'][i])) for i,group in enumerate(g['groups'])]

def inertia(out):
 result,traces=analyze(ROOT/'build/inertia');model=json.loads((ROOT/'parameters/inertial_model.json').read_text());F=json.loads((ROOT/'parameters/fixture_geometry.json').read_text());reg=json.loads((ROOT/'data/processed/inertia/registration_preview.json').read_text());marker=json.loads((ROOT/'parameters/marker_measurements.json').read_text());native=np.array(list(marker['markers_motive_reported_mm'].values()));body=(native-np.array(marker['motor_plate_marker_centroid_motive_mm']))@np.array(reg['body_from_Motive_marker_points_provisional']).T;labels=list(marker['markers_motive_reported_mm']);parts=load_geometry();com=np.array(model['COM_body_m'])*1000;fc=np.array(reg['CAD_FC_bbox_centre_body_mm_preview']);colors=['#df6a32' if l in ['M3','M4','M5','M6'] else '#a43e9e' for l in labels]
 fig=plt.figure(figsize=(12,8));axes=[fig.add_subplot(221,projection='3d'),fig.add_subplot(222),fig.add_subplot(223),fig.add_subplot(224)]
 for v,f,group,color in parts:
  tri=v[f];tri=tri[::max(1,len(tri)//5000)];alpha=.12 if group in ['Propellers','Electronics / details'] else .24;axes[0].add_collection3d(Poly3DCollection(tri,facecolor=color,edgecolors='none',alpha=alpha,rasterized=True))
  for a,ix in zip(axes[1:],[[0,1],[0,2],[1,2]]):a.add_collection(PolyCollection(tri[:,:,ix],facecolor=color,edgecolors='none',alpha=alpha*.8,rasterized=True))
 a=axes[0];a.scatter(*body.T,c=colors,s=35,depthshade=False);a.scatter(*com,c=GOLD,marker='*',s=85,depthshade=False)
 for label,v in zip(labels,body):a.text(*v,label,fontsize=8)
 a.set(xlim=(-115,115),ylim=(-150,150),zlim=(-25,65),xlabel='Body X [mm]',ylabel='Body Y [mm]',zlabel='Body Z [mm]',title='CAD and marker centres');a.set_box_aspect([230,300,90]);a.view_init(-24,-55)
 for a,ix,title in zip(axes[1:],[[0,1],[0,2],[1,2]],['Plan view','Side view','Front view']):
  a.scatter(body[:,ix[0]],body[:,ix[1]],c=colors,s=35,zorder=20);a.scatter(com[ix[0]],com[ix[1]],c=GOLD,marker='*',s=90,zorder=20);a.plot([fc[ix[0]],com[ix[0]]],[fc[ix[1]],com[ix[1]]],color=GOLD,ls='--',zorder=15)
  for label,v in zip(labels,body):a.annotate(label,(v[ix[0]],v[ix[1]]),xytext=(4,4),textcoords='offset points',fontsize=8)
  a.autoscale_view();a.set_aspect('equal');a.set(xlabel='Body '+ 'XYZ'[ix[0]]+' [mm]',ylabel='Body '+'XYZ'[ix[1]]+' [mm]',title=title)
  if ix!=[0,1]:a.set_ylim(-15,65)
 fig.suptitle('Approximate Raven CAD • gold star = adopted COM • marker-centroid origin');fig.subplots_adjust(hspace=.4,wspace=.3,top=.91);save(fig,out,'cad-markers',True)
 fig=plt.figure(figsize=(9,6));a=fig.add_subplot(111,projection='3d');z=F['fixture_plane_body_Z_m']*1000;rect=np.array([[-55,-70,z],[55,-70,z],[55,70,z],[-55,70,z],[-55,-70,z]]);a.plot(*rect.T,color='#66808d',lw=2);a.scatter(*rect[:-1].T,color='#da792f',s=35)
 for axis,color in [('yaw',TEAL),('pitch','#378b60'),('roll','#bd6b34')]:
  pair=np.array(F['pairs'][axis]['contacts_body_m'])*1000;a.plot(*pair.T,color=color,lw=3,label=axis.upper());a.scatter(*pair.mean(0),color=color,s=25)
 a.scatter(*com,c=GOLD,marker='*',s=100,label='Adopted COM');a.scatter(*fc,c=TEAL,s=25,label='CAD FC reference');a.plot([fc[0],com[0]],[fc[1],com[1]],[fc[2],com[2]],ls='--',color=GOLD);a.scatter([0],[0],[0],c='black',s=20,label='Body origin');a.set(xlabel='Body X forward [mm]',ylabel='Body Y left [mm]',zlabel='Body Z up [mm]',xlim=(-75,75),ylim=(-90,90),zlim=(-15,35));a.set_box_aspect([150,180,70]);a.view_init(28,-55);a.legend(fontsize=8,loc='upper left');fig.suptitle('COM 25 mm below FC; 1.962 mm above the lower fixture plane');save(fig,out,'com-fixture')
 fig,axes=plt.subplots(3,2,figsize=(12,9),gridspec_kw={'width_ratios':[3,1]},layout='constrained')
 for j,(axis,trace) in enumerate(traces.items()):
  row=result['rows'][j];start,end=row['fit_window_s'];show=trace['window']|trace['forecast'];x=trace['host'][show]-start;y=np.degrees(trace['theta'][show]);pred=np.degrees(trace['prediction'][show]);a,b=axes[j];a.plot(x,y,color=TEAL,alpha=.3,lw=1,label='Measured');a.plot(x,pred,color=INK,lw=1.7,label='Oscillator model');a.axvspan(10,13,color='#477fb5',alpha=.12);a.set(xlabel='Time from selected window start [s]',ylabel='Vertical twist [deg]',title=f'{axis.upper()} • T={row["period_s"]:.5f} s • fit {start}–{end} s after cue');a.legend(fontsize=8);b.plot(x,y-pred,color=TEAL,lw=.8,alpha=.6);b.axvspan(10,13,color='#477fb5',alpha=.12);b.axhline(0,color='black',lw=.6);b.set(xlabel='Time [s]',ylabel='Residual [deg]',title='Measured − model')
 fig.suptitle('Fit first 10 s; shaded 3 s prediction without refitting');save(fig,out,'oscillation-fits')
 fig=plt.figure(figsize=(12,10));settings=json.loads((ROOT/'parameters/measurement_settings.json').read_text())
 for j,(axis,trace) in enumerate(traces.items()):
  rb=Rotation.from_matrix(trace['mean_world_from_body']);pair=np.array(F['pairs'][axis]['contacts_body_m']);mid=pair.mean(0);bottom=rb.apply(pair-mid);cc=rb.apply(com/1000-mid);direction=bottom[1]-bottom[0];direction[2]=0;direction/=np.linalg.norm(direction);g=settings['geometry_by_axis'][axis];a0=g['top_cm']/200;L=g['length_cm']/100
  def top(h):return np.array([-a0*direction,a0*direction])+[0,0,h]
  height=brentq(lambda h:np.mean(np.linalg.norm(top(h)-bottom,axis=1))-L,.05,1.5);upper=top(height)
  for k in range(2):
   a=fig.add_subplot(3,2,j*2+k+1,projection='3d')
   for v,f,group,color in parts:
    if group not in ['Frame','Motors']:continue
    vv=rb.apply(v/1000-mid);a.add_collection3d(Poly3DCollection(vv[f],facecolor=color,edgecolors='none',alpha=.4,rasterized=True))
   a.plot(*bottom.T,color='#d4762e',lw=2);a.scatter(*bottom.T,color='#d4762e',s=25);a.plot(*upper.T,color='black');a.plot([0,0],[0,0],[-.18,height+.03],color='black',ls=':',lw=1);a.scatter(*cc,c=GOLD,marker='*',s=75,depthshade=False);a.plot([0,cc[0]],[0,cc[1]],[cc[2],cc[2]],color=GOLD,lw=2)
   for b,t in zip(bottom,upper):a.plot([b[0],t[0]],[b[1],t[1]],[b[2],t[2]],color=TEAL,lw=1.6)
   a.set(xlim=(-.17,.17),ylim=(-.17,.17),zlim=(-.18,.12) if k else (-.18,.75),title=f'{axis.upper()} — '+('contact detail' if k else 'two-string suspension'));a.set_box_aspect([.34,.34,.30 if k else .93]);a.set_xticks([-.1,0,.1]);a.set_yticks([-.1,0,.1]);a.set_zlabel('World height [m]',fontsize=8);a.tick_params(labelsize=7,pad=0);a.view_init(17,-52)
   if k:a.text2D(.02,.95,f'COM {abs(cc[2])*1000:.1f} mm below midpoint\nAxis offset {np.linalg.norm(cc[:2])*1000:.1f} mm',transform=a.transAxes,fontsize=8,va='top',color=GOLD)
 fig.suptitle('Mocap mounting poses • inferred contact edges • schematic upper anchors');fig.subplots_adjust(hspace=.25,top=.92);save(fig,out,'suspension')

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--update-results',action='store_true',help='Explicitly overwrite curated figures in media/');args=p.parse_args();base=ROOT/('media' if args.update_results else 'build/media');actuator(base/'identification');inertia(base/'inertia')
