"""Proposed non-3D Betaflight 4.5.5 steady map; no device interface."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from raven_actuator import steady,S

class CollectiveMap:
    def __init__(self,voltage=14.8,cells=4,compensation=0,idle=.055,output_limit=1.):
        if not np.isfinite([voltage,compensation,idle,output_limit]).all() or cells not in (3,4) or not 0<=compensation<=100 or not 0<=idle<output_limit<=1:raise ValueError('Invalid proposed map configuration')
        self.voltage,self.cells,self.compensation=voltage,cells,compensation
        # Reference 4.5.5 mixer_init uses fixed420cV, not vbat_full_cell_voltage.
        full,warning=4.20,3.50
        cell_voltage_cv=int(round(voltage*100))//cells
        goodness=1-np.clip((420-cell_voltage_cv)/70,0,1)
        attenuation=(full-warning)/full*goodness*compensation/100
        self.dshot_low=48+1999*idle;self.dshot_high=2047-1999*(1-output_limit)
        self.dshot_high-=attenuation*(self.dshot_high-self.dshot_low)
        self.u_low=1001+(self.dshot_low-48)*999/1999
        self.u_high=1001+(self.dshot_high-48)*999/1999
        hull=ConvexHull(S['domain']['convex_hull_points']);lo,hi=-np.inf,np.inf
        for a,b,c in hull.equations:
            if abs(a)<1e-12:
                if b*voltage+c>1e-9:raise ValueError('Voltage outside measured support')
            elif a>0:hi=min(hi,(-b*voltage-c)/a)
            else:lo=max(lo,(-b*voltage-c)/a)
        self.support_low=max(lo+1e-6,self.u_low);self.support_high=min(hi-1e-6,self.u_high)
        if self.support_low>=self.support_high:raise ValueError('No measured support after firmware map')
        self.u=np.linspace(self.support_low,self.support_high,4001);self.F=4*steady(self.u,voltage,'physics')['T_N']
        if np.any(np.diff(self.F)<=0):raise ValueError('Nonmonotone supported collective')
        self.rc_grid=np.arange(1050,2001);commands=self.command(self.rc_grid);self.rc_grid=self.rc_grid[(commands>=self.support_low)&(commands<=self.support_high)];self.rc_force=4*steady(self.command(self.rc_grid),voltage,'physics')['T_N'];self.force_bounds=np.array([self.rc_force[0],self.rc_force[-1]])
        self.info={'scope':'NOMINAL PREDICTION: one ACW motor replicated four times; opposite-direction propeller, installation and collective effects unvalidated','firmware_reference':'Betaflight4.5.5','installed_firmware':'Not installed/configured per latest operator statement','voltage_V':voltage,'cell_count_scenario':cells,'settled_internal_cell_voltage_cV':cell_voltage_cv,'vbat_sag_compensation':compensation,'min_check':1050,'dshot_idle_value':550,'motor_output_limit':int(output_limit*100),'thrust_linear':0,'throttle_boost':0,'dynamic_idle':False,'support_external_counts':[self.support_low,self.support_high],'force_support_N':self.force_bounds.tolist(),'voltage_state':'Settled sag-filtered voltage equals scenario bus voltage. No battery-sag model.'}

    def command(self,rc):
        rc=np.asarray(rc,float)
        normalized=np.floor((np.clip(rc,1050,2000)-1050)*1000/950)/1000
        dshot=self.dshot_low+normalized*(self.dshot_high-self.dshot_low)
        return 1001+(dshot-48)*999/1999

    def force(self,rc):
        return 4*steady(self.command(rc),self.voltage,'physics')['T_N']

    def inverse(self,force):
        force=np.asarray(force,float)
        if np.any(~np.isfinite(force)) or np.any(force<self.force_bounds[0]-1e-9) or np.any(force>self.force_bounds[-1]+1e-9):raise ValueError('Requested force outside jointly supported collective range')
        idx=np.clip(np.searchsorted(self.rc_force,force),1,len(self.rc_grid)-1)
        idx=np.where(abs(self.rc_force[idx-1]-force)<=abs(self.rc_force[idx]-force),idx-1,idx)
        return self.rc_grid[idx]
