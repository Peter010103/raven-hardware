"""Native six-DOF MuJoCo plant with four measured rotor-speed states.

The rate PI and linear command mixer are an explicit engineering surrogate,
not identified Raven Betaflight gains or a complete firmware emulation.
"""
import json,sys
from collections import deque
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import root
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from raven_actuator import steady,S
from betaflight import CollectiveMap

class RavenPlant:
    def __init__(self,voltage=14.8,compensation=0,rate_scale=1.,lag_scale=1.,force_scale=1.,inertia_scale=1.,cells=4):
        if not np.isfinite([rate_scale,lag_scale,force_scale,inertia_scale]).all() or min(rate_scale,lag_scale,force_scale,inertia_scale)<=0:raise ValueError('Positive finite sensitivity scales required')
        self.map=CollectiveMap(voltage,cells,compensation);self.voltage=voltage;self.dt=.001;self.inner_dt=.004;self.force_scale=force_scale
        data=json.loads((ROOT/'parameters/inertial_model.json').read_text());self.mass=data['mass_kg'];self.J=np.array(data['I_COM_kg_m2'])*inertia_scale;self.com=np.array(data['COM_body_m']);self.motor_tau=.02189140910403713*lag_scale;self.motor_delay=.0017634598349229545
        self.rotor_reference=np.array([[-.0687435,-.08969,.030],[.0687435,-.08969,.030],[.0687435,.08969,.030],[-.0687435,.08969,.030]])
        self.r=self.rotor_reference-self.com;self.signs=np.array([-1,1,-1,1]);self.mix=np.c_[[-1,-1,1,1],[1,-1,-1,1],self.signs]
        self.rate_tau=np.array([.04,.04,.08])*rate_scale;self.kp=np.diag(np.array(data['I_COM_kg_m2']))/self.rate_tau;self.ki=self.kp/.35
        self.uhover=float(np.interp(self.mass*9.81/self.force_scale,self.map.F,self.map.u))
        def wrench(u):
            o=steady(u,self.voltage,'physics');f=o['T_N']*force_scale;q=o['Q_Nm']*force_scale;return np.r_[f.sum(),np.cross(self.r,np.c_[np.zeros((4,2)),f]).sum(0)+[0,0,self.signs@q]]
        self.wrench=wrench
        nominal_hover=float(np.interp(self.mass*9.81,self.map.F,self.map.u));eps=.01;Jac=np.column_stack([(wrench(np.full(4,nominal_hover)+np.eye(4)[i]*eps)-wrench(np.full(4,nominal_hover)-np.eye(4)[i]*eps))/(2*eps*force_scale) for i in range(4)])
        self.torque_to_command=self.mix@np.linalg.inv(Jac[1:]@self.mix)
        trim=root(lambda u:wrench(u)-[self.mass*9.81,0,0,0],np.full(4,self.uhover))
        if not trim.success or np.linalg.norm(trim.fun)>1e-6:raise ValueError('Hover trim failed')
        self.trim=trim.x;self.initial_integral=np.linalg.lstsq(self.torque_to_command,self.trim-self.uhover,rcond=None)[0]
        self.integral=self.initial_integral.copy();self.speed=steady(self.trim,voltage,'physics')['omega'];self.target=self.speed.copy();self.pending=deque();self.time=0.;self.inner_counter=0;self.u=self.trim.copy();self.saturated_steps=0;self.domain_clips=0
        inert=' '.join(map(str,np.diag(self.J)));geoms=''
        for i,r in enumerate(self.r):
            geoms+=f'<geom type="capsule" fromto="0 0 0 {r[0]} {r[1]} 0" size=".006" rgba=".15 .22 .28 1"/><geom type="cylinder" pos="{r[0]} {r[1]} {r[2]}" size=".0635 .001" contype="0" conaffinity="0" rgba=".3 .6 .8 .35"/>'
        self.xml=f'''<mujoco model="raven_candidate"><compiler angle="radian"/><option timestep=".001" gravity="0 0 -9.81" integrator="RK4"/><visual><global offwidth="1280" offheight="720"/></visual><worldbody><light pos="0 0 4"/><geom type="plane" size="5 5 .1" rgba=".85 .88 .9 1"/><body name="raven_COM"><freejoint/><inertial pos="0 0 0" mass="{self.mass}" diaginertia="{inert}"/><geom type="box" size=".055 .022 .014" rgba=".18 .23 .3 1"/>{geoms}</body></worldbody></mujoco>'''
        self.model=mujoco.MjModel.from_xml_string(self.xml);self.data=mujoco.MjData(self.model);self.body=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,'raven_COM');self.reset([0,0,.5])

    def reset(self,position):
        mujoco.mj_resetData(self.model,self.data);self.data.qpos[:3]=position;self.data.qpos[3]=1;self.time=0.;self.inner_counter=0;self.integral=self.initial_integral.copy();self.speed=steady(self.trim,self.voltage,'physics')['omega'];self.target=self.speed.copy();self.u=self.trim.copy();self.pending.clear();mujoco.mj_forward(self.model,self.data)

    def forces(self):
        f=np.maximum(0,np.polyval(S['rotor_laws']['T_N']['quadratic']['coefficients_a2_a1_a0'],self.speed))*self.force_scale
        q=S['simple']['kQ_Nm_per_rad_s_squared']*self.speed**2*self.force_scale
        return f,np.cross(self.r,np.c_[np.zeros((4,2)),f]).sum(0)+[0,0,self.signs@q]

    def observe(self):
        f,_=self.forces();return np.r_[self.data.qpos,self.data.qvel,f.sum()]

    def advance(self,ctbr):
        rc=float(self.map.inverse(np.clip(ctbr[0],*self.map.force_bounds)));base=float(self.map.command(rc));last_rc=rc
        for _ in range(20):
            if self.inner_counter%4==0:
                error=ctbr[1:]-self.data.qvel[3:6];wanted=self.kp*error+self.integral;delta=self.torque_to_command@wanted;span=self.map.u_high-self.map.u_low
                if np.ptp(delta)>span:delta*=span/np.ptp(delta);self.saturated_steps+=1
                shifted=np.clip(base,self.map.u_low-delta.min(),self.map.u_high-delta.max());raw=shifted+delta
                low,high=self.map.support_low,self.map.support_high
                self.domain_clips+=int(np.any(raw<low-1e-6) or np.any(raw>high+1e-6));self.u=np.clip(raw,low,high)
                if np.allclose(raw,self.u,atol=1e-6):self.integral=np.clip(self.integral+self.ki*error*self.inner_dt,-.3,.3)
                self.pending.append((self.time+self.motor_delay,steady(self.u,self.voltage,'physics')['omega']))
            cursor=self.time;end=self.time+self.dt
            while self.pending and self.pending[0][0]<end-1e-12:
                event,target=self.pending.popleft();event=max(cursor,event);self.speed=self.target+(self.speed-self.target)*np.exp(-(event-cursor)/self.motor_tau);cursor=event;self.target=target
            self.speed=self.target+(self.speed-self.target)*np.exp(-(end-cursor)/self.motor_tau)
            force,torque=self.forces();R=self.data.xmat[self.body].reshape(3,3);self.data.xfrc_applied[self.body,:3]=R@np.array([0,0,force.sum()]);self.data.xfrc_applied[self.body,3:]=R@torque;mujoco.mj_step(self.model,self.data);self.time=end;self.inner_counter+=1
        return {'RC_throttle':last_rc,'motor_external_counts':self.u.tolist(),'motor_speed_rad_s':self.speed.tolist(),'force_N':self.forces()[0].tolist(),'moment_body_Nm':self.forces()[1].tolist()}

    def controller_config(self):
        fmin,fmax=self.map.force_bounds;kt=S['simple']['kT_N_per_rad_s_squared'];kq=S['simple']['kQ_Nm_per_rad_s_squared'];B=np.vstack([np.ones(4),self.r[:,1],-self.r[:,0],self.signs*kq/kt]);allocation=np.linalg.inv(B)[:,1:]
        return dict(scope='raven_simulation_only',mass_kg=self.mass,inertia_kg_m2=json.loads((ROOT/'parameters/inertial_model.json').read_text())['I_COM_kg_m2'],rate_prediction='legacy',rate_kv=(np.diag(np.array(json.loads((ROOT/'parameters/inertial_model.json').read_text())['I_COM_kg_m2']))/np.array([.04,.04,.08])).tolist(),torque_limits_Nm=[.6,.5,.12],allocation_per_m=allocation.tolist(),physical_force_range_N=[float(fmin),float(fmax)],torque_reserve_fraction=.7,collective_tau_up_s=.02189140910403713,collective_tau_down_s=.02189140910403713,pitch_limit_rad_s=9.5,maximum_thrust_N=float(fmax*.95),solver_type='SQP_RTI',rti_iterations=2,control_cost=[.001,.03,.03,.03],state_cost=[80,80,120,8,8,12,12,12,12,.1,.1,.1,.001],nodes=20,prediction_dt_s=.04)
