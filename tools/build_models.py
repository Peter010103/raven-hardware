"""Export standalone simulation assets, mapping predictions and firmware provenance."""
import json,hashlib,shutil,sys
import xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'simulation'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from betaflight import CollectiveMap,ROOT


def main():
    out=ROOT/'build/raven_model';out.mkdir(parents=True,exist_ok=True);records=[];fig,axes=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
    for V,cells in [(10.5,3),(11.1,3),(12.6,3),(14.8,4),(16,4),(16.8,4)]:
        for comp in [0,100]:
            m=CollectiveMap(V,cells,comp);idx=np.linspace(0,len(m.rc_grid)-1,400).astype(int);rc=m.rc_grid[idx];F=m.rc_force[idx];r={'voltage_V':V,'cells':cells,'compensation':comp,'RC_throttle':rc.tolist(),'collective_force_N':F.tolist(),'map':m.info};records.append(r)
            axes[0].plot(rc,F,'-' if not comp else '--',label=f'{V:g}V / {cells}S / comp{comp}');axes[1].plot(F,rc,'-' if not comp else '--')
    axes[0].axhline(.716*9.81,color='black',ls=':',label='716g hover');axes[0].set(xlabel='RC throttle [integer counts]',ylabel='Nominal four-rotor force [N]',title='Supported predictions; missing curve regions unsupported');axes[0].legend(fontsize=7,ncol=2);axes[1].set(xlabel='Requested total force [N]',ylabel='RC throttle [integer counts]',title='MPC inverse: supported branches only');fig.savefig(out/'collective-mapping.svg');fig.savefig(out/'collective-mapping.png',dpi=600);plt.close(fig)
    (out/'mapping_predictions.json').write_text(json.dumps({'scope':'UNVALIDATED AIRCRAFT PREDICTIONS; no fitted collective correction','nominal_collective_correction':1,'curves':records},indent=2)+'\n')
    # Model asset uses the adopted COM as the floating-base origin.
    from plant import RavenPlant
    plant=RavenPlant();(out/'raven.xml').write_text(plant.xml);model=json.loads((ROOT/'parameters/inertial_model.json').read_text());J=np.diag(model['I_COM_kg_m2']);urdf=f'<robot name="raven_candidate"><link name="COM"><inertial><origin xyz="0 0 0"/><mass value="{model["mass_kg"]}"/><inertia ixx="{J[0]}" iyy="{J[1]}" izz="{J[2]}" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="marker_centroid"/><joint name="marker_reference" type="fixed"><parent link="COM"/><child link="marker_centroid"/><origin xyz="'+ ' '.join(map(str,-plant.com))+'"/></joint>'
    for i,r in enumerate(plant.r):urdf+=f'<link name="rotor_{i}"/><joint name="rotor_fixed_{i}" type="fixed"><parent link="COM"/><child link="rotor_{i}"/><origin xyz="'+ ' '.join(map(str,r))+'"/></joint>'
    (out/'raven.urdf').write_text(urdf+'</robot>\n');(out/'mpc_config.json').write_text(json.dumps(plant.controller_config(),indent=2)+'\n')
    manifest={'scope':'SIMULATION CANDIDATE: measured static/dynamic rotor model and adopted suspended-configuration inertia; unknown rate loop','mass_kg':model['mass_kg'],'COM_relative_marker_reference_m':plant.com.tolist(),'root_frame':'Adopted COM, axes Xforward,Yleft,Zup','rotor_centres_relative_COM_m':plant.r.tolist(),'rotor_geometry':'Approximate supplied CAD centres X±68.7435mm/Y±89.69mm; Z30mm display placement, unmeasured. Parallel-Z thrust gives no rZ moment contribution.','rotor_body_yaw_signs_assumed':plant.signs.tolist(),'rate_prior_time_constants_s':plant.rate_tau.tolist(),'motor_speed_tau_s':plant.motor_tau,'motor_speed_delay_s':plant.motor_delay,'hover_trim_external_counts':plant.trim.tolist(),'hover_trim_wrench':[float(x) for x in plant.wrench(plant.trim)],'parameter_SHA256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'parameters/inertial_model.json',ROOT/'parameters/static_models.json',ROOT/'parameters/dynamic_models.json']},'no_battery_sag_identification':True,'installation_and_CW_prop_transfer':'Assumed unity; not empirically fitted','native_XML':'models/raven.xml','URDF':'models/raven.urdf','MPC_config':'parameters/mpc_simulation.json'};(out/'model_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest,indent=2))

    for name in ['raven.xml','raven.urdf']:
        tree=ET.parse(out/name);ET.indent(tree,space='  ');tree.write(out/name,encoding='unicode');
        with (out/name).open('a') as stream:stream.write('\n')
    for name,target in {'raven.xml':'models/raven.xml','raven.urdf':'models/raven.urdf','mpc_config.json':'parameters/mpc_simulation.json','model_manifest.json':'data/provenance/simulation_model.json'}.items():
        destination=ROOT/target;destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(out/name,destination)

if __name__=='__main__':main()
