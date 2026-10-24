"""Offline source-formula, inverse, hover-trim and native rigid-body checks."""
import json,ctypes,subprocess,sys,hashlib
from pathlib import Path
import numpy as np
from betaflight import CollectiveMap,ROOT
from plant import RavenPlant


def function(source,name):
    start=source.rfind('\n',0,source.index(name+'('))+1;brace=source.index('{',start);depth=1;end=brace+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]


def main():
    out=ROOT/'build/raven_validation';out.mkdir(parents=True,exist_ok=True);p=ROOT/'build/betaflight-4.5.5';meta=json.loads((ROOT/'simulation/firmware_reference.json').read_text())
    for r in meta['files']:
        f=p/r['path'].replace('src/main/','').replace('/','_');assert hashlib.sha256(f.read_bytes()).hexdigest()==r['SHA256']
    src=(p/'drivers_dshot.c').read_text();idle=function((p/'drivers_motor.c').read_text(),'getDigitalIdleOffset')
    prefix='''#include <math.h>
#include <stdint.h>
#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MIDDLE 1500
#define PWM_RANGE_MAX 2000
#define DSHOT_MIN_THROTTLE 48
#define DSHOT_MAX_THROTTLE 2047
#define DSHOT_CMD_MOTOR_STOP 0
#define DSHOT_3D_FORWARD_MIN_THROTTLE 1048
#define DSHOT_RANGE 1999
#define FEATURE_3D 1
#define featureIsEnabled(x) 0
#define CONVERT_PARAMETER_TO_PERCENT(x) ((x)*.01f)
#define constrain(x,l,h) ((x)<(l)?(l):((x)>(h)?(h):(x)))
typedef struct { int digitalIdleOffsetValue; } motorConfig_t;
float scaleRangef(float x,float a,float b,float c,float d){return c+(x-a)*(d-c)/(b-a);}
'''
    first=src.index('void dshotInitEndpoints');brace=src.index('{',first);depth=1;end=brace+1
    while depth:depth+=(src[end]=='{')-(src[end]=='}');end+=1
    prefix+='''
#define MAX(a,b) ((a)>(b)?(a):(b))
#define power5(x) ((x)*(x)*(x)*(x)*(x))
typedef struct { uint8_t rcExpo[3], rcRates[3], rates[3]; } rates_t;
static rates_t rates={{0,0,0},{60,60,60},{60,60,60}};
static rates_t *currentControlRateProfile=&rates;
'''
    rate_source=function((p/'fc_rc.c').read_text(),'applyActualRates')
    c=prefix+rate_source+'\n'+idle+'\n'+src[first:end]+'\n'+function(src,'dshotConvertFromExternal')+'\n'+function(src,'dshotConvertToExternal')+'''
float reference_map(int rc,float V,int cells,int comp){
 motorConfig_t cfg={550};float lo,hi,dis,a,b;
 dshotInitEndpoints(&cfg,1.f,&lo,&hi,&dis,&a,&b);
 uint16_t cell_cv=(uint16_t)lrintf(V*100.f)/cells;
 float good=1.f-constrain((420.f-cell_cv)/70.f,0.f,1.f);
 hi-=(.7f/4.2f)*good*(comp/100.f)*(hi-lo);
 int x=(constrain(rc,1050,2000)-1050)*1000/950;
 float m=lo+(hi-lo)*x/1000.f;
 return 1001.f+(m-48.f)*999.f/1999.f;
}
''';(out/'reference.c').write_text(c);subprocess.run(['cc','-shared','-O2','-fPIC',str(out/'reference.c'),'-o',str(out/'reference.so'),'-lm'],check=True);lib=ctypes.CDLL(str(out/'reference.so'));lib.reference_map.argtypes=[ctypes.c_int,ctypes.c_float,ctypes.c_int,ctypes.c_int];lib.reference_map.restype=ctypes.c_float;maxerr=0.;inverse_error=0.
    lib.applyActualRates.argtypes=[ctypes.c_int,ctypes.c_float,ctypes.c_float];lib.applyActualRates.restype=ctypes.c_float
    for axis in range(3):
        for x in np.linspace(-1,1,41):assert abs(lib.applyActualRates(axis,float(x),abs(float(x)))-600*x)<.0001
    for V,n in [(10.5,3),(11.1,3),(12.6,3),(14.8,4),(16.,4),(16.8,4),(14.87,4),(15.13,4),(12.10,3)]:
        for comp in [0,100]:
            m=CollectiveMap(V,n,comp);rc=np.arange(1050,2001,19);expected=np.array([lib.reference_map(int(r),V,n,comp) for r in rc]);maxerr=max(maxerr,float(max(abs(expected-m.command(rc)))));F=np.linspace(*m.force_bounds,301);pred=m.force(m.inverse(F));err=float(max(abs(pred-F)));assert err<=max(np.diff(m.rc_force))/2+1e-7;inverse_error=max(inverse_error,err)
            try:m.inverse(m.force_bounds[1]+.01)
            except ValueError:pass
            else:raise AssertionError('Unsupported inverse accepted')
    assert maxerr<.0004
    plant=RavenPlant();np.testing.assert_allclose(plant.wrench(plant.trim),[plant.mass*9.81,0,0,0],atol=1e-6);assert plant.model.body_mass[plant.body]==plant.mass;np.testing.assert_allclose(plant.model.body_inertia[plant.body],np.diag(plant.J),rtol=1e-12)
    # Independent native torque -> acceleration check about the adopted COM.
    import mujoco
    plant.data.xfrc_applied[plant.body]=[0,0,plant.mass*9.81,.01,.02,.03];mujoco.mj_forward(plant.model,plant.data);np.testing.assert_allclose(plant.data.qacc[3:6],[.01,.02,.03]/np.diag(plant.J),atol=1e-10)
    result={'status':'passed','firmware_reference_commit':meta['commit'],'source_hashes_verified':len(meta['files']),'C_reference_external_count_max_error':maxerr,'inverse_roundtrip_max_force_error_N':inverse_error,'inverse_error_interpretation':'Bounded by one half of largest adjacent integer-RC force increment; not model-identification error','ACTUAL_rate_mapping_600deg_s_linear_checked_against_C':True,'native_mass_inertia_and_torque_acceleration':True,'hover_trim_force_moment':plant.wrench(plant.trim).tolist(),'scope':'Source-derived restricted steady mapping and numeric plant checks; not an installed-firmware or closed-loop flight validation'};(out/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':main()
