"""Render the recorded native MuJoCo simulation to a standalone MP4."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,subprocess
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from matplotlib import font_manager as fm

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);args=p.parse_args();folder=args.run.resolve();rows=[json.loads(x) for x in (folder/'trace.jsonl').read_text().splitlines()];t=np.array([x['time_s'] for x in rows]);s=np.array([x['state'][:7] for x in rows]);xml=(folder/'raven.xml').read_text().replace('</visual>','</visual><asset><texture type="skybox" builtin="gradient" rgb1=".88 .92 .96" rgb2=".98 .99 1" width="512" height="3072"/></asset>');model=mujoco.MjModel.from_xml_string(xml);model.vis.headlight.ambient[:]=[.7,.7,.7];model.vis.headlight.diffuse[:]=[.7,.7,.7];data=mujoco.MjData(model);renderer=mujoco.Renderer(model,height=720,width=1280);camera=mujoco.MjvCamera();camera.lookat[:]=[0,0,.9];camera.distance=5.2;camera.azimuth=90;camera.elevation=-12
    close=mujoco.MjvCamera();close.distance=1.0;close.azimuth=90;close.elevation=-12;font=ImageFont.truetype(fm.findfont('DejaVu Sans'),20)
    path=folder/'raven_flip.mp4';proc=subprocess.Popen(['ffmpeg','-loglevel','error','-y','-f','rawvideo','-vcodec','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r','30','-i','-','-an','-c:v','libx264','-preset','fast','-crf','18','-pix_fmt','yuv420p',str(path)],stdin=subprocess.PIPE)
    for stamp in np.arange(t[0],t[-1],1/30):
        q=np.array([np.interp(stamp,t,s[:,i]) for i in range(7)]);q[3:]/=np.linalg.norm(q[3:]);data.qpos[:]=q;mujoco.mj_forward(model,data);renderer.update_scene(data,camera=camera);frame=Image.fromarray(renderer.render());close.lookat[:]=data.qpos[:3];renderer.update_scene(data,camera=close);detail=Image.fromarray(renderer.render()).crop((384,168,896,552)).resize((320,240));frame.paste(detail,(936,24));draw=ImageDraw.Draw(frame);draw.rectangle((12,12,700,65),fill='white');draw.text((22,24),f'OFFLINE Raven flip simulation | t={stamp:.2f} s',font=font,fill='black');draw.text((946,270),'Close view',font=font,fill='black');proc.stdin.write(np.asarray(frame).tobytes())
    proc.stdin.close();assert proc.wait()==0;renderer.close();print(path)

if __name__=='__main__':main()
