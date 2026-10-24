#!/usr/bin/env python3
"""Rotate/zoom the supported direct-command/voltage thrust surface offline."""
import argparse,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from raven_actuator import surface,HULL

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--save',type=Path);args=p.parse_args()
 if args.save:
  import matplotlib
  matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 u,V=np.meshgrid(np.linspace(1050,2000,80),np.linspace(9.89,16.94,70));T=surface('T_N',u,V);T[HULL.find_simplex(np.c_[u.ravel(),V.ravel()]).reshape(u.shape)<0]=np.nan
 f=plt.figure(figsize=(10,7));a=f.add_subplot(projection='3d');s=a.plot_surface(u,V,T,cmap='viridis',linewidth=0,alpha=.9);a.set(xlabel='Direct MSP motor count',ylabel='Delivered FC voltage [V]',zlabel='One-rotor thrust [N]',title='Raven ACW: supported command–voltage surface');a.view_init(25,-125);f.colorbar(s,ax=a,shrink=.6,label='Thrust [N]')
 if args.save:args.save.parent.mkdir(parents=True,exist_ok=True);f.savefig(args.save,dpi=600,bbox_inches='tight');print(args.save)
 else:plt.show()
if __name__=='__main__':main()
