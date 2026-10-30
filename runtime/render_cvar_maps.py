#!/usr/bin/env python3
"""Render aligned CVaR map panels from saved statistics without re-fusing data."""
import argparse
import importlib.util
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    source=root/'emcp/src/elevation_mapping_cupy/elevation_mapping_cupy/script/elevation_mapping_cupy/risk_mapping.py'
    spec=importlib.util.spec_from_file_location('risk_math',source)
    risk=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(risk)
    panels=[]
    for suffix in ['095','005']:
        with np.load(args.session/('statistics_alpha_'+suffix+'.npz')) as data:
            alpha=float(data['cvar_alpha'])
            score=data['score_mean']-risk.cvar_coefficient(alpha)*np.sqrt(np.maximum(data['variance'],0))
            score=np.where(data['valid'],score,np.nan)
            encoding=str(data['cost_encoding']) if 'cost_encoding' in data else 'continuous'
            thresholds=data['cost_thresholds'] if 'cost_thresholds' in data else (.3,.5,.7,.9)
            cost=risk.score_to_cost(score,encoding,thresholds)
            resolution=float(data['resolution'])
            speed=float(data['reference_speed'])
            panels.append((alpha,cost))
    known=(panels[0][1]>=0)|(panels[1][1]>=0)
    y,x=np.where(known)
    if len(x)==0: raise ValueError('No observed cells to render')
    padding=max(2,int(round(1./resolution)))
    crop=(slice(max(0,y.min()-padding),min(known.shape[0],y.max()+padding+1)),
          slice(max(0,x.min()-padding),min(known.shape[1],x.max()+padding+1)))
    # Gray levels match the legacy RViz Map display with alpha 0.7.
    palette={0:255,30:202,50:166,70:131,100:77}
    fig,axes=plt.subplots(2,1,figsize=(10,8),layout='constrained')
    for axis,(alpha,cost) in zip(axes,panels):
        cost=cost[crop]
        rgb=np.full((*cost.shape,3),(155,173,171),dtype=np.uint8)
        if encoding=='five_level':
            for value,gray in palette.items(): rgb[cost==value]=gray
        else:
            valid=cost>=0
            rgb[valid]=np.rint(255*(1-cost[valid]/100.))[:,None].astype(np.uint8)
        axis.imshow(rgb,origin='lower',interpolation='nearest')
        axis.set_axis_off()
        axis.text(.98,.96,rf'$\alpha={alpha:.2f}$',transform=axis.transAxes,
                  ha='right',va='top',fontsize=22)
    if encoding=='five_level':
        handles=[Patch(facecolor=np.full(3,gray/255.),edgecolor='.4',label=str(cost))
                 for cost,gray in palette.items()]
        fig.legend(handles=handles,title='Cost',loc='outside lower center',ncol=5,frameon=False)
    fig.suptitle(f'{resolution:g} m/cell; reference speed {speed:g} m/s',fontsize=12)
    for suffix in ['png','pdf']:
        path=args.session/('cvar_comparison.'+suffix)
        fig.savefig(path,dpi=200)
        print(path)
    plt.close(fig)


if __name__=='__main__': main()
