#!/usr/bin/env python3
"""Compare output thresholds/alpha on saved statistics; EMA is not re-estimated."""
import argparse
import importlib.util
import json
import math
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
    spec=importlib.util.spec_from_file_location('risk_math',root/'emcp/src/elevation_mapping_cupy/elevation_mapping_cupy/script/elevation_mapping_cupy/risk_mapping.py')
    risk=importlib.util.module_from_spec(spec);spec.loader.exec_module(risk)
    output=args.session/'tuning';output.mkdir(exist_ok=True)
    with np.load(args.session/'statistics_alpha_095.npz') as data:
        mean=data['score_mean'].copy();variance=data['variance'].copy();valid=data['valid'].copy()
        count=data['count'][valid].copy();resolution=float(data['resolution']);gamma=float(data['ema_gamma'])
    sigma=np.sqrt(np.maximum(variance,0))
    quantiles=[0,.05,.25,.5,.75,.9,.95,1]
    report=dict(source_session=str(args.session),ema_gamma_observed=gamma,resolution=resolution,
        quantile_probabilities=quantiles,
        distributions={name:np.quantile(values,quantiles).tolist() for name,values in
                       [('mean_q',mean[valid]),('observation_sigma',sigma[valid]),('count',count)]},
        ema_suggestions=[dict(gamma=g,half_life_observations=math.log(.5)/math.log(1-g)) for g in [.05,.1,.2]],
        notes=['Threshold/alpha comparisons reuse the same final mean/variance/count.',
               'No alternate EMA gamma was run; it requires the ordered projected observations.',
               'These thresholds are exploratory; no ground-truth safety calibration is implied.'],
        comparisons=[])
    thresholds=[('Original',[.3,.5,.7,.9]),('Candidate A',[.2,.35,.5,.65]),('Candidate B',[.15,.3,.45,.6])]
    palette={0:255,30:202,50:166,70:131,100:77}
    y,x=np.where(valid);pad=3
    crop=(slice(max(0,y.min()-pad),min(valid.shape[0],y.max()+pad+1)),
          slice(max(0,x.min()-pad),min(valid.shape[1],x.max()+pad+1)))
    fig,axes=plt.subplots(2,3,figsize=(16,8),layout='constrained')
    for row,alpha in enumerate([.95,.05]):
        q=np.where(valid,mean-risk.cvar_coefficient(alpha)*sigma,np.nan)
        for col,(name,edges) in enumerate(thresholds):
            cost=risk.score_to_cost(q,'five_level',edges)
            percentages={str(value):float((cost[valid]==value).mean()*100) for value in palette}
            report['comparisons'].append(dict(alpha=alpha,thresholds=edges,percentages=percentages,
                coefficient=risk.cvar_coefficient(alpha)))
            image=np.full((*cost.shape,3),(155,173,171),dtype=np.uint8)
            for value,gray in palette.items():image[cost==value]=gray
            axis=axes[row,col]
            axis.imshow(image[crop],origin='lower',interpolation='nearest');axis.set_axis_off()
            axis.set_title('%s; alpha=%.2f\n%s\nfree=%.1f%%, cost 100=%.1f%%'%
                (name,alpha,', '.join('%g'%v for v in edges),percentages['0'],percentages['100']),fontsize=11)
    handles=[Patch(facecolor=np.full(3,gray/255.),edgecolor='.4',label=str(cost)) for cost,gray in palette.items()]
    fig.legend(handles=handles,title='Cost (same scale in every panel)',loc='outside lower center',ncol=5,frameon=False)
    fig.suptitle('Threshold comparison at fixed EMA gamma=%g, %.1f m/cell'%(gamma,resolution))
    fig.savefig(output/'threshold_comparison.png',dpi=180);fig.savefig(output/'threshold_comparison.pdf');plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(15,4),layout='constrained')
    axes[0].hist(mean[valid],bins=40,range=(0,1),density=True,histtype='step',label='EMA mean q')
    for alpha in [.05,.95]:
        q=np.clip(mean[valid]-risk.cvar_coefficient(alpha)*sigma[valid],0,1)
        axes[0].hist(q,bins=40,range=(0,1),density=True,histtype='step',label='CVaR q, alpha=%.2f'%alpha)
    axes[0].axvline(.9,color='k',ls='--',lw=1,label='old free threshold')
    axes[0].set(xlabel='Conservative score',ylabel='Density');axes[0].legend(fontsize=8)
    axes[1].hist(sigma[valid],bins=35,color='#5885a5');axes[1].set(xlabel='Observation sigma',ylabel='Observed cells')
    age=np.arange(40)
    for g in [.05,.1,.2]:axes[2].plot(age,g*(1-g)**age,label='gamma=%g'%g)
    axes[2].set(xlabel='Valid observations ago',ylabel='EMA weight');axes[2].legend()
    fig.savefig(output/'distributions_and_ema.png',dpi=180);plt.close(fig)
    # Historical +sigma behavior is diagnostic only, not a candidate mapping.
    report['historical_sign_diagnostic']={}
    for alpha in [.05,.95]:
        score=mean[valid]+risk.cvar_coefficient(alpha)*sigma[valid]
        report['historical_sign_diagnostic'][str(alpha)]=dict(free_fraction_percent=float((score>=.9).mean()*100))
    (output/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(output=str(output),distributions=report['distributions'],comparisons=report['comparisons']),indent=2))


if __name__=='__main__':main()
