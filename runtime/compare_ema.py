#!/usr/bin/env python3
"""Replay identical recorded world-cell observations through several EMA gains."""
import argparse
import importlib.util
import json
from pathlib import Path
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    parser.add_argument('--wait',action='store_true')
    parser.add_argument('--gammas',type=float,nargs='+',default=[.05,.1,.2])
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if any(not 0<value<1 for value in args.gammas):parser.error('All EMA gains must be in (0,1)')
    deadline=time.monotonic()+1200
    while not (args.session/'result.json').exists():
        if not args.wait:raise RuntimeError('Wait for sensor replay to complete first')
        if time.monotonic()>deadline:raise TimeoutError('Sensor replay did not complete')
        time.sleep(2)
    session=json.loads((args.session/'session.json').read_text())
    trace=args.session/'ema_trace'
    meta=json.loads((trace/'metadata.json').read_text())
    root=Path(__file__).resolve().parents[2]
    spec=importlib.util.spec_from_file_location('risk_math',root/'emcp/src/elevation_mapping_cupy/elevation_mapping_cupy/script/elevation_mapping_cupy/risk_mapping.py')
    risk=importlib.util.module_from_spec(spec);spec.loader.exec_module(risk)
    gammas=list(dict.fromkeys(args.gammas))
    live_gamma=float(meta['baseline_gamma'])
    baseline=next((g for g in gammas if abs(g-live_gamma)<1e-7),live_gamma)
    if baseline not in gammas:gammas.append(baseline)
    states={gamma:np.zeros((3,meta['size'],meta['size']),dtype=np.float32) for gamma in gammas}
    generation=None;channel=None;max_error=0.;observations=0
    files=sorted(trace.glob('frame_*.npz'))
    if not files:raise RuntimeError('No EMA observations recorded')
    for file in files:
        with np.load(file,allow_pickle=False) as data:
            if channel is None:channel=int(data['channel'])
            assert channel==int(data['channel']),'Expected one conservative score channel'
            if generation != int(data['generation']):
                for state in states.values():state.fill(0)
                generation=int(data['generation'])
            x,y,values=data['x'],data['y'],data['observation']
            assert len(np.unique(x*meta['size']+y))==len(x),'Duplicate cell updates within a frame'
            observations+=len(x)
            for gamma,state in states.items():
                effective_gamma=live_gamma if gamma==baseline else float(np.float32(gamma))
                state[:,x,y]=np.stack(risk.ema_update(*state[:,x,y],values,effective_gamma))
            actual=states[baseline][:,x,y]
            np.testing.assert_allclose(actual,data['posterior'],rtol=1e-5,atol=1e-6,
                err_msg='Offline replay differs from live GPU EMA at '+file.name)
            if actual.size:max_error=max(max_error,float(np.abs(actual-data['posterior']).max()))
    output=args.output or args.session/'ema_sweep';output.mkdir(parents=True,exist_ok=True)
    report=dict(schema='controlled_ema_sweep_v1',source_session=str(args.session),
        projected_image_events=len(files),projected_cell_observations=observations,
        baseline_gamma=baseline,baseline_gamma_live=live_gamma,
        baseline_gpu_posteriors_verified=True,max_baseline_abs_error=max_error,
        support_min=session.get('support_min',0.),
        cost_thresholds=session['cost_thresholds'],comparisons=[],
        notes=['Every gamma uses exactly the same projected samples in the same order.',
               'Final full-world states may differ from periodically copied local GridMap exports.'])
    valid=states[baseline][2].T>0
    y,x=np.where(valid);pad=3
    crop=(slice(max(0,y.min()-pad),min(valid.shape[0],y.max()+pad+1)),
          slice(max(0,x.min()-pad),min(valid.shape[1],x.max()+pad+1)))
    palette={0:255,30:202,50:166,70:131,100:77}
    fig,axes=plt.subplots(2,len(gammas),figsize=(5*len(gammas),8),layout='constrained',squeeze=False)
    for col,gamma in enumerate(gammas):
        mean,variance,count=states[gamma].transpose(0,2,1)
        known=count>0
        np.testing.assert_array_equal(known,valid)
        folder=output/('gamma_%03d'%round(gamma*1000));folder.mkdir(exist_ok=True)
        for row,alpha in enumerate([.95,.05]):
            q=np.where(known,mean-risk.cvar_coefficient(alpha)*np.sqrt(np.maximum(variance,0)),np.nan)
            cost=risk.score_to_cost(q,'five_level',session['cost_thresholds'])
            fractions={str(value):float((cost[known]==value).mean()*100) for value in palette}
            report['comparisons'].append(dict(gamma=gamma,alpha=alpha,cost_percentages=fractions,
                median_observation_sigma=float(np.median(np.sqrt(variance[known]))),
                median_mean_q=float(np.median(mean[known])),observed_cells=int(known.sum())))
            np.savez_compressed(folder/('statistics_alpha_%03d.npz'%round(alpha*100)),
                score_mean=mean,cost_mean=1-mean,variance=variance,count=count,valid=known,
                resolution=meta['resolution'],origin=meta['origin'],map_frame='map',cvar_alpha=alpha,
                ema_gamma=gamma,reference_speed=session['reference_speed'],cost_encoding='five_level',
                support_min=session.get('support_min',0.),
                cost_thresholds=session['cost_thresholds'],schema='paper_ema_cvar_v1')
            rgb=np.full((*cost.shape,3),(155,173,171),dtype=np.uint8)
            for value,gray in palette.items():rgb[cost==value]=gray
            axes[row,col].imshow(rgb[crop],origin='lower',interpolation='nearest')
            axes[row,col].set_axis_off()
            axes[row,col].set_title('gamma=%g; alpha=%.2f\nfree=%.1f%%, cost 100=%.1f%%'%
                                    (gamma,alpha,fractions['0'],fractions['100']))
    fig.suptitle('Identical projected observations; thresholds '+str(session['cost_thresholds']))
    fig.legend(handles=[Patch(facecolor=np.full(3,gray/255.),edgecolor='.4',label=str(cost))
                        for cost,gray in palette.items()],title='Cost',loc='outside lower center',ncol=5,frameon=False)
    for suffix in ['png','pdf']:fig.savefig(output/('ema_comparison.'+suffix),dpi=180)
    plt.close(fig)
    (output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
