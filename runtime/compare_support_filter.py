#!/usr/bin/env python3
"""Compare completed support-filter experiments using full-world EMA states."""
import argparse
import importlib.util
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np


def load_state(folder):
    with np.load(folder/'statistics_alpha_095.npz', allow_pickle=False) as data:
        return {key: data[key].copy() for key in data.files}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline',type=Path,help='Folder containing baseline world statistics')
    parser.add_argument('filtered',type=Path,help='Folder containing filtered world statistics')
    parser.add_argument('--extra-filtered',type=Path,action='append',default=[],
                        help='Additional completed filters to display alongside the primary comparison')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    spec=importlib.util.spec_from_file_location('risk_math',root/'emcp/src/elevation_mapping_cupy/elevation_mapping_cupy/script/elevation_mapping_cupy/risk_mapping.py')
    risk=importlib.util.module_from_spec(spec);spec.loader.exec_module(risk)
    states=[load_state(folder) for folder in [args.baseline,args.filtered,*args.extra_filtered]]
    for state in states[1:]:
        for key in ['resolution','origin','ema_gamma','reference_speed','cost_thresholds']:
            np.testing.assert_array_equal(states[0][key],state[key],err_msg='Different '+key)
        assert states[0]['count'].shape==state['count'].shape
    resolution=float(states[0]['resolution'])
    masks=[state['count']>0 for state in states]
    common=masks[0]&masks[1]
    union=np.logical_or.reduce(masks)
    y,x=np.where(union)
    if not len(x):raise ValueError('No observed world cells')
    pad=4
    crop=(slice(max(0,y.min()-pad),min(union.shape[0],y.max()+pad+1)),
          slice(max(0,x.min()-pad),min(union.shape[1],x.max()+pad+1)))
    report=dict(schema='support_filter_comparison_v1',baseline=str(args.baseline),filtered=str(args.filtered),
        additional_filtered=[str(folder) for folder in args.extra_filtered],
        resolution_m=resolution,ema_gamma=float(states[0]['ema_gamma']),cost_thresholds=states[0]['cost_thresholds'].tolist(),
        baseline_known_cells=int(masks[0].sum()),filtered_known_cells=int(masks[1].sum()),
        common_known_cells=int(common.sum()),baseline_only_cells=int((masks[0]&~masks[1]).sum()),
        filtered_only_cells=int((masks[1]&~masks[0]).sum()),
        known_coverage_change_percent=float(100*(masks[1].sum()/masks[0].sum()-1)),
        comparisons=[],notes=[
            'All states use identical EMA gamma and cost thresholds; no cost value is reassigned for display.',
            'Statistics use all observed world cells, not periodically exported local GridMap caches.',
            'Separate bag replays may produce different projection events or SLAM geometry. This is not a paired projection experiment.',
            'Rejected support samples do not update mean, variance or count. Never-accepted cells are unknown, not free.'
        ])
    palette={0:255,30:202,50:166,70:131,100:77}
    fig,axes=plt.subplots(2,len(states),figsize=(6.5*len(states),8),layout='constrained')
    costmaps={}
    for col,(state,known) in enumerate(zip(states,masks)):
        threshold=float(state.get('support_min',0.))
        mean,var=state['score_mean'],state['variance']
        pair_common=masks[0]&known
        for row,alpha in enumerate([.95,.05]):
            q=np.where(known,mean-risk.cvar_coefficient(alpha)*np.sqrt(np.maximum(var,0)),np.nan)
            cost=risk.score_to_cost(q,'five_level',state['cost_thresholds'])
            costmaps[col,alpha]=cost
            n=int(known.sum());nblack=int((cost==100).sum())
            item=dict(support_min=threshold,alpha=alpha,known_cells=n,known_area_m2=n*resolution**2,
                projected_observations=int(state['count'].sum()),cost100_cells=nblack,
                cost100_area_m2=nblack*resolution**2,
                cost_percentages={str(c):float(100*np.mean(cost[known]==c)) for c in palette},
                median_mean_q=float(np.median(mean[known])),median_sigma=float(np.median(np.sqrt(var[known]))),
                common_known_cells=int(pair_common.sum()),
                common_cells_cost100_percent=float(100*np.mean(cost[pair_common]==100)) if pair_common.any() else None)
            report['comparisons'].append(item)
            rgb=np.full((*cost.shape,3),(155,173,171),dtype=np.uint8)
            for c,g in palette.items():rgb[cost==c]=g
            ax=axes[row,col]
            ax.imshow(rgb[crop],origin='lower',interpolation='nearest');ax.set_axis_off()
            title='No support filter' if threshold==0 else 'Accept support >= %g'%threshold
            ax.set_title('%s; alpha=%.2f\nknown=%d; cost 100=%.2f%%'%(title,alpha,n,item['cost_percentages']['100']),fontsize=12)
    old_dark=costmaps[0,.95]==100
    new_dark=costmaps[1,.95]==100
    report['alpha095_cost100_transitions']=dict(
        old_dark_no_longer_observed=int((old_dark&~masks[1]).sum()),
        old_dark_still_observed_but_lower_cost=int((old_dark&masks[1]&~new_dark).sum()),
        old_dark_still_dark=int((old_dark&new_dark).sum()),
        new_dark_previously_not_dark=int((new_dark&~old_dark).sum()))
    fig.suptitle('Support filtering before EMA; gamma=%g; same cost thresholds'%float(states[0]['ema_gamma']),fontsize=14)
    handles=[Patch(facecolor=np.full(3,g/255.),edgecolor='.4',label=str(c)) for c,g in palette.items()]
    handles.append(Patch(facecolor=np.array([155,173,171])/255,edgecolor='.4',label='Unknown'))
    fig.legend(handles=handles,title='Cost',loc='outside lower center',ncol=6,frameon=False)
    args.output.mkdir(parents=True,exist_ok=True)
    for suffix in ['png','pdf']:fig.savefig(args.output/('support_comparison.'+suffix),dpi=180)
    plt.close(fig)
    (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
