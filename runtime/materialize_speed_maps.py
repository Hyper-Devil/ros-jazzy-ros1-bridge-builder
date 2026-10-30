#!/usr/bin/env python3
"""Verify shared projections and export fixed-parameter speed comparisons."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patheffects as effects
from matplotlib.patches import Patch
import numpy as np

from export_cvar_figure import PALETTE, UNKNOWN, sha256
from range_fusion import base_in_camera, observation_range, within_range


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-range-m', type=float, default=None,
                        help='Filter each observation before EMA using its image-time base_link pose')
    parser.add_argument('--bounds', type=float, nargs=4)
    args = parser.parse_args()
    source = args.session.resolve()
    if not (source/'result.json').is_file():
        raise RuntimeError('Complete sensor replay before materializing maps')
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError('Use an empty output directory')
    output.mkdir(parents=True, exist_ok=True)
    session = json.loads((source/'session.json').read_text())
    radius = args.max_range_m
    base_camera = None
    if radius is not None:
        within_range([], radius)
        base_camera = base_in_camera(source/'calibration_used/mapping_rs_bistu.launch')
        session = dict(session, mapping_max_range_m=radius,
                       range_reference_frame='base_link', range_metric='horizontal',
                       range_fusion='per_observation_before_ema_at_image_acquisition_pose')
    for key,expected in [('ema_gamma',.2),('support_min',.15),('fusion_resolution_m',.5)]:
        if not np.isclose(session[key],expected):
            raise ValueError('This figure profile expects the selected final parameters: '+key)
    worker = json.loads((source/'prediction_metadata.json').read_text())
    speeds = session['reference_speeds']
    assert worker['reference_speeds'] == speeds
    channels = session['score_channels']
    trace = source/'ema_trace'
    meta = json.loads((trace/'metadata.json').read_text())
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('risk_math', root/'emcp/src/elevation_mapping_cupy/elevation_mapping_cupy/script/elevation_mapping_cupy/risk_mapping.py')
    risk = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(risk)
    states = [np.zeros((3, meta['size'], meta['size']), np.float32) for _ in speeds]
    baseline_states = states if radius is None else [np.zeros_like(s) for s in states]
    totals = [0]*len(speeds)
    unrestricted_totals = [0]*len(speeds)
    range_audit = []
    errors = [0.]*len(speeds)
    keys = {}
    files = sorted(trace.glob('frame_*.npz'))
    if not files or len(files) % len(speeds):
        raise ValueError('Incomplete multi-channel trace')
    geometry = None
    generation = None
    projected_candidates = 0
    geometry_fields = ['candidate_x','candidate_y','candidate_u','candidate_v']
    for index, path in enumerate(files):
        with np.load(path, allow_pickle=False) as archive:
            data = {key: archive[key].copy() for key in archive.files}
        j = int(data['image_channel'])
        assert j == index % len(speeds), 'Channel order changed'
        key = int(data['channel'])
        if j not in keys: keys[j] = key
        assert key == keys[j], 'Semantic state index changed'
        if j == 0:
            geometry = [data[name] for name in geometry_fields]
            projected_candidates += len(geometry[0])
            if generation != int(data['generation']):
                for state in states: state.fill(0)
                if radius is not None:
                    for state in baseline_states: state.fill(0)
                generation = int(data['generation'])
        assert generation == int(data['generation']), 'Generation changed mid-image'
        for name, expected in zip(geometry_fields, geometry):
            np.testing.assert_array_equal(data[name], expected, err_msg='Projection differs: '+name)
        x,y,values = data['x'],data['y'],data['observation']
        assert len(np.unique(x*meta['size']+y)) == len(x), 'Duplicate cell update'
        assert np.isfinite(values).all() and ((values>=0)&(values<=1)).all()
        candidate_ids = data['candidate_x']*meta['size']+data['candidate_y']
        assert np.isin(x*meta['size']+y, candidate_ids).all(), 'Accepted cell outside shared geometry'
        baseline = baseline_states[j]
        baseline[:,x,y] = np.stack(risk.ema_update(*baseline[:,x,y], values, float(meta['baseline_gamma'])))
        np.testing.assert_allclose(baseline[:,x,y], data['posterior'], rtol=1e-5, atol=1e-6,
                                   err_msg='GPU posterior mismatch: '+path.name)
        if len(x): errors[j] = max(errors[j], float(np.abs(baseline[:,x,y]-data['posterior']).max()))
        unrestricted_totals[j] += len(x)
        if radius is not None:
            distance, base_world = observation_range(data, meta, base_camera)
            keep = within_range(distance, radius)
            if j == 0:
                pose = [data['map_to_camera_rotation'], data['map_to_camera_translation']]
            for name, expected in zip(['map_to_camera_rotation','map_to_camera_translation'],pose):
                np.testing.assert_array_equal(data[name],expected)
            range_audit.append(dict(event=index//len(speeds),speed_m_s=speeds[j],
                base_world_xyz_m=base_world.tolist(),before=int(len(x)),accepted=int(keep.sum()),
                max_accepted_distance_m=float(distance[keep].max()) if keep.any() else None,
                min_rejected_distance_m=float(distance[~keep].min()) if (~keep).any() else None))
            x,y,values=x[keep],y[keep],values[keep]
            state=states[j]
            state[:,x,y]=np.stack(risk.ema_update(*state[:,x,y],values,float(meta['baseline_gamma'])))
        totals[j] += len(x)
    assert len(set(keys.values())) == len(speeds), 'Speeds accidentally share statistics'
    trajectory_path = source/'odometry.csv'
    full_trajectory = np.atleast_1d(np.genfromtxt(trajectory_path, delimiter=',', names=True))
    window = session.get('mapping_window_stamp_s')
    if window:
        verification = json.loads((source/'result.json').read_text())['mapping_window_validation']
        assert verification['passed'] and verification['full_bag_playback_and_slam']
        shutil.copy2(trajectory_path,output/'odometry_full.csv')
        with trajectory_path.open(newline='') as stream:
            reader=csv.DictReader(stream); fields=reader.fieldnames
            rows=[row for row in reader if window[0]<=float(row['stamp'])<window[1]]
        trajectory_path=output/'odometry_window.csv'
        with trajectory_path.open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    trajectory = np.atleast_1d(np.genfromtxt(trajectory_path, delimiter=',', names=True))
    xy = np.column_stack((trajectory['x'],trajectory['y']))
    assert len(xy)>1 and np.isfinite(xy).all() and (np.diff(trajectory['stamp'])>0).all()
    knowns = [state[2].T>0 for state in states]
    common = np.logical_and.reduce(knowns)
    union = np.logical_or.reduce(knowns)
    y,x = np.where(union)
    ox,oy = meta['origin']; resolution = meta['resolution']; pad = 3.
    bounds = [min(ox+x.min()*resolution,xy[:,0].min())-pad,
              max(ox+(x.max()+1)*resolution,xy[:,0].max())+pad,
              min(oy+y.min()*resolution,xy[:,1].min())-pad,
              max(oy+(y.max()+1)*resolution,xy[:,1].max())+pad]
    if args.bounds is not None: bounds = args.bounds
    report = dict(schema='shared_projection_speed_comparison_v1', source_session=str(source),
        bag=session['bag'], reference_speeds_m_s=speeds, raw_worker_metadata=worker,
        shared_parameters={key:session[key] for key in ['support_min','ema_gamma','cost_thresholds',
                            'fusion_resolution_m','cost_encoding','pointcloud_topic']},
        cvar_alpha=.95, shared_projection_events=len(files)//len(speeds),
        shared_candidate_observations=projected_candidates, projection_geometry_verified=True,
        gpu_posteriors_verified=True, trajectory_samples=len(xy),
        trajectory_source=str(trajectory_path),trajectory_full_source=str(source/'odometry.csv'),
        trajectory_full_samples=len(full_trajectory),
        trajectory_scope='mapping_window' if window else 'full_bag',
        mapping_window_offsets_s=session.get('mapping_window_offsets_s'),
        mapping_window_stamp_s=window,
        union_observed_cells=int(union.sum()), intersection_observed_cells=int(common.sum()),
        plot_bounds_xy_m=bounds, maps=[],
        mapping_max_range_m=radius,
        range_fusion=session.get('range_fusion'),
        base_link_in_camera_m=None if base_camera is None else base_camera.tolist(),
        notes=['Speeds query the same predicted curve; they do not change rosbag motion.',
               'All speeds share SLAM, rectification, projection and candidate cells in time order.',
               'Each speed independently accepts support >= 0.15; accepted coverage can differ.',
               'No spatial smoothing or gap filling is applied.'])
    if radius is not None:
        report['notes'] += ['The original unrestricted GPU posteriors are verified first.',
            'Each range map starts empty and replays only in-range observations; this is not a crop of fused statistics.',
            'Distance is measured from world-cell center to image-time base_link, using the same camera TF as projection.']
        (output/'range_audit.json').write_text(json.dumps(range_audit,indent=2)+'\n')
    costs = []
    for j,(speed,state,known) in enumerate(zip(speeds,states,knowns)):
        folder = output/('v_%04d'%round(speed*1000))
        stats = folder/'ema_sweep/gamma_200'
        stats.mkdir(parents=True)
        child_manifest = dict(session, output=str(folder),reference_speed=speed,
                              source_common_session=str(source),query_index=j,
                              trajectory_scope='mapping_window' if window else 'full_bag',
                              ema_trace_dir=str(trace),stopped=True)
        child_worker = dict(worker,reference_speed=speed,query_index=j,
                            source_worker_reference_speed=worker['reference_speed'])
        (folder/'session.json').write_text(json.dumps(child_manifest,indent=2)+'\n')
        (folder/'prediction_metadata.json').write_text(json.dumps(child_worker,indent=2)+'\n')
        shutil.copy2(trajectory_path,folder/'odometry.csv')
        mean,variance,count = state.transpose(0,2,1)
        for alpha in [.95,.05]:
            np.savez_compressed(stats/('statistics_alpha_%03d.npz'%round(alpha*100)),
                score_mean=mean,cost_mean=1-mean,variance=variance,count=count,valid=known,
                resolution=resolution,origin=meta['origin'],map_frame='map',cvar_alpha=alpha,
                ema_gamma=session['ema_gamma'],reference_speed=speed,cost_encoding='five_level',
                support_min=session['support_min'],cost_thresholds=session['cost_thresholds'],
                mapping_window_offsets_s=session.get('mapping_window_offsets_s',[]),
                mapping_max_range_m=np.nan if radius is None else radius,
                schema='paper_ema_cvar_v1')
        q = np.where(known,mean-risk.cvar_coefficient(.95)*np.sqrt(np.maximum(variance,0)),np.nan)
        cost = risk.score_to_cost(q,'five_level',session['cost_thresholds'])
        costs.append(cost)
        report['maps'].append(dict(reference_speed_m_s=speed,channel=channels[j],
            observed_cells=int(known.sum()),area_m2=float(known.sum()*resolution**2),
            accepted_cell_observations=totals[j],max_gpu_posterior_abs_error=errors[j],
            unrestricted_cell_observations=unrestricted_totals[j],
            mean_cost_on_intersection=float(cost[common].mean()) if common.any() else None,
            cost_percentages={str(v):float((cost[known]==v).mean()*100) for v in PALETTE},
            intersection_cost_percentages={str(v):float((cost[common]==v).mean()*100) for v in PALETTE},
            session=str(folder)))
        subprocess.run([sys.executable,str(Path(__file__).with_name('export_cvar_figure.py')),str(folder),
            '--output',str(folder/'figures'),'--alpha','0.95','--plain-map-only','--show-speed',
            '--bounds',*map(str,bounds)],check=True)
    report['cost_changes_on_intersection'] = [dict(from_speed_m_s=speeds[i],to_speed_m_s=speeds[i+1],
        increased_percent=float((costs[i+1][common]>costs[i][common]).mean()*100),
        decreased_percent=float((costs[i+1][common]<costs[i][common]).mean()*100),
        unchanged_percent=float((costs[i+1][common]==costs[i][common]).mean()*100))
        for i in range(len(speeds)-1)]
    extent = [ox,ox+meta['size']*resolution,oy,oy+meta['size']*resolution]
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':13,'pdf.fonttype':42})
    legend = [Patch(facecolor=np.full(3,gray/255.),edgecolor='.4',label=str(value))
              for value,gray in PALETTE.items()]
    legend += [Patch(facecolor=np.array(UNKNOWN)/255.,edgecolor='.4',label='Unknown')]
    figures = []
    for annotated in [False,True]:
        fig,axes = plt.subplots(1,len(speeds),figsize=(24,8.2))
        fig.subplots_adjust(left=.012,right=.988,bottom=.13,top=.9,wspace=.025)
        for ax,speed,cost in zip(axes,speeds,costs):
            rgb = np.full((*cost.shape,3),UNKNOWN,np.uint8)
            for value,gray in PALETTE.items(): rgb[cost==value] = gray
            ax.imshow(rgb,origin='lower',extent=extent,interpolation='nearest')
            ax.set_xlim(bounds[:2]); ax.set_ylim(bounds[2:]); ax.set_aspect('equal'); ax.set_axis_off()
            ax.set_title('v = %g m/s'%speed,fontsize=22,pad=14)
            if annotated:
                line, = ax.plot(xy[:,0],xy[:,1],color='#00cfd3',linewidth=1.5,zorder=4)
                line.set_path_effects([effects.Stroke(linewidth=2.7,foreground='white'),effects.Normal()])
                ax.plot(*xy[0],marker='o',markersize=8,color='#169b62',markeredgecolor='white',zorder=6)
                ax.plot(*xy[-1],marker='*',markersize=12,color='#df4035',markeredgecolor='white',zorder=6)
                for point,label,offset in [(xy[0],'Start',(0,20)),(xy[-1],'End',(14,17))]:
                    text = ax.annotate(label,xy=point,xytext=offset,textcoords='offset points',
                        arrowprops=dict(arrowstyle='-',color='.2',linewidth=.7,shrinkB=7),zorder=7)
                    text.set_path_effects([effects.Stroke(linewidth=3,foreground='white'),effects.Normal()])
            xb,yb = bounds[0]+2,bounds[2]+1.5
            ax.plot([xb,xb+10],[yb,yb],color='.15',linewidth=2)
            ax.text(xb+5,yb+.6,'10 m',ha='center',fontsize=12)
        title=r'$\alpha=0.95$; EMA $\gamma=0.2$; support $\geq0.15$; resolution $0.5$ m'
        if window:title += '; t=%g–%g s'%tuple(session['mapping_window_offsets_s'])
        if radius is not None:title += '; range <= %g m'%radius
        fig.suptitle(title,fontsize=19,y=.98)
        fig.legend(handles=legend,title='Cost',loc='lower center',ncol=6,frameon=False,bbox_to_anchor=(.5,.005))
        name = 'speed_comparison_'+('trajectory' if annotated else 'plain')+'_300dpi'
        for suffix in ['png','pdf']:
            path = output/(name+'.'+suffix)
            fig.savefig(path,dpi=300,facecolor='white')
            figures.append(dict(path=str(path),sha256=sha256(path)))
        plt.close(fig)
    report['figures'] = figures
    (output/'comparison_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
