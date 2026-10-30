#!/usr/bin/env python3
"""Rebuild a single-speed map with an acquisition-time range gate and two alphas."""
import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np

from range_fusion import base_in_camera, observation_range, within_range


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-range-m', type=float, required=True)
    parser.add_argument('--alpha', type=float, choices=[.05, .95],
                        help='Export only this alpha; both statistics remain available for validation')
    args = parser.parse_args()
    within_range([], args.max_range_m)
    source, output = args.session.resolve(), args.output.resolve()
    result = json.loads((source/'result.json').read_text())
    if not result['passed']:
        raise ValueError('Complete and validate the full sensor replay first')
    session = json.loads((source/'session.json').read_text())
    if session.get('reference_speeds'):
        raise ValueError('Use materialize_speed_maps.py for multi-speed traces')
    if output.exists() and any(output.iterdir()):
        raise FileExistsError('Use an empty output directory')
    output.mkdir(parents=True, exist_ok=True)
    worker = json.loads((source/'prediction_metadata.json').read_text())
    assert worker['reference_speed'] == session['reference_speed']
    trace = source/'ema_trace'
    meta = json.loads((trace/'metadata.json').read_text())
    # ROS/C++ stores these parameters as float32 before the trace serializes
    # them; the controller JSON retains the nominal Python float values.
    np.testing.assert_allclose(meta['baseline_gamma'], session['ema_gamma'], rtol=1e-7, atol=1e-8)
    np.testing.assert_allclose(meta['resolution'], session['fusion_resolution_m'], rtol=1e-7, atol=1e-8)
    base_camera = base_in_camera(source/'calibration_used/mapping_rs_bistu.launch')
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('risk_math', root/
        'emcp/src/elevation_mapping_cupy/elevation_mapping_cupy/script/elevation_mapping_cupy/risk_mapping.py')
    risk = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(risk)
    baseline = np.zeros((3, meta['size'], meta['size']), np.float32)
    filtered = np.zeros_like(baseline)
    files = sorted(trace.glob('frame_*.npz'))
    if not files:
        raise ValueError('No projected observations were recorded')
    generation, channel, audit = None, None, []
    max_error = 0.
    for index, path in enumerate(files):
        with np.load(path, allow_pickle=False) as archive:
            data = {key: archive[key].copy() for key in archive.files}
        if index == 0:
            generation, channel = int(data['generation']), int(data['channel'])
        assert int(data['generation']) == generation, 'Unexpected map reset during experiment'
        assert int(data['channel']) == channel and int(data['image_channel']) == 0
        x,y,q = data['x'],data['y'],data['observation']
        assert len(np.unique(x*meta['size']+y)) == len(x), 'Duplicate cell update'
        assert ((x>=0)&(x<meta['size'])&(y>=0)&(y<meta['size'])).all()
        assert np.isfinite(q).all() and ((q>=0)&(q<=1)).all()
        baseline[:,x,y] = np.stack(risk.ema_update(*baseline[:,x,y], q, meta['baseline_gamma']))
        np.testing.assert_allclose(baseline[:,x,y], data['posterior'], rtol=1e-5, atol=1e-6,
                                   err_msg='Recorded GPU posterior mismatch: '+path.name)
        if len(x): max_error = max(max_error, float(abs(baseline[:,x,y]-data['posterior']).max()))
        distance, base_world = observation_range(data, meta, base_camera)
        keep = within_range(distance, args.max_range_m)
        audit.append(dict(event=index,base_world_xyz_m=base_world.tolist(),before=int(len(x)),
            accepted=int(keep.sum()),
            max_accepted_distance_m=float(distance[keep].max()) if keep.any() else None,
            min_rejected_distance_m=float(distance[~keep].min()) if (~keep).any() else None))
        x,y,q = x[keep],y[keep],q[keep]
        filtered[:,x,y] = np.stack(risk.ema_update(*filtered[:,x,y], q, meta['baseline_gamma']))
    assert int(filtered[2].sum()) == sum(a['accepted'] for a in audit)
    assert (filtered[2] <= baseline[2]).all()
    if not (filtered[2]>0).any():
        raise ValueError('No observations passed the requested range gate')
    (output/'range_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    child = dict(session, output=str(output), source_common_session=str(source), stopped=True,
        mapping_max_range_m=args.max_range_m,range_reference_frame='base_link',
        range_metric='horizontal',range_fusion='per_observation_before_ema_at_image_acquisition_pose',
        trajectory_scope='full_bag',alphas=[args.alpha] if args.alpha is not None else [.05,.95])
    if session.get('mapping_window_stamp_s'):
        raise ValueError('This single-speed exporter currently expects full-bag mapping')
    (output/'session.json').write_text(json.dumps(child,indent=2)+'\n')
    shutil.copy2(source/'prediction_metadata.json', output/'prediction_metadata.json')
    shutil.copy2(source/'odometry.csv', output/'odometry.csv')
    trajectory = np.atleast_1d(np.genfromtxt(output/'odometry.csv', delimiter=',', names=True))
    assert len(trajectory)>1 and (np.diff(trajectory['stamp'])>0).all()
    stats = output/'ema_sweep/gamma_200'
    stats.mkdir(parents=True)
    mean, variance, count = filtered.transpose(0,2,1)
    known = count>0
    costs, measurements = {}, []
    for alpha in [.05,.95]:
        np.savez_compressed(stats/('statistics_alpha_%03d.npz'%round(alpha*100)),
            score_mean=mean,cost_mean=1-mean,variance=variance,count=count,valid=known,
            resolution=meta['resolution'],origin=meta['origin'],map_frame='map',cvar_alpha=alpha,
            ema_gamma=session['ema_gamma'],reference_speed=session['reference_speed'],
            cost_encoding=session['cost_encoding'],support_min=session['support_min'],
            cost_thresholds=session['cost_thresholds'],mapping_max_range_m=args.max_range_m,
            prediction_metadata=json.dumps(worker,sort_keys=True),schema='paper_ema_cvar_v1')
        q = np.where(known,mean-risk.cvar_coefficient(alpha)*np.sqrt(np.maximum(variance,0)),np.nan)
        cost = risk.score_to_cost(q,session['cost_encoding'],session['cost_thresholds'])
        costs[alpha] = cost
        measurements.append(dict(alpha=alpha,coefficient=risk.cvar_coefficient(alpha),
            mean_cost=float(cost[known].mean()),
            cost_percentages={str(v):float((cost[known]==v).mean()*100) for v in [0,30,50,70,100]}))
    assert (costs[.95][known]>=costs[.05][known]).all()
    with np.load(stats/'statistics_alpha_005.npz') as lo, np.load(stats/'statistics_alpha_095.npz') as hi:
        for field in ['score_mean','variance','count','valid']:
            np.testing.assert_array_equal(lo[field],hi[field])
    validation = dict(source_session=str(source),bag=session['bag'],reference_speed_m_s=session['reference_speed'],
        mapping_scope='full_bag',radius_m=args.max_range_m,range_reference_frame='base_link',
        base_link_in_camera_m=base_camera.tolist(),shared_projection_events=len(files),
        unrestricted_observations=int(baseline[2].sum()),accepted_observations=int(count.sum()),
        unrestricted_cells=int((baseline[2]>0).sum()),observed_cells=int(known.sum()),
        observed_area_m2=float(known.sum()*meta['resolution']**2),trajectory_samples=len(trajectory),
        raw_gpu_posteriors_verified=True,max_gpu_posterior_abs_error=max_error,
        range_gate_before_ema=True,counts_match_range_audit=True,counts_subset_of_unrestricted=True,
        same_statistics_for_both_alphas=True,alpha_095_cost_ge_005=True,
        strictly_higher_cost_cells=int((costs[.95][known]>costs[.05][known]).sum()),maps=measurements)
    (output/'range_comparison_manifest.json').write_text(json.dumps(validation,indent=2)+'\n')
    export_command = [sys.executable,str(Path(__file__).with_name('export_cvar_figure.py')),str(output),
        '--output',str(output/'figures'),'--plain-map-only','--show-speed']
    if args.alpha is not None:
        export_command += ['--alpha',str(args.alpha)]
    subprocess.run(export_command,check=True)
    print(json.dumps(validation,indent=2))


if __name__ == '__main__':
    main()
