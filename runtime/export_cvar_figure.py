#!/usr/bin/env python3
"""Export measured CVaR maps with the recorded SLAM path and endpoints."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patheffects as effects
from matplotlib.patches import Patch
import numpy as np
from PIL import Image


PALETTE = {0: 255, 30: 202, 50: 166, 70: 131, 100: 77}
UNKNOWN = (155, 173, 171)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--dpi', type=int, default=300)
    parser.add_argument('--statistics', choices=['world', 'rviz'], default='world')
    parser.add_argument('--alpha', type=float, choices=[.95, .05],
                        help='Export one alpha; omit to export both and their combined figure')
    parser.add_argument('--plain-map-only', action='store_true',
                        help='Remove all text, scale bar and legend from the plain version')
    parser.add_argument('--plain-full-bleed', action='store_true',
                        help='Fill the plain PNG/PDF canvas with the map at its physical aspect ratio')
    parser.add_argument('--width-inches', type=float, default=10.)
    parser.add_argument('--auto-aspect', action='store_true',
                        help='Fit the paper height to the world-map aspect ratio')
    parser.add_argument('--bounds', type=float, nargs=4, metavar=('XMIN','XMAX','YMIN','YMAX'))
    parser.add_argument('--show-speed', action='store_true')
    args = parser.parse_args()
    if args.dpi < 300:
        parser.error('Publication exports require at least 300 dpi')
    if args.width_inches <= 0:parser.error('width-inches must be positive')
    if args.plain_full_bleed and not args.plain_map_only:
        parser.error('--plain-full-bleed requires --plain-map-only')
    session = args.session.resolve()
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError('Use an empty output directory to preserve earlier figures')
    output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((session/'session.json').read_text())
    stats = session/'ema_sweep/gamma_200' if args.statistics == 'world' else session
    root = Path(__file__).resolve().parents[2]
    source = root/'emcp/src/elevation_mapping_cupy/elevation_mapping_cupy/script/elevation_mapping_cupy/risk_mapping.py'
    spec = importlib.util.spec_from_file_location('risk_math', source)
    risk = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(risk)
    panels = []
    paths = []
    reference = None
    suffixes = ['095', '005'] if args.alpha is None else [{.95: '095', .05: '005'}[args.alpha]]
    for suffix in suffixes:
        path = stats/('statistics_alpha_'+suffix+'.npz')
        with np.load(path, allow_pickle=False) as archive:
            data = {key: archive[key].copy() for key in archive.files}
        if reference is not None:
            for key in ['score_mean', 'variance', 'count', 'resolution', 'origin', 'ema_gamma',
                        'support_min', 'cost_thresholds', 'reference_speed', 'map_frame']:
                np.testing.assert_array_equal(reference[key], data[key], err_msg='Alpha states differ: '+key)
        reference = data
        for key, expected in [('ema_gamma', manifest['ema_gamma']),
                              ('support_min', manifest['support_min']),
                              ('resolution', manifest['fusion_resolution_m']),
                              ('reference_speed', manifest['reference_speed'])]:
            if not np.isclose(float(data[key]), expected):
                raise ValueError('Session and saved statistics differ: '+key)
        np.testing.assert_array_equal(data['cost_thresholds'], manifest['cost_thresholds'])
        if str(data['cost_encoding']) != 'five_level':
            raise ValueError('Expected the selected five-level encoding')
        known = data['count'] > 0
        alpha = float(data['cvar_alpha'])
        q = np.where(known, data['score_mean']-risk.cvar_coefficient(alpha)*
                     np.sqrt(np.maximum(data['variance'], 0)), np.nan)
        cost = risk.score_to_cost(q, 'five_level', data['cost_thresholds'])
        panels.append((suffix, alpha, cost))
        paths.append(path)
    data = reference
    origin, resolution = data['origin'], float(data['resolution'])
    path_file = session/'odometry.csv'
    trajectory = np.atleast_1d(np.genfromtxt(path_file, delimiter=',', names=True))
    xy = np.column_stack((trajectory['x'], trajectory['y']))
    if len(xy) < 2 or not np.isfinite(xy).all() or not np.all(np.diff(trajectory['stamp']) > 0):
        raise ValueError('Trajectory must contain finite, time-ordered measurements')
    if str(data['map_frame']) != 'map':
        raise ValueError('This export expects the explicit map frame')
    # Maintained profiles use identity map->camera_init (BISTU via odom).
    # No fitted alignment, smoothing or endpoint extrapolation is applied.
    observed = np.logical_or.reduce([panel[2] >= 0 for panel in panels])
    rows, cols = np.where(observed)
    if not len(cols):
        raise ValueError('No observed map cells')
    pad = 3.0
    bounds = [min(origin[0]+cols.min()*resolution, xy[:, 0].min())-pad,
              max(origin[0]+(cols.max()+1)*resolution, xy[:, 0].max())+pad,
              min(origin[1]+rows.min()*resolution, xy[:, 1].min())-pad,
              max(origin[1]+(rows.max()+1)*resolution, xy[:, 1].max())+pad]
    if args.bounds is not None:
        if not np.isfinite(args.bounds).all() or args.bounds[0]>=args.bounds[1] or args.bounds[2]>=args.bounds[3]:
            raise ValueError('Invalid shared world bounds')
        bounds = args.bounds
    extent = [origin[0], origin[0]+observed.shape[1]*resolution,
              origin[1], origin[1]+observed.shape[0]*resolution]
    height = max(3.2, args.width_inches*.95*(bounds[3]-bounds[2])/(bounds[1]-bounds[0])+1.) if args.auto_aspect else 6.5
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 12,
                         'pdf.fonttype': 42, 'ps.fonttype': 42})
    legend = [Patch(facecolor=np.full(3, gray/255.), edgecolor='.4', label=str(cost))
              for cost, gray in PALETTE.items()]
    legend.append(Patch(facecolor=np.array(UNKNOWN)/255., edgecolor='.4', label='Unknown'))
    files = []

    def draw(ax, panel, annotated):
        _, alpha, cost = panel
        rgb = np.full((*cost.shape, 3), UNKNOWN, dtype=np.uint8)
        for value, gray in PALETTE.items():
            rgb[cost == value] = gray
        ax.imshow(rgb, origin='lower', extent=extent, interpolation='nearest')
        ax.set_xlim(bounds[:2]); ax.set_ylim(bounds[2:]); ax.set_aspect('equal')
        ax.set_axis_off()
        if annotated or not args.plain_map_only:
            if args.show_speed:
                ax.text(.035, .95, f'v={manifest["reference_speed"]:g} m/s', transform=ax.transAxes,
                        ha='left', va='top', fontsize=20)
            ax.text(.035 if args.auto_aspect else .965, .95,
                    rf'$\alpha={alpha:.2f}$', transform=ax.transAxes,
                    ha='left' if args.auto_aspect else 'right', va='top', fontsize=20)
            if manifest.get('mapping_max_range_m') is not None:
                ax.text(.965, .87, 'Range <= %g m'%manifest['mapping_max_range_m'],
                        transform=ax.transAxes,ha='right',va='top',fontsize=14)
        if annotated:
            line, = ax.plot(xy[:, 0], xy[:, 1], color='#00cfd3', linewidth=1.65,
                            solid_capstyle='round', solid_joinstyle='round', zorder=4)
            line.set_path_effects([effects.Stroke(linewidth=2.9, foreground='white'), effects.Normal()])
            ax.plot(*xy[0], marker='o', markersize=8, color='#169b62', markeredgecolor='white',
                    markeredgewidth=1.2, zorder=6)
            ax.plot(*xy[-1], marker='*', markersize=13, color='#df4035', markeredgecolor='white',
                    markeredgewidth=.9, zorder=6)
            for point, label, offset in [(xy[0], 'Start', (0, 20)), (xy[-1], 'End', (14, 17))]:
                text = ax.annotate(label, xy=point, xytext=offset, textcoords='offset points',
                                   ha='left', va='bottom', fontsize=12, fontweight='normal', zorder=7,
                                   arrowprops=dict(arrowstyle='-', color='.2', linewidth=.7,
                                                   shrinkA=2, shrinkB=7))
                text.set_path_effects([effects.Stroke(linewidth=3, foreground='white'), effects.Normal()])
        if annotated or not args.plain_map_only:
            # A ten-metre bar in the unobserved lower-left margin.
            xbar, ybar = bounds[0]+2.0, bounds[2]+1.5
            ax.plot([xbar, xbar+10], [ybar, ybar], color='.15', linewidth=2.0, zorder=4)
            ax.text(xbar+5, ybar+.6, '10 m', ha='center', va='bottom', fontsize=11)

    def save(fig, name):
        for suffix in ['png', 'pdf']:
            path = output/(name+'.'+suffix)
            background = np.array(UNKNOWN)/255. if args.plain_full_bleed and '_plain_' in name else 'white'
            fig.savefig(path, dpi=args.dpi, facecolor=background)
            entry = dict(path=str(path), sha256=sha256(path))
            if suffix == 'png':
                with Image.open(path) as image:
                    entry.update(pixel_dimensions=list(image.size), dpi=list(image.info['dpi']))
                    if min(image.info['dpi']) < args.dpi-.1:
                        raise ValueError('PNG DPI metadata was not preserved')
            files.append(entry)
        plt.close(fig)

    for panel in panels:
        for annotated in [False, True]:
            fig, ax = plt.subplots(figsize=(args.width_inches, height))
            fig.subplots_adjust(left=.025, right=.975, top=.98,
                                bottom=.75/height if args.auto_aspect else .115)
            if not annotated and args.plain_full_bleed:
                fig.set_size_inches(args.width_inches,
                                    args.width_inches*(bounds[3]-bounds[2])/(bounds[1]-bounds[0]))
                fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
            draw(ax, panel, annotated)
            if annotated or not args.plain_map_only:
                fig.legend(handles=legend, title='Cost', loc='lower center', ncol=6,
                           frameon=False, bbox_to_anchor=(.5, -.005))
            name = 'cvar_alpha_'+panel[0]+('_trajectory_' if annotated else '_plain_')+str(args.dpi)+'dpi'
            save(fig, name)
    if len(panels) > 1:
        fig, axes = plt.subplots(2, 1, figsize=(10, 12.5))
        fig.subplots_adjust(left=.025, right=.975, top=.99, bottom=.065, hspace=.035)
        for ax, panel in zip(axes, panels):
            draw(ax, panel, True)
        fig.legend(handles=legend, title='Cost', loc='lower center', ncol=6,
                   frameon=False, bbox_to_anchor=(.5, .002))
        save(fig, 'cvar_comparison_trajectory_'+str(args.dpi)+'dpi')
    np.savetxt(output/'trajectory_xy.csv', np.column_stack((trajectory['stamp'], xy)),
               delimiter=',', header='timestamp_s,x_m,y_m', comments='', fmt='%.12f')
    result = dict(schema='cvar_publication_figure_v1', source_session=str(session),
        source_bag=manifest['bag'],
        platform=manifest.get('platform','bistu'),
        pointcloud_topic=manifest.get('pointcloud_topic','/cloud_registered'),
        prediction_metadata=json.loads((session/'prediction_metadata.json').read_text()),
        statistics_source=args.statistics, dpi=args.dpi, alphas=[panel[1] for panel in panels],
        plain_map_only=args.plain_map_only,
        plain_full_bleed=args.plain_full_bleed,
        show_speed=args.show_speed,
        width_inches=args.width_inches, auto_aspect=args.auto_aspect,
        alpha_label_position='upper_left' if args.auto_aspect else 'upper_right',
        parameters={key: manifest[key] for key in ['support_min', 'ema_gamma', 'cost_thresholds',
                    'cost_encoding', 'fusion_resolution_m', 'reference_speed']},
        observed_world_cells=int(observed.sum()), map_frame='map', trajectory_frame='camera_init',
        trajectory_to_map_transform=manifest.get('trajectory_to_map_transform','identity from mapping_rs_bistu.launch'),
        trajectory_samples=len(xy), trajectory_source=str(path_file),
        trajectory_scope=manifest.get('trajectory_scope','full_bag'),
        mapping_window_offsets_s=manifest.get('mapping_window_offsets_s'),
        mapping_window_stamp_s=manifest.get('mapping_window_stamp_s'),
        mapping_max_range_m=manifest.get('mapping_max_range_m'),
        range_fusion=manifest.get('range_fusion'),
        start=dict(timestamp=float(trajectory['stamp'][0]), xy_m=xy[0].tolist()),
        end=dict(timestamp=float(trajectory['stamp'][-1]), xy_m=xy[-1].tolist()),
        plot_bounds_xy_m=bounds, cost_palette=PALETTE, unknown_rgb=UNKNOWN,
        sources=[dict(path=str(path), sha256=sha256(path)) for path in [*paths, path_file]],
        notes=['The trajectory uses recorded SLAM samples inside the mapping window; full-bag poses are retained.'
               if manifest.get('trajectory_scope')=='mapping_window' else
               'The trajectory uses the complete recorded online SLAM odometry.',
               'No fitted transform, smoothing or endpoint extrapolation is used.',
               'PNG is rendered at the declared DPI; PDF keeps annotations and trajectory as vectors.',
               'The physical map cell resolution is unchanged.'], files=files)
    (output/'figure_manifest.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(output=str(output),dpi=args.dpi,observed_cells=int(observed.sum()),
                          trajectory_samples=len(xy),files=[entry['path'] for entry in files]),indent=2))


if __name__ == '__main__':
    main()
