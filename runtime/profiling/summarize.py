#!/usr/bin/env python3
"""Summarize measured invocation time, matched-image latency and processing rate."""
import argparse
import collections
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);args=parser.parse_args()
root=args.root.resolve()
baseline_manifest=json.loads((root/'single_speed_baseline/manifest.json').read_text())
bag_start_ns=round(baseline_manifest['bag_start']*1e9);bag_end_ns=round(baseline_manifest['bag_end']*1e9)
duration=baseline_manifest['bag_end']-baseline_manifest['bag_start'];cut=bag_start_ns+10_000_000_000

def summarize(values):
    x=np.asarray(values,float)
    if not len(x):return dict(count=0)
    return dict(count=len(x),mean_ms=float(x.mean()),median_ms=float(np.median(x)),p95_ms=float(np.percentile(x,95)),
                p99_ms=float(np.percentile(x,99)),min_ms=float(x.min()),max_ms=float(x.max()),std_ms=float(x.std()))

results={};stage_rows=[];latency_rows=[];rates=[]
for mode in ['single_speed','three_speeds','single_speed_baseline']:
    folder=root/mode;manifest=json.loads((folder/'manifest.json').read_text());assert manifest['status']=='complete'
    topics=[json.loads(l) for l in (folder/'topics.jsonl').read_text().splitlines()]
    by_topic=collections.defaultdict(list)
    for item in topics:by_topic[item['topic']].append(item)
    raw=by_topic['/hikrobot_camera_L/image_raw']
    assert len(raw)==582 and len(by_topic['/rslidar_points'])==617
    for values in by_topic.values():assert len({r['stamp_ns'] for r in values})==len(values),'Duplicate topic stamp'
    firstwall=min(r['wall_ns'] for r in raw);lastwall=max(r['wall_ns'] for r in raw)
    ordered=sorted(raw,key=lambda r:r['stamp_ns'])
    cutoffwall=int(np.interp(cut,[r['stamp_ns'] for r in ordered],[r['wall_ns'] for r in ordered]))
    records=[]
    for file in folder.glob('*_*.jsonl'):
        records += [json.loads(l) for l in file.read_text().splitlines()]
    cpp=[]
    if (folder/'mapping_cpp.csv').exists():
        for record in csv.DictReader((folder/'mapping_cpp.csv').open()):
            record={k:(v if k=='stage' else int(v)) for k,v in record.items()};cpp.append(record)
        records += cpp
    def steady(r):
        if r.get('stamp_ns',0):return cut<=r['stamp_ns']<=bag_end_ns
        return cutoffwall<=r['start_ns']<=lastwall
    groups=collections.defaultdict(list)
    for record in records:
        if steady(record):groups[record['stage']].append(record)
    assert not any(r.get('error') for r in records if steady(r))
    stages={}
    for name,values in sorted(groups.items()):
        summary=summarize([(r['end_ns']-r['start_ns'])/1e6 for r in values])
        gpu=summarize([r['gpu_ms'] for r in values if r.get('gpu_ms') is not None])
        stages[name]=dict(wall=summary,gpu=gpu)
        stage_rows.append(dict(mode=mode,stage=name,**summary,gpu_mean_ms=gpu.get('mean_ms'),gpu_p95_ms=gpu.get('p95_ms')))
    rawdict={r['stamp_ns']:r['wall_ns'] for r in raw}
    rectdict={r['stamp_ns']:r['wall_ns'] for r in by_topic['/hikrobot_camera_L/image_rect']}
    preddict={r['stamp_ns']:r['wall_ns'] for r in by_topic['/uam2te/conservative_score']}
    mapdict={r['stamp_ns']:r['wall_ns'] for r in by_topic['/uam2te/mapping_score']}
    finished={r['stamp_ns']:r['end_ns'] for r in cpp if r['stage']=='mapping.image_callback'}
    client_started={r['stamp_ns']:r['start_ns'] for r in records if r['stage']=='client.callback'}
    client_finished={r['stamp_ns']:r['end_ns'] for r in records if r['stage']=='client.callback'}
    latencies={}
    for name,start,end in [('raw_to_rectified',rawdict,rectdict),('raw_to_prediction',rawdict,preddict),
                            ('rectified_to_prediction',rectdict,preddict),('raw_to_mapping_message',rawdict,mapdict),
                            ('raw_to_fusion_complete',rawdict,finished),('mapping_message_to_fusion_complete',mapdict,finished),
                            ('raw_arrival_to_client_start',rawdict,client_started),
                            ('rectified_arrival_to_client_start',rectdict,client_started),
                            ('client_start_to_fusion_complete',client_started,finished),
                            ('client_complete_to_fusion_complete',client_finished,finished)]:
        samples=[(stamp,(end[stamp]-wall)/1e6) for stamp,wall in start.items() if stamp in end and cut<=stamp<=bag_end_ns]
        summary=summarize([value for stamp,value in samples]);summary['negative_samples']=sum(value<0 for stamp,value in samples)
        latencies[name]=summary
        for stamp,value in samples:latency_rows.append(dict(mode=mode,measurement=name,stamp_ns=stamp,latency_ms=value))
    counts={topic:len(rows) for topic,rows in by_topic.items()}
    callbacks=sum(r['stage']=='mapping.image_callback' for r in cpp) if cpp else None
    callback_success=sum(r['stage']=='mapping.image_python_call' for r in cpp) if cpp else None
    cpp_cloud=sum(r['stage']=='mapping.cloud_callback' for r in cpp) if cpp else None
    rate=dict(mode=mode,bag_duration_s=duration,input_images=len(raw),predicted_images=len(preddict),
              image_processing_fraction=len(preddict)/len(raw),inference_fps=len(preddict)/duration,
              fusion_callbacks=callbacks,successful_fusion_callbacks=callback_success,pointcloud_callbacks=cpp_cloud,
              gridmap_publications=counts.get('/navigation_elevation/filtered_elevation_map',0),
              costmap_publications=counts.get('/similarity_costmap',0),
              costmap_fps=counts.get('/similarity_costmap',0)/duration)
    rates.append(rate)
    with np.load(folder/'final_statistics.npz') as z:
        observed_cells=int((z['count']>0).sum());assert observed_cells>0
    monitor=[]
    for row in csv.DictReader((folder/'gpu_monitor.log').open()):
        try:monitor.append(float(row[' utilization.gpu [%]'].strip().split()[0]))
        except (KeyError,ValueError):pass
    results[mode]=dict(manifest=manifest,throughput=rate,topic_counts=counts,steady_stage_timings=stages,
                       steady_matched_latencies=latencies,final_known_cells=observed_cells,
                       mean_gpu_utilization_percent=float(np.mean(monitor)),steady_wall_window_ns=[cutoffwall,lastwall],
                       startup_timings={name:summarize([(r['end_ns']-r['start_ns'])/1e6 for r in records if r['stage']==name])
                                        for name in ['startup.model_load','startup.first_inference']})

def write_csv(path,rows):
    keys=list(rows[0])
    with path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader();writer.writerows(rows)
write_csv(root/'stage_timings_ms.csv',stage_rows)
write_csv(root/'matched_frame_latencies.csv',latency_rows)
write_csv(root/'throughput.csv',rates)
baseline=results['single_speed_baseline']['throughput']['inference_fps']
profiled=results['single_speed']['throughput']['inference_fps']
report=dict(bag=results['single_speed']['manifest']['bag'],bag_duration_s=duration,expected_camera_frames=582,
            steady_state_exclusion_s=10,instrumentation_throughput_reduction_percent=100*(1-profiled/baseline),
            hardware=dict(gpu='NVIDIA RTX PRO 6000 Blackwell Workstation Edition',gpu_memory_mib=97887,driver='595.80',
                          cpu='AMD EPYC 7A23, 48 cores / 96 threads',ros='ROS1 Noetic container',
                          model_python='3.12.13',torch='2.10.0+cu128',torch_cuda='12.8',cupy='13.6.0',
                          model_torch_cpu_threads=4,model_opencv_threads=96),
            interpretation_notes=[
                'Invocation timings use monotonic wall time and include synchronous CUDA completion where applicable. CUDA event spans are also retained.',
                'The first 10 seconds are excluded from steady-state invocation and matched-image latency statistics. Full-bag counts include startup.',
                'Parent timings include child timings; do not add nested phases. Pointcloud/image/grid publications have distinct invocation rates.',
                'All modes run SLAM concurrently. Its internal compute time was not instrumented; odometry counts are recorded.',
                'Raw-to-prediction and raw-to-fusion latency join identical image acquisition stamps, using observer arrival and C++ completion wall time.',
                'Topic-observer timings include independent TCP subscriber delivery and callback scheduling. Short cross-subscriber differences can be negative; raw-to-rectified and mapping-message-to-fusion are diagnostic only, not isolated stage durations.',
                'Raw/rectified arrival to client start is observed pre-inference delivery, buffering and scheduling delay, not a direct measurement of a single queue.',
                'The mapping subscriber may run concurrently with the client finishing diagnostics; client-complete-to-fusion can therefore also be negative.',
                'OccupancyGrid headers carry GridMap acquisition time, not input image identity; no exact image-to-final-OccupancyGrid latency is claimed.',
                'GridMap publisher configured at 3 Hz; global OccupancyGrid at 1 Hz, wall throttled and quantized by GridMap callbacks.',
                'Three speeds share one image forward; online CVaR exporter visualizes first speed while EMCP updates all three semantic channels.',
                'No RViz or per-observation map trace writes; ordinary ROS publication remains enabled.',
                'The established 6 m range maps are offline reconstructions. This measurement covers the existing live mapping chain without that offline range replay.',
                'Single original-code baseline compares throughput with detailed instrumentation removed; both use the topic observer. The 5.2% difference includes run-to-run variation and is not an exact causal overhead estimate. No separate three-speed baseline was run.'
            ],runs=results)
(root/'summary.json').write_text(json.dumps(report,indent=2)+'\n')

plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'pdf.fonttype':42,'axes.spines.top':False,'axes.spines.right':False})
fig,(ax1,ax2)=plt.subplots(1,2,figsize=(13.8,5.3),layout='constrained',gridspec_kw={'width_ratios':[1.1,1]})
labels=['Network forward','Query + restore','Worker serialization','Client decode + checks','Mapping mask + publish','Other callback work']
keys=['worker.network_forward','worker.query_restore_total','worker.serialize_outputs','client.decode_validate_outputs','client.mapping_mask_and_publish']
colors=['#4477AA','#66CCEE','#EE6677','#CCBB44','#228833','#999999']
for i,mode in enumerate(['single_speed','three_speeds']):
    stages=results[mode]['steady_stage_timings'];total=stages['client.callback']['wall']['mean_ms']
    values=[stages[key]['wall']['mean_ms'] for key in keys];values.append(total-sum(values))
    assert values[-1]>=0
    offset=0
    for label,value,color in zip(labels,values,colors):
        ax1.barh(i,value,left=offset,color=color,label=label if i==0 else None,height=.55)
        if value>23:ax1.text(offset+value/2,i,f'{value:.0f}',ha='center',va='center',fontsize=10)
        offset+=value
    ax1.text(total+3,i,f'{total:.1f} ms',va='center',fontsize=10)
ax1.set_yticks([0,1],['Single speed\n0.8 m/s','Three speeds\n0.4 / 0.9 / 1.4'])
ax1.invert_yaxis();ax1.set(xlabel='Mean ROS inference callback time (ms)',title='Inference pipeline: measured wall time',xlim=(0,370))
ax1.legend(loc='upper left',bbox_to_anchor=(0,-.18),ncol=2,fontsize=9,frameon=False)
names=['Pointcloud callback','Image projection + fusion','EMA fusion, per channel','CVaR + publish, one speed']
keys=['mapping.cloud_callback','mapping.image_callback','mapping.ema_world_fusion','risk.cvar_and_publish']
for i,mode in enumerate(['single_speed','three_speeds']):
    stages=results[mode]['steady_stage_timings']
    means=[stages[key]['wall']['mean_ms'] for key in keys]
    p95=[stages[key]['wall']['p95_ms'] for key in keys]
    y=np.arange(len(keys))+(i-.5)*.32
    ax2.barh(y,means,height=.28,color=['#4477AA','#EE6677'][i],label=['Single speed','Three speeds'][i])
    ax2.plot(p95,y,'|',color='black',ms=8)
ax2.set_yticks(np.arange(len(keys)),names);ax2.invert_yaxis()
ax2.set(xlabel='Time per invocation (ms); black mark = P95',title='Mapping stages (EMA is nested)')
ax2.legend(loc='upper left',bbox_to_anchor=(0,-.18),ncol=2,fontsize=9,frameon=False)
fig.savefig(root/'deployment_timing_300dpi.png',dpi=300)
fig.savefig(root/'deployment_timing.pdf')
plt.close(fig)
print(json.dumps(dict(rates=rates,instrumentation_reduction_percent=report['instrumentation_throughput_reduction_percent'],
    latencies={mode:run['steady_matched_latencies'] for mode,run in results.items()}),indent=2))
