#!/usr/bin/env python3
"""Run one instrumented full bag at 1x, with explicitly owned process groups."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET
import xmlrpc.client

HERE=Path(__file__).resolve().parent
RUNTIME=HERE.parent
ROOT=RUNTIME.parents[1]
sys.path.insert(0,str(RUNTIME))
import cvar_session as control

parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--mode',choices=['single','multi'],default='single')
parser.add_argument('--baseline',action='store_true',help='Original worker/client/map with only arrival-time observation')
args=parser.parse_args()
out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
previous=json.loads((RUNTIME.parent/'.runtime/cvar_20261029_103551/session.json').read_text())
speeds=[.8] if args.mode=='single' else [.4,.9,1.4]
channels=['similarity'] if args.mode=='single' else ['similarity','speed_query_1','speed_query_2']
manifest=dict(mode=args.mode,baseline=args.baseline,bag=previous['bag'],config=previous['config'],checkpoint=previous['checkpoint'],
              bag_start=previous['bag_start'],bag_end=previous['bag_end'],
              speeds_m_s=speeds,play_rate=1.,alpha=.95,ema_gamma=.2,support_min=.15,
              map_resolution_m=.5,world_map_size=400,local_map_length_m=15.,
              worker_image_size_wh=[448,336],rectified_image_size_wh=[1920,1440],
              clock='Linux monotonic nanoseconds; CUDA events synchronized at timed GPU stage ends',
              visualization=False,ema_trace=False,online_range_gate=False,
              online_range_note='The established 6 m maps were reconstructed offline from recorded observations; this profiles the current live mapping chain.',
              warmup_exclusion='First 10 seconds of bag playback are excluded from steady-state timings; full counts retain startup.',
              warmup_worker_calls=4,output=str(out),status='starting')
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')

# Snapshot the exact client and add only timing calls / request correlation.
source=ROOT/'uam2te/scripts/ros_workspace/src/ros_interface/scripts/navigation_inference.py'
original=source.read_text();client=original
def replace(old,new):
    global client
    assert client.count(old)==1,old
    client=client.replace(old,new)
replace('import time\n','import time\nfrom runtime_profile import Span, set_stamp\n')
replace('    def on_image(self, msg):\n','    def on_image(self, msg):\n        set_stamp(msg.header.stamp.to_nsec())\n        profile = None\n')
replace('            buffer = BytesIO()','            profile = Span("client.callback", msg.header.stamp.to_nsec())\n            buffer = BytesIO()')
replace("            endpoint = '/infer_speeds'",'            profile.mark("client.encode_request")\n            endpoint = \'/infer_speeds\'')
replace("headers={'Content-Type': 'application/octet-stream'}","headers={'Content-Type': 'application/octet-stream', 'X-Image-Stamp-Ns': str(msg.header.stamp.to_nsec())}")
replace('            with np.load(BytesIO(payload), allow_pickle=False) as maps:',
        '            profile.mark("client.http_infer_roundtrip")\n            with np.load(BytesIO(payload), allow_pickle=False) as maps:')
replace('            # Check the worker identity on each accepted frame.',
        '            profile.mark("client.decode_validate_outputs")\n            # Check the worker identity on each accepted frame.')
replace('            info.header = copy.deepcopy(msg.header)',
        '            profile.mark("client.metadata_check")\n            info.header = copy.deepcopy(msg.header)')
replace('            # A rejected sample must not update mean, variance or count.',
        '            profile.mark("client.prediction_message_publish")\n            # A rejected sample must not update mean, variance or count.')
replace('            if self.mapping_audit_path:',
        '            profile.mark("client.mapping_mask_and_publish")\n            if self.mapping_audit_path:')
replace('            self.last_stamp = msg.header.stamp',
        '            profile.mark("client.diagnostics_and_publish")\n            self.last_stamp = msg.header.stamp')
replace('        finally:\n            self.lock.release()',
        '        finally:\n            if profile is not None: profile.close()\n            self.lock.release()')
(out/'navigation_inference_profiled.py').write_text(client)
(out/'navigation_inference_original.py').write_text(original)
compile(client,str(out/'navigation_inference_profiled.py'),'exec')

# Use the existing mapping launch, swapping only the risk node's timing entry.
mapping_source=ROOT/'emcp/src/elevation_mapping_cupy/elevation_mapping_cupy/launch/navigation.launch'
tree=ET.parse(mapping_source)
for node in tree.getroot().findall('node'):
    if node.get('type')=='generate_risk_costmap.py' and not args.baseline:
        node.set('pkg','cvar_runtime');node.set('type','profile_entry.py')
        node.set('args','risk '+str(mapping_source.parents[1]/'script/generate_risk_costmap.py'))
        ET.SubElement(node,'env',name='UAM2TE_PROFILE_ROLE',value='risk')
    else:ET.SubElement(node,'env',name='UAM2TE_PROFILE_ROLE',value='mapping')
tree.write(out/'mapping.launch')
launch=ET.Element('launch');ET.SubElement(launch,'param',name='/use_sim_time',value='true')
slam=ET.SubElement(launch,'include',file='$(find fast_lio_sam)/launch/mapping_rs_bistu.launch')
for key,value in dict(use_sim_time='true',rviz='false',runtime_metrics_log_path=str(out/'slam_metrics.csv'),calibration_status_log_path=str(out/'slam_calibration.csv')).items():
    ET.SubElement(slam,'arg',name=key,value=value)
client_node=ET.SubElement(launch,'node',pkg='cvar_runtime',type='profile_entry.py',name='navigation_inference',output='screen',required='true',args='client '+str(out/('navigation_inference_original.py' if args.baseline else 'navigation_inference_profiled.py')))
ET.SubElement(client_node,'env',name='UAM2TE_PROFILE_ROLE',value='client')
for key,value in dict(image_topic='/hikrobot_camera_L/image_rect',camera_info_topic='/hikrobot_camera_L/image_rect/camera_info',
                      worker_url='http://127.0.0.1:8765',reference_speed=str(speeds[0]),support_min='.15').items():
    ET.SubElement(client_node,'param',name=key,value=value)
for key,value in dict(reference_speeds=speeds if len(speeds)>1 else [],score_channels=channels).items():
    ET.SubElement(client_node,'rosparam',param=key).text=json.dumps(value)
mapping=ET.SubElement(launch,'include',file=str(out/'mapping.launch'))
params=dict(score_topic='/uam2te/mapping_score',score_channels=json.dumps(channels),
            score_fusions=json.dumps(dict([(c,'ema_variance') for c in channels]+[('default','exponential')])),
            pointcloud_topic='/cloud_registered_body',map_frame='map',base_frame='base_link',reference_speed=str(speeds[0]),
            cvar_alpha='.95',ema_gamma='.2',ema_trace_dir='',resolution='.5',cost_encoding='five_level',
            cost_thresholds='[0.2,0.35,0.5,0.65]',map_length='200',origin_x='-100',origin_y='-100',save_path=str(out/'final_statistics.npz'))
for key,value in params.items():ET.SubElement(mapping,'arg',name=key,value=value)
ET.ElementTree(launch).write(out/'experiment.launch')
snapshot=out/'source_snapshot';snapshot.mkdir()
for file in list(HERE.glob('*.py'))+[RUNTIME/'profile_entry.py',source,mapping_source]:
    shutil.copy2(file,snapshot/file.name)
(out/'source_hashes.json').write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in list(HERE.glob('*.py'))+[source,mapping_source]},indent=2))

started=[]
def start(name,command,gpu=False):
    shell='export UAM2TE_PROFILE_DIR='+shlex.quote(str(out))+' UAM2TE_PROFILE_ROLE='+shlex.quote(name)+'; '
    if args.baseline and name in ['worker','pipeline']:shell='unset UAM2TE_PROFILE_DIR; '
    shell+='export PYTHONPATH='+shlex.quote(str(HERE))+':"$PYTHONPATH"; exec '+shlex.join(command)
    control.start_process(out,name,['bash','-c',shell],gpu=gpu)
    started.append(name)
def save_status(status,**extra):
    manifest.update(status=status,**extra);(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(status=status,**extra)),flush=True)
master=xmlrpc.client.ServerProxy('http://127.0.0.1:11671')
try:
    start('master',['roscore','-p','11671'])
    control.wait_for(lambda:master.getPid('/profile')[0]==1,20,'master')
    control.ros_command(['rosparam','set','/use_sim_time','true'])
    model=[str(ROOT/'uam2te/.venv/bin/python'),str(HERE/'worker.py'),'--config',manifest['config'],'--checkpoint',manifest['checkpoint'],
           '--speed',str(speeds[0]),'--warmup-image',str(ROOT/'cvar_results/2025-06-27-14-30-04_t029670_grass_speed_curve/camera_rectified.png')]
    if len(speeds)>1:model+=['--speeds',*map(str,speeds)]
    if args.baseline:
        model=[str(ROOT/'uam2te/.venv/bin/python'),str(ROOT/'uam2te/scripts/serve_navigation_inference.py'),
               '--config',manifest['config'],'--checkpoint',manifest['checkpoint'],'--reference-speed',str(speeds[0])]
        if len(speeds)>1:model+=['--reference-speeds',*map(str,speeds)]
    start('worker',model,gpu=True)
    control.wait_for(lambda:urllib.request.urlopen('http://127.0.0.1:8765/metadata',timeout=2).status==200,180,'profile worker')
    if args.baseline:
        code='''import cv2,numpy as np,sys
from io import BytesIO
from urllib.request import Request,urlopen
b=BytesIO();np.save(b,cv2.imread(sys.argv[1]),allow_pickle=False)
for i in range(4):
 with urlopen(Request('http://127.0.0.1:8765/'+sys.argv[2],data=b.getvalue()),timeout=60) as r:r.read()
'''
        subprocess.run([str(ROOT/'uam2te/.venv/bin/python'),'-c',code,
                        str(ROOT/'cvar_results/2025-06-27-14-30-04_t029670_grass_speed_curve/camera_rectified.png'),
                        'infer_speeds' if len(speeds)>1 else 'infer'],check=True)
    save_status('worker_ready')
    start('pipeline',['roslaunch',str(out/'experiment.launch')],gpu=True)
    def ready():
        pubs=master.getSystemState('/profile')[2][0]
        return '/navigation_elevation/filtered_elevation_map' in {topic for topic,nodes in pubs}
    control.wait_for(ready,120,'initialized map publisher')
    hook_files=list(out.glob('mapping_*.jsonl'))
    if not args.baseline:
        assert hook_files and any('profiling.emcp_hook_ready' in p.read_text() for p in hook_files),'Embedded Python timing hook did not load'
    start('observer',['/usr/bin/python3',str(HERE/'observe.py')])
    control.wait_for(lambda:(out/'observer_ready').exists(),20,'observer')
    start('gpu_monitor',['nvidia-smi','--query-gpu=timestamp,utilization.gpu,utilization.memory,memory.used,power.draw,clocks.sm','--format=csv','-l','1'])
    bag_command=['rosbag','play','--clock','--pause','--queue=30','-r','1',manifest['bag'],'--topics',
                 '/hikrobot_camera_L/image_raw','/rslidar_points','/imu/data','__name:=profile_bag']
    finish='import pathlib,time,json; pathlib.Path('+repr(str(out/'bag_finished.json'))+').write_text(json.dumps(dict(wall_ns=time.perf_counter_ns())))'
    start('bag',['bash','-c',shlex.join(bag_command)+'; /usr/bin/python3 -c '+shlex.quote(finish)])
    control.wait_for(lambda:master.lookupService('/profile','/profile_bag/pause_playback')[0]==1,60,'paused bag')
    time.sleep(2)
    (out/'ros_parameters.yaml').write_text(control.ros_command(['rosparam','get','/']))
    save_status('playing',play_start_wall_ns=time.perf_counter_ns())
    control.ros_command(['rosservice','call','/profile_bag/pause_playback','data: false'])
    control.wait_for(lambda:(out/'bag_finished.json').exists(),180,'full bag replay')
    save_status('draining',bag_finished=json.loads((out/'bag_finished.json').read_text()))
    time.sleep(5)
    (out/'save_result.txt').write_text(control.ros_command(['rosservice','call','/navigation_risk_costmap/save']))
    save_status('complete',completed_wall_ns=time.perf_counter_ns())
except Exception as exc:
    save_status('failed',error=repr(exc));raise
finally:
    for name in ['bag','gpu_monitor','observer','pipeline','worker','master']:
        if name in started:control.stop_process(out/(name+'.pid.json'))
