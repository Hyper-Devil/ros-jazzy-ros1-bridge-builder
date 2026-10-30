#!/usr/bin/env python3
"""Start and control the mounted-source ROS1 CVaR/RViz experiment."""
import argparse
import datetime
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import socket
import subprocess
import time
import urllib.request
import xmlrpc.client
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
PROJECTS = ROOT.parent
ACTIVE = ROOT/'.runtime/cvar_active.json'
CONTAINER = 'ros1-noetic'


def docker(command, **kwargs):
    return subprocess.run(['docker', 'exec', CONTAINER, 'bash', '-lc', command], check=True, **kwargs)


def start_process(output, name, command, gpu=False, display=':1'):
    pidfile, logfile = output/(name+'.pid.json'), output/(name+'.log')
    record = ('import json,pathlib,sys; p=int(sys.argv[1]); '
              's=pathlib.Path("/proc/%s/stat"%p).read_text().rsplit(")",1)[1].split()[19]; '
              'pathlib.Path(sys.argv[2]).write_text(json.dumps(dict(pid=p,start=s)))')
    inner = ('set -e; /usr/bin/python3 -c '+shlex.quote(record)+' $$ '+shlex.quote(str(pidfile))+
             '; source '+shlex.quote(str(ROOT/'runtime/env.sh'))+' noetic'+(' gpu' if gpu else '')+
             '; export ROS_LOG_DIR='+shlex.quote(str(output/'ros_logs'))+'; exec '+shlex.join(command))
    outer = 'exec setsid --wait bash -c '+shlex.quote(inner)+' > '+shlex.quote(str(logfile))+' 2>&1'
    subprocess.run(['docker','exec','-d','-e','DISPLAY='+display,'-e','QT_X11_NO_MITSHM=1',
                    CONTAINER,'bash','-lc',outer], check=True)


def ros_command(command):
    return docker('source '+shlex.quote(str(ROOT/'runtime/env.sh'))+' noetic; '+shlex.join(command),
                  capture_output=True, text=True).stdout


def wait_for(check, timeout, description):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        try:
            if check(): return
        except Exception: pass
        time.sleep(.3)
    raise RuntimeError('Timed out waiting for '+description)


def rviz_config(topic, alpha, x, cost_encoding='continuous', support_min=0.):
    return {'Panels':[{'Class':'rviz/Displays','Name':'Displays'}, {'Class':'rviz/Views','Name':'Views'}],
        'Preferences':{'PromptSaveOnExit':False},
        'Visualization Manager':{'Class':'','Enabled':True,
            'Global Options':{'Fixed Frame':'map','Background Color':('255; 255; 255' if cost_encoding == 'five_level' else '158; 177; 177'),'Frame Rate':20},
            'Displays':[
                {'Class':'rviz/Map','Name':'CVaR alpha='+alpha,'Enabled':True,'Value':True,
                 'Topic':topic,'Alpha':(.7 if cost_encoding == 'five_level' else 1.),'Color Scheme':'map','Draw Behind':True,'Use Timestamp':False},
                {'Class':'rviz/Path','Name':'SLAM trajectory','Enabled':True,'Value':True,
                 'Topic':'/path','Alpha':1.,'Color':'0; 255; 255','Line Style':'Billboards',
                 'Line Width':.12,'Buffer Length':1,'Pose Style':'None'},
                {'Class':'rviz/PointCloud2','Name':'SLAM global cloud (toggle)','Enabled':False,'Value':False,
                 'Topic':'/fast_lio_sam/mapping/map_global_optimized','Position Transformer':'XYZ',
                 'Color Transformer':'Intensity','Channel Name':'intensity','Use rainbow':True,
                 'Autocompute Intensity Bounds':True,'Style':'Points','Size (Pixels)':2,'Queue Size':1},
                {'Class':'rviz/Image','Name':'Rectified camera','Enabled':False,'Value':False,
                 'Image Topic':'/uam2te/visualization/camera','Transport Hint':'raw','Queue Size':1},
                {'Class':'rviz/Image','Name':'Network q = score x support','Enabled':alpha=='.95','Value':alpha=='.95',
                 'Image Topic':'/uam2te/visualization/q_heatmap','Transport Hint':'raw','Queue Size':1},
                {'Class':'rviz/Image','Name':'Network q overlay','Enabled':alpha=='.95','Value':alpha=='.95',
                 'Image Topic':'/uam2te/visualization/q_overlay','Transport Hint':'raw','Queue Size':1},
                {'Class':'rviz/Image','Name':'Network score (toggle)','Enabled':False,'Value':False,
                 'Image Topic':'/uam2te/visualization/score_heatmap','Transport Hint':'raw','Queue Size':1},
                {'Class':'rviz/Image','Name':'Network support (toggle)','Enabled':False,'Value':False,
                 'Image Topic':'/uam2te/visualization/support_heatmap','Transport Hint':'raw','Queue Size':1},
                {'Class':'rviz/Image','Name':'Mapping support >= %g'%support_min,
                 'Enabled':support_min>0 and alpha=='.95','Value':support_min>0 and alpha=='.95',
                 'Image Topic':'/uam2te/visualization/mapping_mask','Transport Hint':'raw','Queue Size':1},
                {'Class':'rviz/Image','Name':'Network q numeric [0,1] (toggle)','Enabled':False,'Value':False,
                 'Image Topic':'/uam2te/visualization/q','Transport Hint':'raw','Queue Size':1,
                 'Normalize Range':False,'Min Value':0.,'Max Value':1.},
                {'Class':'rviz/Axes','Name':'Robot axes','Enabled':True,'Value':True,
                 'Reference Frame':'base_link','Length':1.,'Radius':.08}],
            'Tools':[{'Class':'rviz/Interact'},{'Class':'rviz/MoveCamera'},{'Class':'rviz/Select'}],
            'Views':{'Current':{'Class':'rviz/TopDownOrtho','Name':'Current View','Target Frame':'<Fixed Frame>',
                                'Scale':10.,'Angle':0.,'X':10.,'Y':0.},'Saved':[]}},
        'Window Geometry':{'Width':1260,'Height':1350,'X':x,'Y':30,'Hide Left Dock':False,'Hide Right Dock':True}}


def start(args):
    if ACTIVE.exists():
        previous = json.loads(ACTIVE.read_text())
        if not previous.get('stopped'): raise RuntimeError('An active session is already recorded; use status/stop first')
    for port in (11671,8765):
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            probe.bind(('127.0.0.1',port))
    for path in (args.bag,args.config,args.checkpoint):
        if not Path(path).is_file(): raise FileNotFoundError(path)
    rellis = args.platform == 'rellis'
    multi = bool(args.reference_speeds)
    if multi:
        if rellis:raise ValueError('Multi-speed profile currently uses BISTU calibration')
        args.reference_speed = args.reference_speeds[0]
        if len(args.reference_speeds) != 3 or len(set(args.reference_speeds)) != 3:
            raise ValueError('The comparison profile requires three distinct speeds')
        args.record_ema = True
    if rellis and not args.calibration_dir:
        raise ValueError('RELLIS requires --calibration-dir for the selected sequence')
    if rellis:
        for name in ['calibration.json','camera.launch']:
            if not (Path(args.calibration_dir)/name).is_file():raise FileNotFoundError(name)
    bag_probe = ('import json,rosbag,sys; b=rosbag.Bag(sys.argv[1]); '
                 'print(json.dumps(dict(start=b.get_start_time(),end=b.get_end_time()))); b.close()')
    bag_info = json.loads(ros_command(['/usr/bin/python3','-c',bag_probe,args.bag]))
    if args.mapping_window and args.mapping_window[1] > bag_info['end']-bag_info['start']:
        raise ValueError('Mapping window extends beyond the bag duration')
    output = ROOT/'.runtime'/datetime.datetime.now(ZoneInfo('Asia/Singapore')).strftime('cvar_%Y%m%d_%H%M%S')
    output.mkdir(parents=True)
    if rellis:shutil.copytree(args.calibration_dir,output/'calibration')
    image_topic='/uam2te/rellis/image_rect' if rellis else '/hikrobot_camera_L/image_rect'
    camera_info_topic=image_topic+'/camera_info'
    playback_topics=['/pylon_camera_node/image_raw','/pylon_camera_node/camera_info',
                     '/os1_cloud_node/points','/os1_cloud_node/imu'] if rellis else \
                    ['/hikrobot_camera_L/image_raw','/rslidar_points','/imu/data']
    manifest = dict(output=str(output),bag=args.bag,config=args.config,checkpoint=args.checkpoint,
                    reference_speed=args.reference_speed,ema_gamma=args.ema_gamma,support_min=args.support_min,
                    alphas=[.95,.05],play_rate=args.play_rate,
                    fusion_resolution_m=args.resolution,cost_encoding=args.cost_encoding,
                    cost_thresholds=args.cost_thresholds,
                    platform=args.platform,image_topic=image_topic,camera_info_topic=camera_info_topic,
                    pointcloud_topic='/cloud_registered_body' if rellis or multi else '/cloud_registered',
                    map_length=args.map_length,origin=[args.origin_x,args.origin_y],
                    calibration_dir=str(output/'calibration') if rellis else None,
                    trajectory_to_map_transform='identity: map -> camera_init' if rellis else 'identity: map -> odom -> camera_init',
                    ema_trace_dir=str(output/'ema_trace') if args.record_ema else '',
                    ros_master_uri='http://127.0.0.1:11671',worker_url='http://127.0.0.1:8765',display=args.display,
                    bag_start=bag_info['start'],bag_end=bag_info['end'],stopped=False)
    if multi:
        manifest.update(reference_speeds=args.reference_speeds,
                        score_channels=['similarity','speed_query_1','speed_query_2'])
    if args.mapping_window:
        manifest.update(mapping_window_offsets_s=args.mapping_window,
                        mapping_window_stamp_s=[bag_info['start']+t for t in args.mapping_window],
                        mapping_window_interval='[start,end)',
                        mapping_audit_path=str(output/'mapping_window_audit.json'))
    ACTIVE.write_text(json.dumps(manifest,indent=2)+'\n')
    (output/'session.json').write_text(json.dumps(manifest,indent=2)+'\n')
    master = xmlrpc.client.ServerProxy(manifest['ros_master_uri'])
    start_process(output,'master',['roscore','-p','11671'])
    wait_for(lambda: master.getPid('/cvar_control')[0] == 1,20,'ROS master')
    ros_command(['rosparam','set','/use_sim_time','true'])
    model_command=['bash',str(ROOT/'runtime/run_model.sh'),'--config',args.config,
                  '--checkpoint',args.checkpoint,'--reference-speed',str(args.reference_speed)]
    if multi:model_command += ['--reference-speeds',*map(str,args.reference_speeds)]
    start_process(output,'model',model_command,gpu=True)
    wait_for(lambda: urllib.request.urlopen(manifest['worker_url']+'/metadata',timeout=2).status == 200,
             120,'model worker')
    with urllib.request.urlopen(manifest['worker_url']+'/metadata',timeout=2) as response:
        (output/'prediction_metadata.json').write_text(json.dumps(json.load(response),indent=2)+'\n')
    launch=ROOT/'runtime'/('cvar_speed_experiment.launch' if multi else 'cvar_rellis_experiment.launch' if rellis else 'cvar_experiment.launch')
    mapping_command=['roslaunch',str(launch),
                  'reference_speed:='+str(args.reference_speed),'output_dir:='+str(output),
                  'resolution:='+str(args.resolution),'cost_encoding:='+args.cost_encoding,
                  'ema_gamma:='+str(args.ema_gamma),'cost_thresholds:='+json.dumps(args.cost_thresholds),
                  'support_min:='+str(args.support_min),
                  'map_length:='+str(args.map_length),'origin_x:='+str(args.origin_x),'origin_y:='+str(args.origin_y),
                  'ema_trace_dir:='+manifest['ema_trace_dir']]
    if rellis:mapping_command += ['calibration_dir:='+str(output/'calibration')]
    if multi:mapping_command += ['reference_speeds:='+json.dumps(args.reference_speeds),
                                  'score_channels:='+json.dumps(manifest['score_channels'])]
    if args.mapping_window:
        mapping_command += ['mapping_window:='+json.dumps(manifest['mapping_window_stamp_s']),
                            'mapping_audit_path:='+manifest['mapping_audit_path']]
    start_process(output,'mapping',mapping_command,gpu=True)
    expected = {'/laserMapping','/rellis_camera_rectifier' if rellis else '/camera_rect_mono','/navigation_inference','/navigation_elevation',
                '/navigation_risk_costmap','/navigation_risk_costmap_alpha_005'}
    def nodes_ready():
        state = master.getSystemState('/cvar_control')[2]
        present = {name for group in state for topic,names in group for name in names}
        return expected <= present
    wait_for(nodes_ready,90,'SLAM, rectifier and mapping nodes')
    start_process(output,'observer',['/usr/bin/python3',str(ROOT/'runtime/watch_cvar_experiment.py'),
                  '--output',str(output)],gpu=False)
    start_process(output,'network_visualizer',['/usr/bin/python3',str(ROOT/'runtime/network_visualizer.py'),
                  '--session',str(output)])
    for alpha,topic,x in [('095','/similarity_costmap',0),('005','/similarity_costmap_alpha_005',1280)]:
        config = output/('CVaR_alpha_'+alpha+'.rviz')
        view=rviz_config(topic,'.'+alpha[1:],x,args.cost_encoding,args.support_min)
        if rellis:view['Visualization Manager']['Views']['Current'].update(Scale=3.,X=120.,Y=0.)
        config.write_text(json.dumps(view,indent=2)+'\n')
        start_process(output,'rviz_'+alpha,['rviz','-d',str(config),'__name:=cvar_rviz_alpha_'+alpha],
                      display=args.display)
    start_process(output,'bag',['rosbag','play','--clock','--pause','--keep-alive','--queue=30','-r',str(args.play_rate),
                  args.bag,'--topics',*playback_topics,'__name:=cvar_bag'])
    wait_for(lambda: master.lookupService('/cvar_control','/cvar_bag/pause_playback')[0] == 1,
             60,'bag pause service')
    time.sleep(2)
    print(ros_command(['rosservice','call','/cvar_bag/pause_playback','data: false']).strip())
    print(json.dumps(manifest,indent=2))


def stop_process(pidfile):
    if not pidfile.exists(): return
    record = json.loads(pidfile.read_text())
    script = '''import os,pathlib,signal,sys,time
p=int(sys.argv[1]); expected=sys.argv[2]
def same():
 try: return pathlib.Path('/proc/%s/stat'%p).read_text().rsplit(')',1)[1].split()[19]==expected
 except (FileNotFoundError,ProcessLookupError): return False
if same():
 try: os.killpg(p,signal.SIGINT)
 except ProcessLookupError: sys.exit(0)
 for _ in range(80):
  if not same(): break
  time.sleep(.1)
 if same():
  try: os.killpg(p,signal.SIGTERM)
  except ProcessLookupError: pass
'''
    docker(shlex.join(['/usr/bin/python3','-c',script,str(record['pid']),record['start']]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['start','status','pause','resume','save','stop'])
    parser.add_argument('--bag')
    parser.add_argument('--config')
    parser.add_argument('--checkpoint')
    parser.add_argument('--platform',choices=['bistu','rellis'],default='bistu')
    parser.add_argument('--calibration-dir')
    parser.add_argument('--map-length',type=float,default=200.)
    parser.add_argument('--origin-x',type=float,default=-100.)
    parser.add_argument('--origin-y',type=float,default=-100.)
    parser.add_argument('--reference-speed',type=float,default=.8)
    parser.add_argument('--reference-speeds',type=float,nargs='+')
    parser.add_argument('--mapping-window',type=float,nargs=2,metavar=('START_S','END_S'))
    parser.add_argument('--resolution',type=float,default=.5)
    parser.add_argument('--cost-encoding',choices=['continuous','five_level'],default='five_level')
    parser.add_argument('--play-rate',type=float,default=.2)
    parser.add_argument('--ema-gamma',type=float,default=.2)
    parser.add_argument('--support-min',type=float,default=.15)
    parser.add_argument('--cost-thresholds',type=float,nargs=4,default=[.2,.35,.5,.65])
    parser.add_argument('--record-ema',action='store_true')
    parser.add_argument('--display',default=':1')
    args = parser.parse_args()
    if args.resolution <= 0 or args.play_rate <= 0: parser.error('Resolution and playback rate must be positive')
    if args.map_length<=0:parser.error('map-length must be positive')
    if args.mapping_window and (not all(math.isfinite(t) for t in args.mapping_window) or
                               not 0 <= args.mapping_window[0] < args.mapping_window[1]):
        parser.error('mapping-window must be increasing nonnegative seconds relative to bag start')
    if not 0 < args.ema_gamma < 1:parser.error('ema-gamma must be in (0,1)')
    if not 0 <= args.support_min <= 1:parser.error('support-min must be in [0,1]')
    if not (0 <= args.cost_thresholds[0] < args.cost_thresholds[1] < args.cost_thresholds[2] < args.cost_thresholds[3] <= 1):
        parser.error('cost-thresholds must be four increasing values in [0,1]')
    if args.action == 'start': return start(args)
    session = json.loads(ACTIVE.read_text())
    output = Path(session['output'])
    if args.action == 'status':
        state = output/'status.json'
        status = json.loads(state.read_text()) if state.exists() else session
        if status.get('playback_complete'): status['bag_elapsed']=status['bag_duration']
        print(json.dumps(status,indent=2))
    elif args.action in ('pause','resume'):
        state = output/'status.json'
        if state.exists() and json.loads(state.read_text()).get('playback_complete'):
            if args.action == 'pause':
                print('Playback is complete; the final scene is already frozen.')
                return
            raise RuntimeError('Playback is complete. Stop/start a fresh session to reset EMA before replaying.')
        print(ros_command(['rosservice','call','/cvar_bag/pause_playback',
                           'data: '+('true' if args.action == 'pause' else 'false')]))
    elif args.action == 'save':
        for name in ['/navigation_risk_costmap/save','/navigation_risk_costmap_alpha_005/save']:
            print(ros_command(['rosservice','call',name]))
        request=json.dumps(dict(resolution=.1,destination=str(output/'slam')))
        for name in ['/save_map','/save_pose']:
            print(ros_command(['rosservice','call',name,request]))
    elif args.action == 'stop':
        for name in ['bag','network_visualizer','observer','rviz_005','rviz_095','speed_maps','mapping','model','master']:
            stop_process(output/(name+'.pid.json'))
        session['stopped'] = True
        ACTIVE.write_text(json.dumps(session,indent=2)+'\n')


if __name__ == '__main__': main()
