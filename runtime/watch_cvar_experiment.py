#!/usr/bin/python3
"""Observe the experiment, save comparison maps and retain their provenance."""
import argparse
import csv
import json
from pathlib import Path
import time
import threading
import shlex
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rospy
import rosnode
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from fast_lio_sam.srv import save_map, save_pose


def freeze_scene(poses, session):
    """Keep the completed scene viewable without advancing simulation time.

    Original fixed calibration edges and the final SLAM pose become latched TF.
    This is only called once sensor playback has ended; replay requires a new
    session, which removes this observer and its frozen transforms.
    """
    assert poses, 'No final SLAM pose available'
    root=Path(__file__).resolve().parents[2]
    rellis=session.get('platform')=='rellis'
    launch=ET.parse(root/'stepp_adapter/third_party/FAST_LIO_SAM/FAST_LIO_SAM/launch/mapping_rs_bistu.launch') if not rellis else None
    transforms=[]; nodes=['/cvar_bag']
    # RELLIS uses latched tf2 static publishers that remain valid after playback.
    # BISTU's periodic tf publishers need the existing conversion below.
    for node in ([] if rellis else launch.getroot().findall('node')):
        if node.get('pkg') != 'tf' or node.get('type') != 'static_transform_publisher': continue
        args=shlex.split(node.get('args'))
        x,y,z,yaw,pitch,roll=map(float,args[:6])
        transform=TransformStamped()
        transform.header.frame_id,transform.child_frame_id=args[6:8]
        transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=x,y,z
        q=quaternion_from_euler(roll,pitch,yaw)
        transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w=q
        transforms.append(transform)
        nodes.append('/'+node.get('name'))
    rosnode.kill_nodes(nodes)
    time.sleep(.5)
    _,x,y,z,qx,qy,qz,qw=poses[-1]
    transform=TransformStamped()
    transform.header.frame_id,transform.child_frame_id='camera_init','body'
    transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=x,y,z
    transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w=qx,qy,qz,qw
    transforms.append(transform)
    broadcaster=tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)
    return broadcaster


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    output = args.output
    session = json.loads((output/'session.json').read_text())
    rospy.init_node('cvar_experiment_observer')
    lock = threading.RLock()
    maps, counts, poses, frames = {}, {}, [], {}
    mapping_stamps, before_window_observed = [], [0]
    window = session.get('mapping_window_stamp_s')
    clock, end_seen = [None], [None]
    bridge = CvBridge()
    def count(name): counts[name] = counts.get(name,0)+1
    def on_map(msg,alpha):
        with lock:
            count('map_'+alpha)
            maps[alpha] = msg
            if window and clock[0] is not None and clock[0] < window[0]:
                before_window_observed[0] = max(before_window_observed[0], sum(v >= 0 for v in msg.data))
    def on_mapping(msg):
        with lock:
            mapping_stamps.append(msg.header.stamp.to_nsec())
    def on_odom(msg):
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        with lock:
            count('odometry')
            poses.append([msg.header.stamp.to_sec(),p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    def on_clock(msg):
        clock[0] = msg.clock.to_sec()
        if clock[0] >= session['bag_end']-.1 and end_seen[0] is None: end_seen[0] = time.monotonic()
    def on_image(msg):
        elapsed = msg.header.stamp.to_sec()-session['bag_start']
        for target in (20,50):
            if elapsed >= target and target not in frames:
                path=output/('camera_t%03d.png'%target)
                if cv2.imwrite(str(path),bridge.imgmsg_to_cv2(msg,'bgr8')):
                    frames[target] = dict(path=str(path),stamp=msg.header.stamp.to_sec(),frame=msg.header.frame_id)
    subscriptions = [rospy.Subscriber('/clock',Clock,on_clock,queue_size=1),
        rospy.Subscriber('/Odometry',Odometry,on_odom,queue_size=10),
        rospy.Subscriber('/uam2te/conservative_score',Image,lambda msg:count('predictions'),queue_size=1,buff_size=32*1024*1024),
        rospy.Subscriber(session.get('image_topic','/hikrobot_camera_L/image_rect'),Image,on_image,queue_size=1,buff_size=32*1024*1024)]
    if window:
        subscriptions.append(rospy.Subscriber('/uam2te/mapping_score',Image,on_mapping,
                                              queue_size=1,buff_size=128*1024*1024))
    for alpha,topic in [('095','/similarity_costmap'),('005','/similarity_costmap_alpha_005')]:
        subscriptions.append(rospy.Subscriber(topic,OccupancyGrid,on_map,callback_args=alpha,queue_size=1,
                                               buff_size=32*1024*1024))
    finished = (output/'result.json').exists()
    frozen = None
    if finished:
        previous=json.loads((output/'result.json').read_text())
        counts.update(previous['counts'])
        frames.update({int(key):value for key,value in previous['frames'].items()})
        with (output/'odometry.csv').open() as stream:
            poses.extend([[float(row[key]) for key in ['stamp','x','y','z','qx','qy','qz','qw']]
                          for row in csv.DictReader(stream)])
        frozen=freeze_scene(poses,session)
    while not rospy.is_shutdown():
        with lock:
            status = dict(clock=clock[0],bag_elapsed=min(clock[0]-session['bag_start'],session['bag_end']-session['bag_start']) if clock[0] else None,
                          bag_duration=session['bag_end']-session['bag_start'],counts=dict(counts),
                          maps={},frames=frames,playback_complete=finished)
            if finished: status['bag_elapsed']=session['bag_end']-session['bag_start']
            for alpha,msg in maps.items():
                data = np.asarray(msg.data,dtype=np.int8)
                valid = data >= 0
                status['maps'][alpha] = dict(observed_cells=int(valid.sum()),
                    min=int(data[valid].min()) if valid.any() else None,
                    max=int(data[valid].max()) if valid.any() else None,
                    mean=float(data[valid].mean()) if valid.any() else None)
            (output/'status.tmp').write_text(json.dumps(status,indent=2)+'\n')
            (output/'status.tmp').replace(output/'status.json')
        if not finished and end_seen[0] is not None and time.monotonic()-end_seen[0] > 10:
            try:
                assert len(maps) == 2 and counts.get('predictions',0) > 0, 'No usable model maps'
                for service in ['/navigation_risk_costmap/save','/navigation_risk_costmap_alpha_005/save']:
                    response = rospy.ServiceProxy(service,Trigger)()
                    assert response.success,response.message
                with np.load(output/'statistics_alpha_095.npz') as high, np.load(output/'statistics_alpha_005.npz') as low:
                    assert np.array_equal(high['count'],low['count']), 'Exporter histories differ'
                    assert np.array_equal(high['score_mean'],low['score_mean']), 'Exporter means differ'
                    assert np.array_equal(high['variance'],low['variance']), 'Exporter variances differ'
                    valid = high['valid']
                    assert valid.any()
                    resolution = float(high['resolution'])
                    assert np.isclose(resolution,session.get('fusion_resolution_m',.1))
                    with lock:
                        a=np.asarray(maps['095'].data,dtype=np.int16)
                        b=np.asarray(maps['005'].data,dtype=np.int16)
                    seen = (a>=0)&(b>=0)
                    assert np.all(a[seen]>=b[seen]), 'CVaR monotonicity check failed'
                    encoding=session.get('cost_encoding','continuous')
                    cost_values={alpha:sorted(set(msg.data)) for alpha,msg in maps.items()}
                    if encoding == 'five_level':
                        for values in cost_values.values():
                            assert set(values)<={-1,0,30,50,70,100}, values
                    report=dict(passed=True,observed_cells=int(valid.sum()),
                        fusion_resolution_m=resolution,cost_encoding=encoding,cost_values=cost_values,
                        max_observations=float(high['count'].max()),
                        same_statistics_for_both_alphas=True,
                        alpha_095_cost_ge_005=True,strictly_higher_cells=int((a[seen]>b[seen]).sum()),
                        reference_speed=session['reference_speed'],ema_gamma=session['ema_gamma'],
                        support_min=session.get('support_min',0.),
                        counts=dict(counts),frames=frames)
                for alpha,msg in maps.items():
                    data=np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
                    rgb=np.full((*data.shape,3),(158,177,177),dtype=np.uint8)
                    valid=data>=0; gray=np.rint(255*(1-data[valid]/100.)).astype(np.uint8)
                    rgb[valid]=gray[:,None]
                    cv2.imwrite(str(output/('costmap_alpha_'+alpha+'.png')),cv2.cvtColor(np.flipud(rgb),cv2.COLOR_RGB2BGR))
                map_result=rospy.ServiceProxy('/save_map',save_map)(.1,str(output/'slam'))
                pose_result=rospy.ServiceProxy('/save_pose',save_pose)(.1,str(output/'slam'))
                report.update(slam_map_saved=map_result.success,slam_poses_saved=pose_result.success)
                if window:
                    audit=json.loads((output/'mapping_window_audit.json').read_text())
                    lo,hi=[rospy.Time.from_sec(t).to_nsec() for t in window]
                    assert audit['window_stamp_ns']==[lo,hi]
                    admitted=[stamp for stamp,allowed in audit['decisions'] if allowed]
                    assert admitted and mapping_stamps, 'No mapping frames in the requested window'
                    assert all(lo<=stamp<hi for stamp in admitted+mapping_stamps), 'Mapping outside window'
                    assert set(mapping_stamps)<=set(admitted), 'Unexpected mapping publisher'
                    assert before_window_observed[0]==0, 'Map history existed before window'
                    assert audit['predicted_frames']==counts['predictions'], 'Prediction audit is incomplete'
                    before=sum(stamp<lo for stamp,allowed in audit['decisions'])
                    after=sum(stamp>=hi for stamp,allowed in audit['decisions'])
                    assert before and after, 'Full-bag inference was not retained'
                    report['mapping_window_validation']=dict(passed=True,interval='[start,end)',
                        offsets_s=session['mapping_window_offsets_s'],window_stamp_ns=[lo,hi],
                        admitted_mapping_frames=len(admitted),independently_received_frames=len(mapping_stamps),
                        admitted_first_last_stamp_ns=[min(admitted),max(admitted)],
                        predicted_frames_before=before,predicted_frames_after=after,
                        observed_cells_before_window=before_window_observed[0],
                        full_bag_playback_and_slam=True)
                with (output/'odometry.csv').open('w') as stream:
                    writer=csv.writer(stream);writer.writerow(['stamp','x','y','z','qx','qy','qz','qw']);writer.writerows(poses)
                (output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
                print(json.dumps(report),flush=True)
                frozen=freeze_scene(poses,session)
                finished=True
            except Exception as exc:
                rospy.logerr('Final experiment snapshot failed: %s',exc)
                (output/'snapshot_error.txt').write_text(str(exc)+'\n')
        time.sleep(2)


if __name__ == '__main__': main()
