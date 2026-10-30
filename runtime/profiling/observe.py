#!/usr/bin/python3
"""Timestamp ROS arrivals without deserializing large image/map arrays."""
import json
import os
from pathlib import Path
import struct
import threading
import time
import rospy

root=Path(os.environ['UAM2TE_PROFILE_DIR'])
handle=(root/'topics.jsonl').open('w',buffering=1)
lock=threading.Lock()
topics=['/hikrobot_camera_L/image_raw','/hikrobot_camera_L/image_rect','/rslidar_points',
        '/Odometry','/cloud_registered_body','/uam2te/conservative_score','/uam2te/mapping_score',
        '/navigation_elevation/filtered_elevation_map','/similarity_costmap']
def callback(msg,topic):
    now=time.perf_counter_ns()
    _,sec,nsec=struct.unpack_from('<III',msg._buff,0)
    with lock:handle.write(json.dumps(dict(topic=topic,wall_ns=now,stamp_ns=sec*1000000000+nsec,bytes=len(msg._buff)),separators=(',',':'))+'\n')
rospy.init_node('deployment_profile_observer')
subs=[rospy.Subscriber(topic,rospy.AnyMsg,callback,callback_args=topic,queue_size=100,buff_size=64*1024*1024) for topic in topics]
(root/'observer_ready').write_text('ready\n')
rospy.spin()
