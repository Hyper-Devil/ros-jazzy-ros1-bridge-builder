#!/usr/bin/python3
"""Publish completed, verified world costmaps for selectable RViz displays."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from geometry_msgs.msg import PoseStamped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('comparison', type=Path)
    parser.add_argument('--ranges', action='store_true', help='Publish the six maps in a 6 m / 8 m collection')
    args = parser.parse_args()
    folders = [args.comparison/('range_%dm'%r) for r in (6,8)] if args.ranges else [args.comparison]
    reports = [json.loads((folder/'comparison_manifest.json').read_text()) for folder in folders]
    entries = []
    for report in reports:
        if not report['gpu_posteriors_verified'] or not report['projection_geometry_verified']:
            raise ValueError('Use verified completed maps')
        prefix = '/range_comparison/r_%dm'%report['mapping_max_range_m'] if args.ranges else '/speed_comparison'
        entries.extend((entry,prefix) for entry in report['maps'])
    report = reports[0]
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('risk_math',root/'emcp/src/elevation_mapping_cupy/elevation_mapping_cupy/script/elevation_mapping_cupy/risk_mapping.py')
    risk = importlib.util.module_from_spec(spec); spec.loader.exec_module(risk)
    source = json.loads((Path(report['source_session'])/'session.json').read_text())
    rospy.init_node('range_comparison_maps' if args.ranges else 'speed_comparison_maps')
    publishers = []
    for entry,prefix in entries:
        speed = entry['reference_speed_m_s']
        with np.load(Path(entry['session'])/'ema_sweep/gamma_200/statistics_alpha_095.npz') as a:
            q = np.where(a['valid'],a['score_mean']-risk.cvar_coefficient(float(a['cvar_alpha']))*
                         np.sqrt(np.maximum(a['variance'],0)),np.nan)
            cost = risk.score_to_cost(q,'five_level',a['cost_thresholds'])
            message = OccupancyGrid()
            message.header.frame_id = 'map'
            message.header.stamp = rospy.Time.from_sec(source['bag_end'])
            message.info.resolution = float(a['resolution'])
            message.info.width = cost.shape[1]; message.info.height = cost.shape[0]
            message.info.origin.position.x = float(a['origin'][0])
            message.info.origin.position.y = float(a['origin'][1])
            message.info.origin.orientation.w = 1.
            message.data = cost.ravel().tolist()
        topic = prefix+'/v_%04d/costmap'%round(speed*1000)
        publisher = rospy.Publisher(topic,OccupancyGrid,queue_size=1,latch=True)
        publisher.publish(message)
        publishers.append(publisher)
        rospy.loginfo('Published v=%g m/s: %s',speed,topic)
    path = RosPath()
    path.header.frame_id = 'map'
    path.header.stamp = rospy.Time.from_sec(source['bag_end'])
    with Path(report.get('trajectory_source',str(Path(report['source_session'])/'odometry.csv'))).open() as stream:
        for row in csv.DictReader(stream):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.from_sec(float(row['stamp']))
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = [float(row[k]) for k in ['x','y','z']]
            pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w = [float(row[k]) for k in ['qx','qy','qz','qw']]
            path.poses.append(pose)
    publisher = rospy.Publisher(('/range_comparison' if args.ranges else '/speed_comparison')+'/trajectory',RosPath,queue_size=1,latch=True)
    publisher.publish(path); publishers.append(publisher)
    rospy.spin()


if __name__ == '__main__':
    main()
