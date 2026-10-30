#!/usr/bin/python3
"""Publish the selected verified alpha maps from one range-filtered EMA state."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as RosPath


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('comparison', type=Path)
    args = parser.parse_args()
    folder = args.comparison.resolve()
    report = json.loads((folder/'range_comparison_manifest.json').read_text())
    assert report['raw_gpu_posteriors_verified'] and report['same_statistics_for_both_alphas']
    session = json.loads((folder/'session.json').read_text())
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('risk_math',root/
        'emcp/src/elevation_mapping_cupy/elevation_mapping_cupy/script/elevation_mapping_cupy/risk_mapping.py')
    risk = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(risk)
    rospy.init_node('range_alpha_comparison_maps')
    publishers = []
    for alpha in session.get('alphas', [.05,.95]):
        suffix = '%03d'%round(alpha*100)
        with np.load(folder/'ema_sweep/gamma_200'/('statistics_alpha_'+suffix+'.npz')) as a:
            q = np.where(a['valid'],a['score_mean']-risk.cvar_coefficient(alpha)*
                         np.sqrt(np.maximum(a['variance'],0)),np.nan)
            cost = risk.score_to_cost(q,'five_level',a['cost_thresholds'])
            msg = OccupancyGrid()
            msg.header.frame_id = 'map'
            msg.header.stamp = rospy.Time.from_sec(session['bag_end'])
            msg.info.resolution = float(a['resolution'])
            msg.info.height,msg.info.width = cost.shape
            msg.info.origin.position.x,msg.info.origin.position.y = map(float,a['origin'])
            msg.info.origin.orientation.w = 1.
            msg.data = cost.ravel().tolist()
        topic = '/range_comparison/alpha_'+suffix+'/costmap'
        pub = rospy.Publisher(topic,OccupancyGrid,queue_size=1,latch=True)
        pub.publish(msg)
        publishers.append(pub)
        rospy.loginfo('Published %s',topic)
    path = RosPath()
    path.header.frame_id = 'map'
    path.header.stamp = rospy.Time.from_sec(session['bag_end'])
    with (folder/'odometry.csv').open() as stream:
        for row in csv.DictReader(stream):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.from_sec(float(row['stamp']))
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = [float(row[k]) for k in ['x','y','z']]
            pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w = [float(row[k]) for k in ['qx','qy','qz','qw']]
            path.poses.append(pose)
    pub = rospy.Publisher('/range_comparison/trajectory',RosPath,queue_size=1,latch=True)
    pub.publish(path)
    publishers.append(pub)
    rospy.spin()


if __name__ == '__main__':
    main()
