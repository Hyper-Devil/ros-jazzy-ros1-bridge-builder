#!/usr/bin/env python3
"""ROS endpoints for checking late-join maps/TF and custom bridge conversions."""
import argparse
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros', type=int, choices=[1, 2], required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.ros == 1:
        import rospy
        from geometry_msgs.msg import TransformStamped, Twist
        from tf2_msgs.msg import TFMessage
        from nav_msgs.msg import OccupancyGrid
        from grid_map_msgs.msg import GridMap
        from elevation_map_msgs.msg import ChannelInfo
        from std_msgs.msg import Float32MultiArray
        rospy.init_node('runtime_check_ros1')
        received = []
        subscriber = rospy.Subscriber('/cmd_vel', Twist, lambda m: received.append((m.linear.x, m.angular.z)))
        map_pub = rospy.Publisher('/similarity_costmap', OccupancyGrid, queue_size=1, latch=True)
        tf_pub = rospy.Publisher('/tf_static', TFMessage, queue_size=1, latch=True)
        grid_pub = rospy.Publisher('/runtime_bench/grid_1to2', GridMap, queue_size=1)
        info_pub = rospy.Publisher('/runtime_bench/info_1to2', ChannelInfo, queue_size=1)
        time.sleep(2)
        occupancy = OccupancyGrid()
        occupancy.header.frame_id = 'runtime_check_map'
        occupancy.info.resolution, occupancy.info.width, occupancy.info.height = .1, 3, 2
        occupancy.info.origin.orientation.w = 1.
        occupancy.data = [0, 25, 50, 75, 100, -1]
        map_pub.publish(occupancy)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'runtime_check_map', 'runtime_check_base'
        transform.transform.translation.x, transform.transform.rotation.w = 1.25, 1.
        tf_pub.publish(TFMessage([transform]))
        grid = GridMap()
        grid.info.header.frame_id, grid.info.header.stamp = 'runtime_check_map', rospy.Time(123, 456)
        grid.info.resolution, grid.info.length_x, grid.info.length_y = .1, .3, .2
        grid.info.pose.orientation.w = 1.
        grid.layers = ['risk']
        grid.data = [Float32MultiArray(data=[0., .25, .5, .75, 1., -1.])]
        channel = ChannelInfo()
        channel.header.frame_id, channel.channels = 'runtime_check_camera', ['similarity']
        end = time.monotonic()+12
        while time.monotonic() < end:
            grid_pub.publish(grid)
            info_pub.publish(channel)
            time.sleep(.2)
        assert any(abs(x-1.25)<1e-6 and abs(z-.3)<1e-6 for x,z in received), received
        result = {'cmd_vel_2to1': True, 'cmd_vel_messages': len(received)}
    else:
        import rclpy
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from geometry_msgs.msg import Twist
        from tf2_msgs.msg import TFMessage
        from nav_msgs.msg import OccupancyGrid
        from grid_map_msgs.msg import GridMap
        from elevation_map_msgs.msg import ChannelInfo
        rclpy.init()
        node = rclpy.create_node('runtime_check_ros2')
        result = {}
        transient = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        def on_map(msg):
            assert msg.header.frame_id == 'runtime_check_map'
            assert list(msg.data) == [0,25,50,75,100,-1], msg.data
            result['late_join_costmap_1to2'] = True
        def on_tf(msg):
            if any(t.child_frame_id == 'runtime_check_base' and abs(t.transform.translation.x-1.25)<1e-6
                   for t in msg.transforms): result['late_join_tf_static_1to2'] = True
        def on_grid(msg):
            assert msg.header.frame_id == 'runtime_check_map'
            assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (123,456)
            assert msg.layers == ['risk'] and list(msg.data[0].data) == [0.,.25,.5,.75,1.,-1.]
            result['grid_map_header_and_data_1to2'] = True
        def on_info(msg):
            assert msg.header.frame_id == 'runtime_check_camera' and msg.channels == ['similarity']
            result['channel_info_1to2'] = True
        subscriptions = [node.create_subscription(OccupancyGrid, '/similarity_costmap', on_map, transient),
                         node.create_subscription(TFMessage, '/tf_static', on_tf, transient),
                         node.create_subscription(GridMap, '/runtime_bench/grid_1to2', on_grid, 10),
                         node.create_subscription(ChannelInfo, '/runtime_bench/info_1to2', on_info, 10)]
        publisher = node.create_publisher(Twist, '/cmd_vel', 1)
        command = Twist()
        command.linear.x, command.angular.z = 1.25, .3
        started = time.monotonic()
        while time.monotonic()-started < 8:
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=.1)
        assert len(result) == 4, result
        node.destroy_node()
        rclpy.shutdown()
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__': main()
