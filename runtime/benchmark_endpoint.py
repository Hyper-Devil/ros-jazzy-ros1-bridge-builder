#!/usr/bin/env python3
"""Low-overhead Python sensor endpoints; run with each ROS's system Python."""
import argparse
from array import array
import json
from pathlib import Path
import struct
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros', type=int, choices=[1, 2], required=True)
    parser.add_argument('--role', choices=['pub', 'sub'], required=True)
    parser.add_argument('--kind', choices=['image', 'cloud'], default='image')
    parser.add_argument('--topic', required=True)
    parser.add_argument('--size', type=int, default=2764800)
    parser.add_argument('--hz', type=float, default=30.)
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if args.ros == 1:
        import rospy
        from sensor_msgs.msg import Image, PointCloud2, PointField
        rospy.init_node('runtime_bench_'+args.role, anonymous=True)
    else:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image, PointCloud2, PointField
        rclpy.init()
        node = rclpy.create_node('runtime_bench_'+args.role)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    cls = Image if args.kind == 'image' else PointCloud2
    records = []
    def callback(msg):
        now = time.time_ns()
        sequence = struct.unpack('!Q', memoryview(msg.data)[:8])[0]
        stamp = msg.header.stamp
        sec, nanosec = (stamp.secs, stamp.nsecs) if args.ros == 1 else (stamp.sec, stamp.nanosec)
        records.append((sequence, (now-sec*1000000000-nanosec)/1000000, now))
    if args.role == 'sub':
        if args.ros == 1:
            subscriber = rospy.Subscriber(args.topic, cls, callback, queue_size=1, buff_size=32*1024*1024)
        else:
            subscriber = node.create_subscription(cls, args.topic, callback, qos)
        end = time.monotonic()+args.duration+8
        while time.monotonic() < end:
            if args.ros == 2: rclpy.spin_once(node, timeout_sec=.05)
            else: time.sleep(.05)
        sequences = sorted(set(row[0] for row in records))
        latency = sorted(row[1] for row in records)
        percentile = lambda q: latency[min(len(latency)-1, int((len(latency)-1)*q))] if latency else None
        result = dict(received=len(sequences), first=sequences[0] if sequences else None,
                      last=sequences[-1] if sequences else None, p50_ms=percentile(.5),
                      p95_ms=percentile(.95), p99_ms=percentile(.99),
                      received_hz=(len(records)-1)/((records[-1][2]-records[0][2])/1e9) if len(records)>1 else 0,
                      missing_within_stream=(sequences[-1]-sequences[0]+1-len(sequences)) if sequences else None)
    else:
        if args.ros == 1: publisher = rospy.Publisher(args.topic, cls, queue_size=1)
        else: publisher = node.create_publisher(cls, args.topic, qos)
        message = cls()
        message.header.frame_id = 'runtime_bench'
        if args.kind == 'image':
            message.height, message.width = 1, args.size//3
            message.encoding, message.step = 'rgb8', message.width*3
            size = message.step
        else:
            message.height, message.width, message.point_step = 1, args.size//16, 16
            message.row_step, message.is_dense = message.width*16, True
            message.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                              for i, name in enumerate(['x','y','z','intensity'])]
            size = message.row_step
        payload = bytearray(size) if args.ros == 1 else array('B', bytes(size))
        time.sleep(2.)
        started, sent = time.monotonic(), 0
        while time.monotonic()-started < args.duration:
            stamp = time.time_ns()
            if args.ros == 1:
                message.header.stamp = rospy.Time(stamp//1000000000, stamp%1000000000)
                struct.pack_into('!Q', payload, 0, sent)
                message.data = bytes(payload)
            else:
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1000000000)
                struct.pack_into('!Q', payload, 0, sent)
                message.data = payload
            publisher.publish(message)
            sent += 1
            pause = started+sent/args.hz-time.monotonic()
            if pause > 0: time.sleep(pause)
        result = dict(sent=sent, payload_bytes=size, requested_hz=args.hz, elapsed=time.monotonic()-started)
    Path(args.output).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
    if args.ros == 2: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
