#!/usr/bin/env python3
"""Isolated three-container sensor benchmark; owns and cleans its test processes."""
import argparse
import http.client
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import time
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]
ENV = ROOT/'runtime/env.sh'
ENDPOINT = ROOT/'runtime/benchmark_endpoint.py'


def docker(container, command, **kwargs):
    return subprocess.run(['docker', 'exec', container, 'bash', '-lc', command], check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--udp-only', action='store_true')
    parser.add_argument('--check-bridge', action='store_true',
                        help='Check late-join maps/TF, cmd_vel and custom messages instead of load testing')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    children, log_handles = [], []
    docker_endpoint = os.environ.get('DOCKER_HOST') or subprocess.check_output(
        ['docker', 'context', 'inspect', '--format', '{{.Endpoints.docker.Host}}'], text=True).strip()
    def start(container, mode, name, command):
        pidfile = f'/tmp/robotics-bench-{os.getpid()}-{name}.pid'
        env = ('export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; ' if args.udp_only else '')
        inner = f'echo $$ > {shlex.quote(pidfile)}; source {shlex.quote(str(ENV))} {mode}; {env}exec {command}'
        handle = open(output/(name+'.log'), 'w')
        log_handles.append(handle)
        process = subprocess.Popen(['docker', 'exec', container, 'bash', '-lc',
                                    'exec setsid --wait bash -c '+shlex.quote(inner)], stdout=handle, stderr=subprocess.STDOUT)
        children.append((container, pidfile, process))
        return process
    def cpu(container):
        # The inherited Noetic image masks /sys/fs/cgroup. Read Docker's own
        # cumulative counters instead of relying on the container mount layout.
        if not docker_endpoint.startswith('unix://'): return None
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.connect(docker_endpoint[len('unix://'):])
            request = f'GET /containers/{container}/stats?stream=false&one-shot=true HTTP/1.0\r\nHost: localhost\r\n\r\n'
            connection.sendall(request.encode())
            response = http.client.HTTPResponse(connection)
            response.begin()
            if response.status != 200: raise RuntimeError(response.read().decode())
            usage = json.loads(response.read()).get('cpu_stats', {}).get('cpu_usage', {}).get('total_usage')
            return usage/1000 if usage else None
    try:
        with socket.socket() as check: check.bind(('127.0.0.1', 11671))
        start('ros1-noetic', 'noetic', 'master', 'roscore -p 11671')
        for _ in range(100):
            try:
                if xmlrpc.client.ServerProxy('http://127.0.0.1:11671').getPid('/benchmark')[0] == 1: break
            except Exception: time.sleep(.1)
        else: raise RuntimeError('Test ROS Master did not start')
        topics = ROOT/'runtime/benchmark_topics.yaml'
        if args.check_bridge:
            topics = output/'check-topics.yaml'
            topics.write_text((ROOT/'runtime/topics.yaml').read_text()+'''
  - topic: /runtime_bench/grid_1to2
    type: grid_map_msgs/msg/GridMap
    queue_size: 1
  - topic: /runtime_bench/info_1to2
    type: elevation_map_msgs/msg/ChannelInfo
    queue_size: 1
''')
        start('ros1-ros2-bridge', 'bridge', 'bridge',
              'env BRIDGE_TOPICS='+shlex.quote(str(topics))+' bash '+shlex.quote(str(ROOT/'runtime/run_bridge.sh')))
        time.sleep(3.)
        if args.check_bridge:
            endpoint = shlex.quote(str(ROOT/'runtime/check_bridge.py'))
            pub = start('ros1-noetic', 'noetic', 'check_ros1',
                        f'/usr/bin/python3 {endpoint} --ros 1 --output '+shlex.quote(str(output/'ros1.json')))
            # ROS2 joins after ROS1 has published the latched map and static TF.
            time.sleep(5.)
            sub = start('ros2-jazzy', 'jazzy', 'check_ros2',
                        f'/usr/bin/python3 {endpoint} --ros 2 --output '+shlex.quote(str(output/'ros2.json')))
            assert sub.wait(timeout=30) == 0, 'ROS2 bridge check failed'
            assert pub.wait(timeout=20) == 0, 'ROS1 bridge check failed'
            result = {**json.loads((output/'ros1.json').read_text()), **json.loads((output/'ros2.json').read_text())}
            (output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
            print(json.dumps(result), flush=True)
            return
        containers = ['ros1-noetic', 'ros1-ros2-bridge', 'ros2-jazzy']
        results = []
        for kind, size, hz in [('image', 2764800, 30.), ('cloud', 1600000, 10.)]:
            for origin, destination in [(1, 2), (2, 1)]:
                name = f'{kind}_{origin}to{destination}'
                topic = '/runtime_bench/'+name
                common = f'{shlex.quote(str(ENDPOINT))} --kind {kind} --topic {topic} --size {size} --hz {hz} --duration {args.duration}'
                a, b = ('ros1-noetic' if origin == 1 else 'ros2-jazzy'), ('ros1-noetic' if destination == 1 else 'ros2-jazzy')
                sub = start(b, 'noetic' if destination == 1 else 'jazzy', name+'_sub',
                            f'/usr/bin/python3 {common} --ros {destination} --role sub --output {shlex.quote(str(output/(name+"_sub.json")))}')
                time.sleep(1.)
                before = {container: cpu(container) for container in containers}
                begun = time.monotonic()
                pub = start(a, 'noetic' if origin == 1 else 'jazzy', name+'_pub',
                            f'/usr/bin/python3 {common} --ros {origin} --role pub --output {shlex.quote(str(output/(name+"_pub.json")))}')
                assert pub.wait(timeout=args.duration+30) == 0, f'Publisher failed: {name}'
                assert sub.wait(timeout=20) == 0, f'Subscriber failed: {name}'
                elapsed = time.monotonic()-begun
                after = {container: cpu(container) for container in containers}
                usage = {container: ((after[container]-before[container])/1e6/elapsed*100
                                     if after[container] is not None and before[container] is not None else None)
                         for container in containers}
                sent = json.loads((output/(name+'_pub.json')).read_text())
                received = json.loads((output/(name+'_sub.json')).read_text())
                result = dict(name=name, **sent, **received, cpu_percent_of_one_core=usage)
                result['loss_percent'] = 100*(1-received['received']/sent['sent'])
                result['payload_MB_per_s'] = received['received_hz']*sent['payload_bytes']/1e6
                results.append(result)
                print(json.dumps(result), flush=True)
        (output/'result.json').write_text(json.dumps(dict(udp_only=args.udp_only, results=results,
            scope='Python endpoints + C++ bridge, same-host clock; synthetic payloads, not model inference'), indent=2)+'\n')
    finally:
        for container, pidfile, process in reversed(children):
            if process.poll() is None:
                docker(container, 'if test -f '+shlex.quote(pidfile)+'; then kill -INT -- -"$(cat '+shlex.quote(pidfile)+')" 2>/dev/null || true; fi')
                try: process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    docker(container, 'kill -KILL -- -"$(cat '+shlex.quote(pidfile)+')" 2>/dev/null || true')
                    process.wait(timeout=5)
        for handle in log_handles: handle.close()


if __name__ == '__main__': main()
