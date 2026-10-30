#!/usr/bin/env python3
"""Inspect host UDP limits; explicitly apply or restore a recorded temporary change."""
import argparse
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
TARGET = {'net.core.rmem_max': 67108864, 'net.core.wmem_max': 67108864,
          'net.ipv4.ipfrag_high_thresh': 134217728, 'net.ipv4.ipfrag_time': 3}


def read(key):
    return int(subprocess.check_output(['sysctl', '-n', key], text=True).strip())


def counters():
    result = {}
    lines = Path('/proc/net/snmp').read_text().splitlines()
    for header, values in zip(lines[::2], lines[1::2]):
        keys, numbers = header.split(), values.split()
        if keys[0] in ('Udp:', 'Ip:'):
            result[keys[0][:-1]] = dict(zip(keys[1:], map(int, numbers[1:])))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--apply', action='store_true')
    mode.add_argument('--restore', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    before = {key: read(key) for key in TARGET}
    report = dict(time_ns=time.time_ns(), values=before, recommended=TARGET, counters=counters())
    if args.apply:
        output = args.output or ROOT/'.runtime'/f'network-before-{time.time_ns()}.json'
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2)+'\n')
        for key, value in TARGET.items():
            subprocess.run(['sysctl', '-w', f'{key}={value}'], check=True)
        report['backup'] = str(output)
        report['after'] = {key: read(key) for key in TARGET}
    elif args.restore:
        previous = json.loads(args.restore.read_text())['values']
        for key in TARGET:
            subprocess.run(['sysctl', '-w', f'{key}={previous[key]}'], check=True)
        report['after'] = {key: read(key) for key in TARGET}
    elif args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
