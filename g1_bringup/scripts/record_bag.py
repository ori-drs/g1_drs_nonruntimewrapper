#!/usr/bin/env python3
"""Record a G1 dataset bag on PC2: robot state, Livox lidar and IMU, RealSense colour and depth.

Start the robot stack and the camera first (g1_drs_nonruntimewrapper), then record:

    ros2 launch g1_bringup robot.launch.xml camera:=true
    ros2 run g1_bringup record_bag.py --label lab_walk                  # Ctrl+C to stop
    ros2 run g1_bringup record_bag.py --label lab_walk --duration 120
    ros2 run g1_bringup record_bag.py --groups state lidar static       # no camera
    ros2 run g1_bringup record_bag.py --dry-run                         # checks only

The same file is kept in two repositories, identical (check with diff):
  g1_drs_nonruntimewrapper/g1_bringup/scripts/record_bag.py   run with `ros2 run g1_bringup ...`
  g1_drs/tools/recording/record_bag.py                        run with `python3 record_bag.py ...`
The topics are listed in groups in recording.yaml: next to the script (g1_drs), or in
g1_bringup/config (g1_drs_nonruntimewrapper). --config selects another file.

Before recording, it checks that every topic has a publisher, that PC2's clock follows PC1
(chrony) and that large messages can be received without loss (net.core.rmem_max). The recorder
runs with a 32 MB DDS receive buffer, a queue of 2 s per topic and, on Foxy, a write cache.
Without these, ros2 bag record on PC2 lost 2-15 % of the images, lidar scans and 1 kHz state
messages.

After recording, it checks each topic's message rate and writes into the bag folder:
recording_info.yaml (checks, publishers, clock status, command) and robot_description.urdf (the
URDF that robot_state_publisher used; /robot_description itself is not recorded, because when
replayed it would replace the URDF of the replay tools).
"""

import argparse
import datetime
import importlib
import math
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time

import yaml

RECEIVE_BUFFER_BYTES = 32 * 1024 * 1024
CYCLONE_RECEIVE_BUFFER_XML = ('<CycloneDDS><Domain><Internal><SocketReceiveBufferSize max="32MB"/>'
                              '</Internal></Domain></CycloneDDS>')
FOXY_CACHE_MESSAGES = 200        # Foxy counts the cache in messages (later distros: bytes)
QUEUE_SECONDS = 2.0              # recorder queue per topic, in seconds of messages
MIN_FREE_GB = 20.0               # ~5 min at the full data rate (~60 MB/s)
MAX_CLOCK_OFFSET_S = 1e-3
RATE_TOLERANCE = 0.95            # flag topics below 95 % of their nominal rate
RMEM_FIX = ('sudo sysctl -w net.core.rmem_max=33554432   '
            '# persistent: echo net.core.rmem_max=33554432 | '
            'sudo tee /etc/sysctl.d/60-dds-receive-buffer.conf')


def default_config_path():
    """recording.yaml next to the script, in ../config (g1_bringup source), or in g1_bringup's share."""
    script_dir = Path(__file__).resolve().parent
    for candidate in (script_dir / 'recording.yaml', script_dir.parent / 'config' / 'recording.yaml'):
        if candidate.is_file():
            return candidate
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('g1_bringup')) / 'config' / 'recording.yaml'
    except Exception:
        sys.exit('ERROR: recording.yaml not found; pass --config')


def select_topics(config, groups, extra_topics):
    topics = {}
    for group in groups:
        if group not in config['groups']:
            sys.exit(f'ERROR: unknown group "{group}"; groups: {", ".join(config["groups"])}')
        topics.update(config['groups'][group] or {})
    for topic in extra_topics:
        topics.setdefault(topic, {})
    return topics


def discover(topics, timeout_s):
    """Types, publishers and publisher reliability of each topic (waits for discovery).

    Also returns the URDF from /robot_description (None if nobody publishes it).
    """
    import rclpy
    from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('record_bag_preflight')
    urdf = []
    node.create_subscription(String, '/robot_description', lambda msg: urdf.append(msg.data),
                             QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
    try:
        deadline = time.monotonic() + timeout_s
        while True:
            types = dict(node.get_topic_names_and_types())
            if (all(topic in types for topic in topics) and urdf) or time.monotonic() > deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.2)
        settle = time.monotonic() + 0.5  # let the publisher list settle
        while time.monotonic() < settle:
            rclpy.spin_once(node, timeout_sec=0.1)
        info = {}
        for topic in topics:
            publishers = node.get_publishers_info_by_topic(topic) if topic in types else []
            # bare DDS writers (PC1) carry placeholder node names such as _CREATED_BY_BARE_DDS_APP_
            names = sorted({'(non-ROS, PC1)' if p.node_name.startswith('_')
                            else f'{p.node_namespace.rstrip("/")}/{p.node_name}' for p in publishers})
            info[topic] = {
                'types': sorted(types.get(topic, [])),
                'publishers': names,
                'reliable': bool(publishers) and all(
                    p.qos_profile.reliability == QoSReliabilityPolicy.RELIABLE for p in publishers),
            }
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return info, (urdf[-1] if urdf else None)


def type_support_missing(type_names):
    """Message types whose Python package cannot be imported (workspace not sourced)."""
    missing = []
    for type_name in type_names:
        package, _, name = type_name.split('/')
        try:
            getattr(importlib.import_module(f'{package}.msg'), name)
        except (ImportError, AttributeError):
            missing.append(type_name)
    return missing


def chrony_status():
    if not shutil.which('chronyc'):
        return None
    try:
        out = subprocess.run(['chronyc', '-c', 'tracking'], capture_output=True, text=True,
                             timeout=5, check=False).stdout.strip().split(',')
        return {'reference': out[1], 'system_time_offset_s': float(out[4]),
                'rms_offset_s': float(out[6]), 'leap_status': out[13]}
    except (IndexError, ValueError, subprocess.TimeoutExpired):
        return None


def receive_buffer_max():
    try:
        return int(Path('/proc/sys/net/core/rmem_max').read_text())
    except (OSError, ValueError):
        return None


def qos_overrides(topics, info):
    """Recorder subscription QoS: a deep queue, reliability matching the publishers."""
    overrides = {}
    for topic, spec in topics.items():
        if spec.get('latched'):
            continue  # rosbag2 matches transient-local publishers by itself
        overrides[topic] = {
            'history': 'keep_last',
            'depth': max(20, int(math.ceil(QUEUE_SECONDS * spec.get('rate', 100)))),
            'reliability': 'reliable' if info[topic]['reliable'] else 'best_effort',
            'durability': 'volatile',
        }
    return overrides


def rate_check(bag_path, topics):
    metadata = yaml.safe_load((bag_path / 'metadata.yaml').read_text())
    metadata = metadata['rosbag2_bagfile_information']
    duration = metadata['duration']['nanoseconds'] / 1e9
    counts = {entry['topic_metadata']['name']: entry['message_count']
              for entry in metadata['topics_with_message_count']}
    result = {}
    for topic, spec in topics.items():
        count = counts.get(topic, 0)
        row = {'messages': count}
        if count == 0:
            row['status'] = 'MISSING'
        elif spec.get('latched') or not spec.get('rate'):
            row['status'] = 'OK'
        else:
            rate = count / duration if duration > 0 else 0.0
            row.update(rate_hz=round(rate, 1), nominal_hz=spec['rate'],
                       status='OK' if rate >= RATE_TOLERANCE * spec['rate'] else 'LOW')
        result[topic] = row
    return duration, result


def main():
    parser = argparse.ArgumentParser(
        description='Record a G1 dataset bag on PC2', epilog=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output-dir', default='~/logs/raw/online_rosbag_output',
                        help='Parent folder of the bag folders (default: %(default)s)')
    parser.add_argument('--label', default='g1', help='Bag folder label (default: %(default)s)')
    parser.add_argument('--groups', nargs='+', metavar='GROUP',
                        help='Topic groups from the config (default: its default_groups)')
    parser.add_argument('--add-groups', nargs='+', default=[], metavar='GROUP',
                        help='Groups added to the default ones, e.g. torso_imu raw')
    parser.add_argument('--topics', nargs='+', default=[], metavar='TOPIC',
                        help='Extra topics to record')
    parser.add_argument('--duration', type=float, help='Stop after this many seconds')
    parser.add_argument('--max-bag-size', type=int, default=10_000_000_000,
                        help='Split the bag at this size [bytes] (default: 10 GB)')
    parser.add_argument('--config', type=Path, default=None,
                        help='Topic group file (default: g1_bringup/config/recording.yaml)')
    parser.add_argument('--discovery-timeout', type=float, default=5.0,
                        help='Seconds to wait for all topics to appear (default: %(default)s)')
    parser.add_argument('--force', action='store_true',
                        help='Record even if checks fail (missing topics are recorded once they appear)')
    parser.add_argument('--dry-run', action='store_true', help='Run the checks, do not record')
    args = parser.parse_args()

    if not shutil.which('ros2'):
        sys.exit('ERROR: ros2 is not on PATH. Source the ROS 2 workspace first.')
    config = yaml.safe_load((args.config or default_config_path()).read_text())
    groups = (args.groups or config['default_groups']) + args.add_groups
    topics = select_topics(config, groups, args.topics)

    label = re.sub(r'[^a-zA-Z0-9_-]', '_', args.label).strip('_') or 'g1'
    parent = Path(args.output_dir).expanduser().resolve()
    bag_path = parent / f'record_{label}_{datetime.datetime.now():%Y-%m-%d-%H-%M-%S}'
    distro = os.environ.get('ROS_DISTRO', '')
    rmw = os.environ.get('RMW_IMPLEMENTATION', '')

    # ---- checks
    print(f'Groups: {" ".join(groups)}    discovering {len(topics)} topics ...', flush=True)
    info, urdf = discover(list(topics), args.discovery_timeout)
    problems = []
    print(f'\n{"topic":36s} {"type":40s} publishers')
    for topic in topics:
        entry = info[topic]
        types = ', '.join(entry['types']) or '-'
        publishers = ', '.join(entry['publishers']) or 'NONE'
        print(f'{topic:36s} {types:40s} {publishers}')
        if not entry['publishers']:
            problems.append(f'{topic} has no publisher')
    if urdf is None:
        print('WARNING: no /robot_description (is robot.launch.xml running?); '
              'robot_description.urdf will not be saved.')
    missing_types = type_support_missing(sorted({t for e in info.values() for t in e['types']}))
    if missing_types:
        problems.append(f'message definitions not found: {", ".join(missing_types)} '
                        '(source the workspace that builds them)')

    rmem = receive_buffer_max()
    if rmem is not None and rmem < RECEIVE_BUFFER_BYTES:
        problems.append(f'net.core.rmem_max is {rmem} bytes: images and lidar scans will be lost.'
                        f'\n    Fix: {RMEM_FIX}')
    chrony = chrony_status()
    if chrony is not None:
        print(f'\nClock: chrony reference {chrony["reference"]}, offset '
              f'{chrony["system_time_offset_s"] * 1e3:+.3f} ms, {chrony["leap_status"]}')
        if (chrony['leap_status'] != 'Normal'
                or abs(chrony['system_time_offset_s']) > MAX_CLOCK_OFFSET_S):
            problems.append('PC2 clock is not synchronized to PC1 '
                            '(sudo service chrony restart; see g1_drs tools/pc2_time_sync)')
    parent.mkdir(parents=True, exist_ok=True)
    free_gb = shutil.disk_usage(parent).free / 1e9
    print(f'Free space in {parent}: {free_gb:.0f} GB')
    if free_gb < MIN_FREE_GB:
        problems.append(f'only {free_gb:.1f} GB free')

    if problems:
        print('\nChecks failed:\n  - ' + '\n  - '.join(problems))
        if not args.force:
            sys.exit('Not recording. Fix the above or pass --force.')
        print('--force: recording anyway.')
    else:
        print('\nAll checks passed.')

    # ---- recorder setup
    with tempfile.NamedTemporaryFile('w', suffix='_qos.yaml', dir=parent, delete=False) as qos_file:
        yaml.safe_dump(qos_overrides(topics, info), qos_file, sort_keys=False)
    command = ['ros2', 'bag', 'record', '-o', str(bag_path), '-b', str(args.max_bag_size),
               '--qos-profile-overrides-path', qos_file.name]
    if distro == 'foxy':
        command += ['--max-cache-size', str(FOXY_CACHE_MESSAGES)]
    command += list(topics)
    env = os.environ.copy()
    if 'cyclonedds' in rmw:
        uri = env.get('CYCLONEDDS_URI', '')
        env['CYCLONEDDS_URI'] = f'{uri},{CYCLONE_RECEIVE_BUFFER_XML}' if uri else CYCLONE_RECEIVE_BUFFER_XML
    else:
        print(f'WARNING: RMW_IMPLEMENTATION is "{rmw}", not CycloneDDS; the receive buffer is not set.')

    print(f'\nBag folder: {bag_path}\nCommand: {" ".join(command)}', flush=True)
    if args.dry_run:
        os.unlink(qos_file.name)
        return 0

    # ---- record. The recorder gets its own session so that Ctrl+C reaches only this script,
    # which then sends the recorder exactly one SIGINT (clean metadata.yaml).
    started = datetime.datetime.now()
    process = subprocess.Popen(command, env=env, start_new_session=True)

    def stop(reason):
        if process.poll() is None:
            print(f'\n{reason}; finalizing the bag ...', flush=True)
            process.send_signal(signal.SIGINT)

    signal.signal(signal.SIGTERM, lambda *_: stop('SIGTERM'))
    deadline = time.monotonic() + args.duration if args.duration else None
    try:
        while process.poll() is None:
            if deadline is not None and time.monotonic() >= deadline:
                stop(f'{args.duration:g} s recorded')
                deadline = None
            time.sleep(0.2)
    except KeyboardInterrupt:
        stop('Ctrl+C')
    while True:
        try:
            return_code = process.wait()
            break
        except KeyboardInterrupt:
            print('Still finalizing the bag, please wait ...', flush=True)

    if not (bag_path / 'metadata.yaml').exists():
        os.unlink(qos_file.name)
        print('ERROR: no metadata.yaml; see the recorder output above.')
        return return_code or 1

    # ---- report
    shutil.move(qos_file.name, bag_path / 'record_qos_overrides.yaml')
    os.chmod(bag_path / 'record_qos_overrides.yaml', 0o644)
    duration, rates = rate_check(bag_path, topics)
    print(f'\nRecorded {duration:.1f} s into {bag_path}')
    print(f'{"topic":36s} {"messages":>9s} {"rate":>8s} {"nominal":>8s}  status')
    for topic, row in rates.items():
        rate = f'{row["rate_hz"]:.1f}' if 'rate_hz' in row else '-'
        nominal = str(row.get('nominal_hz', '-'))
        print(f'{topic:36s} {row["messages"]:9d} {rate:>8s} {nominal:>8s}  {row["status"]}')
    bad = [topic for topic, row in rates.items() if row['status'] != 'OK']
    print('All topics OK.' if not bad else f'CHECK: {", ".join(bad)}')

    if urdf is not None:
        (bag_path / 'robot_description.urdf').write_text(urdf)
    recording_info = {
        'recorded_with': 'g1_bringup record_bag.py',
        'label': label,
        'host': socket.gethostname(),
        'start_local_time': started.isoformat(timespec='seconds'),
        'clock_note': "PC2's clock follows PC1's (chrony), which is not UTC",
        'ros_distro': distro,
        'rmw_implementation': rmw,
        'groups': groups,
        'command': command,
        'checks_failed': problems,
        'clock_sync': chrony,
        'net_core_rmem_max': rmem,
        'robot_description': 'robot_description.urdf' if urdf is not None else None,
        'topics': {topic: {'types': info[topic]['types'], 'publishers': info[topic]['publishers'],
                           **rates[topic]} for topic in topics},
        'duration_s': round(duration, 3),
    }
    (bag_path / 'recording_info.yaml').write_text(
        yaml.safe_dump(recording_info, sort_keys=False, width=120))
    print(f'Wrote {bag_path / "recording_info.yaml"}')
    return return_code


if __name__ == '__main__':
    sys.exit(main())
