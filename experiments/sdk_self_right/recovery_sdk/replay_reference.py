"""Send recorded measurements only into the offline C++ stall detector."""
import argparse
import json
import math
from pathlib import Path
import subprocess


def recordings(path):
    active = None
    with path.open(encoding='utf-8') as log:
        for raw in log:
            item = json.loads(raw)
            event = item['event']
            if event == 'factory_recovery_started':
                active = dict(start=item['elapsed_s'], left=False, samples=[])
            elif active is not None:
                if event == 'factory_direction_left':
                    active['left'] = True
                    active['direction_s'] = item['elapsed_s'] - active['start']
                elif event == 'recovery_joint_sample' and item.get('recovering'):
                    active['samples'].append(item)
                elif event in ('soft_emergency_observed', 'factory_fault'):
                    active = None
                elif event == 'factory_reported_success':
                    if active['left'] and active['samples']:
                        active['duration_s'] = item['elapsed_s'] - active['start']
                        yield active
                    active = None


def first_pose(samples, threshold):
    start = samples[0]['sample_monotonic_s']
    for sample in samples:
        x, y, z, w = sample['orientation']
        cosine = max(-1., min(1., 1. - 2. * (x*x + y*y)))
        if math.degrees(math.acos(cosine)) <= threshold:
            return round(sample['sample_monotonic_s']-start, 3)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('replay', type=Path)
    parser.add_argument('audit', type=Path)
    args = parser.parse_args()
    trials = []
    for trial in recordings(args.audit):
        trials.append(trial)
        if len(trials) == 3:
            break
    if len(trials) != 3:
        parser.error('Need three complete, uninterrupted factory left recoveries')
    replay_rows = []
    for i, trial in enumerate(trials, 1):
        samples = trial['samples']
        print(json.dumps(dict(trial=i, samples=len(samples),
                              duration_s=round(trial['duration_s'], 3),
                              left_direction_s=round(trial['direction_s'], 3),
                              first_tilt_below_120_s=first_pose(samples, 120.),
                              first_tilt_below_20_s=first_pose(samples, 20.))))
        for row in samples:
            values = (row['orientation'] + row['position'] + row['velocity'])
            if len(values) != 28 or not all(math.isfinite(v) for v in values):
                parser.error('Unexpected joint/IMU sample')
            ms = round(row['sample_monotonic_s'] * 1000)
            replay_rows.append(' '.join(map(str, [f'trial{i}', ms, *values])))
    subprocess.run([str(args.replay.resolve())], input='\n'.join(replay_rows)+'\n',
                   text=True, check=True)


if __name__ == '__main__':
    main()
