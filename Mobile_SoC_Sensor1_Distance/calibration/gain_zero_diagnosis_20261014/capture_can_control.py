"""Receive-only control: no J-Link access during the measurement interval."""
import argparse
import json
from pathlib import Path
import sys
import threading
import time

sys.dont_write_bytecode = True
sys.path.insert(0, r'C:/Users/SongYoungBin/Desktop/Soc/Mobile_SoC_Experiments/calibration_tools')
from mobile_telemetry import Capture, mobile_protocol_observed


def main():
    p = argparse.ArgumentParser()
    p.add_argument('folder', type=Path)
    p.add_argument('--seconds', type=float, default=1800)
    a = p.parse_args()
    a.folder.mkdir(parents=True, exist_ok=False)
    c = Capture(a.folder)
    c.phase = 'stationary_30cm_CAN_only_control'
    worker = threading.Thread(target=c.run, daemon=True)
    worker.start()
    start = time.monotonic()
    report = 0
    status_update_retries = 0
    try:
        if not c.ready.wait(8) or c.failure:
            raise RuntimeError(('CAN not ready', c.failure))
        while time.monotonic() - start < a.seconds and not (a.folder/'STOP').exists():
            status = {'host_epoch_s': time.time(), 'elapsed_seconds': time.monotonic()-start,
                      'ids': dict(c.ids), 'error_count': len(c.errors), 'failure': c.failure,
                      'latest_snapshot': c.latest_snapshot, 'JLink_accesses': 0,
                      'sensor_commands_sent': 0, 'motor_commands_sent': 0,
                      'status_update_retries': status_update_retries}
            temp = a.folder/'status.tmp'
            temp.write_text(json.dumps(status, indent=2), encoding='utf-8')
            try:
                temp.replace(a.folder/'status.json')
            except PermissionError:
                # A Windows reader may temporarily deny file replacement.
                # Keep receiving CAN and retry the ancillary status next cycle.
                status_update_retries += 1
            if c.latest_snapshot and not (a.folder/'READY.json').exists():
                (a.folder/'READY.json').write_text(json.dumps(status, indent=2), encoding='utf-8')
                print(json.dumps({'kind': 'ready', **status}), flush=True)
            if time.monotonic()-report >= 20:
                print(json.dumps({'kind': 'status', **status}), flush=True)
                report = time.monotonic()
            if c.failure:
                raise RuntimeError(c.failure)
            time.sleep(1)
    finally:
        c.stop.set(); worker.join(5)
        summary = c.summary()
        summary.update({'mobile_four_frame_protocol_observed': mobile_protocol_observed(c.ids),
                        'JLink_accesses': 0, 'sensor_commands_sent': 0, 'motor_commands_sent': 0,
                        'status_update_retries': status_update_retries})
        (a.folder/'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        print(json.dumps({'kind': 'stopped', 'ids': dict(c.ids), 'error_count': len(c.errors)}), flush=True)


if __name__ == '__main__':
    main()
