"""One bounded RAM batch after the CAN-only interval; preserve pre/post CAN."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

root = Path(__file__).resolve().parent
can_root = root/'live_boot_03_can_control'
out = root/'live_boot_03_end_RAM'
pre = json.loads((can_root/'status.json').read_text(encoding='utf-8'))
(root/'control_pre_RAM_status.json').write_text(json.dumps(pre, indent=2), encoding='utf-8')
meta = {'read_started_epoch_s': time.time(), 'confirmation_observed_by_utc': '2026-10-14 13:03:43 UTC', 'note': 'No requested SWD accesses during the preceding CAN-only180s; this subprocess reads one batch after that interval.'}
run = subprocess.run([sys.executable, str(root/'collect_live_boot.py'), str(out), '--identity', str(root/'live_boot_01/firmware_identity_v8/result.json'), '--seconds', '0.1', '--no-can', '--cli'], timeout=20)
meta['read_ended_epoch_s'] = time.time()
meta['read_exit_code'] = run.returncode
time.sleep(15)
post = json.loads((can_root/'status.json').read_text(encoding='utf-8'))
(root/'control_post_RAM_status.json').write_text(json.dumps(post, indent=2), encoding='utf-8')
meta['post_observed_epoch_s'] = time.time()
meta['pre_status_age_seconds'] = meta['read_started_epoch_s']-pre['host_epoch_s']
meta['post_status_age_seconds'] = meta['post_observed_epoch_s']-post['host_epoch_s']
meta['status_snapshots_fresh'] = (meta['pre_status_age_seconds'] < 3 and meta['post_status_age_seconds'] < 3)
(root/'control_RAM_timing.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')
print(json.dumps({'meta': meta, 'pre_status_bits': pre.get('latest_snapshot', {}).get('status_bits'), 'post_status_bits': post.get('latest_snapshot', {}).get('status_bits')}, indent=2), flush=True)
