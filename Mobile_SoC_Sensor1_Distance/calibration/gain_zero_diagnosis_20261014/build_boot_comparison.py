"""Compare completed reboot trials using committed audit records."""
import collections
import csv
import json
from pathlib import Path

root = Path(__file__).resolve().parent
trials = [(1, root/'live_boot_01/continuous', 2), (2, root/'live_boot_02_after_reply', 0), (3, root/'live_boot_03_end_RAM', 0)]
result = {'trials': [], 'firmware_changes_during_diagnosis': 0, 'motor_commands_sent': 0}
for number, folder, epoch in trials:
    info = json.loads((folder/'inspection_latest.json').read_text(encoding='utf-8'))['epochs'][str(epoch)]
    with (folder/'audit.csv').open(encoding='utf-8', newline='') as f:
        rows = [{k: float(v) if k == 'host_read_epoch_s' else int(v) for k, v in r.items()} for r in csv.DictReader(f) if int(r['epoch']) == epoch]
    first = info['first_ready_model']
    after = [r for r in rows if r['tick_ms'] >= first['tick_ms']]
    duration = collections.Counter()
    bit_duration = collections.Counter()
    allowed_ms = 0
    for a, b in zip(rows, rows[1:]):
        lo = max(a['tick_ms'], first['tick_ms'])
        hi = min(b['tick_ms'], 180000)
        if hi <= lo:
            continue
        ms = hi-lo
        reason = (a['flags'] >> 8) & 63
        duration[str(reason)] += ms
        for bit, name in [(1, 'OBJECT'), (2, 'TOF_NEAR'), (4, 'TOF_UNTRUSTED'), (8, 'RESIDUAL'), (16, 'INVALID'), (32, 'WAIT_CLEAR')]:
            if reason & bit:
                bit_duration[name] += ms
        if a['flags'] & 32768:
            allowed_ms += ms
    g = info['last_guard']
    row = {'trial': number, 'capture_folder': str(folder), 'epoch': epoch, 'first_ready_tick_ms': first['tick_ms'],
           'first_accepted_pair_tick_ms': rows[0]['tick_ms'], 'first_fit_collection_span_ms': first['tick_ms']-rows[0]['tick_ms'],
           'gain': first['coefficient']/10000, 'coefficient': first['coefficient'], 'offset': first['offset'],
           'numerator': first['numerator'], 'denominator': first['denominator'], 'sensor_variance': first['sensor_variance'],
           'correlation': info['bootstrap_check']['correlation'], 'integer_moments_match': info['bootstrap_check']['matches_recorded_integer_moments'],
           'additional_model_updates_after_first_fit': len(info['model_changes'])-1, 'noise_ready_at_read': info['last_noise_reference']['ready'],
           'noise_reference_samples_max_in_audit': max(r['noise_reference_samples'] for r in rows),
           'no_noise_ready_flag_any_audit': not any(r['flags'] & 65536 for r in rows),
           'tof_reference_mm': g['tof_reference'], 'tof_sigma_mm': g['tof_noise'],
           'tof_near_enter_mm': g['tof_reference']-max(5,4*g['tof_noise']),
           'tof_near_release_mm': g['tof_reference']-max(3,2*g['tof_noise']),
           'first_fit_to180s_reason_duration_ms': dict(duration), 'first_fit_to180s_reason_bit_duration_ms': dict(bit_duration),
           'first_fit_to180s_allowed_ms': allowed_ms,
           'first_invalid_audit_tick_ms': next((r['tick_ms'] for r in rows if r['flags']&3 != 3), None),
           'last_read_tick_ms': info['last_tick_ms'], 'last_invalid_skips': info['last_invalid_skips'],
           'last_valid_cycles': info['last_valid_cycles'], 'audit_first_sequence': rows[0]['sequence'], 'audit_last_sequence': rows[-1]['sequence'],
           'audit_sequence_gaps': sum(max(0,b['sequence']-a['sequence']-1) for a,b in zip(rows,rows[1:]))}
    result['trials'].append(row)

can_root = root/'live_boot_03_can_control'
with (can_root/'snapshots.csv').open(encoding='utf-8', newline='') as f:
    snapshots = list(csv.DictReader(f))
gap, index = max((int(b['host_monotonic_ns'])-int(a['host_monotonic_ns']), i+1) for i, (a,b) in enumerate(zip(snapshots,snapshots[1:])))
new = snapshots[index:]
result['CAN_only_trial_recording_quality'] = {'largest_gap_seconds': gap/1e9, 'new_boot_first_frame_monotonic_ns': int(new[0]['host_monotonic_ns']),
    'new_boot_last_frame_monotonic_ns': int(new[-1]['host_monotonic_ns']), 'new_boot_recorded_span_s': (int(new[-1]['host_monotonic_ns'])-int(new[0]['host_monotonic_ns']))/1e9,
    'new_boot_snapshots': len(new), 'invalid_pair_snapshots': sum(r['ch0_valid'] != '1' or r['ch1_valid'] != '1' for r in new),
    'warning': 'CAN logger stopped on Windows status-file replacement error. Full180s CAN was not captured. End-of-test committed RAM audit covers startup through~237s with invalid_skips0. Status pre/post files were stale; their repeated255 values cannot verify immediate pre/post SWD health.'}
timing = json.loads((root/'control_RAM_timing.json').read_text(encoding='utf-8'))
for phase, field in [('pre', 'read_started_epoch_s'), ('post', 'post_observed_epoch_s')]:
    status = json.loads((root/f'control_{phase}_RAM_status.json').read_text(encoding='utf-8'))
    timing[phase+'_status_age_seconds'] = timing[field]-status['host_epoch_s']
timing['status_snapshots_fresh'] = all(timing[k] < 3 for k in ('pre_status_age_seconds', 'post_status_age_seconds'))
(root/'control_RAM_timing_audited.json').write_text(json.dumps(timing, indent=2), encoding='utf-8')
result['control_end_read_timing'] = timing
(root/'boot_comparison.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
with (root/'boot_comparison.csv').open('w', encoding='utf-8-sig', newline='') as f:
    keys = ['trial', 'first_ready_tick_ms', 'gain', 'numerator', 'denominator', 'sensor_variance', 'correlation', 'integer_moments_match', 'first_fit_collection_span_ms', 'additional_model_updates_after_first_fit', 'noise_ready_at_read', 'tof_reference_mm', 'tof_sigma_mm', 'tof_near_enter_mm', 'first_fit_to180s_allowed_ms', 'first_invalid_audit_tick_ms', 'last_read_tick_ms']
    w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore'); w.writeheader(); w.writerows(result['trials'])
print(json.dumps(result, indent=2))
