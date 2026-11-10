"""Summarize original records; tolerate an unfinished last line during capture."""
import collections
import csv
import json
import math
from pathlib import Path
import sys

root = Path(sys.argv[1])
states = []
for line in (root/'states.jsonl').read_text(encoding='utf-8').splitlines():
    try:
        states.append(json.loads(line))
    except json.JSONDecodeError:
        pass
audit = []
with (root/'audit.csv').open(encoding='utf-8', newline='') as f:
    for row in csv.DictReader(f):
        try:
            audit.append({k: float(v) if k == 'host_read_epoch_s' else int(v) for k, v in row.items()})
        except (TypeError, ValueError):
            pass
out = {'event_label_note': 'In the initial process compact event payloads, object was a mistaken label for flag_algorithmDO (detector enabled), NOT object presence. Object is decoded from audit/evaluation flags bit2 or CAN line_bits bit6. Original files retained.', 'epochs': {}}
for epoch in sorted({s['epoch'] for s in states}):
    ss = [s for s in states if s['epoch'] == epoch]
    aa = [a for a in audit if a['epoch'] == epoch]
    fits = [a for a in aa if a['flags'] & (1 << 14)]
    last = ss[-1]
    reasons = collections.Counter(s['learning_guard']['reason'] for s in ss)
    durations = collections.Counter()
    for a, b in zip(aa, aa[1:]):
        if 0 <= b['tick_ms'] - a['tick_ms'] <= 2000:
            durations[str((a['flags'] >> 8) & 63)] += b['tick_ms'] - a['tick_ms']
    changes = []
    previous = None
    for a in fits:
        key = (a['coefficient'], a['learning_updates'])
        if key != previous:
            changes.append({k: a[k] for k in ('sequence', 'tick_ms', 'coefficient', 'offset', 'numerator', 'denominator', 'sensor_variance', 'learning_updates', 'mean_ch0', 'mean_ch1')})
        previous = key
    bootstrap_check = None
    if fits:
        first = fits[0]
        pairs = {}
        for a in aa:
            if 1 <= a['learning_updates'] <= first['learning_updates']:
                pairs.setdefault(a['learning_updates'], (a['learning_input_ch0'], a['learning_input_ch1']))
        keys = range(first['learning_updates'] - 79, first['learning_updates'] + 1)
        if all(k in pairs for k in keys):
            window = [pairs[k] for k in keys]
            m0 = sum(p[0] for p in window)//80
            m1 = sum(p[1] for p in window)//80
            num = sum((p[0]-m0)*(p[1]-m1) for p in window)
            den = sum((p[1]-m1)**2 for p in window)
            var = sum((p[0]-m0)**2 for p in window)
            bootstrap_check = {'samples': 80, 'first_update_used': first['learning_updates']-79, 'last_update_used': first['learning_updates'], 'mean_ch0': m0, 'mean_ch1': m1, 'numerator': num, 'denominator': den, 'sensor_variance': var, 'correlation': num/math.sqrt(den*var) if den*var else None, 'matches_recorded_integer_moments': all(v == first[k] for k, v in [('mean_ch0', m0), ('mean_ch1', m1), ('numerator', num), ('denominator', den), ('sensor_variance', var)])}
    postfit = [a for a in aa if fits and a['tick_ms'] >= fits[0]['tick_ms']]
    longest_wait, longest_allowed = 0, 0
    wait_start, allowed_start = None, None
    for a in postfit:
        reason = (a['flags'] >> 8) & 63
        if reason == 32:
            if wait_start is None: wait_start = a['tick_ms']
            longest_wait = max(longest_wait, a['tick_ms']-wait_start)
        else:
            if wait_start is not None: longest_wait = max(longest_wait, a['tick_ms']-wait_start)
            wait_start = None
        if a['flags'] & (1 << 15):
            if allowed_start is None: allowed_start = a['tick_ms']
            longest_allowed = max(longest_allowed, a['tick_ms']-allowed_start)
        else:
            if allowed_start is not None: longest_allowed = max(longest_allowed, a['tick_ms']-allowed_start)
            allowed_start = None
    out['epochs'][str(epoch)] = {'state_samples': len(ss), 'first_tick_ms': ss[0]['uwTick'], 'last_tick_ms': last['uwTick'], 'audit_rows': len(aa), 'first_audit': aa[0] if aa else None, 'first_ready_model': fits[0] if fits else None, 'model_changes': changes, 'reason_state_counts': dict(reasons), 'reason_audit_duration_ms': dict(durations), 'last_model': last['model'], 'last_guard': last['learning_guard'], 'last_noise_reference': last['noise_reference'], 'last_learning_updates': last['drift_eval_learning_updates'], 'last_valid_cycles': last['drift_eval_valid_cycles'], 'last_invalid_skips': last['drift_eval_invalid_skips'], 'last_candidate_rejects': last['drift_learning_candidate_rejects'], 'last_can': last.get('latest_can_snapshot'), 'object_audit_count': sum(bool(a['flags'] & 4) for a in aa)}
    out['epochs'][str(epoch)].update({'bootstrap_check': bootstrap_check, 'longest_WAIT_CLEAR_ms': longest_wait, 'longest_allowed_ms_after_first_fit': longest_allowed, 'postfit_allowed_records': sum(bool(a['flags'] & (1 << 15)) for a in postfit), 'peak_noise_reference_samples': max((s['noise_reference']['sample_count'] for s in ss), default=0)})
path = root/'inspection_latest.json'
path.write_text(json.dumps(out, indent=2), encoding='utf-8')
print(json.dumps(out, indent=2))
