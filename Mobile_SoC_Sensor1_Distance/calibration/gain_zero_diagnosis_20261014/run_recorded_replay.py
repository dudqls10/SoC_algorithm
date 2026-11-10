import csv
import hashlib
import json
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parent
project = root.parents[1]
capture = root/'live_boot_01/continuous'
rows = []
with (capture/'evaluation.csv').open(encoding='utf-8', newline='') as f:
    for row in csv.DictReader(f):
        if row['epoch'] == '2' and 22782 <= int(row['tick_ms']) <= 180000:
            rows.append({k: int(v) for k, v in row.items() if k != 'host_read_epoch_s'})
data = root/'boot1_recorded_replay.csv'
with data.open('w', newline='', encoding='ascii') as f:
    writer = csv.writer(f)
    for previous, current in zip(rows, rows[1:]):
        assert current['flags'] & 3 == 3
        writer.writerow([current['tick_ms'], current['filtered_ch0'], current['filtered_ch1'], previous['post_ema'], bool(current['flags'] & 4)*1, current['tof_mm']])
batch = root/'run_recorded_replay.cmd'
batch.write_text('''@echo off
call "C:\\Program Files\\Microsoft Visual Studio\\18\\Insiders\\VC\\Auxiliary\\Build\\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0"
cl /nologo /utf-8 /std:c11 /W4 /WX /Od /I"../../Core/Inc" replay_boot1_guard.c /Fe:replay_boot1_guard.exe /link /INCREMENTAL:NO
if errorlevel 1 exit /b 1
replay_boot1_guard.exe boot1_recorded_replay.csv
exit /b %errorlevel%
''', encoding='ascii')
run = subprocess.run(['cmd.exe', '/d', '/c', str(batch)], capture_output=True, text=True, encoding='cp949', errors='replace', timeout=50, creationflags=subprocess.CREATE_NO_WINDOW)
(root/'recorded_replay_console.log').write_text(run.stdout+'\n'+run.stderr, encoding='utf-8')
records = [json.loads(line) for line in run.stdout.splitlines() if line.startswith('{')]
out = {'exit_code': run.returncode, 'records': records, 'hardware_commands': 0, 'firmware_changes': 0,
       'scope': 'First completed reboot, recorded filtered pairs/ToF/object flags, fixed original first model. Production guard/noise headers. Approximate cycle timing from evaluation tick, valid ToF quality assumed; not a bit-exact full firmware replay. Fixed ToF/object cases are offline counterfactuals and are not proposed firmware changes.',
       'input_sha256': hashlib.sha256(data.read_bytes()).hexdigest(), 'input_records': len(rows)-1}
(root/'recorded_replay_result.json').write_text(json.dumps(out, indent=2), encoding='utf-8')
print(json.dumps(out, indent=2))
assert run.returncode == 0, run.stdout+run.stderr
