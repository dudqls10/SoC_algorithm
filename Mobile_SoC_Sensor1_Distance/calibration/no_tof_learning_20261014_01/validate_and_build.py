"""Host regressions and the actual Sensor1 firmware build; no hardware commands."""
import difflib
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

root = Path(__file__).resolve().parent
project = root.parents[1]
diagnosis = project/'calibration/gain_zero_diagnosis_20261014'
host = root/'host_tests'; host.mkdir(exist_ok=True)
main = (project/'Core/Src/main.c').read_text(encoding='utf-8-sig')
assert 'learning_tof_valid' not in main
assert 'DriftLearningGuardStepCapOnly(&drift_learning_guard' in main
start = main.index('static uint8_t TrainDelayedQ1Pair(')
brace = main.index('{', start); depth = 1; end = brace+1
while depth:
    depth += (main[end] == '{')-(main[end] == '}'); end += 1
trainer = main[start:end]
(host/'train_delayed_production.inc').write_text(trainer+'\n', encoding='utf-8')
shutil.copy2(diagnosis/'offline/production_types.h', host)
fixture = (diagnosis/'offline/gain_zero_harness.c').read_text(encoding='utf-8')
fixture = fixture.replace('DriftLearningGuardStep(&drift_learning_guard,i*50U,8000000,10000000,1,0,i,300,1);',
    'DriftLearningGuardStepCapOnly(&drift_learning_guard,i*50U,8000000,10000000,1,0);')
fixture = fixture.replace('DriftLearningGuardStep(&drift_learning_guard,tick,fdc.ch300,fdc.ch302,\n                              1,detected,1U+tick/50U,300,1);',
    'DriftLearningGuardStepCapOnly(&drift_learning_guard,tick,fdc.ch300,fdc.ch302,1,detected);')
assert 'DriftLearningGuardStep(' not in fixture
(host/'integration.c').write_text(fixture, encoding='utf-8')
shutil.copy2(root/'cap_only_guard_tests.c', host)
replay = (diagnosis/'replay_boot1_guard.c').read_text(encoding='utf-8')
replay = replay.replace('Case cases[4]', 'Case cases[5]').replace('i < 4', 'i < 5').replace('labels[4]', 'labels[5]')
replay = replay.replace('DriftLearningGuardStep(&c->guard, tick, ch0, ch1, 1, detected,\n                                   tick / 100U, optical, 1);',
    'if (i == 4U) DriftLearningGuardStepCapOnly(&c->guard, tick, ch0, ch1, 1, detected);\n            else DriftLearningGuardStep(&c->guard, tick, ch0, ch1, 1, detected, tick / 100U, optical, 1);')
replay = replay.replace('"fixed_ToF_and_object_zero_counterfactual"};',
    '"fixed_ToF_and_object_zero_counterfactual", "production_CAP_only_guard"};')
(host/'recorded_replay.c').write_text(replay, encoding='utf-8')
shutil.copy2(diagnosis/'boot1_recorded_replay.csv', host/'input.csv')
include = str(project/'Core/Inc')
batch = ['@echo off', 'call "C:\\Program Files\\Microsoft Visual Studio\\18\\Insiders\\VC\\Auxiliary\\Build\\vcvars64.bat" >nul',
         'if errorlevel 1 exit /b 1', 'cd /d "%~dp0"']
for source, exe, args in [('integration.c', 'integration.exe', ''), ('cap_only_guard_tests.c', 'cap_only_guard_tests.exe', ''), ('recorded_replay.c', 'recorded_replay.exe', ' input.csv')]:
    batch += [f'cl /nologo /utf-8 /std:c11 /W4 /WX /Od /I"{include}" {source} /Fe:{exe} /link /INCREMENTAL:NO',
              'if errorlevel 1 exit /b 1', exe+args, 'if errorlevel 1 exit /b 1']
batch.append('exit /b 0')
(host/'run.cmd').write_text('\n'.join(batch)+'\n', encoding='ascii')
if '--resume-build' in sys.argv:
    previous = json.loads((root/'validation_result.json').read_text(encoding='utf-8'))
    assert previous['host_exit_code'] == 0
    run = subprocess.CompletedProcess([], 0, (host/'console.log').read_text(encoding='utf-8'), '')
else:
    run = subprocess.run(['cmd.exe', '/d', '/c', str(host/'run.cmd')], cwd=host, capture_output=True, text=True, encoding='cp949', errors='replace', timeout=50, creationflags=subprocess.CREATE_NO_WINDOW)
(host/'console.log').write_text(run.stdout+'\n'+run.stderr, encoding='utf-8')
records = [json.loads(line) for line in run.stdout.splitlines() if line.startswith('{')]
result = {'host_exit_code': run.returncode, 'records': records, 'firmware_build_success': False, 'hardware_commands': 0}
(root/'validation_result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
assert run.returncode == 0, run.stdout+run.stderr
cap_replay = next(r for r in records if r.get('case') == 'production_CAP_only_guard')
assert cap_replay['noise_ready'] == 1 and cap_replay['near_cycles'] == 0
toolchain = Path(r'C:/ST/STM32CubeIDE_1.17.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.1.0.202410251130/tools/bin')
make = Path(r'C:/ST/STM32CubeIDE_1.17.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845/tools/bin/make.exe')
env = os.environ.copy(); env['PATH'] = str(toolchain)+os.pathsep+str(make.parent)+os.pathsep+env.get('PATH', '')
build = subprocess.run([str(make), '-j4', 'SHELL=cmd.exe', 'all'], cwd=project/'Debug', env=env, capture_output=True, text=True, errors='replace', timeout=90, creationflags=subprocess.CREATE_NO_WINDOW)
(root/'build.log').write_text(build.stdout+'\n'+build.stderr, encoding='utf-8')
result['firmware_build_exit_code'] = build.returncode
assert build.returncode == 0, build.stdout+build.stderr
after = root/'firmware'; after.mkdir(exist_ok=False)
elf = project/'Debug'/(project.name+'.elf')
for fmt, ext in [('binary', 'bin'), ('ihex', 'hex')]:
    destination = project/'Debug'/(project.name+'.'+ext)
    subprocess.run([str(toolchain/'arm-none-eabi-objcopy.exe'), '-O', fmt, str(elf), str(destination)], check=True, capture_output=True, timeout=15, creationflags=subprocess.CREATE_NO_WINDOW)
for ext in ('elf', 'bin', 'hex', 'map'):
    shutil.copy2(project/'Debug'/(project.name+'.'+ext), after)
result['firmware_build_success'] = True
result['firmware_hashes'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in after.iterdir()}
result['firmware_bin_bytes'] = (after/(project.name+'.bin')).stat().st_size
result['new_build_warnings'] = [line for line in (build.stdout+build.stderr).splitlines() if 'warning:' in line]
result['source_diffs'] = []
for relative in ('Core/Src/main.c', 'Core/Inc/drift_learning_guard.h', 'Core/Inc/drift_noise_reference.h', 'Core/Inc/sensor1_distance_calibration.h'):
    a = root/'before'/relative; b = project/relative
    diff = ''.join(difflib.unified_diff(a.read_text(encoding='utf-8-sig').splitlines(True), b.read_text(encoding='utf-8-sig').splitlines(True), fromfile='before/'+relative, tofile=relative))
    if diff:
        name = relative.replace('/', '_')+'.diff'
        (root/name).write_text(diff, encoding='utf-8'); result['source_diffs'].append(name)
(root/'validation_result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
print(json.dumps(result, indent=2), flush=True)
