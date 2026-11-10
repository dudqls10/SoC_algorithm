"""Compile production learning functions in a host-only diagnostic fixture."""
import hashlib,json,os,re,subprocess,sys
from pathlib import Path
sys.stdout.reconfigure(encoding='utf-8')
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parents[2]
main=(PROJECT/'Core/Src/main.c').read_text(encoding='utf-8-sig')
header=(PROJECT/'Core/Inc/main.h').read_text(encoding='utf-8-sig')
match=re.search(r'static uint8_t TrainDelayedQ1Pair\([^;{}]*\)\s*\{',main)
assert match
depth=1;end=match.end()
while depth:
    depth+=(main[end]=='{')-(main[end]=='}');end+=1
function=main[match.start():end]
(ROOT/'train_delayed_production.inc').write_text(function+'\n',encoding='utf-8')
types=list(re.finditer(r'typedef\s+struct\s*\{[^{}]*\}\s*(\w+)\s*;',header,re.S))
sample=next(m for m in types if 'uint32_t fdc300[100]' in m.group())
comp=next(m for m in types if m.group(1)=='driftCompensation')
(ROOT/'production_types.h').write_text(comp.group()+'\n'+sample.group()+'\ntypedef '+sample.group(1)+' ProductionSample;\n',encoding='utf-8')
batch='''@echo off
call "C:\\Program Files\\Microsoft Visual Studio\\18\\Insiders\\VC\\Auxiliary\\Build\\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0"
cl /nologo /utf-8 /std:c11 /W4 /WX /Od /I"../../../Core/Inc" gain_zero_harness.c /Fe:gain_zero_harness.exe /link /INCREMENTAL:NO
if errorlevel 1 exit /b 1
gain_zero_harness.exe
exit /b %errorlevel%
'''
(ROOT/'run_tests.cmd').write_text(batch,encoding='ascii')
run=subprocess.run(['cmd.exe','/d','/c',str(ROOT/'run_tests.cmd')],cwd=ROOT,capture_output=True,text=True,encoding='cp949',errors='replace',timeout=60,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
(ROOT/'test_console.log').write_text(run.stdout+'\n'+run.stderr,encoding='utf-8')
records=[json.loads(line) for line in run.stdout.splitlines() if line.startswith('{')]
inputs=[PROJECT/'Core/Src/main.c',PROJECT/'Core/Inc/main.h']+[PROJECT/'Core/Inc'/name for name in ['drift_learning_guard.h','drift_noise_reference.h','drift_model_math.h','ema_precision.h']]
result={'success':run.returncode==0,'exit_code':run.returncode,'hardware_commands':0,'firmware_changes':0,
        'production_function_extracted_verbatim':True,'production_trainer_sha256':hashlib.sha256(function.encode()).hexdigest(),
        'source_hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
        'scope':'Synthetic post-first-fit integration fixture. Actual production trainer/types/guard/noise/math/EMA. Main detector ready and valid ToF assumed; object flags supplied explicitly. Not an Oct14 boot reconstruction.',
        'records':records}
(ROOT/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(run.stdout+run.stderr)
assert run.returncode==0,run.returncode
