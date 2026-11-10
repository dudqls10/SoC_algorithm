"""Verify the known Sensor1 image before programming this reviewed revision."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

sys.dont_write_bytecode = True
root = Path(__file__).resolve().parent
sys.path.insert(0, str(root.parent/'gain_zero_diagnosis_20261014'))
from probe_live_readonly import parse_elf, JLINK


def run_script(folder, name, commands, timeout):
    script = folder/(name+'.jlink')
    script.write_text('\n'.join(commands)+'\n', encoding='ascii')
    run = subprocess.run([str(JLINK), '-device', 'STM32F722RE', '-if', 'SWD', '-speed', '4000', '-SelectEmuBySN', '59768883', '-AutoConnect', '0', '-ExitOnError', '1', '-NoGui', '1', '-CommanderScript', str(script)], capture_output=True, text=True, timeout=timeout)
    log = run.stdout+'\n'+run.stderr
    (folder/(name+'.log')).write_text(log, encoding='utf-8')
    if run.returncode or any(x in log.lower() for x in ('unknown command', 'syntax error', 'command string error')):
        raise RuntimeError((name, run.returncode, log[-1800:]))
    return log


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--program', action='store_true')
    a = p.parse_args()
    folder = root/('flash_apply' if a.program else 'board_preflight')
    folder.mkdir(exist_ok=False)
    validation = json.loads((root/'validation_result.json').read_text())
    assert validation['host_exit_code'] == 0 and validation['firmware_build_success']
    elf_path = root/'firmware/Mobile_SoC_Sensor1_Distance.elf'
    image = root/'firmware/Mobile_SoC_Sensor1_Distance.bin'
    assert hashlib.sha256(image.read_bytes()).hexdigest() == validation['firmware_hashes'][image.name]
    old_elf, old_segments, _ = parse_elf(root/'before/Debug/Mobile_SoC_Sensor1_Distance.elf')
    assert hashlib.sha256(old_elf).hexdigest() == '4ec2d08914b313e4735c68d23808e82a5215cbb8a7f3b9418b444ef769e515f6'
    cpuid = folder/'cpuid.bin'; backup = folder/'board_flash_before.bin'
    commands = ['exec DisableAutoUpdateFW', 'exec ForceAttachTarget=1', 'exec SetAllowStopMode=0', 'exec InhibitConnectRetries=1', 'connect',
                f'savebin "{cpuid.as_posix()}", 0xE000ED00, 4',
                f'savebin "{backup.as_posix()}", 0x08000000, 0x80000', 'exit']
    run_script(folder, 'identify_and_backup', commands, 20)
    cpu = int.from_bytes(cpuid.read_bytes(), 'little')
    assert (cpu >> 4) & 0xfff == 0xc27, 'Unexpected target core'
    original = backup.read_bytes(); assert len(original) == 0x80000
    matches = all(original[s['address']-0x08000000:s['address']-0x08000000+len(s['data'])] == s['data'] for s in old_segments)
    assert matches, 'Current board flash no longer matches the three-trial Sensor1 image'
    result = {'epoch_s': time.time(), 'target': 'STM32F722RE', 'cpuid': hex(cpu), 'before_expected_elf_matches': matches,
              'board_flash_backup_sha256': hashlib.sha256(original).hexdigest(), 'board_flash_backup': str(backup),
              'motor_commands_sent': 0, 'custom_SoC_flash_commands': 0, 'programmed': False,
              'candidate_bin_sha256': hashlib.sha256(image.read_bytes()).hexdigest(), 'candidate_elf': str(elf_path)}
    if a.program:
        readback = folder/'board_flash_readback.bin'
        commands = ['exec DisableAutoUpdateFW', 'connect', 'h',
                    f'loadbin "{image.as_posix()}", 0x08000000',
                    f'verifybin "{image.as_posix()}", 0x08000000',
                    f'savebin "{readback.as_posix()}", 0x08000000, 0x{image.stat().st_size:X}', 'r', 'g', 'exit']
        run_script(folder, 'program_and_verify', commands, 60)
        assert readback.read_bytes() == image.read_bytes(), 'Programmed bytes failed readback comparison'
        result.update({'programmed': True, 'full_readback_matches': True, 'bytes': image.stat().st_size,
                       'full_sensor_power_cycle_pending': True, 'reset_halt_for_programming': True})
    else:
        result['reset_halt_write_commands_sent'] = 0
    (folder/'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
