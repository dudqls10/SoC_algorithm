"""Identify running STM32 flash using explicit attach-only J-Link V8 settings."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import time

JLINK = Path(r'C:/ST/STM32CubeIDE_1.17.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.jlink.win32_2.3.0.202409170845/tools/bin/JLink.exe')
SOC = Path(r'C:/Users/SongYoungBin/Desktop/Soc')
OLD = Path(r'C:/Users/SongYoungBin/.codex/visualizations/2026/10/02/01a0fcdc-55de-7463-a319-ade97ad00ae3/step_sigma_once_20261013/firmware_reference/Mobile_SoC.elf')


def parse_elf(path):
    data = path.read_bytes()
    h = struct.unpack_from('<16sHHIIIIIHHHHHH', data)
    assert data[:6] == b'\x7fELF\x01\x01'
    segments = []
    for i in range(h[10]):
        typ, off, va, pa, fs, ms, flags, align = struct.unpack_from('<8I', data, h[5] + i * h[9])
        if typ == 1 and fs and 0x08000000 <= pa < 0x08200000:
            segments.append({'address': pa, 'data': data[off:off + fs]})
    sections = [struct.unpack_from('<10I', data, h[6] + i * h[11]) for i in range(h[12])]
    symbols = {}
    for sec in sections:
        if sec[1] != 2:
            continue
        st = sections[sec[6]]
        strings = data[st[4]:st[4] + st[5]]
        for i in range(sec[4], sec[4] + sec[5], sec[9]):
            name, value, size, info, other, idx = struct.unpack_from('<IIIBBH', data, i)
            name = strings[name:].split(b'\0', 1)[0].decode(errors='replace')
            if name and size:
                symbols[name] = {'address': value, 'size': size}
    return data, segments, symbols


def main():
    p = argparse.ArgumentParser()
    p.add_argument('folder', type=Path)
    a = p.parse_args()
    a.folder.mkdir(parents=True, exist_ok=False)
    candidates = [OLD, SOC/'Mobile_SoC/Debug/Mobile_SoC.elf', SOC/'Mobile_SoC_Experiments/Mobile_SoC_Sensor1_Distance/Debug/Mobile_SoC_Sensor1_Distance.elf']
    images = [(x, *parse_elf(x)) for x in candidates if x.exists()]
    extent = max(s['address'] + len(s['data']) - 0x08000000 for _, _, segs, _ in images for s in segs)
    cpuid = a.folder/'cpuid.bin'
    flash = a.folder/'flash.bin'
    commands = ['exec DisableAutoUpdateFW', 'exec ForceAttachTarget=1', 'exec SetAllowStopMode=0', 'exec InhibitConnectRetries=1', 'connect',
                f'savebin "{cpuid.as_posix()}", 0xE000ED00, 4',
                f'savebin "{flash.as_posix()}", 0x08000000, 0x{extent:X}', 'exit']
    script = a.folder/'read.jlink'
    script.write_text('\n'.join(commands)+'\n', encoding='ascii')
    result = {'epoch_s': time.time(), 'halt_reset_write_commands_sent': 0, 'identified': False, 'jlink_executable': str(JLINK)}
    try:
        proc = subprocess.run([str(JLINK), '-device', 'STM32F722RE', '-if', 'SWD', '-speed', '1000', '-SelectEmuBySN', '59768883', '-AutoConnect', '0', '-ExitOnError', '1', '-NoGui', '1', '-CommanderScript', str(script)], capture_output=True, text=True, timeout=35)
        log = proc.stdout+'\n'+proc.stderr
        (a.folder/'jlink.log').write_text(log, encoding='utf-8')
        result['process_exit_code'] = proc.returncode
        result['jlink_log_tail'] = log[-3500:]
        assert proc.returncode == 0, 'J-Link attachment failed; see jlink.log'
        assert not any(x in log.lower() for x in ('unknown command', 'syntax error', 'command string error'))
        cpu = int.from_bytes(cpuid.read_bytes(), 'little')
        result['cpuid'] = hex(cpu)
        assert (cpu >> 4) & 0xfff == 0xc27, 'Attached target is not Cortex-M7'
        actual = flash.read_bytes()
        matches = []
        for path, data, segments, symbols in images:
            details = [{'address': hex(s['address']), 'bytes': len(s['data']), 'matches': actual[s['address']-0x08000000:s['address']-0x08000000+len(s['data'])] == s['data']} for s in segments]
            matched = bool(details) and all(s['matches'] for s in details)
            matches.append({'path': str(path), 'elf_sha256': hashlib.sha256(data).hexdigest(), 'matches': matched, 'segments': details})
            if matched and not result['identified']:
                result['identified'] = True
                result['matched_elf'] = str(path)
                (a.folder/'symbols.json').write_text(json.dumps(symbols, indent=2), encoding='utf-8')
        result['candidates'] = matches
    except Exception as e:
        result['failure'] = repr(e)
    (a.folder/'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
