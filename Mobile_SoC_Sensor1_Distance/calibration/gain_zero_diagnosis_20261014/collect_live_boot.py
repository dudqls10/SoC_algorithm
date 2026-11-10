"""Receive CAN and read committed RAM traces; never halt/reset/write target memory."""
import argparse
import csv
import ctypes as C
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import threading
import time

sys.dont_write_bytecode = True
from probe_live_readonly import parse_elf, JLINK

OLD_LAYOUT = Path(r'C:/Users/SongYoungBin/.codex/visualizations/2026/10/02/01a0fcdc-55de-7463-a319-ade97ad00ae3/noise_live_20261013/layout.json')
TOOLS = Path(r'C:/Users/SongYoungBin/Desktop/Soc/Mobile_SoC_Experiments/calibration_tools')
REASONS = {1: 'OBJECT', 2: 'TOF_NEAR', 4: 'TOF_UNTRUSTED', 8: 'RESIDUAL', 16: 'INVALID', 32: 'WAIT_CLEAR'}


class Reader:
    def __init__(self, elf, native=True):
        self.data, self.segments, self.symbols = parse_elf(elf)
        if native:
            self.j = C.CDLL(str(JLINK.parent/'JLink_x64.dll'))
            self.j.JLINKARM_ExecCommand.argtypes = [C.c_char_p, C.c_char_p, C.c_int]
            self.j.JLINKARM_Open.restype = C.c_char_p
            self.j.JLINKARM_ReadMemU8.argtypes = [C.c_uint32, C.c_uint32, C.c_void_p, C.c_void_p]
        self.layout = json.loads(OLD_LAYOUT.read_text())
        for kind, name, seq in [('evaluation', 'eval_trace', 'eval_trace_sequence'), ('audit', 'drift_model_audit', 'drift_model_audit_sequence')]:
            spec = self.layout[kind]
            assert self.symbols[name]['size'] == spec['bytes']
            spec['address'] = self.symbols[name]['address']
            spec['sequence_address'] = self.symbols[seq]['address']
        assert self.symbols['drift_learning_guard']['size'] == 296
        assert self.symbols['drift_noise_reference']['size'] == 48
        assert self.symbols['dComps']['size'] == 200
        extras = ['uwTick', 'n_data', 'count_sampling', 'peroid_sampling', 'n_samplingDI', 'countDO', 'flag_di', 'flag_algorithmDO', 'flag_DOinit', 'ema2_initialized', 'ema2_selected_alpha', 'mobile_fdc_raw_ch0', 'mobile_fdc_raw_ch1', 'mobile_tof_quality_valid', 'mobile_tof_range_status', 'mobile_tof_tick', 'fdc_recovery_retry_count', 'fdc_recovery_retry_success_count', 'fdc_diag_first_valid', 'fdc_diag_first_error', 'eval_trace_sequence', 'eval_tim7_interrupts', 'eval_tim7_coalesced']
        self.scalar_names = sorted({name for name, s in self.symbols.items() if 0x20000000 <= s['address'] < 0x20080000 and s['size'] <= 8 and (name.startswith(('drift_', 'fdc_diag_')) or name in extras)})
        self.state_names = self.scalar_names + ['drift_learning_guard', 'drift_noise_reference', 'dComps', 'state']
        assert self.symbols['state']['size'] == 13
        self.ranges = []
        for name in sorted(self.state_names, key=lambda n: self.symbols[n]['address']):
            s = self.symbols[name]
            lo, hi = s['address'] & ~3, (s['address'] + s['size'] + 3) & ~3
            if self.ranges and lo <= self.ranges[-1][1] + 32:
                self.ranges[-1][1] = max(hi, self.ranges[-1][1])
            else:
                self.ranges.append([lo, hi])

    def command(self, text):
        reply = C.create_string_buffer(1024)
        rc = self.j.JLINKARM_ExecCommand(text.encode('ascii'), reply, len(reply))
        if rc < 0:
            raise RuntimeError((text, rc, reply.value.decode(errors='replace')))

    def connect(self):
        for cmd in ['DisableAutoUpdateFW', 'ForceAttachTarget=1', 'SetAllowStopMode=0', 'InhibitConnectRetries=1']:
            self.command(cmd)
        self.j.JLINKARM_EMU_SelectByUSBSN(59768883)
        error = self.j.JLINKARM_Open()
        if error:
            raise RuntimeError(error.decode(errors='replace'))
        self.command('Device = STM32F722RE')
        self.j.JLINKARM_TIF_Select(1)
        self.j.JLINKARM_SetSpeed(1000)
        if self.j.JLINKARM_Connect() < 0:
            raise RuntimeError('Attach unavailable')
        cpuid = int.from_bytes(self.read(0xE000ED00, 4), 'little')
        if (cpuid >> 4) & 0xfff != 0xc27:
            raise RuntimeError('Target is not Cortex-M7')
        for seg in self.segments:
            if self.read(seg['address'], len(seg['data'])) != seg['data']:
                raise RuntimeError('Running flash differs from identified ELF')
        for name, expected in [('drift_model_audit_entry_size', 152), ('drift_model_audit_depth', 1152), ('eval_trace_depth', 512)]:
            # Unreferenced metadata can be removed by --gc-sections. The
            # matched ELF's ring symbol sizes are checked unconditionally.
            if name in self.symbols and self.u32(name) != expected:
                raise RuntimeError((name, 'RAM trace schema mismatch'))

    def close(self):
        self.j.JLINKARM_Close()

    def read(self, address, count):
        data, status = C.create_string_buffer(count), C.create_string_buffer(count)
        got = self.j.JLINKARM_ReadMemU8(address, count, data, status)
        if got != count:
            raise RuntimeError(('ReadMemU8', hex(address), count, got))
        return data.raw

    def u32(self, name):
        return int.from_bytes(self.read(self.symbols[name]['address'], 4), 'little')

    def state(self):
        tick0 = self.u32('uwTick')
        seq0 = self.u32('drift_model_audit_sequence')
        blocks = [(lo, hi, self.read(lo, hi - lo)) for lo, hi in self.ranges]
        def value(name):
            s = self.symbols[name]
            for lo, hi, b in blocks:
                if lo <= s['address'] and s['address'] + s['size'] <= hi:
                    return b[s['address']-lo:s['address']-lo+s['size']]
            raise KeyError(name)
        out = {n: int.from_bytes(value(n), 'little') for n in self.scalar_names}
        out['ema2_selected_alpha'] = struct.unpack('<f', value('ema2_selected_alpha'))[0]
        g = value('drift_learning_guard')
        guard = dict(zip(('coefficient', 'offset', 'noise', 'residual_mean', 'clear_sum'), struct.unpack_from('<qq3d', g)))
        guard.update(dict(zip(('tof_reference', 'tof_noise'), struct.unpack_from('<2d', g, 152))))
        guard.update(dict(zip(('reference_tick', 'clear_tick', 'last_tof_sequence', 'residual_high_tick', 'clear_count'), struct.unpack_from('<5I', g, 168))))
        guard.update(dict(zip(('ready', 'allowed', 'reason', 'clear_timer', 'residual_high_timer', 'residual_count', 'residual_head', 'tof_count', 'tof_near', 'pending_count', 'pending_head'), g[284:295])))
        guard['reason_names'] = [v for k, v in REASONS.items() if guard['reason'] & k]
        n = struct.unpack_from('<3d5I2B', value('drift_noise_reference'))
        noise = dict(zip(('mean', 'm2', 'measured_sigma', 'sample_count', 'clear_since_tick', 'first_sample_tick', 'last_tick', 'sigma_counts', 'collecting', 'ready'), n))
        d = value('dComps')
        model = {k: struct.unpack_from('<q', d, off)[0] for k, off in [('coefficient', 48), ('offset', 80), ('numerator', 120), ('denominator', 152)]}
        model['gain'] = model['coefficient'] / 10000.0
        model['post_ema'] = int.from_bytes(d[:4], 'little')
        out['object_detected'] = value('state')[9]
        tick1 = self.u32('uwTick')
        seq1 = self.u32('drift_model_audit_sequence')
        out.update({'host_epoch_s': time.time(), 'host_monotonic_ns': time.monotonic_ns(), 'tick_before': tick0, 'tick_after': tick1, 'audit_sequence_before': seq0, 'audit_sequence_after': seq1, 'model_transition_during_read': seq1 != seq0, 'learning_guard': guard, 'noise_reference': noise, 'model': model})
        if tick1 < tick0:
            raise RuntimeError('Reboot during state read')
        return out

    def ring(self, kind, previous):
        spec = self.layout[kind]
        before = int.from_bytes(self.read(spec['sequence_address'], 4), 'little')
        lo, hi = max(1, previous + 1, before - spec['depth'] + 3), before - 1
        if hi < lo:
            return [], previous, 0
        chunks = []
        start = lo
        while start <= hi:
            slot = (start - 1) % spec['depth']
            count = min(hi - start + 1, spec['depth'] - slot)
            raw = self.read(spec['address'] + slot * spec['entry_size'], count * spec['entry_size'])
            chunks.append((start, count, raw))
            start += count
        after = int.from_bytes(self.read(spec['sequence_address'], 4), 'little')
        if after < before or after - before >= spec['depth'] - 3:
            raise RuntimeError('Ring changed epoch or overran while reading')
        safe_lo = max(lo, after - spec['depth'] + 3)
        rows = []
        for start, count, raw in chunks:
            for i in range(count):
                seq = start + i
                entry = raw[i * spec['entry_size']:(i + 1) * spec['entry_size']]
                if seq < safe_lo or int.from_bytes(entry[:4], 'little') != seq:
                    continue
                row = {}
                for name, f in spec['fields'].items():
                    b = entry[f['offset']:f['offset']+f['size']]
                    if name == 'stage_us':
                        for k, label in enumerate(('read', 'kalman', 'drift', 'ema', 'calibration', 'tof', 'curve', 'fusion')):
                            row[label+'_us'] = int.from_bytes(b[k*4:k*4+4], 'little')
                    else:
                        row[name] = int.from_bytes(b, 'little', signed=f['signed'])
                rows.append(row)
        return rows, (rows[-1]['sequence'] if rows else previous), max(0, lo - previous - 1)


class CLIReader(Reader):
    """Bound each attach/read subprocess; a native DLL failure cannot hang CAN."""
    def __init__(self, elf, folder):
        super().__init__(elf, native=False)
        self.folder = folder/'batch_latest'
        self.folder.mkdir()
        self.prepared = False
        self.cache = []

    def close(self):
        self.prepared = False

    def connect(self):
        self.refresh()
        self.prepared = True

    def refresh(self):
        requests = [('cpuid', 0xE000ED00, 4), ('tick_before', self.symbols['uwTick']['address'], 4)]
        for kind, spec in self.layout.items():
            if kind in ('audit', 'evaluation'):
                requests.append((kind+'_before', spec['sequence_address'], 4))
        requests.extend((f'state_{i}', lo, hi-lo) for i, (lo, hi) in enumerate(self.ranges))
        requests.extend((kind+'_ring', self.layout[kind]['address'], self.layout[kind]['bytes']) for kind in ('audit', 'evaluation'))
        for kind in ('audit', 'evaluation'):
            requests.append((kind+'_after', self.layout[kind]['sequence_address'], 4))
        requests.append(('tick_after', self.symbols['uwTick']['address'], 4))
        requests.extend((f'flash_{i}', seg['address'], len(seg['data'])) for i, seg in enumerate(self.segments))
        commands = ['exec DisableAutoUpdateFW', 'exec ForceAttachTarget=1', 'exec SetAllowStopMode=0', 'exec InhibitConnectRetries=1', 'connect']
        for name, addr, size in requests:
            commands.append(f'savebin "{(self.folder/(name+".bin")).as_posix()}", 0x{addr:08X}, 0x{size:X}')
        commands.append('exit')
        script = self.folder/'read.jlink'
        script.write_text('\n'.join(commands)+'\n', encoding='ascii')
        proc = subprocess.run([str(JLINK), '-device', 'STM32F722RE', '-if', 'SWD', '-speed', '4000', '-SelectEmuBySN', '59768883', '-AutoConnect', '0', '-ExitOnError', '1', '-NoGui', '1', '-CommanderScript', str(script)], capture_output=True, text=True, timeout=8)
        log = proc.stdout+'\n'+proc.stderr
        (self.folder/'jlink.log').write_text(log, encoding='utf-8')
        if proc.returncode != 0 or any(x in log.lower() for x in ('unknown command', 'syntax error', 'command string error')):
            raise RuntimeError(('Bounded CLI read failed', proc.returncode, log[-800:]))
        raw = {name: (self.folder/(name+'.bin')).read_bytes() for name, _, _ in requests}
        if (int.from_bytes(raw['cpuid'], 'little') >> 4) & 0xfff != 0xc27:
            raise RuntimeError('Unexpected target CPUID')
        for i, seg in enumerate(self.segments):
            if raw[f'flash_{i}'] != seg['data']:
                raise RuntimeError('Running flash no longer matches identified ELF')
        self.cache = [(addr, addr+size, raw[name]) for name, addr, size in requests if name.startswith('state_') or name.endswith('_ring')]
        self.meta = {name: int.from_bytes(raw[name], 'little') for name in ('tick_before', 'tick_after', 'audit_before', 'audit_after', 'evaluation_before', 'evaluation_after')}
        if self.meta['tick_after'] < self.meta['tick_before']:
            raise RuntimeError('Reboot during CLI batch')

    def state(self):
        if self.prepared:
            self.prepared = False
        else:
            self.refresh()
        self.read_counts = {}
        self.mode = 'state'
        s = super().state()
        s['read_transport'] = 'bounded_JLink_CLI_4000kHz'
        return s

    def ring(self, kind, previous):
        self.mode = kind
        self.read_counts = {}
        return super().ring(kind, previous)

    def read(self, address, count):
        if count == 4:
            if address == self.symbols['uwTick']['address']:
                key = 'tick'
            elif address == self.layout['audit']['sequence_address']:
                key = 'audit'
            elif address == self.layout['evaluation']['sequence_address']:
                key = 'evaluation'
            else:
                key = None
            if key:
                nth = self.read_counts.get(key, 0)
                self.read_counts[key] = nth + 1
                return self.meta[key+('_before' if nth == 0 else '_after')].to_bytes(4, 'little')
        for lo, hi, data in self.cache:
            if lo <= address and address+count <= hi:
                return data[address-lo:address-lo+count]
        raise RuntimeError(('Range not in completed CLI batch', hex(address), count))


def compact(s):
    return {'tick_ms': s['uwTick'], 'gain': s['model']['gain'], 'numerator': s['model']['numerator'], 'denominator': s['model']['denominator'], 'n_data': s.get('n_data'), 'learning_updates': s['drift_eval_learning_updates'], 'reason': s['learning_guard']['reason_names'], 'allowed': s['learning_guard']['allowed'], 'residual_mean': s['learning_guard']['residual_mean'], 'noise': s['learning_guard']['noise'], 'noise_ready': s['noise_reference']['ready'], 'noise_samples': s['noise_reference']['sample_count'], 'detector_enabled': s.get('flag_algorithmDO'), 'object_detected': s.get('object_detected'), 'DOinit': s.get('flag_DOinit'), 'alpha': s['ema2_selected_alpha']}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('folder', type=Path)
    p.add_argument('--identity', required=True, type=Path)
    p.add_argument('--seconds', type=float, default=1200)
    p.add_argument('--no-can', action='store_true')
    p.add_argument('--cli', action='store_true')
    a = p.parse_args()
    identity = json.loads(a.identity.read_text())
    assert identity['identified']
    elf = Path(identity['matched_elf'])
    a.folder.mkdir(parents=True, exist_ok=False)
    r = CLIReader(elf, a.folder) if a.cli else Reader(elf)
    (a.folder/'layout.json').write_text(json.dumps(r.layout, indent=2))
    (a.folder/'symbols.json').write_text(json.dumps(r.symbols, indent=2))
    event_file = (a.folder/'events.jsonl').open('w', encoding='utf-8')
    states = (a.folder/'states.jsonl').open('w', encoding='utf-8')
    def event(kind, **kw):
        record = {'host_epoch_s': time.time(), 'kind': kind, **kw}
        event_file.write(json.dumps(record, ensure_ascii=False)+'\n'); event_file.flush()
        print(json.dumps(record, ensure_ascii=False), flush=True)
    capture, worker = None, None
    csv_files, writers = {}, {}
    connected, last_tick, epoch = False, None, 0
    previous = {'audit': 0, 'evaluation': 0}
    count = 0
    last_report = 0
    start = time.monotonic()
    try:
        if not a.no_can:
            sys.path.insert(0, str(TOOLS))
            from mobile_telemetry import Capture
            can_dir = a.folder/'can'; can_dir.mkdir()
            capture = Capture(can_dir)
            capture.phase = 'stationary_30cm_boot_diagnosis'
            worker = threading.Thread(target=capture.run, daemon=True); worker.start()
            if not capture.ready.wait(8) or capture.failure:
                raise RuntimeError(('CAN unavailable', capture.failure))
        while time.monotonic() - start < a.seconds and not (a.folder/'STOP').exists():
            cycle = time.monotonic()
            try:
                if not connected:
                    r.connect(); connected = True
                    event('attached', elf_sha256=hashlib.sha256(r.data).hexdigest())
                s = r.state()
                if last_tick is not None and s['uwTick'] < last_tick:
                    epoch += 1
                    previous = {'audit': 0, 'evaluation': 0}
                    event('reboot_detected', epoch=epoch, previous_tick_ms=last_tick, new_tick_ms=s['uwTick'])
                last_tick = s['uwTick']
                s['epoch'] = epoch
                if capture and capture.latest_snapshot:
                    s['latest_can_snapshot'] = capture.latest_snapshot.copy()
                states.write(json.dumps(s, ensure_ascii=False)+'\n'); states.flush()
                for kind in previous:
                    rows, seq, missed = r.ring(kind, previous[kind])
                    if missed:
                        event('ring_gap', epoch=epoch, ring=kind, skipped_sequences=missed, initial_attach=previous[kind] == 0)
                    previous[kind] = seq
                    for row in rows:
                        row = {'epoch': epoch, 'host_read_epoch_s': s['host_epoch_s'], **row}
                        if kind not in writers:
                            csv_files[kind] = (a.folder/(kind+'.csv')).open('w', newline='', encoding='utf-8')
                            writers[kind] = csv.DictWriter(csv_files[kind], fieldnames=list(row)); writers[kind].writeheader()
                        writers[kind].writerow(row)
                    if kind in csv_files:
                        csv_files[kind].flush()
                count += 1
                if count == 1:
                    (a.folder/'READY.json').write_text(json.dumps(compact(s), indent=2))
                    event('ready', state=compact(s), can_enabled=capture is not None)
                if time.monotonic() - last_report > 20:
                    event('state', epoch=epoch, state=compact(s), can_frames=dict(capture.ids) if capture else None)
                    last_report = time.monotonic()
            except Exception as exc:
                if connected or count == 0:
                    event('read_unavailable', detail=repr(exc))
                connected = False
                r.close()
            time.sleep(max(0.05, 0.5 - (time.monotonic()-cycle)))
    finally:
        r.close()
        if capture:
            capture.stop.set(); worker.join(5)
            (a.folder/'can_summary.json').write_text(json.dumps(capture.summary(), indent=2), encoding='utf-8')
        event('stopped', state_samples=count, reboot_epochs=epoch, halt_reset_write_commands_sent=0, sensor_commands_sent=0, motor_commands_sent=0)
        for f in csv_files.values(): f.close()
        states.close(); event_file.close()


if __name__ == '__main__':
    main()
