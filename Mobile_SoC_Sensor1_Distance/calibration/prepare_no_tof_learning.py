"""Preserve the current Sensor1 source/image before removing optical learning gates."""
import hashlib
import json
from pathlib import Path
import shutil

project = Path(__file__).resolve().parent.parent
root = project/'calibration/no_tof_learning_20261014_01'
root.mkdir(exist_ok=False)
files = ['Core/Src/main.c', 'Core/Inc/drift_learning_guard.h', 'Core/Inc/drift_noise_reference.h',
         'Core/Inc/sensor1_distance_calibration.h', 'Debug/Mobile_SoC_Sensor1_Distance.elf',
         'Debug/Mobile_SoC_Sensor1_Distance.bin', 'Debug/Mobile_SoC_Sensor1_Distance.hex',
         'Debug/Mobile_SoC_Sensor1_Distance.map', 'Debug/makefile', '.cproject']
manifest = {}
for relative in files:
    source = project/relative
    destination = root/'before'/relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    manifest[relative] = hashlib.sha256(source.read_bytes()).hexdigest()
(root/'before_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
print(json.dumps({'revision': str(root), 'saved_files': len(files), 'before_elf_sha256': manifest['Debug/Mobile_SoC_Sensor1_Distance.elf']}, indent=2))
