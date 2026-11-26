"""Build only the private JPEG adapter; no installation or service changes."""
from pathlib import Path
import hashlib
import json
import subprocess

root = Path(__file__).resolve().parents[1]
command = ['cc', '-O2', '-fPIC', '-shared', '-Wall', '-Wextra', '-Werror',
           str(root/'strict_jpeg.c'), '-o', str(root/'libstrictjpeg.so'), '-ljpeg']
subprocess.run(command, check=True)
report = dict(command=command, source_sha256=hashlib.sha256((root/'strict_jpeg.c').read_bytes()).hexdigest(),
              binary_sha256=hashlib.sha256((root/'libstrictjpeg.so').read_bytes()).hexdigest(),
              libraries=subprocess.check_output(['ldd',str(root/'libstrictjpeg.so')], text=True))
(root/'strict_jpeg_build.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
