#!/usr/bin/env python3
"""Fetch the licensed, pinned upstream source used only inside the container."""
from pathlib import Path
import hashlib,urllib.request
p=Path(__file__).resolve().parents[1]/'docker/vendor/slam-toolbox-2.6.10.tar.gz'
expected='5cc31b72e89903ee1b2c44699e93a800e4e425f3b9eee17d7c0c20e12a66f8af'
if not p.exists():
    p.parent.mkdir(parents=True,exist_ok=True)
    with urllib.request.urlopen('https://codeload.github.com/SteveMacenski/slam_toolbox/tar.gz/8293d21fe5c816d0405e0cc1f3eb46c70659c267',timeout=180) as response:
        temporary=p.with_suffix('.download');temporary.write_bytes(response.read())
    if hashlib.sha256(temporary.read_bytes()).hexdigest()!=expected:raise RuntimeError('Upstream archive checksum mismatch')
    temporary.replace(p)
if hashlib.sha256(p.read_bytes()).hexdigest()!=expected:raise RuntimeError('Upstream archive checksum mismatch')
print('Verified slam_toolbox 2.6.10 source archive')
