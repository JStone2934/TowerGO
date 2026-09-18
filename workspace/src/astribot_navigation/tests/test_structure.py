from pathlib import Path
import ast,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]

def test_python_parses_and_packages_exist():
    for path in ROOT.rglob('*.py'):ast.parse(path.read_text())
    assert len(list(ROOT.glob('*/package.xml')))==5
    for path in ROOT.glob('*/package.xml'):ET.parse(path)
def test_no_hardware_sdk_imports():
    for folder in ROOT.glob('astribot_nav_*'):
        for path in folder.rglob('*.py'):
            tree=ast.parse(path.read_text())
            for n in ast.walk(tree):
                if isinstance(n,ast.Import):assert all(not x.name.startswith('astribot_sdk') for x in n.names)
                if isinstance(n,ast.ImportFrom):assert not (n.module or '').startswith('astribot_sdk')
def test_no_host_network_or_device_mount():
    text=(ROOT/'scripts/container.sh').read_text()
    assert '--network none' in text
    assert '--network host' not in text and '--device ' not in text
