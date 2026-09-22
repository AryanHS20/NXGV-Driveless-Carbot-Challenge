"""Run mocked runtime tests in a subprocess so fake ROS cannot leak into colcon."""
from pathlib import Path
import subprocess
import sys


def test_runtime_regressions():
    root = Path(__file__).resolve().parents[3]
    result = subprocess.run(
        [sys.executable, '-B', '-m', 'unittest', 'discover', '-s', str(root/'tests'), '-p', 'test_*.py'],
        cwd=root, capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
