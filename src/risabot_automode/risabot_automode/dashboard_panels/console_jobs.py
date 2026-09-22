"""One bounded, isolated planning worker; no shell or robot-process control.

The HTTP thread never waits for a fit/search. Only the fixed planning worker
can be started here. Killing it cannot stop a ROS node, driver, or launch.
"""
from collections import deque
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import uuid


class JobConflict(ValueError):
    pass


class PlanningJobs:
    OPERATIONS = frozenset(('fit', 'mission'))
    MAX_INPUT_BYTES = 2_000_000
    MAX_OUTPUT_BYTES = 8_000_000

    def __init__(self, timeout=90.0):
        self.timeout = timeout
        self.lock = threading.RLock()
        self.job = None
        self.process = None
        self.closed = False
        self.logs = deque(maxlen=80)

    def start(self, operation, payload, revision):
        if operation not in self.OPERATIONS:
            raise ValueError('Unknown planning operation')
        encoded = json.dumps(payload, allow_nan=False).encode('utf-8')
        if len(encoded) > self.MAX_INPUT_BYTES:
            raise ValueError('Planning input too large')
        with self.lock:
            if self.closed:
                raise JobConflict('Planning service is shutting down')
            if self.job and self.job['state'] in ('running', 'stopping'):
                raise JobConflict(f"{self.job['operation']} is running. Stop it before starting another job.")
            directory = tempfile.TemporaryDirectory(prefix='carbot-plan-')
            root = Path(directory.name)
            (root / 'input.json').write_bytes(encoded)
            env = os.environ.copy()
            for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
                env[key] = '1'
            command = [sys.executable, '-u', str(Path(__file__).with_name('console_worker.py')),
                       operation, str(root)]
            try:
                process = subprocess.Popen(
                    command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, env=env,
                    creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            except Exception:
                directory.cleanup()
                raise
            self.process = process
            self.logs.clear()
            self.job = dict(id=uuid.uuid4().hex, operation=operation, revision=revision,
                            state='running', started=time.monotonic(), error=None, result=None)
            job = self.job
            threading.Thread(target=self._read_log, args=(process, job), daemon=True).start()
            threading.Thread(target=self._watch, args=(process, job, directory), daemon=True).start()
            return self.snapshot()

    def _read_log(self, process, job):
        # Bounded reads also handle a broken worker printing without newlines.
        with process.stdout:
            for chunk in iter(lambda: process.stdout.readline(512), b''):
                with self.lock:
                    if self.job is job:
                        self.logs.append(chunk.decode('utf-8', errors='replace').rstrip())

    def _watch(self, process, job, directory):
        error, result = None, None
        state = 'failed'
        try:
            try:
                code = process.wait(timeout=self.timeout)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                code = None
                error = f'Planning timed out after {self.timeout:g}s. Reduce inputs or retry.'
            if code == 0:
                path = Path(directory.name) / 'result.json'
                if path.stat().st_size > self.MAX_OUTPUT_BYTES:
                    raise ValueError('Planning output exceeds size limit')
                result = json.loads(path.read_text(encoding='utf-8'))
                state = 'succeeded'
            elif error is None:
                error = f'Planning worker exited ({code}). Check the log, then retry.'
        except Exception as exc:
            error = f'Planning result unavailable: {exc}'
        finally:
            try:
                directory.cleanup()
            except OSError as exc:
                error = f'{error or "Planning completed"}; temporary-file cleanup failed: {exc}'
            with self.lock:
                if job['state'] == 'stopping':
                    state, error, result = 'cancelled', 'Stopped by operator. You can retry.', None
                job.update(state=state, error=error, result=result,
                           elapsed=round(time.monotonic() - job['started'], 3))
                if self.job is job:
                    self.process = None

    def cancel(self):
        with self.lock:
            if self.process and self.job['state'] == 'running':
                self.job['state'] = 'stopping'
                # This is our fixed computation worker, never an arbitrary PID.
                self.process.kill()
            return self.snapshot()

    def snapshot(self, include_result=False):
        with self.lock:
            if self.job is None:
                return None
            result = {key: value for key, value in self.job.items()
                      if key not in ('result', 'started')}
            result['elapsed'] = self.job.get('elapsed', round(time.monotonic() - self.job['started'], 3))
            result['log'] = list(self.logs)
            if include_result:
                result['result'] = self.job['result']
            return result

    def close(self):
        with self.lock:
            self.closed = True
            self.cancel()
