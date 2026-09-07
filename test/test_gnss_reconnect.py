#!/usr/bin/env python3
"""
Drive the real gnss_node binary through the field failure: port dies, stays gone, returns
as a different PTY. Asserts on the node's own log output.
"""
import os
import pty
import re
import signal
import subprocess
import sys
import tempfile
import time

def find_binary():
    """
    Locate the built nmea_serial_driver.

    Set GNSS_NODE_BIN to point at a binary built any other way (e.g. a plain g++ link on
    a dev box where colcon is not used).
    """
    env = os.environ.get('GNSS_NODE_BIN')
    if env and os.path.exists(env):
        return env
    try:
        prefix = subprocess.check_output(
            ['ros2', 'pkg', 'prefix', 'reach_ros_node'], text=True).strip()
        cand = os.path.join(prefix, 'lib', 'reach_ros_node', 'nmea_serial_driver')
        if os.path.exists(cand):
            return cand
    except Exception:
        pass
    print('Cannot find the nmea_serial_driver binary. Build the package, or set '
          'GNSS_NODE_BIN to a locally built one.', file=sys.stderr)
    sys.exit(2)


BIN = find_binary()
TMP = tempfile.mkdtemp(prefix='gnss-test-')
LINK = os.path.join(TMP, 'ttyGPSFIX')
GGA = b'$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n'

results = []


def check(name, cond, detail=''):
    results.append((name, cond, detail))
    print(('  [PASS] ' if cond else '  [FAIL] ') + name + (('  (%s)' % detail) if not cond and detail else ''))


def new_pty():
    m, s = pty.openpty()
    if os.path.islink(LINK) or os.path.exists(LINK):
        os.unlink(LINK)
    os.symlink(os.ttyname(s), LINK)
    return m, s


def start(extra=None):
    args = [BIN, '--ros-args', '-p', 'serial_port:=' + LINK, '-p', 'baud_rate:=57600']
    for k, v in (extra or {}).items():
        args += ['-p', '%s:=%s' % (k, v)]
    return subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1, env={**os.environ,
                                                       'RCUTILS_LOGGING_USE_STDOUT': '1',
                                                       'RCUTILS_LOGGING_BUFFERED_STREAM': '0'})


def drain(proc, seconds, sink):
    """Collect output for a while without blocking on a quiet process."""
    import select
    end = time.time() + seconds
    while time.time() < end:
        r, _, _ = select.select([proc.stdout], [], [], 0.1)
        if r:
            line = proc.stdout.readline()
            if not line:
                break
            sink.append(line.rstrip())


print('=== gnss_node reconnect harness ===\n')

# ---------------------------------------------------------------- reconnect cycle
print('--- Port dies, stays gone, then returns as a different PTY ---')
master, slave = new_pty()
proc = start({'reopen_timeout_s': '0.0'})   # never self-exit in this scenario
log = []
drain(proc, 1.5, log)

for _ in range(3):
    os.write(master, GGA)
drain(proc, 1.0, log)
check('opens the port at startup', any('serial opened' in l for l in log),
      '\n'.join(log[-5:]))

# the adapter leaves the bus
first_pts = os.readlink(LINK)
os.close(master)
os.close(slave)
os.unlink(LINK)
drain(proc, 6.0, log)

check('detects the read error',
      any('serial read error' in l for l in log), '\n'.join(log[-5:]))
check('reports the port is unavailable',
      any('reopen failed' in l for l in log), '\n'.join(log[-5:]))

# The old build logged every 2s forever. Count attempts over the outage.
fails = [l for l in log if 'reopen failed' in l or 'still unavailable' in l]
check('throttles failure logging (<=3 lines in ~6s outage, was ~3/6s flat)',
      len(fails) <= 3, 'got %d lines' % len(fails))

# the adapter comes back on a *different* pts. Hold a spare pair open across the gap so
# the kernel cannot hand back the number we just released.
_spare = pty.openpty()
master, slave = new_pty()
second_pts = os.readlink(LINK)
check('device returned as a different PTY node', first_pts != second_pts,
      '%s vs %s' % (first_pts, second_pts))

drain(proc, 12.0, log)
check('reopens the port by path', any('serial reopened' in l for l in log),
      '\n'.join(log[-8:]))

reopened_at = len(log)
for _ in range(3):
    os.write(master, GGA)
drain(proc, 2.0, log)
after = log[reopened_at:]
check('no new serial errors after the reconnect',
      not any('read error' in l or 'reopen failed' in l for l in after),
      '\n'.join(after[-6:]))
check('node is still running after the reconnect', proc.poll() is None,
      'exit code %s' % proc.poll())

proc.send_signal(signal.SIGINT)
try:
    proc.wait(timeout=5)
except subprocess.TimeoutExpired:
    proc.kill()
os.close(master); os.close(slave)
os.close(_spare[0]); os.close(_spare[1])
if os.path.islink(LINK):
    os.unlink(LINK)

# ---------------------------------------------------------------- absent at startup
print('\n--- Port absent at startup: must not abort ---')
proc = start({'reopen_timeout_s': '0.0'})
log2 = []
drain(proc, 3.0, log2)
alive = proc.poll() is None
check('survives a missing port at startup (old build aborted)', alive,
      'exit code %s' % proc.poll())
check('says it will keep trying',
      any('will keep trying' in l for l in log2), '\n'.join(log2[-5:]))
proc.send_signal(signal.SIGINT)
try:
    proc.wait(timeout=5)
except subprocess.TimeoutExpired:
    proc.kill()

# ---------------------------------------------------------------- bounded exit
print('\n--- Bounded outage: exits non-zero so the launcher can respawn ---')
proc = start({'reopen_timeout_s': '3.0'})
log3 = []
drain(proc, 12.0, log3)
rc = proc.poll()
if rc is None:
    proc.kill(); proc.wait()
check('exits after reopen_timeout_s', rc is not None, 'still running')
check('exit code is non-zero (a fault, not a clean stop)', rc == 1, 'rc=%s' % rc)
check('explains why it is exiting',
      any('exiting so the launcher can respawn' in l for l in log3), '\n'.join(log3[-5:]))

print('\n============================================')
passed = sum(1 for _, c, _ in results if c)
print('  passed: %d   failed: %d' % (passed, len(results) - passed))
sys.exit(0 if passed == len(results) else 1)
