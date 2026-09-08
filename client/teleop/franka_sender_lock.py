"""Advisory mutual exclusion between the processes that send UDP targets to the torque servo.

Only one sender may drive the servo at a time (teleop bridge, a policy loop, an automatic collector). The
servo on the RT host latches the first sender and drops the rest, so a second sender would
silently do nothing; this lock makes the collision loud instead. The lock is a flock() on a file,
so a crashed holder releases it automatically.

Usage (keep the returned object alive for the life of the process):

    from franka_sender_lock import acquire_sender_lock
    _lock = acquire_sender_lock("my_sender")     # exits 75 if another sender holds it
"""
import fcntl
import os
import sys
import time

LOCK_PATH = "/tmp/franka_sender.lock"
EXIT_BUSY = 75  # same code family as franka-ctl "busy"


def acquire_sender_lock(name, path=LOCK_PATH):
    f = open(path, "a+")
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        f.seek(0)
        holder = f.read().strip() or "unknown"
        print("franka_sender_lock: another UDP sender is active (%s); refusing to start %s" % (holder, name),
              file=sys.stderr)
        sys.exit(EXIT_BUSY)
    f.seek(0)
    f.truncate()
    f.write("%s pid=%d since=%d\n" % (name, os.getpid(), int(time.time())))
    f.flush()
    try:
        os.chmod(path, 0o666)
    except OSError:
        pass
    return f
