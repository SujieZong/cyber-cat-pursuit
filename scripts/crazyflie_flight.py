#!/usr/bin/env python3
"""Fly the Crazyflie target patterns over a Crazyradio, outside ROS 2.

The drone is deliberately *not* a ROS 2 node. cflib runs its own link and
callback threads, which deadlock against an rclpy executor when both live in
one process; keeping the Crazyflie on a plain cflib script sidesteps that
entirely. The two robots are coupled only through the physical world — the
TurtleBot sees the drone, and that is the whole interface.

All patterns are *relative* MotionCommander primitives rather than absolute
setpoints. With only a Flow deck for positioning there is no global frame to
be absolute in, and relative moves degrade gracefully as position drifts.

Usage:
    python3 scripts/crazyflie_flight.py circle
    python3 scripts/crazyflie_flight.py eight --height 0.5 --repeat 3
    python3 scripts/crazyflie_flight.py square --side 0.7
    python3 scripts/crazyflie_flight.py manual

Ctrl-C lands the drone rather than cutting the motors.
"""

import argparse
import logging
import signal
import sys
import termios
import time
import tty
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M/E7E7E7E7E7'
CACHE_DIR = Path.home() / '.cache' / 'crazyflie'

# The marker ball adds mass the stock tune does not expect, so the drone needs
# noticeably longer than the usual ~1 s to settle before it will track a
# pattern cleanly. Measured empirically; shortening it brings back wobble.
STABILISE_SEC = 4.0

_mc = None  # set once airborne, for the signal handler


def _land_and_exit(_sig, _frame):
    print('\nInterrupt — landing.')
    try:
        if _mc is not None:
            _mc.land()
            time.sleep(2)
    except Exception as exc:  # noqa: BLE001 — never let cleanup mask the exit
        print(f'Could not land cleanly: {exc}')
    sys.exit(0)


# ── Patterns ──────────────────────────────────────────────────────────────

def fly_circle(mc, args):
    for i in range(args.repeat):
        print(f'Circle {i + 1}/{args.repeat}')
        mc.circle_right(args.radius, args.speed)


def fly_eight(mc, args):
    for i in range(args.repeat):
        print(f'Figure-8 {i + 1}/{args.repeat}')
        mc.circle_right(args.radius, args.speed)
        mc.circle_left(args.radius, args.speed)


def fly_square(mc, args):
    # MotionCommander has no "fly a side" primitive, so each edge is a timed
    # velocity command. Corners overshoot slightly; acceptable for a target.
    leg = args.side / args.speed
    for i in range(args.repeat):
        print(f'Square {i + 1}/{args.repeat}')
        for start in (mc.start_forward, mc.start_left,
                      mc.start_back, mc.start_right):
            start(args.speed)
            time.sleep(leg)
        mc.stop()
        time.sleep(1.0)


MANUAL_KEYS = """
  arrows      translate            u / d   up / down by step
  q / e       yaw left / right     space   hover
  l           land and exit        ^C      emergency land
"""


def _read_key():
    """Read one keypress, decoding arrow escape sequences, without Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':  # CSI escape — an arrow key
            if sys.stdin.read(1) == '[':
                return {'A': 'UP', 'B': 'DOWN',
                        'C': 'RIGHT', 'D': 'LEFT'}.get(sys.stdin.read(1), '')
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def fly_manual(mc, args):
    print(MANUAL_KEYS)
    moves = {
        'UP': mc.start_forward, 'DOWN': mc.start_back,
        'LEFT': mc.start_left, 'RIGHT': mc.start_right,
    }
    turns = {'q': mc.start_turn_left, 'e': mc.start_turn_right}

    while True:
        key = _read_key()
        if key in moves:
            moves[key](args.speed)
        elif key in turns:
            turns[key](30)
        elif key in ('u', 'd'):
            # stop() first: move_distance is blocking and will not compose
            # with an in-flight velocity command.
            mc.stop()
            mc.move_distance(0, 0, args.step if key == 'u' else -args.step)
        elif key == ' ':
            mc.stop()
        elif key == 'l':
            return


PATTERNS = {
    'circle': fly_circle,
    'eight': fly_eight,
    'square': fly_square,
    'manual': fly_manual,
}


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('pattern', choices=sorted(PATTERNS))
    ap.add_argument('--uri', default=URI)
    ap.add_argument('--height', type=float, default=0.33,
                    help='takeoff altitude in m (tested 0.33-0.5)')
    ap.add_argument('--speed', type=float, default=0.5, help='m/s')
    ap.add_argument('--radius', type=float, default=0.7,
                    help='circle / figure-8 radius in m')
    ap.add_argument('--side', type=float, default=0.7,
                    help='square edge length in m')
    ap.add_argument('--step', type=float, default=0.1,
                    help='manual altitude step in m')
    ap.add_argument('--repeat', type=int, default=3)
    return ap.parse_args(argv)


def main(argv=None):
    global _mc
    args = parse_args(argv)
    signal.signal(signal.SIGINT, _land_and_exit)
    logging.basicConfig(level=logging.ERROR)

    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    cflib.crtp.init_drivers()

    print(f'Connecting to {args.uri} ...')
    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=str(CACHE_DIR))) as scf:
        with MotionCommander(scf, default_height=args.height) as mc:
            _mc = mc
            print(f'Takeoff to {args.height} m, settling {STABILISE_SEC} s ...')
            time.sleep(STABILISE_SEC)
            PATTERNS[args.pattern](mc, args)
            print('Landing.')


def _self_check():
    """Pattern dispatch and argument defaults, without a radio attached."""
    assert parse_args(['circle']).repeat == 3
    assert parse_args(['eight', '--height', '0.5']).height == 0.5
    assert set(PATTERNS) == {'circle', 'eight', 'square', 'manual'}

    # Square must issue 4 timed legs per repeat and stop at each corner set.
    calls = []

    class FakeMC:
        def __getattr__(self, name):
            return lambda *a: calls.append((name, a))

    fly_square(FakeMC(), parse_args(['square', '--repeat', '2']))
    assert [c[0] for c in calls].count('start_forward') == 2, calls
    assert [c[0] for c in calls].count('stop') == 2, calls
    print('self-check ok')


if __name__ == '__main__':
    if '--self-check' in sys.argv:
        _self_check()
    else:
        main()
