#!/usr/bin/env python3
# kinova_gen3_ros2/test/gains_cli.py
"""Shared --profile/--kq/--zeta/--torque-limit flags -> ImpedanceGains.

Any custom flag makes the goal PROFILE_CUSTOM; the unset custom fields fall
back to medium kq / zeta 0.5 / the ceiling torque limits, so a lone
`--kq 60` is a valid goal and `--torque-limit 0` exercises the #64 floor
rejection deliberately.
"""

from rammp_arm_interfaces.msg import ImpedanceGains

PROFILES = {
    "default": ImpedanceGains.PROFILE_SESSION_DEFAULT,
    "soft": ImpedanceGains.PROFILE_SOFT,
    "medium": ImpedanceGains.PROFILE_MEDIUM,
    "stiff": ImpedanceGains.PROFILE_STIFF,
}

_KQ_MEDIUM = [80.0, 80.0, 80.0, 80.0, 30.0, 30.0, 30.0]
_TORQUE_CEIL = [39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0]


def add_gains_args(ap):
    ap.add_argument("--profile", choices=sorted(PROFILES), default="default")
    ap.add_argument("--kq", default=None, help="one value or 7-list; implies CUSTOM")
    ap.add_argument("--zeta", type=float, default=None, help="implies CUSTOM")
    ap.add_argument(
        "--torque-limit", default=None, help="one value or 7-list; implies CUSTOM"
    )


def build_gains(args):
    spec = ImpedanceGains()
    if args.kq is None and args.zeta is None and args.torque_limit is None:
        spec.profile = PROFILES[args.profile]
        return spec
    spec.profile = ImpedanceGains.PROFILE_CUSTOM
    spec.custom.kq = _seven(args.kq, _KQ_MEDIUM)
    spec.custom.zeta = args.zeta if args.zeta is not None else 0.5
    spec.custom.torque_limit = _seven(args.torque_limit, _TORQUE_CEIL)
    return spec


def _seven(s, default):
    if s is None:
        return list(default)
    v = [float(x) for x in s.split(",")]
    if len(v) == 1:
        return v * 7
    if len(v) != 7:
        raise SystemExit(f"ERROR: expected 1 or 7 values, got {len(v)}: {s}")
    return v
