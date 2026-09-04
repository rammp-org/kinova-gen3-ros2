#!/usr/bin/env python3
"""Exercise every ROS control surface against a RUNNING kinova_gen3_node.

    ros2 run kinova_gen3_ros2 kinova_gen3_node --sim --urdf ...     # or the real arm
    python3 run_conformance.py

Attaches to whatever node is up; never launches one. The arbitration mode is fixed at
launch, so the runner reads it from /control_status and SKIPS the checks that need the
other mode, naming them, rather than passing silently.

Ordering is deliberate: state, then the e-stop, then arbitration, then streaming, then
motion. Nothing that commands the arm runs until the stop path has been proven in this
session -- and if the e-stop checks fail, the run aborts before any motion.

    --sections a,b     run only these sections
    --no-motion        skip everything that commands the arm
    --list             print the checks and exit
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

import checks_arbitration  # noqa: F401  (registers checks on import)
import checks_gripper  # noqa: F401
import checks_motion  # noqa: F401
import checks_state  # noqa: F401
import checks_streaming  # noqa: F401
from harness import FAIL, PASS, REGISTRY, SKIP, Ctx, Result

SECTION_ORDER = ["state", "arbitration", "streaming", "motion", "gripper"]
# The e-stop must be proven before anything commands the arm. These run first within
# their section, and a failure aborts the run.
ESTOP_CRITICAL = ("engaging /estop latches", "a fresh clear releases")

GREEN, RED, YELLOW, DIM, RESET = (
    "\033[32m",
    "\033[31m",
    "\033[33m",
    "\033[2m",
    "\033[0m",
)
MARK = {
    PASS: f"{GREEN}PASS{RESET}",
    FAIL: f"{RED}FAIL{RESET}",
    SKIP: f"{YELLOW}SKIP{RESET}",
}


def detect_mode(ctx) -> str:
    cs = ctx.control_status()
    if cs is None:
        return "unknown"
    return "enforced" if cs.arbitration_enabled else "disabled"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sections", default=",".join(SECTION_ORDER))
    ap.add_argument("--no-motion", action="store_true")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    wanted = [s.strip() for s in args.sections.split(",") if s.strip()]

    if args.list:
        for sec in SECTION_ORDER:
            for s, name, _, motion, mode in REGISTRY.checks:
                if s == sec:
                    tags = []
                    if motion:
                        tags.append("MOTION")
                    if mode:
                        tags.append(mode)
                    suffix = f"  [{', '.join(tags)}]" if tags else ""
                    print(f"  {sec:<12s} {name}{suffix}")
        return 0

    rclpy.init()
    node = Node("kinova_conformance")
    ctx = Ctx(node)

    print("waiting for the node's latched status topics ...")
    if ctx.control_status(fresh=False) is None:
        print(f"{RED}no /control_status. Is kinova_gen3_node running?{RESET}")
        rclpy.shutdown()
        return 2
    mode = detect_mode(ctx)
    print(f"attached. arbitration_mode = {mode}\n")

    results, aborted = [], False
    try:
        for sec in SECTION_ORDER:
            if sec not in wanted:
                continue
            checks = [c for c in REGISTRY.checks if c[0] == sec]
            if not checks:
                continue
            print(f"{DIM}--- {sec} ---{RESET}")
            for _, name, fn, needs_motion, needs_mode in checks:
                if needs_motion and args.no_motion:
                    r = Result(name, SKIP, "--no-motion")
                elif needs_mode and needs_mode != mode:
                    r = Result(
                        name,
                        SKIP,
                        f"needs arbitration_mode={needs_mode}, " f"node is {mode}",
                    )
                else:
                    try:
                        r = fn(ctx)
                        r.name = name
                    except Exception as e:  # a raising check is a
                        r = Result(
                            name, FAIL, f"{type(e).__name__}: {e}"
                        )  # failing one
                results.append((sec, r))
                print(f"  {MARK[r.status]}  {name}")
                if r.detail:
                    print(f"        {DIM}{r.detail}{RESET}")

                if (
                    r.status == FAIL
                    and sec == "arbitration"
                    and any(k in name for k in ESTOP_CRITICAL)
                ):
                    print(
                        f"\n{RED}ABORTING: the e-stop path failed. Nothing that "
                        f"commands the arm will be run.{RESET}"
                    )
                    aborted = True
                    break
            print()
            if aborted:
                break
    finally:
        print(
            f"{DIM}restoring: closing any session, releasing ownership, clearing "
            f"e-stop ...{RESET}"
        )
        ctx.cleanup()
        node.destroy_node()
        rclpy.shutdown()

    npass = sum(1 for _, r in results if r.status == PASS)
    nfail = sum(1 for _, r in results if r.status == FAIL)
    nskip = sum(1 for _, r in results if r.status == SKIP)
    print(f"\n{npass} passed, {nfail} failed, {nskip} skipped   (mode={mode})")
    if nskip:
        print(f"{YELLOW}skipped:{RESET}")
        for sec, r in results:
            if r.status == SKIP:
                print(f"  {sec}/{r.name} -- {r.detail}")
    if nfail:
        print(f"{RED}failed:{RESET}")
        for sec, r in results:
            if r.status == FAIL:
                print(f"  {sec}/{r.name}\n      {r.detail}")
    return 1 if (nfail or aborted) else 0


if __name__ == "__main__":
    sys.exit(main())
