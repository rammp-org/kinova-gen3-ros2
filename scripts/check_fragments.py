#!/usr/bin/env python3
"""Validate our fragments against sheppy -- the thing that will run them.

We deliberately do NOT use rammp-module-template's validate_fragment.py. It
requires a `container:` mapping, so it rejects the compose: shape sheppy
supports, and past that it checks only that container.image is a non-empty
string -- too strict and too loose at once. It is also copied into every module
repo, so each copy drifts on its own as sheppy's vocabulary grows, and none of
them is the thing that actually runs the fragment.

This reaches into sheppy's launcher internals, which is not ideal. It is the
interim until `sheppy validate` exists -- rammp-org/sheppy#14.
"""
import sys

import yaml
from sheppy.launch.docker import DockerLauncher
from sheppy.launch.docker.compose import load_service, service_to_docker_args


def check(path: str) -> list:
    with open(path) as f:
        frag = yaml.safe_load(f) or {}
    errors = list(DockerLauncher().validate(frag) or [])

    # validate() checks the fragment's shape. The compose SERVICE it points at
    # is only translated at launch, so translate it here too -- otherwise a bad
    # key inside the service sails through and surfaces on the robot.
    ref = frag.get("compose") or {}
    if ref.get("file") and ref.get("service"):
        try:
            service = load_service(ref["file"], ref["service"], {})
        except (OSError, KeyError) as e:
            return errors + [f"cannot load {ref['file']}#{ref['service']}: {e}"]
        _, _, _, errs, warns = service_to_docker_args(service)
        errors += list(errs)
        for w in warns:
            print(f"  {path}: warning: {w}")
    return errors


def main() -> int:
    failed = False
    for path in sys.argv[1:]:
        errs = check(path)
        if errs:
            failed = True
            for e in errs:
                print(f"  {path}: {e}")
        else:
            print(f"  {path}: ok")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
