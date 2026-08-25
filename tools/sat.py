#!/usr/bin/env python3
import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

SAT = Path(__file__).resolve().parent.parent / "satellite"

TARGETS = {
    "classic": {
        "idf_target": "esp32",
        "args": [],
        "port_glob": "ttyUSB",
    },
    "s3": {
        "idf_target": "esp32s3",
        "args": ["-B", "build.s3", "-DIDF_TARGET=esp32s3", "-DSDKCONFIG=sdkconfig.s3"],
        "port_glob": "ttyACM",
    },
}


def idf_py():
    on_path = shutil.which("idf.py")
    if on_path:
        return on_path
    idf_path = os.environ.get("IDF_PATH")
    if idf_path:
        candidate = Path(idf_path) / "tools" / "idf.py"
        if candidate.exists():
            return str(candidate)
    sys.exit("cannot find idf.py -- activate the IDF environment first "
             "(get_idf), which sets IDF_PATH")


def guess_port(glob):
    try:
        found = sorted(p for p in os.listdir("/dev") if p.startswith(glob))
    except OSError:
        return None
    return "/dev/" + found[0] if found else None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command",
                    help="idf.py command: build, flash, monitor, menuconfig, "
                         "fullclean, size ...")
    ap.add_argument("target", choices=sorted(TARGETS),
                    help="which satellite")
    ap.add_argument("--port", "-p", default=None,
                    help="serial port; guessed from the target if omitted")
    args, rest = ap.parse_known_args()

    spec = TARGETS[args.target]

    cmd = [idf_py(), *spec["args"]]

    if args.command in ("flash", "monitor", "erase-flash", "app-flash"):
        port = args.port or guess_port(spec["port_glob"])
        if not port:
            sys.exit(f"no /dev/{spec['port_glob']}* found for the {args.target} "
                     f"satellite -- pass --port")
        cmd += ["-p", port]

    cmd.append(args.command)
    cmd += [a for a in rest if a != "--"]

    print("+ " + " ".join(cmd), file=sys.stderr)
    raise SystemExit(subprocess.call(cmd, cwd=SAT))


if __name__ == "__main__":
    main()
