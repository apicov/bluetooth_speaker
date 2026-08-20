#!/usr/bin/env python3
"""
Build, flash and monitor the two satellites without retyping the incantation.

One source tree produces two images that coexist permanently:

    classic   sdkconfig.defaults + sdkconfig.defaults.esp32   -> build/,    sdkconfig
    s3        sdkconfig.defaults + sdkconfig.defaults.esp32s3 -> build.s3/, sdkconfig.s3

The classic one is plain `idf.py`. The S3 one is not, and the difference is the
whole reason this exists:

    idf.py -B build.s3 -DIDF_TARGET=esp32s3 -DSDKCONFIG=sdkconfig.s3 <cmd>

-DSDKCONFIG is needed on EVERY invocation, not just the first. Omit it with
-B build.s3 and the S3 build directory is reconfigured from the classic
`sdkconfig` -- a satellite built for the wrong target's pins and, with the ML
work, for the wrong feature set, reported by nothing louder than behaviour. That
is the mistake this closes.

    tools/sat.py build   classic|s3
    tools/sat.py flash   classic|s3 [--port ...]
    tools/sat.py monitor classic|s3 [--port ...]
    tools/sat.py menuconfig s3

It assembles an argument list and execs idf.py. There is no logic of its own to
drift from the build, and anything it does not recognise is passed straight
through:

    tools/sat.py build s3 -- --verbose

Needs the IDF environment already active, the same as idf.py itself:

    get_idf        # or: . ~/.espressif/tools/activate_idf_v6.0.1.sh
"""
import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

SAT = Path(__file__).resolve().parent.parent / "satellite"

# Everything that differs between the two, in one place. `args` is what turns a
# bare idf.py into the right one; empty for the classic build, which is the
# default idf.py behaviour and deliberately not spelled out.
TARGETS = {
    "classic": {
        "idf_target": "esp32",
        "args": [],
        # Classic boards here are devkits with a USB-UART bridge, so a CP2102 or
        # CH340 enumerating as ttyUSB*.
        "port_glob": "ttyUSB",
    },
    "s3": {
        "idf_target": "esp32s3",
        "args": ["-B", "build.s3", "-DIDF_TARGET=esp32s3", "-DSDKCONFIG=sdkconfig.s3"],
        # The XIAO has no bridge chip -- the S3's own USB peripheral enumerates,
        # so it is ttyACM*. Getting this wrong is the other half of the mix-up
        # this tool exists to prevent.
        "port_glob": "ttyACM",
    },
}


def idf_py():
    """How to invoke idf.py here.

    Two ways, because activating the IDF does not necessarily put it on PATH --
    the activate script sets IDF_PATH and the toolchain, and whether `idf.py` is
    also a command depends on how the environment was entered. Preferring PATH
    and falling back to $IDF_PATH/tools keeps this working under both, rather
    than under whichever one it was written on.
    """
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
    """First /dev device matching the target's expected pattern, or None."""
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
    # parse_known_args and NOT nargs=REMAINDER. REMAINDER swallows everything
    # after the positionals including flags this parser defines, so
    # `sat.py monitor s3 --port /dev/ttyACM9` guessed a port AND passed --port
    # through to idf.py -- two ports on one command line, the wrong one winning.
    args, rest = ap.parse_known_args()

    spec = TARGETS[args.target]

    cmd = [idf_py(), *spec["args"]]

    # Only serial commands take a port. Passing one to `build` is harmless but
    # it would be in the printed line, which is the thing being trusted here.
    if args.command in ("flash", "monitor", "erase-flash", "app-flash"):
        port = args.port or guess_port(spec["port_glob"])
        if not port:
            sys.exit(f"no /dev/{spec['port_glob']}* found for the {args.target} "
                     f"satellite -- pass --port")
        cmd += ["-p", port]

    cmd.append(args.command)
    cmd += [a for a in rest if a != "--"]

    # Printed in full, every time. The point of this tool is that the long form
    # is easy to get wrong, not that it should become invisible -- and a reader
    # debugging a build needs to know exactly what ran.
    print("+ " + " ".join(cmd), file=sys.stderr)
    raise SystemExit(subprocess.call(cmd, cwd=SAT))


if __name__ == "__main__":
    main()
