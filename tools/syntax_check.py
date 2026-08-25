#!/usr/bin/env python3
import json
import shlex
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def db_for(project: Path):
    db = project / "build" / "compile_commands.json"
    if not db.exists():
        sys.exit(f"no {db} -- run a full `idf.py build` once to create it")
    return json.loads(db.read_text())


def patched_config(cmd, sets, unsets, tmp: Path):
    real = None
    for a in cmd:
        if a.startswith("-I") and a.endswith("/config"):
            cand = Path(a[2:]) / "sdkconfig.h"
            if cand.exists():
                real = cand
                break
    if real is None:
        sys.exit("could not find the generated sdkconfig.h on the include path")

    touched = set(unsets) | {s.partition("=")[0] for s in sets}
    kept = [l for l in real.read_text().splitlines()
            if not any(l.startswith(f"#define {s} ") or l.startswith(f"#define {s}\t")
                       for s in touched)]
    for s in sets:
        sym, _, val = s.partition("=")
        kept.append(f"#define {sym} {val or 1}")
    (tmp / "sdkconfig.h").write_text("\n".join(kept) + "\n")
    return f"-I{tmp}"


def check(entry, sets, unsets) -> bool:
    cmd = shlex.split(entry["command"])
    out = [a for i, a in enumerate(cmd)
           if a != "-c" and not (a == "-o" or (i and cmd[i - 1] == "-o"))]
    out.append("-fsyntax-only")

    tmp = None
    if sets or unsets:
        tmp = Path(tempfile.mkdtemp(prefix="syntax-check-"))
        out.insert(1, patched_config(cmd, sets, unsets, tmp))
    try:
        r = subprocess.run(out, cwd=entry["directory"], capture_output=True, text=True)
    finally:
        if tmp:
            shutil.rmtree(tmp, ignore_errors=True)

    name = Path(entry["file"]).relative_to(ROOT)
    if r.returncode == 0:
        if r.stderr.strip():
            print(f"  {name}: OK (with warnings)")
            print("\n".join("    " + l for l in r.stderr.strip().splitlines()[:12]))
        else:
            print(f"  {name}: OK")
        return True
    print(f"  {name}: FAILED")
    print("\n".join("    " + l for l in r.stderr.strip().splitlines()[:40]))
    return False


def main():
    args, sets, unsets, rest = sys.argv[1:], [], [], []
    i = 0
    while i < len(args):
        if args[i] == "--with" and i + 1 < len(args):
            sets.append(args[i + 1])
            i += 2
        elif args[i] == "--without" and i + 1 < len(args):
            unsets.append(args[i + 1])
            i += 2
        else:
            rest.append(args[i])
            i += 1
    if len(rest) != 1:
        sys.exit(__doc__)
    target = Path(rest[0])

    if target.is_dir():
        project, want = target, None
    else:
        project, want = target.parents[1], (ROOT / target).resolve()

    entries = db_for(ROOT / project)
    maindir = ROOT / project / "main"
    mine = [e for e in entries if maindir in Path(e["file"]).parents]
    if not mine:
        sys.exit(f"no entries for {project}/main in compile_commands.json")

    known = {Path(e["file"]).resolve() for e in mine}
    template = mine[0]
    for path in sorted(maindir.glob("*.c")):
        if path.resolve() not in known:
            cloned = dict(template)
            cloned["command"] = template["command"].replace(
                template["file"], str(path))
            cloned["file"] = str(path)
            mine.append(cloned)
            print(f"  (cloning flags for {path.name}, new since the last build)")

    if want:
        mine = [e for e in mine if Path(e["file"]).resolve() == want]
        if not mine:
            sys.exit(f"{target} is not a .c file under {project}/main")

    label = " ".join([f"+{s}" for s in sets] + [f"-{u}" for u in unsets])
    print(f"{project}: {len(mine)} source file(s)" + (f"  [{label}]" if label else ""))
    ok = all([check(e, sets, unsets) for e in mine])
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
