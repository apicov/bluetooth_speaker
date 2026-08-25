#!/usr/bin/env python3
"""
Syntax-check firmware sources without idf.py.

Reuses the exact compile command a previous `idf.py build` recorded in
build/compile_commands.json, but runs it with -fsyntax-only, so it needs no
working IDF virtualenv and writes no objects. It catches what a full build
catches short of linking: unbalanced preprocessor nesting, missing symbols,
type errors, bad format strings.

A fast check, not a substitute for a build. Two limits are worth knowing:

  * A stale compile_commands.json describes the file list and flags of the last
    real build, so a NEW source file has no entry until the project is
    configured again. This works around that by cloning a sibling's flags, and
    says when it has.
  * It only compiles the branches the LAST BUILD's sdkconfig selected. Code
    behind a Kconfig symbol that was `n` is never seen, which is exactly where
    an edit rots unnoticed. Use --with / --without to reach those branches:

        tools/syntax_check.py satellite
        tools/syntax_check.py satellite/main/play.c              # one file
        tools/syntax_check.py satellite --with CONFIG_DANCEFLOOR_ENABLE_MARKER \\
                                        --with CONFIG_DANCEFLOOR_MARKER_GPIO=4
        tools/syntax_check.py satellite --with CONFIG_DANCEFLOOR_OUT_MONO \\
                                        --without CONFIG_DANCEFLOOR_OUT_STEREO

Overrides are applied by writing a patched COPY of the generated sdkconfig.h to
a temporary directory and putting it first on the include path. Passing plain
-D does not work: it lands before the real sdkconfig.h, which then redefines
the symbol, and IDF builds with -Werror.

KCONFIG DEPENDENCIES ARE NOT MODELLED. `depends on` means real menuconfig would
define a parent and its children together, so force them together -- otherwise
the result is an "undeclared" error that is an artifact of the override rather
than a defect in the code.
"""
import json
import shlex
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

## @brief The dancefloor tree. Every path printed below is relative to it.
ROOT = Path(__file__).resolve().parent.parent


def db_for(project: Path):
    """
    @brief Load one project's compile_commands.json.
    @param project  The project directory.
    @return The parsed entries. Exits with a message if there is no database.
    """
    db = project / "build" / "compile_commands.json"
    if not db.exists():
        sys.exit(f"no {db} -- run a full `idf.py build` once to create it")
    return json.loads(db.read_text())


def patched_config(cmd, sets, unsets, tmp: Path):
    """
    @brief Write a shadowing sdkconfig.h into a temporary directory.

    The real one is found on the recorded include path, copied with the
    touched symbols removed, and the overrides appended.

    @param cmd     The recorded compile command, for its -I flags.
    @param sets    Symbols to define, as NAME or NAME=VALUE; a bare NAME is 1.
    @param unsets  Symbols to remove.
    @param tmp     Where to write the copy.
    @return The -I flag that puts the copy first on the include path.
    """
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
    """
    @brief Compile one file with -fsyntax-only and print the verdict.
    @param entry   One compile_commands.json entry.
    @param sets    Symbols to define. @see patched_config
    @param unsets  Symbols to remove.
    @return True if it compiled; warnings still count as a pass.
    """
    # shlex, not split(): the command carries -DIDF_VER=\"v6.0.1\", and naive
    # splitting tears the quotes off and the compiler reports an unterminated
    # string from <command-line>.
    cmd = shlex.split(entry["command"])
    # Drop the object output and the compile-only flag; add syntax-only.
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
    """
    @brief Parse the arguments, pick the files, and check them all.

    The target may be a project directory or one .c file inside one.
    """
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
    # Only this project's own sources; components come along via the includes.
    maindir = ROOT / project / "main"
    mine = [e for e in entries if maindir in Path(e["file"]).parents]
    if not mine:
        sys.exit(f"no entries for {project}/main in compile_commands.json")

    # Files added since the last configure have no entry. Every source in one
    # component compiles with the same flags, so clone a sibling's and swap the
    # filename -- which is what lets this check a split before idf.py has seen
    # it.
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


## @cond
# The entry point, not API. With anything other than one target it prints this
# module's docstring as usage, so that docstring is user-facing text.
if __name__ == "__main__":
    main()
## @endcond
