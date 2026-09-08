English | [中文](CONTRIBUTING.zh-CN.md)

# Contributing

Small rules, enforced by review.

Code comments, docstrings, log strings and commit messages are English only. Documentation is
bilingual: every `.md` has a `.zh-CN.md` twin and a language toggle on its first line; keep both
in sync. A CJK scan is part of review: `grep -rlP '[\x{4e00}-\x{9fff}]' --exclude='*.zh-CN.md'
rt-host client docs` should list only the files still named under "Known gaps" in
`rt-host/README.md`, and that list should only ever shrink.

No absolute home paths in new code. Use a variable (`FRANKA_ROOT`, `$HOME`,
`$(dirname "$0")`) instead of `/home/<user>/...`. The existing `rt-host/` scripts still carry
`/home/rongxuan_zhou/franka` and are listed as a known gap; do not add more.

Run `bash -n` and `shellcheck` on every changed shell script, and `python3 -m py_compile` on
every changed Python file. Anything that runs on the host or in the client's mirror daemon is
stdlib only.

The interface is frozen. Any change to addresses, datagram formats, file names, rates, verbs or
exit codes in `docs/INTERFACE.md` bumps the protocol tag (`FRST1` to `FRST2`) and is a
coordinated change on both halves in one PR.

Nothing that moves the robot goes in without the matching checklist step in
`docs/ACCEPTANCE.md` being re-run and the numbers recorded.

**Never commit** private keys, `keys/`, datasets, recordings or compiled binaries. `.gitignore`
covers the usual names, but check `git status` before committing.
