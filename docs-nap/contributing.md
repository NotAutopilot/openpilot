# Contributing

Pull requests welcome against `nap-dev`. For anything non-trivial, open an issue first.

## Branches

| openpilot branch | opendbc branch | purpose |
|------------------|----------------|---------|
| `nap-dev`        | `nap-dev`      | active development |
| `nap-alpha`      | `nap-alpha`    | tester builds (mirrors `nap-dev` when stable) |
| `nap-release`    | `nap-release`  | production release (what `installer.comma.ai/NotAutopilot/openpilot/nap-release` serves) |
| `naponsp-dev`    | `naponsp-dev`  | sunnypilot fork (keep sunnypilot-specific code here only) |

Do not push sunnypilot-specific code (MADS, `CP_SP`, etc.) to `nap-dev` or `nap-alpha`.

## Submodules

`panda` and `opendbc_repo` use NotAutopilot forks. After checkout:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

When bumping the `opendbc_repo` pointer, push the submodule change first, then the pointer bump in the main repo.

## Local build (macOS)

```bash
source .venv/bin/activate
scons -j$(sysctl -n hw.ncpu)
```

## CI build (Linux x86_64)

Reproduces the GitHub Actions build locally:

```bash
docker build -t nap-ci -f Dockerfile.openpilot .
docker run --rm nap-ci bash -c "source .venv/bin/activate && scons -j\$(nproc)"
```

## Pre-AP safety tests

```bash
source .venv/bin/activate
python -m pytest opendbc_repo/opendbc/safety/tests/test_tesla_preap.py -v
```

Safety-critical code (`opendbc_repo/opendbc/safety/**`) must ship with test coverage. See the existing test file for the pattern.

## Pre-push checklist

Before pushing to any `nap-*` branch:

1. Local `scons` succeeds
2. `pytest opendbc_repo/opendbc/safety/tests/test_tesla_preap.py` green
3. If touching `safety/`, run the Docker CI build too
4. If bumping `opendbc_repo`, confirm the submodule SHA has been pushed upstream

## Release notes

When pushing user-facing changes to `nap-dev`, update the top section of `RELEASES.md`. The comma updater shows this on the device's Software update screen.

Format:
```
Version X.Y.Z (YYYY-MM-DD)
========================
* Short description of change
* Another change
```

Only the text before the first blank line is displayed. Keep it concise. File is parsed by `system/updated/updated.py:parse_release_notes()`.

## Bug reports

Report in `#bug-reports` on [Discord](https://discord.gg/notautopilot), not DMs. Include:

- What happened (symptom, not just "didn't work")
- Car year/model (must be pre-AP Model S)
- NAP branch + commit if known
- Route ID from [comma connect](https://connect.comma.ai) if applicable
