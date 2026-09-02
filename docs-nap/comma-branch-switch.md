# Switching branches on a Comma

Use this when moving a device between `nap-dev` and `naponsp-dev` (or
any other NAP branch). Do not reboot until panda matches `pandad.cc`.

naponsp layout: sources live under `openpilot/`. nap-dev layout: they
live at the repo root. The gitlinks are the same idea either way.

## Checklist

1. Fetch and check out, fast-forward only:

   ```bash
   git fetch origin
   git checkout naponsp-dev
   git merge --ff-only origin/naponsp-dev
   ```

2. Update **every** gitlink, not only `opendbc_repo`:

   ```bash
   git submodule sync --recursive
   git submodule update --init
   ```

   On naponsp that is at least: `panda`, `opendbc_repo`, `msgq_repo`,
   `rednose_repo`, `teleoprtc_repo`, `tinygrad_repo`, and
   `openpilot/sunnypilot/neural_network_data`. `openpilot/cereal` is
   in-tree here, not a submodule.

   `git submodule update --init opendbc_repo` is not enough. That is
   how a leftover nap-dev panda SHA survived a naponsp pull and broke
   device scons.

3. Confirm panda is checked out, not missing:

   ```bash
   git submodule status panda
   ```

   A `-` prefix means panda was not updated. Fix with
   `git submodule update --init panda`.

4. Panda SHA must equal the gitlink:

   ```bash
   git -C panda rev-parse HEAD
   git ls-tree HEAD panda
   ```

   They must be the same commit. On current `naponsp-dev` that is
   `649a6281f4ce9b25cf5572b6de7829a7f17df7a0`.

5. Every `health.` field `pandad.cc` uses must exist in
   `panda/board/health.h` at that SHA. Host pytest
   `openpilot/selfdrive/pandad/tests/test_panda_health_pin.py` checks
   this. On the device, grep is enough:

   ```bash
   python3 - <<'PY'
   import re, pathlib
   src = pathlib.Path("openpilot/selfdrive/pandad/pandad.cc").read_text()
   hdr = pathlib.Path("panda/board/health.h").read_text()
   used = set(re.findall(r"\bhealth\.([A-Za-z_][A-Za-z0-9_]*)", src))
   missing = sorted(name for name in used if name not in hdr)
   print("ok" if not missing else missing)
   PY
   ```

6. Leftover untracked nap-dev dirs at the naponsp repo root (`cereal/`,
   `selfdrive/`, `system/`) are debris from the other layout. Report
   them. Do not wipe them blindly.

7. Host scons of pandad does **not** prove the device panda SHA. The
   host can be on `649a6281` while the Comma is still on nap-dev
   `282fae70`. Update the device submodule, then scons **on the
   device**.

8. Never reboot until step 4 passes. Never `systemctl` or reboot unless
   Jack says so.
