# notautopilot (NAP) — Pre-AP Tesla on openpilot

## Pre-push checklist

Before pushing to any NAP branch (nap-dev, nap-alpha, nap-release), run both builds and safety tests:

### 1. Local build (macOS)
```bash
source .venv/bin/activate
scons -j$(sysctl -n hw.ncpu)
```

### 2. Docker CI build (Linux x86_64)
Verifies the build works on Linux, including tinygrad model compilation and panda firmware.
```bash
docker build -t nap-ci -f Dockerfile.openpilot .
docker run --rm nap-ci bash -c "source .venv/bin/activate && scons -j\$(nproc)"
```

### 3. Pre-AP safety tests
```bash
source .venv/bin/activate
python -m pytest opendbc_repo/opendbc/safety/tests/test_tesla_preap.py -v
```

## Release notes

When pushing user-facing changes to nap-dev, update the top section of `RELEASES.md`. The comma updater shows this on the device's Software update screen. Format:

```
Version X.Y.Z (YYYY-MM-DD)
========================
* Short description of change
* Another change
```

Only the text before the first blank line (`\n\n`) is displayed. Keep it concise. The file is parsed by `system/updated/updated.py:parse_release_notes()`.

## Branch structure

| openpilot branch | opendbc branch | purpose |
|-----------------|---------------|---------|
| nap-dev | nap-dev | active development |
| nap-alpha | nap-alpha | tester builds (mirrors nap-dev when stable) |
| nap-release | nap-release | production release |
| naponsp-dev | naponsp-dev | sunnypilot fork (SP-specific code only here) |

**Never push sunnypilot-specific code (MADS, CP_SP) to nap-dev or nap-alpha.**

## Submodules

panda and opendbc use NotAutopilot forks. After checkout, sync URLs:
```bash
git submodule sync --recursive
git submodule update --init --recursive
```

## Key safety invariants

- Panda steering params (`tesla_preap.h` PREAP_STEERING_PARAMS) must match `VehicleModel(TESLA_MODEL_S_PREAP)`. Verify with:
  ```python
  from opendbc.car.tesla.interface import CarInterface
  from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor
  VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_PREAP"))
  print(calc_slip_factor(VM))  # must match tesla_preap.h slip_factor
  ```
- GTW radar emulation must use `.rx_all` hook (not `.rx`) — it needs to see all CAN traffic, not just rx_checks whitelist messages.
- EPAS error codes 6/7/8/9 trigger `steering_disengage` in both panda and carstate.
