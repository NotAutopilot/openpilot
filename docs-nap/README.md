# NAP Developer Docs

Documentation for contributors working on NotAutopilot. These are NAP-specific — upstream openpilot docs live in `docs/` (comma.ai).

## Start here

- **[contributing.md](contributing.md)** — branch structure, pre-push checklist, submodules, how to submit bugs
- **[architecture.md](architecture.md)** — Pre-AP Model S design overview: standalone safety model, radar emulation, pedal interceptor
- **[safety-model.md](safety-model.md)** — panda safety invariants for the Pre-AP target
- **[engagement.md](engagement.md)** — stalk FSM, pedal-vs-no-pedal engagement paths, brake behavior
- **[stoplight-green.md](stoplight-green.md)** — Optional ding when a red light turns green while stopped in Drive; HUD RED LIGHT / GREEN - GO chip. Road camera only, `stoplightd`, default Off. Never brakes or steers.

## Layout

```
openpilot-nap/
├── docs/            # upstream openpilot docs (comma.ai)
├── docs-nap/        # ← you are here
├── opendbc_repo/    # submodule: NotAutopilot/opendbc fork
├── panda/           # submodule: commaai/panda (tracked upstream)
├── selfdrive/
└── ...
```

The Pre-AP car port lives in `opendbc_repo/opendbc/car/tesla/preap/` and the standalone panda safety mode in `opendbc_repo/opendbc/safety/modes/tesla_preap.h`.
