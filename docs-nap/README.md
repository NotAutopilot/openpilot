# NAP Developer Docs

Documentation for contributors working on NotAutopilot. These are NAP-specific — upstream openpilot docs live in `docs/` (comma.ai).

## Start here

- **[contributing.md](contributing.md)** — branch structure, pre-push checklist, submodules, how to submit bugs
- **[architecture.md](architecture.md)** — Pre-AP Model S design overview: standalone safety model, radar emulation, pedal interceptor
- **[safety-model.md](safety-model.md)** — panda safety invariants for the Pre-AP target
- **[engagement.md](engagement.md)** — stalk FSM, pedal-vs-no-pedal engagement paths, brake behavior

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
