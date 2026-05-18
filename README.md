<div align="center">

<h1>NotAutopilot</h1>

<p>
  <b>Self driving for pre-Autopilot Tesla Model S.</b>
  <br>
  Lane keeping, adaptive cruise, and driver monitoring for the 25,000+ pre-AP vehicles Tesla left behind.
</p>

<h3>
  <a href="https://notautopilot.com">Website</a>
  <span> · </span>
  <a href="https://notautopilot.com/wiki">Wiki</a>
  <span> · </span>
  <a href="https://discord.gg/notautopilot">Discord</a>
  <span> · </span>
  <a href="https://notautopilot.com/how-it-works">How It Works</a>
</h3>

Install on your comma 3X: `https://installer.comma.ai/NotAutopilot/openpilot/nap-release`

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

</div>

---

## What is NotAutopilot?

NotAutopilot (NAP) is a fork of [comma.ai's openpilot](https://github.com/commaai/openpilot) that brings driver assistance to 2012–2014 pre-Autopilot Tesla Model S vehicles. These cars shipped without forward-facing cameras, radar, or any driver assistance — NAP adds all of it.

Built on the foundation of the [Tinkla project](https://tinkla.us/t/index) (which first proved openpilot could work on pre-AP Teslas) and [xnor-tech's openpilot](https://github.com/xnor-tech/openpilot) (which added working AP1 Model S support), NAP extends support to pre-Autopilot vehicles with a standalone safety model, Bosch radar integration, and Comma Pedal longitudinal control.

### Features

- **Lateral control** — Steering via EPAS with full panda safety validation
- **Longitudinal control** — Comma Pedal interceptor with zero-torque learning, smooth engagement, and three driving profiles (Aggressive, Standard, Chill)
- **Radar** — Bosch radar support with GTW emulation for lead car detection and adaptive following
- **Driver monitoring** — Stock openpilot driver monitoring camera
- **On-device settings** — Full NAP settings panel for pedal calibration, radar configuration, EPAS firmware management, and tuning

### Hardware Required

1. **2012–2014 Tesla Model S** (pre-Autopilot, no AP ECU)
2. **[comma 3X](https://comma.ai/shop)** (or comma 3)
3. **[Tesla Model S HW1 harness](https://comma.ai/shop/car-harness)** 
4. **[Comma Pedal](https://github.com/commaai/openpilot/wiki/Comma-Pedal)** — for longitudinal (throttle/regen) control
5. **Bosch radar** *(optional)* — for adaptive cruise with lead car following

## Installation

On your comma device, enter this URL during setup or in Software settings under Custom Fork:

```
https://installer.comma.ai/NotAutopilot/openpilot/nap-release
```

## Branches

| Branch | Install URL | Description |
|--------|------------|-------------|
| `nap-release` | `NotAutopilot/openpilot/nap-release` | **Recommended.** Stable release for daily driving. |
| `nap-staging` | — | Pre-release testing. Coming soon. |
| `nap-alpha` | `NotAutopilot/openpilot/nap-alpha` | Alpha builds for testers. May have rough edges. |
| `nap-dev` | — | Active development. Expect things to break. |

## Lineage

```
comma.ai/openpilot
  └── sunnypilot/sunnypilot
        └── xnor-tech/openpilot (AP1 Model S support)
              └── NotAutopilot/openpilot (Pre-AP Model S support)

boggyver/openpilot (Tinkla — the original Pre-AP Tesla fork)
  └── inspiration, radar emulation, pedal architecture → NotAutopilot
```

## Acknowledgments

NotAutopilot wouldn't exist without:

- **[Boggyver](https://github.com/boggyver) and the [Tinkla Project](https://tinkla.us/t/index)** — First proved openpilot could work on pre-AP Teslas. The foundation everything is built on.
- **[Lukas Loetkolben](https://github.com/lukasloetkolben)** — Clean isolated safety model approach that inspired the SAFETY_TESLA_PREAP refactor.
- **[xnor-tech](https://github.com/xnor-tech)** — Working AP1/HW1-4 Tesla support in openpilot.
- **[comma.ai](https://comma.ai)** — openpilot itself. The hard parts were already solved.
- **Johnmr1, SeriouslySerious, Pod042, 1FrostlySlime** — Early testers who drove broken builds so others wouldn't have to.

## Contributing

Pull requests are welcome against the `nap-dev` branch. See [`docs-nap/`](docs-nap/) for branch structure, build instructions, pre-push checklist, safety invariants, and the engagement flow.

## User Data

By default, openpilot uploads driving data to comma servers. You can access your data through [comma connect](https://connect.comma.ai/). Data collection can be disabled in settings.

## Licensing

NotAutopilot is released under the [MIT License](LICENSE). This repository includes original work as well as code derived from [openpilot by comma.ai](https://github.com/commaai/openpilot) and the [Tinkla project by boggyver](https://github.com/boggyver/openpilot), both released under the MIT license.

> **THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
> YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
> NO WARRANTY EXPRESSED OR IMPLIED.**

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys' fees and costs) which arise out of, relate to or result from any use of this software by user.

Any user of this software shall also indemnify and hold harmless the NotAutopilot project, its contributors, maintainers, and affiliates from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys' fees and costs) which arise out of, relate to or result from any use of this software by user.
