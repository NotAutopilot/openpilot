openpilot in simulator
=====================

openpilot implements a [bridge](run_bridge.py) that allows it to run in the [MetaDrive simulator](https://github.com/metadriverse/metadrive).

This NAP tree packs a Tesla Model S Pre-AP (`TESLA_MODEL_S_PREAP`, VIN `5YJSA1H13EFP20460`) instead of the upstream Honda Civic. CARLA is gone from comma openpilot; do not bring it back. Log replay is a separate path (`docs-nap/off-car-replay.md`).

See `docs-nap/metadrive-sim.md` for the Pre-AP CAN contract.

## Launching openpilot
First, start openpilot.
``` bash
# Run locally
./tools/sim/launch_openpilot.sh
```

If MetaDrive is not installed (or you only need selfdrived), the bridge falls back to a CAN plant:

``` bash
./run_bridge.py --plant --no-keyboard
```

## Bridge usage
```
$ ./run_bridge.py -h
usage: run_bridge.py [-h] [--joystick] [--high_quality] [--dual_camera]
Bridge between the simulator and openpilot.

options:
  -h, --help            show this help message and exit
  --joystick
  --high_quality
  --dual_camera
```

#### Bridge Controls:
- Auto-engage is a Tesla stalk double-pull (MAIN) once selfdrive is engageable.
- Manual engage: press 2 twice within ~400ms. 1 raises set speed, 4 lowers it.
- To disengage, press 3 (stalk cancel) or "S" (user brake).

#### All inputs:

```
| key  |   functionality                         |
|------|-----------------------------------------|
|  1   | Stalk up (SET / RES / +)                |
|  2   | Stalk pull MAIN (engage, double-pull)   |
|  3   | Stalk cancel (push away)                |
|  4   | Stalk down (DECEL / -)                  |
|  r   | Reset Simulation                        |
|  i   | Toggle Ignition                         |
|  q   | Exit all                                |
| wasd | Control manually                        |
```

## MetaDrive

### Launching Metadrive
Start bridge processes located in tools/sim:
``` bash
./run_bridge.py
```
