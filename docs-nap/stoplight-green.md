# Stoplight green chime (`stoplightd`)

While the car sits stopped in Drive, `stoplightd` looks at the top of the
road camera frame about 4 times a second. When it sees a lit red signal it
puts **RED LIGHT** in the HUD (under MAX on 3X, top-right on comma four).
When that red goes out and a green comes on in the same signal head, the
HUD shows **GREEN - GO** for 3 s and soundd plays one ding
(`light_green.wav`, E6 to A6 bell, never quieter than 0.7 volume). One
ding per stop.

It never brakes, steers, or changes engagement. It works engaged or not.

## Toggle

`NAPStoplightChime` (default **Off**). Settings → NAP → "Green light
chime" (3X and comma four). Off keeps the process from starting, so there
are no camera reads at all. A factory reset restores Off.

## Pipeline

| piece | file | what |
| --- | --- | --- |
| gate | `selfdrive/stoplightd/gate.py` | stopped in Drive: `livePose.velocityDevice` < 0.25 m/s for 1 s, no `wrongGear` / `reverseGear` in `onroadEvents`. Closes > 0.8 m/s or when either input goes stale. |
| detector | `selfdrive/stoplightd/detect.py` | numpy only. NV12 half-res grid, top 56 % of the frame. Red: V-128 ≥ 40 with red hue (not amber); green: 128-V ≥ 40 and U-128 ≥ -8. 4x4 cells, 8-connected blobs, lamp-sized and compact. ~5 ms per frame on x86. |
| lamp sanity | `selfdrive/stoplightd/tracker.py` `lamp_filter` | every frame, before tracking: lens size 10-110 px; red aspect ≤ 1.6 (green ≤ 2.0: night halo smear); bright lens (peak luma ≥ 100 red / 110 green) and a red needs a blown core or chroma ≥ 56 (dull red paint has neither); 2.5-34 lamp sizes above the horizon (row 0.674 of the frame on Scallywag), so lead-car lights at or below the camera and tower / roof lights far too high for their size drop out; not cut by the top edge; two like-sized reds side by side near the horizon (a car's lights) are both dropped. |
| tracker | `selfdrive/stoplightd/tracker.py` | red is confirmed after 4 hits over 0.6 s **at one spot** (mean wobble ≤ max(4 px, 0.2 size): a lens does not move while stopped; billboard content does). Each confirmed red is one **head**. A head may ding when its red goes out and a **new** green comes on in the **same head** (same column, 0.45-3.2 lens sizes below, ≤ max(8 px, 0.3 size) sideways; or beside it for a horizontal head) within 1 s, seen twice over 0.15 s and steady. |
| head arbitration | `tracker.py` `_may_ding` | Centrality = distance from the frame's middle (0 ahead, 1 edge). The driver's head is the most central. A head does **not** ding if: it already dinged for this change; it is a side head (> 0.35 off center) and a more central main head is still red (turn arrow / cross street going green while his light stays red); a clearly more central head (by 0.12) already dinged this stop; another head dinged < 4 s ago (heads of one approach change together). A side head that dinged first never blocks a more central head that changes later. The same head can ding again only after a fresh red of ≥ 5 s (a new cycle). At most 4 dings per stop. HUD: RED LIGHT / GREEN follow the most central head. |
| daemon | `selfdrive/stoplightd/stoplightd.py` | connects to the road VisionIPC stream only once the gate opens. Little cores 0-3, nice 19. Publishes `stoplightNAP` at 4 Hz while gated, 1 Hz otherwise. Started onroad only when the toggle is On; restarted if it crashes. |
| sound | `selfdrive/ui/soundd.py`, `selfdrive/ui/stoplight_chime.py` | soundd subscribes to `stoplightNAP` and plays the ding when `chime` is set and no selfdrived alert sound is playing. No new `EventName`, so no alert ordinal drift between nap-dev and nap-alpha. |
| HUD | `selfdrive/ui/onroad/stoplight_chip.py` | chip state; drawn by both HUD renderers. |

`stoplightNAP` reuses `CustomReserved6` (Event @113). It does not read
`carState`. Speed comes from `livePose` and gear from `onroadEvents`, so
the daemon does not add a `carState` subscriber.

## Calibration data

From Scallywag's own drives, Oct 23 - Nov 11 2026 (routes `000001c2` to
`000001fc`, 60 drives). 63 stops in Drive of 5 s or more were found from
qlogs, checked by eye on qcamera, and the full-res road camera was pulled
for each.

* 24 real red-to-green stops (day, overcast, rain, dusk, night, arrows):
  **24 / 24 ding**, 0.2 - 3.1 s before the car moved, one ding each. Same
  result at half the frame rate.
* Stop 16 (route `000001ca`): the right-hand ball went green 85 s before he
  moved; he was waiting on the more central left arrow, which went green
  1.3 s before he moved. Both ding now (the first version only dinged the
  ball).
* 22 stops without a change (lots, garages, stop signs, a digital
  billboard flipping red to green content) and one right turn on red:
  **0 dings**.
* RED LIGHT chip on stops with no signal ahead: store sign (stop 61) and
  tower lights (stop 22) gone; a gas-price sign (stop 18) down from ~15 s to
  ~4 s. A lead SUV's high brake light dead ahead (stop 17) still shows it
  (~16 s); it can never ding (no green appears below it).

Fixtures: `selfdrive/stoplightd/tests/samples/` (22 crops, see its README)
and `selfdrive/stoplightd/tests/sequences.json.gz.b64` (per-frame lamps for all 52
labeled stops).

## Limits

* Road camera only (narrow). First in line under a tall overhead mast the
  head can be above the frame; then there is no ding.
* "Yours" is a guess from position: the most central head. If your light
  is far off to the side (> 0.35 of the half width) while a more central
  head stays red, a change in your light stays quiet until the central one
  changes too. On these drives that never happened; the reverse (his light
  a little right of a central red turn head, stop 10) is handled.
* The horizon row is fixed at 0.674 of the frame (Scallywag's 3X mount).
  A very different mount pitch moves it; the height checks then need it
  updated.
* A lead vehicle's high brake light straight ahead can show RED LIGHT. It
  cannot ding.
* A red arrow that turns into a green arrow counts like a round lamp.
