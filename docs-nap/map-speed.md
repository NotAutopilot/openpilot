# OSM Map Speed (MAX)

Live OpenStreetMap speed limits on comma 3X, wired into NAP's existing cruise **MAX** (`CS.vCruise` / HUD "MAX"). This is not a second longitudinal controller.

## What MAX is

The on-road HUD box labeled **MAX** is the cruise set speed (`carState.vCruiseCluster`). This feature maps OSM `maxspeed` onto that value (and the long planner's `v_cruise`) while NAP already owns software cruise.

```
GNSS → mapd (offline OSM sqlite) → liveMapDataNAP
  → card.py (pre-AP pedal software cruise only)
       CS.vCruise / vCruiseCluster   ← HUD MAX
  → plannerd cap + map_track_decel (comfort a; lead can still brake more)
  → LongitudinalMpc.update(radarState, v_cruise)
       constraint = min(lead0, lead1, cruise_obstacle(v_cruise))
```

Licenses: pfeiferj/mapd and sunnypilot SLA are MIT; we did **not** vendor the Go `mapd` binary or sunnypilot UI. NAP keeps a Python OSM querier (ODbL map data) on the existing `vCruise` path. The sunnypilot-specific stack stays on `naponsp-dev` per [contributing.md](contributing.md). Map data is © OpenStreetMap contributors ([ODbL](https://www.openstreetmap.org/copyright)).

## Safety invariants

- **Pedal mode** (`openpilotLongitudinalControl`, not `pcmCruise`): Cap/Follow may change `vCruise`.
- **No-pedal / stock CC**: display only. We do not spoof stalk +/- to chase map limits.
- **Cap**: `MAX = min(driver set, OSM limit + offset)`. Never raises.
- **Follow** (preferred): MAX tracks the OSM limit. A manual stalk set — **above or below** posted `a` — holds that absolute MAX until the posted value changes to `b`, then MAX **rebases to `b`** (driving or long-paused). Do not continuously overwrite sticky toward posted every frame while posted is unchanged. GPS / match drop is posted unknown: keep held MAX, never invent a posted.
- **Sharp curve:** snapshot HUD MAX / sticky / held at curve entry (lat accel or steer). Temporary corner slowing may lower published MAX (`limit_accel_in_turns` plus a comfort lat-accel cap). After lat accel / steer are straight-ish, restore that pre-curve set. Do not let the bend permanently rebase sticky or the map target (OSM flicker on a ramp must not bounce a pre-curve 60 down to the live eco target). A posted change that is still there after exit is a real new zone and may rebase.
- **Engage / SET:** **Double SET** (initial engage from fully disengaged, or second SET in the window while already in session) forgets sticky: maps on + posted known → HUD MAX and `pedal_speed_kph` = current posted (+ offset); maps off / posted unknown → current traveled speed. **One SET** after a brake long pause resumes longitudinal only at the held MAX (already rebased if posted changed). Pedal write-back is **take-speed-now**, **resume-held**, a **real stalk step**, or **Follow/system raise** when the posted limit increased. A sticky hold must **not** write every frame (that ate stalk +/- after CI.update). Stalk +/- is a 1 or 5 mph `pedal_speed` step; button events are extra. An ego jump is not a stalk.
- **Display / Off**: no control change.
- **Lookahead (Cap/Follow):** a **lower** OSM maxspeed ahead eases MAX down so you reach about the new limit as you enter that way. A **higher** limit ahead does **not** raise MAX early — Follow raises only once GPS is on the faster segment.
- **Roundabout:** detect OSM `junction=roundabout` / a compact closed circulating way out to **~140 m** (`RB_DECEL_ONSET_M`; was 200 m until Oct 1; must still have fired by 80–100 m) — not a big steer, and not a sharp town corner or signalized cross. Kinematic decel toward **15–20 mph** (OSM ring maxspeed when tagged, often 20) so 40–45 mph plans **a ≤ −1.0** until near ring speed. `aTarget` stays **≤ 0** while approaching / on the ring (no +a after a lead clears). Long enable inside the funnel applies full ease on that frame. Outer path bias **~3.2 m** (right in RHT / US) on the last 50 m of approach and while circulating — enough to counter a ~3 m inside cut. A short RB way is **not** dropped as a min-zone stub. Yield-before-merge, continue-circulate desire, and a UI chip are later tips.
- **A falling MAX must decelerate** (Cap/Follow, pedal mode, no overriding lead). The stock MPC cruise column is a virtual lead ~`get_safe_obstacle_distance(v_ego)` ahead with `V_EGO_COST=0`, so a 70→45 mph drop would not bind. Planner `min()`s MPC with `map_track_decel` at the **Accel 5** comfort `a` (0.80 m/s² at Normal). Accel 1–10 does not change this brake. Tesla `get_preap_accel_limits` still clips to −1.5 m/s².
- **A rising MAX must accelerate** (Follow, no overriding lead). MPC also will not climb to a higher MAX. Planner commands `map_track_accel` at Accel 1–10 when ego is below MAX and MPC is not braking — **except** when `radarState.leadOne` is valid: a lead owns follow, so map climb must not replace ~0 / slight+ MPC `a` toward MAX (that punched through a slower lead, then overshot and would not rematch). `map_track_decel` when above MAX still mins in with a lead present. Stalk up / Follow posted raise also writes that higher MAX onto `pedal_speed`. Hypermile **Hill Climb** (IMU pitch only; maps-elevation lookahead NOT included) may add grade to that climb when clearly under MAX **and there is no lead** so Accel 1 does not sag; it never raises MAX and does not invent `+g·sin` in the deadband.
- **Lead outranks map.** Map only sets the cruise ceiling plus comfort decel/accel. `mpc.update` is always `mpc.update(radarState, v_cruise)` after the map cap.
- Panda TX whitelist, pedal gating, and engagement FSM are unchanged.
- Default **Off** until US maps are downloaded and you pick a mode.

## US map data

The US speed-limits sqlite is **not in git** (too large; ODbL still requires attribution). After flash, download the prebuilt asset over Wi-Fi:

| | |
|---|---|
| Release tag | `osm-us-speed-limits-v4` on `jmbrunick/openpilot` |
| Asset | `speed_limits_us.sqlite.zst` |
| SHA-256 | of the **zst** (`ASSET_SHA256`), verified **before** decompress. Dest sqlite is not hashed unless `SQLITE_SHA256` is set. `--sha256 ''` skips. |
| Install path | `/data/media/0/osm/speed_limits.sqlite` |
| Staging | `/data/media/0/osm/.download/` on the dest filesystem (not `/tmp`) |
| Size | **~204 MiB zst → ~570 MiB sqlite**. Fetch needs **854 MiB** free on `/data`. |

On the comma 3X: **Settings → NAP → Map Speed Limit → Download US Maps** (offroad), or `python -m scripts.nap.fetch_osm_maps`. That is the first-install of the published US pack. Frequent local updates: **Refresh maps** (or `python -m scripts.nap.refresh_osm_maps`) — live OSM within **100 miles** (~160.9 km), merged into the installed sqlite. `mapd` reloads the sqlite every ~15s onroad — no reboot. To unlock those buttons without parking, Settings → triple-tap **NAP** → **Force Offroad** (see [force-offroad.md](force-offroad.md)).

Download US Maps retries timeouts and dropped connections up to 8 times with backoff ("Retrying… (n)" on screen) and resumes with HTTP Range from `/data/media/0/osm/.download/*.partial`. If it still fails, tap Download again: it resumes from the kept partial for the same asset. Only a SHA-256 mismatch deletes the partial.

If a previous download died with ENOSPC, remove leftovers then retry:

```bash
rm -f /data/media/0/osm/*.partial /data/media/0/osm/.download/*
df -h /data
```

A previous good sqlite is left in place. Optional override path: param `NAPMapSpeedDbPath`.

Publish a new US pack from a PC (Geofabrik PBF ~11 GB):

```bash
osmium tags-filter us-latest.osm.pbf w/highway w/maxspeed -o us-maxspeed.osm.pbf
python scripts/nap/build_osm_speed_limits.py --pbf us-maxspeed.osm.pbf \
  --out speed_limits_us.sqlite --zst

# Overlay MN statutory fills (~100 miles around Benson) onto that US pack:
python scripts/nap/build_osm_speed_limits.py --pbf minnesota-latest.osm.pbf \
  --fill-mn-statutory --benson --merge-into speed_limits_us.sqlite \
  --out speed_limits_us.sqlite --zst
```

Fills are NAP estimates (`source=MN_169.14`, Minn. Stat. 169.14). Do **not** upload guessed `maxspeed` tags to osm.org. Meta on the sqlite records tagged vs filled counts.

Put the **zst** SHA-256 in `selfdrive/mapd/maps_manifest.py` **and** bump `selfdrive/mapd/maps-index.json` (see [Publishing a US pack](#publishing-a-us-pack)). For a PC-only smaller region file: `python scripts/nap/download_osm_speed_limits.py --lat … --lon … --radius-km 30`.

## Refresh maps (100 miles)

**Refresh maps** is one button: a local Overpass update, not a published-pack download. Radius is **100 miles** (~160.9 km) around the vehicle — the bounding box that contains that circle.

On the 3X (offroad, Wi-Fi):

1. **Location.** Wait up to **10s** for a GNSS fix (`gpsLocationExternal`, then `gpsLocation`). Refresh accepts the last received plausible lat/lon (not 0,0), including samples older than 2.5s and accuracy up to 200 m (yard/tree cover). mapd's onroad MAX match still uses 2.5s / 50 m. Otherwise last stored GPS (`LastGPSPosition` param or `/data/params/d/LastGPSPosition`). A successful fix is persisted so the next offroad tap works. If still nothing: start openpilot onroad until the GPS icon/fix is up for about a minute, then retry. Maps will **not** guess a city.
2. **Query OSM.** Reuses Overpass / `scripts.nap.download_osm_speed_limits` / mapd builders for highway ways in that box. Does **not** download the full US Geofabrik PBF on the 3X. Tagged numeric `maxspeed` is authoritative. When the box intersects Minnesota, the query includes fillable highways **without** maxspeed and applies Minn. Stat. 169.14 estimates locally (never uploaded to osm.org) so Refresh maps does not wipe unmarked pack fills.
3. **Merge.** Copy the installed `/data/media/0/osm/speed_limits.sqlite` onto `/data` (not `/tmp`), delete/replace `way_id`s whose bbox intersects the 100-mile box, insert the Overpass ways (tagged + MN fills), keep the rest of the US pack. If no sqlite is installed yet, Download US Maps runs first, then the overlay — never a 100-mile-only dest file.
4. **Atomic install.** `os.replace` onto dest. Overpass timeout / HTTP / merge failure prints a real error and leaves the previous good sqlite. Retry-safe. mapd reloads ~15s onroad — no reboot. Progress (querying OSM, merging, installing) shows on the existing script-runner UI.

Overpass can be slow in a dense metro. Wait or retry; the old maps stay put.

## Roundabouts in the pack (schema v4)

Pack schema **v4** (`meta.pack_schema = 4`) adds `rb_ways` / `rb_nodes`: every OSM way tagged `junction=roundabout` (with or without `maxspeed` / `lanes`) plus the highway ways that touch a ring (approach/exit roads), stored in travel direction (two-way roads both ways). Rings without `maxspeed` get no speed-limit row, so the MAX/limit behavior is unchanged; they only feed roundabout detection and ring geometry. `build_osm_speed_limits.py` includes them by default; `--no-roundabouts` builds the old v3 layout. v3 packs still load (no rb tables = no ring geometry).

- **Getting rings on the device:** Settings → NAP → Map Speed Limit → **Refresh maps**. The 100-mile Overpass query now also fetches `junction=roundabout` ways and their approach roads, and the merge adds `rb_ways` even onto an installed v3 pack. The v4 US first-install pack (`osm-us-speed-limits-v4`) includes rings for all 50 states + DC, so Download US Maps adds them.
- **Advisory and mini-roundabouts:** a ring's `maxspeed:advisory` is stored when it is lower than the posted `maxspeed` (or when there is no posted limit). `highway=mini_roundabout` nodes become a synthetic 7 m ring (way_id = −node id) in PBF builds, so the funnel sees them; roads through the node are its approaches. Refresh maps (Overpass) does not add mini-roundabouts yet.
- **Ring slow-down target:** when the ring geometry is known, `roundaboutSpeedLimit` uses a comfort speed from the lane radius (lateral ≤ ~2.5 m/s², clamped 15–20 mph; e.g. Soco Rd/Dellwood ≈ 15.8 mph) instead of the default 20 mph.
- **Roundabout Steering Assist** (`NAPRoundaboutAssist`, Settings → NAP → Driving Mannerisms, **default Off**, Tesla Pre-AP only): on a mapped ring, blends the model curvature toward the lane circle (hold right entry until ~8–10 m before the ring, correction slewed ≤ 0.07/s, weight from GNSS/map-match confidence). The camera path keeps authority on lane edges and driver torque overrides as usual. It depends on GNSS accuracy (validated only in replay), hence default Off. mapd publishes the matched ring to `NAPRoundaboutRing`.

## Publishing a US pack

Download US Maps still uses the GitHub Release + `maps-index.json`. Bump those when you republish a full US sqlite from a PC:

1. Build a new US sqlite and zst on a PC (same `build_osm_speed_limits.py` flow as above).
2. Attach `speed_limits_us.sqlite.zst` to a **new** GitHub Release on `jmbrunick/openpilot` (e.g. `osm-us-speed-limits-v3`). Do not replace the in-git JSON with the 204MB zst.
3. SHA-256 the **zst** (`sha256sum speed_limits_us.sqlite.zst`).
4. Bump `selfdrive/mapd/maps-index.json`:
   - `revision` — integer or dotted semver, must be **greater** than the previous value (current first-install is `"4"`)
   - `asset_url` — Release download URL for the new zst
   - `asset_name` — usually `speed_limits_us.sqlite.zst`
   - `sha256` — hex digest of the zst
   - `bytes` — zst size (optional)
   - `notes` — optional
5. Keep first-install constants in `selfdrive/mapd/maps_manifest.py` in sync when you intend a new clone/flash to get this pack (`RELEASE_TAG`, `ASSET_SHA256`, `RELEASE_REVISION`, sizes).
6. Merge the JSON (and manifest) to `nap-dev`. Download US Maps fetches:

   `https://raw.githubusercontent.com/jmbrunick/openpilot/nap-dev/selfdrive/mapd/maps-index.json`

## Settings → NAP → Map Speed Limit

All map-speed controls live in this submenu (main NAP stays uncluttered). TICI and mici:

- **Map Speed (MAX)** (`NAPMapSpeedMode`): Off / Display / Cap / Follow
- **Map Speed Offset** (`NAPMapSpeedOffsetMph`): -5 / 0 / +5 mph
- **Lookahead** (`NAPMapSpeedLookahead`): Off / Late / Normal (default) / Early
- **Refresh maps**: live OSM within 100 miles, merged into the installed US sqlite
- **Download US Maps**: first install of the current published pack
- **OSM Map Data** / **Map revision**: installed pack status (Refresh does not bump revision)
- Cap/Follow require the pedal interceptor
- **Acceleration** 1–10 is on **Settings → NAP → Driving Mannerisms** (`NAPMapSpeedAccel`) — not this page

## Anticipatory decreases and Accel

`mapd` probes 40–600 m along GPS heading **and** walks the matched OSM way to its end (then the next way) so a short intermediate maxspeed is published as `nextSpeedLimit`. Policy uses **decreases only**.

**Minimum zone length:** a lower limit (current match or `nextSpeedLimit`) that is gone within **`MIN_ZONE_LENGTH_M` (76 m / ~250 ft)** along the travel heading is ignored. OSM/GPS at a crossroads can briefly snap onto a side street or a tiny fill stub and look like a short speed zone on the highway. Real posted drops that continue past that threshold still apply (US 12 Benson 60→50 is ~760 m of tagged 50; DeGraff 30 is hundreds of meters). This is a general matcher rule, not a Benson/DeGraff hardcode. Bearing / class route continuity from the off-route filter still applies. Lookahead Off still disables anticipatory easing.

| Lookahead | Comfort decel | Extra margin | Start no farther than |
|---|---|---|---|
| Off | — | — | change only after GPS is on the slower way |
| Late | 1.20 m/s² | 110 m | 360 m |
| Normal (default) | 0.80 m/s² | 110 m | 600 m |
| Early | 0.55 m/s² | 230 m | 600 m |

When the upcoming drop is inside that window, MAX interpolates from the current limit at `d = kinematic + margin` to the new limit at the sign. Brake stays Accel-5 **0.80 m/s²** at Normal (not raised). The +110 m is the measured 50→30 shortfall (42 mph at the sign vs 30); every decrease starts at `kin + 110 m`, not a 50→30-only window. A 10 mph drop (60→50) is not filtered. `mapd` walks the matched OSM way to its end (then the next way) so a short intermediate limit is not skipped by 40 m heading probes. A stub shorter than 76 m (~250 ft) along heading is not a posted drop.

**HUD current speed** (top-middle on the 3X) is wheel/ESP `vEgo` only. `vEgoCluster` is Tesla `DI_digitalSpeed`, which pre-AP also uses as `cruiseState.speed`. Map-speed writes MAX into `vCruise` / `pedal_speed` / `cruiseState.speed` (90 kph = **56 mph**). LIMIT/MAX may show the map limit; the live number must not.

**Acceleration** (Settings → NAP → Driving Mannerisms) scales **Follow climb** (MAX rising, no overriding lead) **and** lead gap-close +a. Same Accel 1–10 envelope, including last-mph taper (Accel 1 baby-steps ~5 mph out; Accel 7–10 stay brisk). Brake uses Accel 5 (`map_brake_a_ms2` / `map_track_decel`). Adaptive Accel no longer uses the full cruise profile to punch a large follow gap.

| Accel | Factor | MAX climb / lead-close `a` (Normal) | Used for |
|---|---|---|---|
| 1 | 0.45 | **0.36 m/s²** | climb + catch-up (gentlest; last ~5 mph is a baby-step) |
| **5** | 1.00 | **0.80 m/s²** | climb, catch-up, *and* all map braking |
| 10 | 2.00 | **1.60 m/s²** | climb + catch-up (quickest, clamped); Accel 7–10 stay brisk to the target |

MAX-rise and lead-close `a = clamp(0.30, 1.60, a_lookahead × factor)` inside ~200 m (Bosch). Changing Accel 1 vs 10 must not change brake feel. Rapid / FCW / a real stop still own MPC danger; non-rapid mild closes are floored at overlay MILD. A higher limit ahead may still be published as `nextSpeedLimit`; Cap/Follow ignore it until `speedLimit` itself is the higher value.

## How to test

**Off-device:**

```bash
pytest selfdrive/mapd/tests/test_map_speed_policy.py \
  selfdrive/mapd/tests/test_osm_db.py \
  selfdrive/mapd/tests/test_fetch_maps.py \
  selfdrive/mapd/tests/test_local_refresh.py \
  selfdrive/car/tesla/tests/test_preap_sticky_max.py \
  selfdrive/car/tesla/tests/test_preap_blinker_lat_pause.py \
  selfdrive/selfdrived/tests/test_preap_regen.py \
  selfdrive/controls/lib/tests/test_driver_lateral_handoff.py \
  selfdrive/controls/lib/tests/test_curve_max_hold.py -q
```

Policy tests include lead precedence, fetch of a tiny sqlite over HTTP, and Refresh maps location/merge (no network).

**Offroad Refresh maps (3X, Wi-Fi):**

1. After a drive with GPS (so `LastGPSPosition` is stored), park, Settings → NAP → Map Speed Limit → **Refresh maps** → Start.
2. Script runner should show location source (GNSS or last stored GPS), querying OSM, merging, installing. Must not name a guessed city.
3. `ls -l --time-style=full-iso /data/media/0/osm/speed_limits.sqlite` — mtime updates; size stays US-pack scale (hundreds of MB), not a tiny 100-mile-only file.
4. Unplug GPS / no LastGPSPosition (fresh flash, never driven): Refresh maps must error and leave any existing sqlite.
5. Airplane mode / Overpass down: error text, previous sqlite kept. Retry later.
6. No sqlite yet: Refresh maps downloads the US pack first, then overlays. Download US Maps still does first-install only.

**On comma 3X, parked, GPS lock:** Mode = Display. Confirm the LIMIT sign matches a known posted limit (OSM, not NAR). After Refresh maps, a local OSM edit (new maxspeed) should show within ~15s onroad — no reboot.

**On-road, pedal mode:**

1. Cap, **no lead**: set MAX above the posted limit; the car must decelerate toward that MAX (Accel 5 ≈ 0.80 m/s²). Raising the stalk cannot exceed the cap.
2. Cap/Follow, **slower lead**: the car still slows for radar `leadOne`.
3. Follow, Lookahead Normal, **no lead**. On a known drop (50→30), Accel **1 and 10 must feel the same brake**, and ego should be near 30 at the sign (not still ~42). On a known rise (35→45, after GPS is on the faster way), Accel 1 climbs lazily and Accel 10 quicker — the pedal target and the car must actually speed up. Lookahead = Off: MAX and decel start only after GPS matches the slower way. On a continuous highway through a grid of cross streets (US 12 SE Benson→DeGraff is the example, not a special case), MAX must **not** ease for a side street or a ~250 ft stub; it may ease for a real on-route 60→50→30 that lasts longer than 76 m.
4. Drive toward a **higher** limit: MAX must **not** rise until you are on the faster segment.
5. Follow: set 55 in a 65 — MAX stays 55 until the posted limit changes; set 45 in a 65 — same. Posted 65→45 (or 65→70), driving or long-paused, rebases MAX to the new posted. Brake pause then one SET resumes the held MAX (55 in a 65, or the rebased posted). Double SET with maps on takes current posted; maps off / unknown posted takes current traveled speed. GPS glitch must not wipe sticky or invent a posted. Double-pull engage with a valid limit: MAX **and** the car start at the posted limit immediately.
6. Top-middle live speed must match wheel/ESP (about 45 if that is actual), not MAX (56) and not LIMIT.
7. Cancel is a full disengage (held MAX forgotten). Brake is a long pause: one SET resumes held MAX; double SET takes posted or current speed as above. A latched driver turn does not pause long.
8. **Sharp curve, Hypermile On / Step Down Off:** if MAX is 60 before the bend (sticky or that displayed set), the car may slow and HUD MAX may move through the corner. After exit MAX must return to **60**, not stay on the live eco posted target. A real posted change that is still there after the bend may rebase.

**No-pedal:** LIMIT sign only; stock CC set speed is unchanged. Sticky MAX / one-SET resume is pedal software cruise only.

## Build notes

- Python-only `selfdrive/mapd`. Device needs `zstandard` (already an openpilot dep) to decompress the Release asset.
- Cereal: `liveMapDataNAP` on custom reserved 0. Rebuild so `params_keys.h` picks up the new params.
- `mapd` is an onroad managed process in `system/manager/process_config.py`.
