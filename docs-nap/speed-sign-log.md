# Speed Sign Logger (log-only)

On-drive MUTCD numeric speed-sign logger for comma 3X. **Default off.** Log-only: it does not write sqlite, does not change cruise / HUD **MAX** (`vCruise`), and does not query osm.org.

Stock `modelV2` has no `speedSign` head. This is a separate process (`speedsignd`) that reads the ROAD camera + GNSS and runs a compact **YOLOv8 ONNX** (US traffic-sign classes, including 55/60). The old numpy template matcher is tests/dev only — it does not see real roadside signs.

**Driving model first.** Route 0000017a showed the failure was the GPU, not the CPU budget. modeld sat at 22–26 ms until speedsignd's tinygrad realize dispatched onto the same Adreno (QCOM) modeld uses, and modeld jumped to 300–710 ms. Inference is now CPU only: `onnxruntime` `CPUExecutionProvider` with `intra_op_num_threads=1` when that package is installed, otherwise tinygrad `DEV=CPU` (CLANG/LLVM renderer, `CPU_COUNT=1`, GPU and QCOM forced off). A startup assertion refuses any other device. The first realize compiles (~108 s on the road test) and is refused while engaged (`reason=compile-wait`). 4 Hz YOLO can still show “driving model is lagging”, ~35% `modeld` frame drops, **TAKE CONTROL IMMEDIATELY**, or “Communication Issue Between Processes.”

**Logger On keeps running YOLO while openpilot is engaged** (`selfdriveState.active`, or state in enabled / softDisabling / overriding). The process and SIGN / NO WT HUD still start. The plate shows the held mph while engaged. It does **not** show **WAIT** for that — WAIT was the old pause, and it is why a drive that was engaged 95% of the time never read a sign.

Report `swaglog | grep "speedsignd timing"` (`read_interval_ms`, `infer_ms`, `cpu_share`, `threads`, `backend`, `engaged`, `backoff`, `reason`). `backend=` must be `onnxruntime-cpu`, `tinygrad-clang`, or `tinygrad-cpu`. Do not treat a blank plate as “drive parked” or “weights are bad.”

**Engaged and manual driving both collect** — including moving, no assist. This is **not** gated on park or Force Offroad. Logger On, and log speed-limit signs + GNSS to JSONL. OSM upload is future work.

Unknown cereal is **not** a pause. After a ~2 s startup, unread `selfdriveState` allows the same throttled detect and logs a warning. SubMaster polls at **20 Hz** so 100 Hz `selfdriveState` alive/valid cannot flap the gate; YOLO stays ≤ **1 Hz**, and a long infer idles at least 3× that infer before the next one (about 25% of one core). Engaging does **not** abandon an in-flight ONNX.

Detect is **1 Hz** or slower, `SCHED_IDLE` (else nice 19), little **core 2** only, one thread — engaged and manual. Core 0 is the UI, 1 is sensord, 3 is pandad/encoderd, 4 is controlsd, 7 is modeld. **If TAKE CONTROL or lag comes back, turn Speed Sign Logger Off.** Do not raise `modeld` priority. A back-off line in swaglog means speedsignd saw `modelV2` frame drops, a skipped frame id, `modelExecutionTime` above 50 ms, its own CPU share over 25%, or a model/controls alert, and rested. A frame skip or modeld over 50 ms pauses at least 10 s (`reason=model-skip` or `reason=model-exec`).

## Enable

1. **Install weights** (once, Wi-Fi) — Settings → NAP → **Install weights** (offroad / Force Offroad), or SSH below. Without `/data/media/0/nap/speed_sign.onnx` the onroad SIGN plate shows **NO WT** and will not light mph on real signs.
2. **Settings → NAP → Speed Sign Logger** → On  
   (comma 4 / mici: **Settings → NAP → speed sign logger**)
3. Or: `Params().put_bool("NAPSpeedSignLog", True)` / `echo -n 1 > /data/params/d/NAPSpeedSignLog`
4. Go onroad. Manager starts `speedsignd` only when the param is true **and** the car is onroad.
5. **Drive, including engaged** (moving is OK). SIGN shows the mph while openpilot is **actively controlling**. It does not show **WAIT** for engage. If the plate stays blank on a clear sign, grab swaglog.

Off (default): the process does not run. Logger On never changes that default.

Reset to Defaults turns the logger back off. Weights on `/data` stay.

Optional detect rate (default 1 Hz, clamped 0.2–4): `NAP_SPEED_SIGN_HZ=0.5` in the process environment. Do not raise this on a 3X. While engaged, a short infer still waits out that period. A long infer waits `4× infer` (the infer plus 3× idle). It is not 0 Hz.

Manual detect is **cheap-path** so Logger On is less likely to starve `modeld`:

- one right-side window of the 1928×1208 ROAD frame, **x=1300, y=520, 628×320** (through x=1928, y=840), letterboxed into the 320 ONNX — never a full-frame RGB convert, and not the old 1208² letterbox (that left an 80 ft sign at about 20 px). Route 0000017a signs sat at about y=557–805, x=1110–1914. Scale is 320/628 ≈ 0.51. The v2 detector was trained on 640 crops resized to 320 (scale 0.5), so this window stays. A sign ≥ 60 px tall stays ≥ 30 px in the model.
- **CPU only.** `onnxruntime` is not in the stock 3X image (openpilot ships tinygrad for modeld). The fallback is tinygrad on `DEV=CPU` with the CLANG renderer, `CPU_COUNT=1`, and `GPU`/`QCOM` set to 0. Startup logs `speedsignd cpu-only asserted backend=...` and raises if the device is not CPU.
- **No compile while driving.** The first realize is the ~108 s on-device compile. It runs only while disengaged. Engaged reads wait (`reason=compile-wait`) until that realize has finished.
- **1 ONNX / BLAS thread** (`OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS`, `TINYGRAD_NUM_THREADS`, `CPU_COUNT`, and the rest, all forced to 1)
- pinned to **little core 2** (modeld stays FIFO on core 7)
- `NAP_SPEED_SIGN_INFER_CAP_MS` default **800** is logged only. It does **not** add a second wait. Pressure adds another 3× infer of rest. `modelV2.frameDropPerc` backs off before the lag alert. Any skipped `frameId`, or `modelExecutionTime` above 0.050 s, pauses at least 10 s.
- `os.sched_yield()` while an infer is busy

YOLOv8n-320 is about **2.2 GFLOP**, roughly 4× cheaper than the YOLOv8s (~8.7 GFLOP) that produced the route 0000017a numbers. Those 3.8 s infers and the 108 s first realize were the **GPU** path, not a CPU measurement. On one A55 (core 2) with tinygrad CLANG, expect about **1–4 s** per infer. This environment cannot time a 3X, so that range is the FLOP ratio, not a device measurement. The 25% duty cap still applies: start-to-start gap is about **4–16 s** (`read_interval_ms` ≈ 4× `infer_ms`, `cpu_share` ≤ 0.250). A ~1 s infer gaps about 4 s; a ~4 s infer gaps about 16 s. The first realize/compile still waits for disengaged (`reason=compile-wait`) and should be shorter than 108 s, but it is not a measured number. Mean well under 500 ms is still not the expectation on stock 3X tinygrad.

Onroad, `swaglog` prints `speedsignd detect paused (controlling=True enabled=… active=… state=… alive=… valid=…)` when you SET, `speedsignd abandon in-flight ONNX` if a YOLO was still running, and every infer:

```
speedsignd timing read_interval_ms=32000 infer_ms=8000 cpu_share=0.250 threads=1 backend=tinygrad-clang engaged=1 backoff=0 reason=pace
speedsignd infer 8000ms backend=tinygrad-clang frame=1928x1208 letterbox=320 crop=1300,520 628x320 … peak=0.72/speedLimit65 n_over=1 sl_peak=0.72/speedLimit65 … refine=50:0.71 class=65 luma=90/35 chroma=1 prep=18 sess=7900 raw=[(50, 0.71)] hud=[(50, 0.71)] jsonl=[]
```

`refine=` is the existing crop digit read (`refine_mph`), not a new digit reader. HUD and JSONL light only after **two reads from different frames** agree. Class and OCR on the same frame are one vote when they match, and **no vote** when they contradict (v2 often reads 60 as 30 and 30 as 50; a matching wrong OCR must not lock that in). A class-only hit does not vote. OCR-only (the second look) is one vote. `sl_unreadable` and `neg_*` never become an mph; `sl_unreadable` may schedule the OCR second look. Last accepted mph holds **~45 s**. The agree window is **12 s**, so two reads meet when infer is up to about 3 s. A slower infer can miss a sign. That is a miss, not a wrong plate.

Every finished read logs a greppable line:

```
speedsignd timing read_interval_ms=17200 infer_ms=4300 cpu_share=0.250 threads=1 engaged=1 backoff=0 reason=pace
```

`read_interval_ms` is the gap between read starts. On a ~4.3 s infer it should sit near **4× `infer_ms`** (about 17200), and `cpu_share` near **0.250** or lower. `threads=1`. `engaged=1` while openpilot is controlling. `reason=pace` is the duty cycle. `reason=model-drop`, `model-skip`, or `cpu-budget` means it rested early. A back-off is:

```
speedsignd timing backoff=1 reason=model-lag extra_ms=3980 infer_ms=3980 engaged=1
```

A 15 s summary still prints `speedsignd timing hz=… infer_ms mean=… max=… n=… skip=… engaged=…`. If TAKE CONTROL / “driving model is lagging” / Communication Issue comes back, Logger Off. The same `speedsignd timing` lines show whether detect was running.

## Install weights on the 3X

No large weights in git (~12 MB ONNX). Same idea as the OSM map pack: fetch onto `/data` with a SHA-256 check. A new sha replaces the file at the same path. The old v1 weights must not stay.

On the 3X (parked / Force Offroad, Wi-Fi): **Settings → NAP → Install weights** → Start. Settings shows **Speed Sign Weights: Installed**, **Out of date**, or **Missing**. **Out of date** means the file is present but the sha is not v2 — tap Install weights again; the download replaces it. Progress and errors stay on that runner screen (does not block the rest of the UI, does not reboot). speedsignd will not load a mismatched sha (the plate stays **NO WT** until the file matches).

Or SSH:

```bash
python -m scripts.nap.install_speed_sign_weights
```

That downloads GitHub Release `speed-sign-onnx-v2` / `speed_sign.onnx` and writes:

| | |
|---|---|
| Install path | `/data/media/0/nap/speed_sign.onnx` |
| Override | env `NAP_SPEED_SIGN_ONNX` or `--out` |
| SHA-256 | `weights_manifest.ASSET_SHA256` (checked on download) |

If the Release asset is not up yet, export once on a PC (needs `pip install ultralytics onnx`), then copy:

```bash
# PC
python -m scripts.nap.export_speed_sign_onnx --out ./speed_sign.onnx
scp ./speed_sign.onnx comma@<device>:/tmp/speed_sign.onnx

# 3X
python -m scripts.nap.install_speed_sign_weights /tmp/speed_sign.onnx
```

Or pass `--url` at a local `file://` / HTTP path. `--sha256 ''` skips the digest (local experiments only).

Shipped detector: GitHub Release `speed-sign-onnx-v2`, file `speed_sign.onnx`, YOLOv8n, imgsz 320, opset 12, input `images` 1×3×320×320, output `[1, 17, 2100]`. Classes, in order: `sl_25`, `sl_30`, `sl_35`, `sl_40`, `sl_45`, `sl_50`, `sl_55`, `sl_60`, `sl_65`, `sl_70`, `sl_unreadable`, `neg_yellow`, `neg_other`. SHA-256 `1131200b7fe2d400596e09d7680fc3a26a08dcd8952acfdf261170bdb22d3d28`, 12658953 bytes. Runtime: `onnxruntime` CPUExecutionProvider if that package is installed (it is not in the stock 3X image), else tinygrad `OnnxRunner` on `DEV=CPU` / CLANG with GPU and QCOM off. The startup log names which one loaded. Backend on the 3X is `tinygrad-clang`.

## What Justin should see

After weights are installed and the logger is **On**, **drive manually** (do not engage openpilot). Pass a **clear, unobstructed MUTCD R2-1** (white SPEED LIMIT plate) in daylight — e.g. a roadside **55** or **60**. Moving is the intended path. Parked / Force Offroad is not required to detect.

- SIGN shows the mph **while engaged**. It does not show **WAIT** just because openpilot is controlling. A blank plate on a clear sign is a miss, not a pause (report `swaglog | grep "speedsignd timing"`).
- After two reads agree (the OCR second look is the next frame or two, not the next YOLO), a large opaque **SIGN** plate appears with that mph on the **left** (driver) side of the onroad UI.
- **Yes** / **No** (“is this accurate?”) sit under the plate on 3X (beside it on comma 4). They are **stubs** right now — they do not change cruise, HUD MAX, or map speed. Later, Yes may confirm the marker (JSONL + optional OSM); No may discard a wrong read.
- It holds **~45 s** after the last accepted detection (the old **3.0 s** hold blanked between multi-second infers), then hides (Yes/No hide with it).
- A JSONL row is appended only with a live GNSS fix (same mph near the last write is skipped ~8 s / ~40 m).
- HUD **MAX** / cruise / OSM LIMIT do not change.

If the plate never appears, see **SIGN never lights** below.

## Log path

| | |
|---|---|
| Device | `/data/media/0/nap/speed_signs.jsonl` |
| PC / tests | `~/.comma/media/0/nap/speed_signs.jsonl` |
| Override | env `NAP_SPEED_SIGN_LOG` |

One JSON object per line:

```json
{"t":1710000000.12,"lat":45.315,"lon":-95.601,"bearing":87.2,"mph":45,"conf":0.81}
```

| Field | Meaning |
|---|---|
| `t` | Unix time (seconds) |
| `lat`, `lon` | GNSS fix used for the row |
| `bearing` | GNSS bearing (deg), or `null` |
| `mph` | MUTCD posted integer (5–85 step 5, or 100) |
| `conf` | Detector confidence 0–1 |

A row is written only with a live GNSS fix. The on-road HUD still updates from every **confirmed** detection (cereal `liveSpeedSignNAP`), including without GPS.

Pull the log off the device:

```bash
scp comma@<device>:/data/media/0/nap/speed_signs.jsonl .
```

## On-road HUD

When the logger is **On**, a large opaque **SIGN** plate shows the mph the camera is reading. Display-only — it does not change MAX or cruise.

| | |
|---|---|
| Where (3X) | Left / driver side, below the MAX box. Large plate (200×248). |
| Where (comma 4) | Left / driver side (top-left), same idea |
| Yes / No | Shown only with a live mph. 3X: stacked under the plate (full-width, ~112 px tall). comma 4: beside the plate. **No-op stubs** (cloudlog debug only). |
| Confirm | **2** reads of the same mph from **different frames**, inside **12 s**. Class and OCR on one frame are one vote when they match, and no vote when they contradict. Class-only does not vote. JSONL uses the same two-read rule, conf ≥ **0.40**. |
| Hold | **~45 s** after the last accepted detection (old hold was **3.0 s**), then it hides |
| Detect rate | Default **1 Hz** while engaged and while manual (env `NAP_SPEED_SIGN_HZ`, clamped 0.2–4), including while moving. A long infer idles 3× that infer. SubMaster **20 Hz**. |
| Engaged plate | The mph, not **WAIT**. YOLO keeps running. |
| Source | cereal `liveSpeedSignNAP` (not the JSONL file) |

White MUTCD-style plate, black digits, red border, **SIGN** label so it is not confused with HUD MAX or OSM LIMIT.

If the logger is On and ONNX failed to load, the same left-side plate shows **NO WT** (not a mph) — **no Yes/No**. If the logger is On and openpilot is **controlling**, the plate still shows the mph when a sign is read — detect is not paused. A blank plate with weights Installed means no confirmed detection yet — that is normal. Report `swaglog | grep "speedsignd timing"` if reads are many seconds apart or `engaged=0` for the whole drive.

Yes/No do not write params, JSONL, or sqlite in this revision. The hook is `on_confirm_accuracy(mph, yes)` — future work may JSONL-confirm a good read and optionally push the marker to OSM.

## SIGN never lights

Speed Sign Logger On, onroad, but SIGN stays dark or never shows mph — almost always missing weights, not a bad model.

1. Look at the onroad plate: **NO WT** means `/data/media/0/nap/speed_sign.onnx` is absent or failed to load. Settings → NAP → Speed Sign Weights should say **Missing**.
2. Park or turn on Force Offroad. Tap **Install weights** (Wi-Fi). Wait for the runner to finish — do not leave it spinning forever; an error prints on that screen. Or SSH: `python -m scripts.nap.install_speed_sign_weights`.
3. File should be ~12 MB (12658953 bytes). `ls -l /data/media/0/nap/speed_sign.onnx`. Settings should flip to **Installed**. **Out of date** means the old file is still there — run Install weights again.
4. No full reboot required: speedsignd retries ONNX every ~15 s. `swaglog` should show `speedsignd starting … backend=yolo-onnx` or `ONNX loaded after retry backend=yolo-onnx` (also `hz=` / `nice=19`).
5. Engage or stay manual. Pass a clear, unobstructed MUTCD R2-1. Two agreeing reads light SIGN while **controlling**. `swaglog | grep "speedsignd timing"` should show `engaged=1`, `threads=1`, `cpu_share` at or under `0.250`, and `read_interval_ms` about 4× `infer_ms`. A blank plate with weights Installed means no detection yet (night, glare, tiny sign — see Accuracy limits).

Do not treat a blank plate as “the detector is running.” Blank + Installed = no confirmed sign. **WAIT** is not the engaged state. Blank + Missing / **NO WT** = install weights.

If you see **TAKE CONTROL IMMEDIATELY** or “Communication Issue Between Processes,” turn **Speed Sign Logger Off** and use **nap-release**.

## Accuracy limits (honest)

This is a small CPU detector at **1 Hz** (was 4 Hz) on one native **628×320** right-side window, including while **controlling**. It is **not** a modeld head and is **not** used for control. It must not starve `modeld`: little core 2, SCHED_IDLE or nice 19, 1 thread, idle at least 3× each infer, and a back-off when `modelV2` drops frames, skips a frame id, or `modelExecutionTime` is above 50 ms, or this process's own CPU share goes over 25%. modeld stays FIFO on core 7; controlsd stays on core 4. `swaglog` logs `speedsignd detect paused/running` with `enabled` / `active` / `state` / alive / valid on those edges and every infer `backend=` `peak=` `refine=` `luma=` `chroma=` `prep=` `sess=` `raw=` plus `speedsignd timing read_interval_ms=… infer_ms=… cpu_share=… threads=… backend=… engaged=… backoff=… reason=…`. Offline val on this window: 105 hits, 52 misreads, 58 misses (30/35/60 read as 50, 30 read as 45). Held-out route 17a: 5 hits, 39 misreads (60 as 30 twenty-one times, 30 as 50 seventeen times), 7 misses. `YOLO_MIN_CONF` stays **0.40**. Two different frames must agree, and a class/OCR contradiction does not vote, so those misreads miss instead of lighting. If both frames are wrong the same way, the plate can still be wrong.

**Usually works:** daylight, dry, a standard white R2-1 facing the car, large enough in the ROAD frame (near / mid roadside, not a speck on the horizon). 55 and 60 are in the trained class set.

**Often misses or misreads:**

- Night, rain, snow, heavy shadow, glare / washout, dirty lens
- Temporary orange / work-zone plaques, electronic VMS, banner-style overlays
- Metric (km/h) plates, yellow advisory plaques, school-zone assemblies when the number is secondary
- Strong perspective (sign almost edge-on), motion blur at high speed, occlusion (trees, trucks)
- Far freeway signs that occupy only a few pixels after the 320² letterbox
- 25 vs 35 vs 55 confusion when the glyph is tiny or partly clipped (similar white plates)
- Non-MUTCD artwork, stickers, or a sign not in the LISA/Roboflow mix

Empty road / sky is quiet at the 0.40 threshold (the model’s background scores are ~0). Debounce exists to kill single-frame flashes, not to invent detections in the dark.

Bring-your-own ONNX is still accepted if it matches the YOLO layout above or the older `[N,6] = x,y,w,h,mph,conf` custom layout.

## What this does not do

- No sqlite writes (OSM map speed is unchanged)
- No cruise / `vCruise` / HUD MAX overlay
- No osm.org / Overpass
- No modeld / `modelV2` head
- The SIGN plate is display-only (same process; not a second controller)
- Yes/No accuracy buttons are display-only stubs (no JSONL confirm, no OSM upload)
