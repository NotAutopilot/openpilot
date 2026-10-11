# Speed Sign Logger (log-only)

On-drive MUTCD numeric speed-sign logger for comma 3X. **Default off.** Log-only: it does not write sqlite, does not change cruise / HUD **MAX** (`vCruise`), and does not query osm.org.

Stock `modelV2` has no `speedSign` head. This is a separate process (`speedsignd`) that reads the ROAD camera + GNSS and runs a compact **YOLOv8 ONNX** (US traffic-sign classes, including 55/60). The old numpy template matcher is tests/dev only — it does not see real roadside signs.

**Driving model first.** Route 0000017a showed the failure was the GPU, not the CPU budget. modeld sat at 22–26 ms until speedsignd's tinygrad realize dispatched onto the same Adreno (QCOM) modeld uses, and modeld jumped to 300–710 ms. Inference is CPU only. The stock 3X image has neither `onnxruntime` nor `cv2.dnn` (route 00000180 logged `onnxruntime-absent(ModuleNotFoundError)` and `cv2-dnn-absent` and fell back to tinygrad CLANG). **Install weights** also installs pinned onnxruntime **1.31.0** CPU (`cp312`, `manylinux_2_28_aarch64`) into `/data/media/0/nap/pylib` and speedsignd loads it with `sys.path`. That wheel is not pip-installed and does not touch the system venv. Session options stay `CPUExecutionProvider`, `intra_op_num_threads=1`. Tinygrad `DEV=CPU` / CLANG remains the slow fallback only, and its first realize is refused while engaged (`reason=compile-wait`), including when the first frame arrives already engaged at boot. 4 Hz YOLO can still show “driving model is lagging”, ~35% `modeld` frame drops, **TAKE CONTROL IMMEDIATELY**, or “Communication Issue Between Processes.”

**Logger On keeps running YOLO while openpilot is engaged** (`selfdriveState.active`, or state in enabled / softDisabling / overriding). The process and SIGN / NO WT HUD still start. The plate shows the held mph while engaged. It does **not** show **WAIT** for that — WAIT was the old pause, and it is why a drive that was engaged 95% of the time never read a sign.

Report `swaglog | grep "speedsignd timing"` (`read_interval_ms`, `infer_ms`, `cpu_share`, `threads`, `os_threads`, `backend`, `engaged`, `backoff`, `reason`). `backend=` must be `onnxruntime-cpu`, `opencv-dnn-cpu`, `tinygrad-clang`, or `tinygrad-cpu`. `threads=` is the pinned compute pool and must be `1`. `os_threads=` is every OS thread in the process. Do not treat a blank plate as “drive parked” or “weights are bad.”

**Engaged and manual driving both collect** — including moving, no assist. This is **not** gated on park or Force Offroad. Logger On, and log speed-limit signs + GNSS to JSONL. OSM upload is future work.

Unknown cereal is **not** a pause. After a ~2 s startup, unread `selfdriveState` allows the same throttled detect and logs a warning. SubMaster polls at **20 Hz** so 100 Hz `selfdriveState` alive/valid cannot flap the gate; YOLO stays ≤ **1 Hz**, and a long infer idles at least 3× that infer before the next one (about 25% of one core). Engaging does **not** abandon an in-flight ONNX.

Detect is **1 Hz** or slower, `SCHED_IDLE` (else nice 19), little **core 2** only, one thread — engaged and manual. Core 0 is the UI, 1 is sensord, 3 is pandad/encoderd, 4 is controlsd, 7 is modeld. **If TAKE CONTROL or lag comes back, turn Speed Sign Logger Off.** Do not raise `modeld` priority. A back-off line in swaglog means speedsignd saw `modelV2` frame drops, a skipped frame id, `modelExecutionTime` above 50 ms, its own CPU share over 25%, or a model/controls alert, and rested. A frame skip or modeld over 50 ms pauses at least 10 s (`reason=model-skip` or `reason=model-exec`).

## Enable

1. **Install weights** (once, Wi-Fi) — Settings → NAP → **Install weights** (offroad / Force Offroad), or SSH below. That installs the ONNX and the CPU runtime. Without `/data/media/0/nap/speed_sign.onnx` the onroad SIGN plate shows **NO WT**. With the ONNX but without the runtime, Settings says **runtime missing** and the plate shows **NO RT**. Neither will light mph on real signs.
2. **Settings → NAP → Speed Sign Logger** → On  
   (comma 4 / mici: **Settings → NAP → speed sign logger**)
3. Or: `Params().put_bool("NAPSpeedSignLog", True)` / `echo -n 1 > /data/params/d/NAPSpeedSignLog`
4. Go onroad. Manager starts `speedsignd` only when the param is true **and** the car is onroad.
5. **Drive, including engaged** (moving is OK). SIGN shows the mph while openpilot is **actively controlling**. It does not show **WAIT** for engage. If the plate stays blank on a clear sign, grab swaglog.

Off (default): the process does not run. Logger On never changes that default.

Reset to Defaults turns the logger back off. Weights on `/data` stay.

Optional detect rate (default 1 Hz, clamped 0.2–4): `NAP_SPEED_SIGN_HZ=0.5` in the process environment. Do not raise this on a 3X. While engaged, a short infer still waits out that period. A long infer waits `4× infer` (the infer plus 3× idle). It is not 0 Hz.

Manual detect is **cheap-path** so Logger On is less likely to starve `modeld`:

- two right-side tiles of the 1928×1208 ROAD frame, alternating one read at a time, letterboxed into the 320 ONNX — never a full-frame RGB convert, and not the old 1208² letterbox (that left an 80 ft sign at about 20 px). The lower tile is **x=1300, y=520, 628×320** (through x=1928, y=840). Route 0000017a signs sat at about y=557–805, x=1110–1914, inside that tile. Town routes sat at y=340–795, x=968–1928, and y=340 is above the lower tile, so the second tile is **x=1288, y=260, 640×320** (through x=1928, y=580). It covers y=340 and overlaps the lower tile from y=520 to y=580. The x=968–1288 tail is not a third tile: at the measured 627 ms infer, two tiles already use the 25% budget and each band is seen about every 5 s; a third tile would push that to about 7.5 s. Scale on the lower tile is 320/628 ≈ 0.51. A sign ≥ 60 px tall stays ≥ 30 px in the model.
- **CPU only.** Order: vendored or system `onnxruntime` `CPUExecutionProvider` (the 3X install puts 1.31.0 on `sys.path` from `/data/media/0/nap/pylib`), else `cv2.dnn` with `DNN_BACKEND_OPENCV` and `DNN_TARGET_CPU` when `cv2.dnn.readNetFromONNX` exists (it does not on this 3X), else tinygrad on `DEV=CPU` with the CLANG renderer, `CPU_COUNT=1`, `BEAM=0`, `NOOPT=0`, and `GPU`/`QCOM` set to 0. Startup logs `speedsignd cpu-only asserted backend=...` plus `threads=` (the BLAS pool, must be 1) and `os_threads=` (every OS thread) and raises if the device is not CPU. OpenCL is forced off.
- **No compile while engaged.** The first realize runs only when `selfdriveState` is known and not controlling. Unknown cereal is not that window. A boot whose first frame is already engaged does not compile. The worker re-reads that flag before realize. Engaged reads wait (`reason=compile-wait`). The kernels are written under `/data/media/0/nap/speed_sign_cache/<sha>/` so that clang compile happens once per weights sha. `model-skip` rest is capped at **60 s** (route 0000017f logged a 531828 ms backoff after a 177 s compile).
- **1 ONNX / BLAS thread** (`OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS`, `TINYGRAD_NUM_THREADS`, `CPU_COUNT`, and the rest, all forced to 1)
- pinned to **little core 2** (modeld stays FIFO on core 7)
- `NAP_SPEED_SIGN_INFER_CAP_MS` default **800** is logged only. It does **not** add a second wait. Pressure adds another 3× infer of rest. `modelV2.frameDropPerc` backs off before the lag alert. Any skipped `frameId`, or `modelExecutionTime` above 0.050 s, pauses at least 10 s.
- `os.sched_yield()` while an infer is busy

YOLOv8n-320 is about **2.2 GFLOP**. Expected timing on one Cortex-A55 after Install weights:

| backend | where it exists | expected infer | read gap |
|---|---|---|---|
| `onnxruntime-cpu` | vendored 1.31.0 wheel (this is the 3X path) | **627 ms** median on route 00000181 (the x86 scale estimate was ~300 ms) | **~2.5 s** steady (`4× infer`); each tile ~5 s |
| `opencv-dnn-cpu` | not in the stock 3X image | **~300–500 ms** if some other image has it | **~1.2–2 s** |
| `tinygrad-clang` | always, slow fallback | **~5800 ms** steady, first compile **~177 s** | multi-second, and the compile must not run while engaged |

Route 0000017e measured tinygrad CLANG at about **5800 ms** (0.38 GFLOP/s) after a ~3 minute first compile. Route 0000017f (nap-dev) measured the first compile at **177 s**, finished while engaged, `threads=11`, then a false `model-skip` backoff of **531828 ms**. That steady CLANG time is the kernel, not a per-call recompile: ClangJITCompiler emits freestanding C at `-O2` and does not run LLVM's loop/SLP vectorizer, so the kernels stay scalar. `BEAM` and `NOOPT` were already off. A one-thread bench of this same file on onnxruntime **1.31.0** `CPUExecutionProvider` (AVX-512 Xeon at 2.4 GHz) measured **12 ms** (~180 GFLOP/s). **cv2.dnn** on that host was **25 ms**. One A55 at 1.8 GHz peaks near 14 GFLOP/s (8 FP32 FLOP/cycle). The Xeon peak is on the order of 150 GFLOP/s, so scaling the 12 ms ORT result by peak (14/150) is about **130 ms** if the A55 build keeps the same fraction of peak. ORT was near peak on x86; an A55 at 30–50% of its peak is about **300–500 ms**. The x86 scale suggested **~300 ms** and a gap of about **1.2 s**. Route 00000181 measured the A55 at **627 ms** median, `read_interval` median **2.6 s**, `cpu_share` **0.22**, modeld **22 ms** with 0% drops and no commIssue, `backend=onnxruntime-cpu`. Steady duty stays **≤ 0.25**: start-to-start is **max(1 s, 4× infer)**, about **2.5 s** at 627 ms (`read_interval_ms` ≈ 2500, `cpu_share` ≤ 0.250, `threads=1`, `reason=pace`). The two tiles alternate, so each tile is about every **5 s** when nothing is bursting. A class score ≥ 0.15 schedules two more YOLO reads on that same tile immediately (three frames, about 0.63 s apart, ~1.9 s of core time). 3×627 ms = 1.88 s, inside the 2.5 s allowed in a 10 s window. The idle after the burst waits until the trailing 10 s of infer time is back to ≤ 2.5 s. A model-skip or model-exec pause still cancels the burst. `reason=burst` is that immediate re-read. The 3X does not have cv2.dnn, so do not expect `opencv-dnn-cpu` there. The tinygrad CLANG fallback stays multi-second and is not the path after Install weights. The first session still waits until disengaged (`reason=compile-wait`), including a boot that is already engaged, and is then cached for that weights sha. `model-skip` extra rest is capped at 60 s.

Onroad, `swaglog` prints `speedsignd detect paused (controlling=True enabled=… active=… state=… alive=… valid=…)` when you SET, `speedsignd abandon in-flight ONNX` if a YOLO was still running, and every infer:

```
speedsignd timing read_interval_ms=32000 infer_ms=8000 cpu_share=0.250 threads=1 backend=tinygrad-clang engaged=1 backoff=0 reason=pace
speedsignd infer 8000ms backend=tinygrad-clang frame=1928x1208 letterbox=320 crop=1300,520 628x320 … peak=0.72/speedLimit65 n_over=1 sl_peak=0.72/speedLimit65 … refine=50:0.71 class=65 luma=90/35 chroma=1 prep=18 sess=7900 raw=[(50, 0.71)] hud=[(50, 0.71)] jsonl=[]
```

`refine=` is the existing crop digit read (`refine_mph`), not a new digit reader. It may replace the class only with a real US limit (5–85, step 5). It must not store 4, 46, or 100, and it must not turn a valid class into a dropped digit (route 00000181 produced a 5 from a 45). JSONL still lights only after **two reads from different frames** agree. The HUD does too, except one read at **≥ 0.70** on a real limit lights immediately and holds **~45 s** unless a different limit is read within **3 s**. 0.60 still needs the second read. `YOLO_MIN_CONF` stays **0.40**. Class and OCR on the same frame are one vote when they match, and **no vote** when they contradict (v2 often reads 60 as 30 and 30 as 50; a matching wrong OCR must not lock that in). A class-only hit below 0.70 does not vote. OCR-only (the second look) is one vote. `sl_unreadable` and `neg_*` never become an mph; `sl_unreadable` may schedule the OCR second look, and any speed-sign class ≥ 0.15 schedules the YOLO burst instead. The two-read window is **12 s**. Route 00000181 had five one-off class reads (45@0.60, 45@0.77, 30@0.75, 45@0.89, 30@0.81) and none had a second agreeing read inside 12 s, so the plate stayed dark under the old two-read rule. Route 0000017e stayed at mph 0 because CLANG plus a false model-skip stretched the gap to ~40 s, longer than the window. A slower infer can still miss a sign. That is a miss, not a wrong plate.

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

On the 3X (parked / Force Offroad, Wi-Fi): **Settings → NAP → Install weights** → Start. That downloads both files. Settings shows **Speed Sign Weights: Installed**, **Out of date**, or **Missing**, and **runtime onnxruntime 1.31.0** or **runtime missing**. **Out of date** means the ONNX is present but the sha is not v2 — tap Install weights again; the download replaces it. **runtime missing** means the wheel is not importable yet — the same button installs it; the onroad plate shows **NO RT** (NO WT still wins when the ONNX itself is missing). Progress and errors stay on that runner screen (does not block the rest of the UI, does not reboot). speedsignd will not load a mismatched sha (the plate stays **NO WT** until the file matches).

Or SSH:

```bash
python -m scripts.nap.install_speed_sign_weights
```

That downloads GitHub Release `speed-sign-onnx-v2` / `speed_sign.onnx` and writes:

| | |
|---|---|
| ONNX path | `/data/media/0/nap/speed_sign.onnx` |
| Runtime path | `/data/media/0/nap/pylib` (unzipped wheel, not the system venv) |
| Override | env `NAP_SPEED_SIGN_ONNX` or `--out`; pylib via `NAP_SPEED_SIGN_PYLIB` |
| ONNX SHA-256 | `1131200b7fe2d400596e09d7680fc3a26a08dcd8952acfdf261170bdb22d3d28` |
| Runtime | onnxruntime `1.31.0` `cp312` `manylinux_2_28_aarch64`, 21420804 bytes |
| Runtime SHA-256 | `35758d7606d578ec5b9d65f6e8a1f488013194c3f6097038a3223cb26d35ef9a` |

If the Release asset is not up yet, export once on a PC (needs `pip install ultralytics onnx`), then copy:

```bash
# PC
python -m scripts.nap.export_speed_sign_onnx --out ./speed_sign.onnx
scp ./speed_sign.onnx comma@<device>:/tmp/speed_sign.onnx

# 3X
python -m scripts.nap.install_speed_sign_weights /tmp/speed_sign.onnx
```

Or pass `--url` at a local `file://` / HTTP path. `--sha256 ''` skips the digest (local experiments only).

Shipped detector: GitHub Release `speed-sign-onnx-v2`, file `speed_sign.onnx`, YOLOv8n, imgsz 320, opset 12, input `images` 1×3×320×320, output `[1, 17, 2100]`. Classes, in order: `sl_25`, `sl_30`, `sl_35`, `sl_40`, `sl_45`, `sl_50`, `sl_55`, `sl_60`, `sl_65`, `sl_70`, `sl_unreadable`, `neg_yellow`, `neg_other`. SHA-256 `1131200b7fe2d400596e09d7680fc3a26a08dcd8952acfdf261170bdb22d3d28`, 12658953 bytes. Runtime: pinned CPU wheel `onnxruntime-1.31.0-cp312-cp312-manylinux_2_28_aarch64.whl` (SHA-256 `35758d7606d578ec5b9d65f6e8a1f488013194c3f6097038a3223cb26d35ef9a`, 21420804 bytes), unzipped into `/data/media/0/nap/pylib` and imported via `sys.path` when the system package is absent. The ELF is AArch64, needs only glibc 2.28 (AGNOS 18.4 is newer), and does not link CUDA. Else `cv2.dnn` on the CPU target when the image has `readNetFromONNX` (this 3X does not), else tinygrad `OnnxRunner` on `DEV=CPU` / CLANG with GPU and QCOM off and the kernel cache under `/data`. The startup log names which one loaded. After Install weights the 3X path is `onnxruntime-cpu`. `tinygrad-clang` is the slow fallback and must not compile while engaged.

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
| Confirm | **2** reads of the same mph from **different frames**, inside **12 s**, or **one** read ≥ **0.70** on a real limit (5–85, step 5) unless a different limit arrives within **3 s**. Class and OCR on one frame are one vote when they match, and no vote when they contradict. Class-only below 0.70 does not vote. JSONL still uses the two-read rule, conf ≥ **0.40**. |
| Hold | **~45 s** after the last accepted detection (old hold was **3.0 s**), then it hides |
| Detect rate | Default **1 Hz** while engaged and while manual (env `NAP_SPEED_SIGN_HZ`, clamped 0.2–4), including while moving. A long infer idles 3× that infer. SubMaster **20 Hz**. |
| Engaged plate | The mph, not **WAIT**. YOLO keeps running. |
| Source | cereal `liveSpeedSignNAP` (not the JSONL file) |

White MUTCD-style plate, black digits, red border, **SIGN** label so it is not confused with HUD MAX or OSM LIMIT.

If the logger is On and ONNX failed to load, the same left-side plate shows **NO WT** (not a mph) — **no Yes/No**. If the ONNX is installed and the CPU runtime is not, the plate shows **NO RT** and Settings says **runtime missing** — tap Install weights again (it installs the runtime too). If the logger is On and openpilot is **controlling**, the plate still shows the mph when a sign is read — detect is not paused. A blank plate with weights Installed and the runtime present means no confirmed detection yet — that is normal. Report `swaglog | grep "speedsignd timing"` if reads are many seconds apart or `engaged=0` for the whole drive. `threads=` must be 1. `os_threads=` is the whole process.

Yes/No do not write params, JSONL, or sqlite in this revision. The hook is `on_confirm_accuracy(mph, yes)` — future work may JSONL-confirm a good read and optionally push the marker to OSM.

## SIGN never lights

Speed Sign Logger On, onroad, but SIGN stays dark or never shows mph — almost always missing weights, not a bad model.

1. Look at the onroad plate: **NO WT** means `/data/media/0/nap/speed_sign.onnx` is absent or failed to load. Settings → NAP → Speed Sign Weights should say **Missing**. **NO RT** means the ONNX is present and the CPU runtime is not — Settings says **runtime missing**.
2. Park or turn on Force Offroad. Tap **Install weights** (Wi-Fi). That installs the ONNX and the runtime. Wait for the runner to finish — do not leave it spinning forever; an error prints on that screen. Or SSH: `python -m scripts.nap.install_speed_sign_weights`.
3. ONNX should be ~12 MB (12658953 bytes). `ls -l /data/media/0/nap/speed_sign.onnx`. Runtime: `ls /data/media/0/nap/pylib/onnxruntime`. Settings should flip to **Installed** and must not say **runtime missing**. **Out of date** means the old ONNX is still there — run Install weights again.
4. No full reboot required: speedsignd retries ONNX every ~15 s. `swaglog` should show `speedsignd starting … backend=onnxruntime-cpu` (also `threads=1`, `os_threads=`, `hz=` / `nice=19`). Leave Logger On while disengaged until that first session finishes, then engage. A boot that is already engaged waits (`reason=compile-wait`) and does not run the multi-minute CLANG compile.
5. Engage or stay manual. Pass a clear, unobstructed MUTCD R2-1. Two agreeing reads, or one read ≥ 0.70, light SIGN while **controlling**. `swaglog | grep "speedsignd timing"` should show `backend=onnxruntime-cpu`, `engaged=1`, `threads=1`, `os_threads=`, `cpu_share` at or under `0.250`, and `read_interval_ms` about 4× `infer_ms` (about 2500 when infer is ~627 ms; `reason=burst` is a short back-to-back confirm). A blank plate with weights Installed and the runtime present means no detection yet (night, glare, tiny sign — see Accuracy limits).

Do not treat a blank plate as “the detector is running.” Blank + Installed = no confirmed sign. **WAIT** is not the engaged state. Blank + Missing / **NO WT** = install weights.

If you see **TAKE CONTROL IMMEDIATELY** or “Communication Issue Between Processes,” turn **Speed Sign Logger Off** and use **nap-release**.

## Accuracy limits (honest)

This is a small CPU detector at **1 Hz** (was 4 Hz) on two native right-side tiles (628×320 at y=520 and 640×320 at y=260), including while **controlling**. It is **not** a modeld head and is **not** used for control. It must not starve `modeld`: little core 2, SCHED_IDLE or nice 19, 1 thread, idle at least 3× each infer, and a back-off when `modelV2` drops frames, skips a frame id, or `modelExecutionTime` is above 50 ms, or this process's own CPU share goes over 25%. modeld stays FIFO on core 7; controlsd stays on core 4. `swaglog` logs `speedsignd detect paused/running` with `enabled` / `active` / `state` / alive / valid on those edges and every infer `backend=` `peak=` `refine=` `luma=` `chroma=` `prep=` `sess=` `raw=` plus `speedsignd timing read_interval_ms=… infer_ms=… cpu_share=… threads=… backend=… engaged=… backoff=… reason=…`. Offline val on this window: 105 hits, 52 misreads, 58 misses (30/35/60 read as 50, 30 read as 45). Held-out route 17a: 5 hits, 39 misreads (60 as 30 twenty-one times, 30 as 50 seventeen times), 7 misses. `YOLO_MIN_CONF` stays **0.40**. Two different frames must agree, and a class/OCR contradiction does not vote, so those misreads miss instead of lighting. If both frames are wrong the same way, the plate can still be wrong.

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
