NAP sight-distance cap, LOG ONLY (2026-10-11)
========================
* Pre-AP shadow (`NAPSightCap`, default 1 = shadow; 0 off). Nothing reads it: speed, MAX, curve follow, CurveMaxHold, the #222 cases and the -0.22 EV settle are unchanged.
* Sight distance = where the model path's lateral std reaches 2 m (`model_sight`, shared with low-visibility steering, whose path std read now lives there unchanged).
* Would fire when sight drops below half its 2-12 s median for 0.5 s at 25-55 mph, with rising path curvature or a map roundabout ahead, unless the model was already red with a short path. Vehicles never count as sight loss: any lead (moving or stopped), adjacent car or parked car within 150 m suppresses it and the reason is logged. Steps aside while the roundabout funnel is live. Glare-only low-vis never feeds it.
* Logs `sightcap {json}` (errorLogMessage, no new capnp fields): sight, baseline, cap speed (slow to a 30 m bend at 15 mph inside the sight at 1.5 m/s², 2 m/s³ jerk), would-be accel, min with curve follow, leads, pitch, crest, night/sun, lane-line probs, blinker, confidence, low-vis.

NAP gap lock (2026-10-05)
========================
* Pre-AP, default Off (`NAPGapLock`). Hold the engage stalk toward you for 2 seconds to keep the current radar gap (8–80 m) instead of the time gap. Long can be on, paused, or not yet taken; a finished hold resumes it. Same follow law, a custom distance. The 3X shows the lock while it is on. A tip, cancel, brake, or One-Pedal pause returns to Follow Distance. Another 2 second hold stores a new distance. No panda flash.

NAP release firm lead brake when the gap opens or the lead leaves the lane (2026-09-23)
========================
* Pre-AP: a slightly negative lead acceleration no longer holds a full brake when the car ahead is pulling away with gap left (Scallywag 09:57 and 10:05). That firm brake now needs a real close — still coming together, a closing residual, or already inside the follow gap. After a short run of the gap opening, the command publishes the mild ease (−0.22), not a zero coast and not the full brake. A lead that slides out of the lane (about 2.5 m off the path) with gap still left does the same (08:57:50–52). An on-path close, including a hard brake for a slowing lead, stays firm. A broken construction path is pickier about starting a new lead that is already a couple of meters to the side. No panda flash.

NAP City/Hwy follow follows MAX, near-FD coast cliffs floor (2026-09-23)
========================
* City vs highway Follow Distance now follows **MAX**, for both the stalk bump and the live gap. **City** when MAX is under 50 mph. **Highway** when MAX is 50 or higher (exactly 50 is highway), including while the car is still coming up to that set speed and while it slows under a highway MAX. Traveled speed is used only when MAX is unset. A first-detent bump steps that band. A full press still nudges MAX. Slowing from highway speed under MAX 75 no longer opens the larger city gap.
* Near Follow Distance, a one-frame hard brake while the plan is still coasting (gap only a few meters long of the set follow, closing under about 1 m/s, lead not braking) is held to the mild ease instead of a full dump. A real brake for a slowing lead ahead — closing up, lead braking, or the firm-match path — stays firm. Near-bumper, forward collision warning, and a stop request stay firm. No panda flash.

NAP stalk tip writes the matching City or Highway Follow Distance (2026-09-23)
========================
* Pre-AP stalk **bump** (first detent, 1 mph) steps Follow Distance 1–7 on the matching band. **Highway** when ego is above 50 mph, or whenever MAX is above 50 (a bump while still accelerating up to a highway set speed stays on Highway, even under 50). **City** only when ego is at or under 50 mph and MAX is also at or under 50. HUD **Follow Distance: N** shows that band. Full press (2nd detent / 5 mph) still nudges MAX. Live follow blend (hysteresis around 50, highway follow from about 30 mph with a lead and MAX above 50) is unchanged. No panda flash.

NAP follow-smooth mid-gap rematch (2026-09-20)
========================
* Pre-AP comfort on a held radar lead: mid-gap Accel rematch no longer pulses while still slowly closing (~0.8–2.0 m/s, slack 12–50 m) unless a same-speed / opening catch-up is already latched (100 m start / too-close recovery still Accel through that band; latch also clears settle so a matched 0–25 m hold cannot freeze rematch while the gap is wrong). A *settled* follow that opens or hunts is not catch-up. Ceiling is a trickle (~0.10), not Accel 1–10. Slack > 50 still Accel. After acquire, small ±a chatter slews both ways so +0.25 ↔ −0.02 cannot flip gas↔regen — catch-up / too-close recovery skips that slew. When planner aTarget is ~0, VirtualDAS cannot dump firm regen (ef 10:18:42: plant −1.23 then rematch +0.40) — effort floor is MILD −0.22. Grade / pitch hold, rapid / near-bumper / FCW / #222 residual-close / planner ≤ −0.5 stay full −a/+a. No panda flash.

NAP skip mild floor on closing / near-gap braking lead (2026-09-20)
========================
* Pre-AP: `#216` kept the MILD −0.22 floor until confirmed rapid ≥ 6 / near-bumper / FCW, so a held lead that hard-braked (closing 1.8→4.4, aLead ~−1, dRel 38→25) never escaped slight-lift. Soft-limit now skips when residual close (worsened beyond ego a, including a planner-frame Δv that is smaller than the 0.15 rise gate) or measured aLead shows brake on an owned / path-synced lead. That also unlocks match-aLead (not `k·v_rel` dump). Closing ≥ 1.5 still reacts, including a cut-in. Matched / slow-close and large-slack e4 stay MILD / #218 glide. Map decel above MAX still mins in on a same-speed lead. Firm 0.55 / full hard-brake still wait on the rapid confirm. Rapid gate stays 6. No panda flash.

NAP roundabout detect + speed ease + outer bias (2026-09-20)
========================
* Map `junction=roundabout` (or a compact closed circulating way already in the OSM pack) ~200 m out — not a big steer, and not every sharp town corner / signalized cross. Kinematic decel toward **15–20 mph** (OSM ring maxspeed when present, often 20) so 40–45 mph plans **a ≤ −1.0** until near ring speed; `aTarget` stays ≤ 0 in the funnel (no +a after a lead clears). Long enable inside the funnel applies full ease immediately. ~3.2 m outer path bias on entry + circulating (right in RHT / US) to counter a ~3 m inside cut. Yield-before-merge / continue-circulate / UI chip are later. No panda flash.

NAP matched-speed glide + earlier settle (2026-09-19)
========================
* After #214 acquire + #216 slight-lift + #219 MAX ceiling: when matched or slightly slower near Follow Distance, leftover mild −a glides to 0; rematch trickle still finishes / holds grade under MAX. Last-meter closing, braking lead, or already inside FD is not a glide. +3 m kinematic gap bias. Inside FD still closing slowly commands the mild floor. Rapid / near-bumper / FCW keep full −a. No panda flash.

NAP MAX is a hard ceiling on lead rematch (2026-09-19)
========================
* Pre-AP: if the lead is faster than MAX/set, **hold ≤ MAX and let the gap open**. Lead rematch / remaining-close no longer commands Accel-1 (+0.36) to keep up past MAX (ea 11:46: 62.2 on a 60). At/above MAX, lead-close `a_env` is 0, not Accel peak. Catch-up under MAX, #214 acquire slew, #216 slight-lift floor, and emergency −a kept. No panda flash.

NAP EV slight-lift comfort path (2026-09-19)
========================
* Pre-AP Model S EV: non-emergency approach/follow is **slight lift only** (MILD −0.22). Full throttle lift / deep −a is reserved for rapid close / near-bumper / FCW. Closing ≳ 1.5 and near-gap aLead no longer skip the floor or acquire slew (that still punched −0.996 at 80–130 m). #214 acquire slew kept. No panda flash.

NAP lead acquire slew + small MPC bite soften (2026-09-19)
========================
* First lead latch no longer punches regen then rematch +a (e4 09:53:19: aTarget −0.46 → +0.05 in ~0.5 s at 118 m). Acquire window slews both ways. Large-slack small MPC adjustments (e4 −2.33 at ~40 m / 9.5 m/s) stay at the mild floor. Rapid close / near-bumper / FCW keep full −a. No panda flash.

NAP town look-ahead for reverse-digitized / short first-way 30s (2026-09-19)
========================
* Map Lookahead Early: `nextSpeedLimit` now arms for along-route town drops when OSM digitized the street opposite travel, and when the first 30 piece is only ~250 ft but the contiguous same-limit run is the real town. Side-road / stub blips under ~250 ft still ignored. No panda flash.

NAP port of #192 flash tip onto nap-dev (2026-09-18)
========================
* Tonight ship to **nap-dev** of the stacked flash tip through **#192** (`c3570c957`). Same Pre-AP longitudinal behavior as `cursor/alead-opening-gate-bc6e`. No panda flash. Hypermile / Hill Climb / map-climb-owns-follow kept.
* **#187** Accel-owns-catch-up + MPC mild floor + city/hwy Follow Distance.
* **#188** skip MPC −MILD floor when closing / lead braking; far mild close must not steal cruise +a.
* **#189** analog `DI_pedalPos` as `gasDEPRECATED`; SET-while-gas arms long.
* **#190** hard-block rematch / cruise +a while closing on a live or held lead; farther radar lead ID.
* **#191** settle rematch deadband + Accel-proportional gap hunt; hwy FD from ~30 mph when MAX > 50.
* **#192** do not match aLeadK on far opening leads (slack ≳ 20 m / gap opening).

NAP One-Pedal SET-while-gas lift keeps long (2026-09-18)
========================
* **One-Pedal Long On:** double-SET (or one SET) **while the foot is already on gas**, then lift, must **keep `enableLongControl`**, hold MAX, and **ACQUIRE** the interceptor on that falling edge — not dive into stock Tesla one-pedal regen. #185's pause-gate DI **> 1** extra kick treated lift-through DI 1–2 as a from-rest tip-in (phantom rising edge after stock `gasPressed` fell), dropped long, zeroed cruise speed, and left `aTarget` climbing while `aEgo` was stock regen (~−1.5). Log: `1c95345a3286a5db|000000dc--9b28358130` 13:16:14 (SET→lift **197 ms**), 13:15:30 (7.5 s regen hole), 13:17:19 success when DI dropped through 1 in one sample. **Now:** orig kick takes `interceptor_di`; `_one_pedal_armed_with_gas` holds lift-to-start until DI is fully off; overlay skips extra kick on falling edge / armed. Light tip-in after a real rest still pauses. Toggle default **Off**.

NAP Accel-owns-catch-up + city/hwy Follow Distance (2026-09-17)
========================
* **Accel 1–10 owns all +a**, including lead gap-close: same Mannerisms gradient as open-road / MAX climb (Accel 1 baby-steps the last ~5 mph; Accel 7–10 stay brisk). No separate 0.20/0.30/0.50 catch-up curve. Close-cap covers the Bosch window (**~200 m**) so a 160–180 m lead does not punch cruise 1.6. Brief hold on `leadOne` flicker. **Hard/rapid regen** (0.55) waits for **4 consecutive** in-window samples (~0.20 s); mild ease is immediate. **Non-rapid MPC −a is floored at mild (~0.22)** so min(MPC, overlay) cannot dump ~−2.5; FCW / rapid / a real stop still own danger.
* **City vs highway Follow Distance** on Driving Mannerisms (TICI + mici): City (<~50 mph) and Highway (>~50 mph), 1–7. Fluid blend with hysteresis around 50 — not a step. Dropping below 50 while following, ego is slightly slower than the lead so the gap opens to city FD, then holds. Rising above 50, t_follow creeps toward hwy FD. Only with a valid lead + long engaged. Migrates from single `NAPFollowDistance`. **Not Hypermile** — available whenever Mannerisms FD is used. Stalk tip steps the active band. Hypermile / Hill Climb / map-climb-owns-follow kept. No panda flash.

NAP follow soften + One-Pedal pause DI>1 (2026-09-17)
========================
* Port of the nap-release **#181 stack** plus pause sensitivity: **mild / normal closes stay light** (~0.22 m/s²) instead of the TTC-floor 0.26 / short-TTC 0.55 path. **Rapid / dumping** only (`v_rel` ≥ ~13 mph) still uses 0.55. Overlay **slew both ways**; near-gap opening rematch **+a trickles** (0.08 / 0.12). Large-gap lead-close cap stays **0.20 / 0.30 / 0.50**. **One-Pedal SET resume** (pause latch is one SET, including SET-while-gas; opendbc skip_resume). Pause gate interceptor DI **> 1** (stock gasPressed stays **> 2**). Hypermile / Hill Climb / map-climb-owns-follow unchanged. No panda flash.

NAP revert tip-brake soft-glide (2026-09-15)
========================
* **#157 reverted:** Pre-AP silent long pause brake cancel is **firm/stock again** (#156-era RELEASE). When `real_brake_pressed` drops software long, interceptor **RELEASEs immediately** to Tesla regen — no tip/hold classifier, no −0.30 start, no ~2.5 s comfort-shaped glide. On-car: the soft cancel felt worse than stock regen. **Kept:** One-Pedal Long (#170–#172) accelerator pause + SET latch, #152 A+B and #153 A3 gas-lift climb. Full cancel / FCW / AEB / hard lead unchanged. No `GAS_COMMAND` rewrite, no PostEngageCoast / climb latch / pedal-hold, no locationd or safety change.

NAP One-Pedal Long pause holds through lift (2026-09-15)
========================
* Follow-up to #171 after on-car: **One-Pedal Long On + long already holding + gas takeover** still **pauses** long (lat stays). **Lift / coast / regen must not resume** — Justin took over at ~60, decelerated to ~50, then long climbed back (A+B / A3). #171 only latched on a single interceptor `gasPressed` rising edge; stock `gasPressedOverride` ending still makes `CC.longActive` true, and a missed/soft edge left `enableLongControl` true so lift ACQUIREd. **Now:** sticky `_one_pedal_had_long_at_rest` + controller `_saw_long_without_gas` latch any gas after long was holding with the foot off (not only that one edge). Latch holds `enableLongControl` false and blocks interceptor ACQUIRE until a **single SET**. **Keep:** foot already on the throttle + SET still arms long; lift starts A+B/A3. Brake pause resume stays as today (firm/stock RELEASE). Toggle default **Off**. **nap-dev only — no nap-release twin until Justin signs off.**

NAP One-Pedal Long SET-to-resume latch (2026-09-15)
========================
* Follow-up to #170 after on-car: **One-Pedal Long On + long already active + gas from rest** still **pauses** long (lat stays, not `USER_DISABLE`). That pause is now a **held latch** (`_one_pedal_pause_latched`) until a **single SET**. **Lift alone must not resume** long / re-ACQUIRE — stock `gasPressedOverride` ending makes `CC.longActive` true again; the latch blocks interceptor ACQUIRE and re-drops long if it comes back without SET. **Keep:** foot already on the throttle + one SET (or two) still arms long; lift starts A+B/A3. Brake pause resume stays as today (this latch is gas-pause only). Toggle default **Off**. **nap-dev only — no nap-release twin until Justin signs off.**

NAP One-Pedal Long (2026-09-15)
========================
* Settings → NAP → **Driving Mannerisms** → **One-Pedal Long** (`NAPOnePedalLong`, default **Off**). Pedal-interceptor Pre-AP only. **On + OP long already active + accelerator rising from rest** (interceptor `gasPressed`, not `DI_pedalPos`): **pauses longitudinal the same way brake does today** — long lets go, lateral stays, sticky MAX, one SET resumes, no `pedalCruiseDisabled` chime. Not `USER_DISABLE` / full session cancel / take-control (`EventName.pedalPressed` is suppressed while this toggle is On, even if DisengageOnAccelerator is On). **SET while the foot is already on the accelerator** (one-stalk or two-stalk) still **arms long**; lift starts the stock A+B grace expire + aEgo seed / A3 climb — One-Pedal Long does not pause or block that handoff. After a from-rest pause the interceptor **RELEASEs** (`ENABLE=0`, command=0) and **stays RELEASED on lift** so Tesla physical pedal / stock lift-regen is the one-pedal path (same envelope as `REGEN_MAX` ≈ −1.5 m/s²). Resume-long after pause matches brake pause (sticky MAX + one SET). Limits: do **not** re-ACQUIRE on lift after a from-rest pause (that is the ENABLE 0↔1 chatter from earlier `GAS_COMMAND` overlays), do **not** rewrite driver pedal DI while ENABLE=1, cannot invent friction beyond interceptor regen. **Off** = stock: gas overrides then A+B/A3 resume climb. Standstill wait-for-gas resume, A+B/A3 engage-while-gas, firm brake cancel, FCW/AEB / #163–#167 / #169 unchanged. **nap-dev only — no nap-release twin until Justin signs off.**

NAP lead-approach earlier normal-close ease (2026-09-15)
========================
* Pre-AP lead-approach **normal closes** ease **earlier and a bit stronger**, so Justin does not ride up then take a late 0.55 / MPC bite. Comfort peak stays **0.55** (Early map). FCW / AEB / MPC hard brake untouched. Follow Distance stalk **1–7** still owns the settle gap (maneuver Follow 1 stays **23.5 m ± 0.5**). Tip-brake glide (#157) and #163/#164/#165/#167 are not this path.
* **Why it was soft:** `a = −v_rel² / (2 · slack)` at far slack is a nibble (|a| ~0.06–0.13). Catch-up +a ignored that nibble, so ego kept pulling in until the 0.55 peak. **Tune (normal close):** when time-to-follow-gap (`slack / v_rel`) is **≤ 20 s** *and* kinematic |a| is already past **0.15**, command at least **0.26** m/s² (or `v_rel / 12` if smaller) so regen is felt. Do **not** boost a far/catch-up nibble — that parked Follow 1 ~0.6 m long of the selected gap. Bleed tapers as `v_rel` drops. Far slack (TTC **> 20 s**, or |a_kin| still a nibble) stays the light kinematic, not a hard early brake.
* **Rapid / dumping:** `v_rel ≥ 6.0` m/s (~13 mph) or TTC **≤ 8 s** keeps kinematics up to the **0.55** peak (already above the light floor). Clear-close skip is **2.0** m/s (~4.5 mph, was 2.5) so a 5 mph close starts at first reliable radar. Need-path head-start **28 s** (was 24). A nibble must not steal Accel-1 rematch / Follow 1–7 catch-up. Enter/exit **0.55 / 0.20**, slew **0.05**/frame, Bosch **200 m** ceiling unchanged. **nap-dev only — no nap-release twin until Justin signs off.**

NAP no lat re-engage below 10 mph (2026-09-15)
========================
* Soft-lat **On**: the driver-turn blinker **no re-enable** gate (`lat_reenable_inhibited`) also applies whenever **v_ego < 10 mph** (`LAT_REENABLE_MIN_V_EGO_MPH`, strictly below, mph→m/s). Keep control if we still have it. Already yielded / blending / lat-down stay yielded — do not finish a take-back blend. After speed crosses 10 mph, resume is yield + the normal **0.15 s** hands-off confirm + **1 s** blend. Blinker-on still blocks take-back at any speed. Above 10 mph with the blinker off, prior resume is unchanged. Soft-lat **Off**, ALC tip/hold, long / follow, FCW / AEB, and #163/#164/#165 are not this path. **nap-dev only — no nap-release twin until Justin signs off (batching a release group).**

NAP DM look-away pause below 2 mph (2026-09-15)
========================
* Looking-away / distraction / eyes-off-road DM alerts do **not** fire below **2 mph** (`DM_LOOKAWAY_GATE_MPH`), even if **Simulate Look** and/or **False Alert Ignore** are On. Same stock standstill pause (hold before the green prompt; recover if already orange), plus creeping at a light — Tesla `CS.standstill` is only true when fully stopped, so 1 mph still nagged. Toggles stay as set; this is a speed gate, not Sim Look / FAI Off. Above 2 mph, Sim Look / FAI / stock 3 / 5 / 11 s are unchanged. FCW / AEB / hard cancels (hands-on ≥ 2 / stalk / door / reverse) unchanged. **nap-dev only — no nap-release twin until Justin signs off after a road test.**

NAP tip-brake soft-glide (2026-09-14)
========================
* **Tip-brake comfort ramp (reverted):** Pre-AP silent long pause classified digital Applied like the stalk. A **short tip** (Applied < **0.30 s**, then lift, not firm `aEgo`) that knocked software long off kept interceptor ENABLE and ran a **comfort-shaped regen ramp** over **~2.5 s**: gentle first (`a ≈ −0.30`), then quadratic ease-in to `REGEN_MAX`. **Reverted — Justin asked for firm/stock brake cancel again** (same as #156). One-Pedal Long (#170–#172) is the accelerator path and stays.

NAP revert soft brake-cancel regen (2026-09-14)
========================
* **R1 reverted:** Pre-AP silent long pause brake cancel is firm/stock again. When `real_brake_pressed` drops software long, interceptor **RELEASEs immediately** to Tesla regen — no tip/hold classifier, no 0.50 s coast, no 0.75 s 0→stock ramp. On-car feedback: engage/accel (A+B + A3) felt better; the soft cancel made braking worse. **Kept:** #152 A+B (expire engage grace on gas→long + seed last non-negative `aEgo`) and #153 A3 (`max(aEgo, planner climb)` so Mannerisms Acceleration 1–10 drives open-road climb). Full cancel / FCW / AEB / hard lead unchanged. No `GAS_COMMAND` rewrite, no PostEngageCoast / climb latch / pedal-hold, no locationd or safety change. **Current again** after #157 soft-glide was also reverted.

NAP regen ramp on light brake-cancel (2026-09-14)
========================
* **R1 follow-up (reverted):** Pre-AP silent long pause: a **short/light** brake tap that knocks software long off now **ramps interceptor regen** from coast (`accel_request=0`) to stock (`REGEN_MAX` −1.5 m/s²) over **0.75 s** (tunable 0.5–1.0), then RELEASEs. ENABLE stays on; VDAS interpolates commanded accel so pedal DI walks into regen. **Not** a 0.50 s coast plateau then sudden stock bite, and **not** full regen on the first frame. Distinguishes tip vs hold like the stalk: Applied shorter than **0.30 s** without firm `aEgo` is a tip; a **held** pedal or **deeper** decel (`aEgo <= −1.5`) RELEASEs immediately so friction + stock regen stay firm. A3 Accel 1–10 gas-lift climb from #153 is unchanged. Full session cancel / FCW / AEB / hard lead braking unchanged. No `GAS_COMMAND` DI rewrite, no PostEngageCoast / climb latch, no locationd change. **Reverted — Justin asked for firm/stock brake cancel.**

NAP soft regen + gas-lift Accel climb (2026-09-14)
========================
* **R1 (reverted):** Pre-AP silent long pause: a **short/light** brake tap that knocks software long off now eases toward **coast** for **0.50 s** (tunable 0.3–0.8) before the interceptor RELEASEs to stock Tesla regen. Distinguishes tip vs hold like the stalk: Applied shorter than **0.30 s** without firm `aEgo` is a tip; a **held** pedal or **deeper** decel (`aEgo <= −1.5`) RELEASEs immediately so friction + stock regen stay firm. Full session cancel / FCW / AEB / hard lead braking unchanged. **Superseded, then fully reverted** (coast/ramp both felt worse than stock regen).
* **A3 (kept):** After gas→long handoff, open-road VDAS seed is `max(aEgo, planner climb)` so takeover uses Mannerisms **Acceleration** 1–10 (`NAPMapSpeedAccel`) toward MAX — not only the post-lift `aEgo`. Personality / MAX still caps. Lead hard decel / FCW / should-stop (`actuators.accel < 0`) still seed 0. No `GAS_COMMAND` DI rewrite, no PostEngageCoast / climb latch. **nap-dev only — no nap-release twin until Justin signs off.**

NAP gas-lift long handoff (2026-09-13)
========================
* After engage-on-gas then lift (or a gas override while software long is already on), Pre-AP no longer restarts the 0.5 s `ENGAGE_GRACE` a=0 floor, and VDAS is seeded from the last non-negative `aEgo` (MAX-capped) instead of `commanded_accel=0`. Lead hard decel / FCW / should-stop and brake still win. First engage without gas is unchanged (grace still ramps from 0). No `GAS_COMMAND` rewrite, no PostEngageCoast / climb-to-MAX / pedal-hold overlay, no panda or locationd change. **nap-dev only — no nap-release twin until Justin signs off.**

NAP Acceleration settings placement (2026-09-13)
========================
* Settings → NAP → **Driving Mannerisms** now has **Acceleration** 1–10 (`NAPMapSpeedAccel`, default 5). Same temperament as before: scales MAX climb / open-road accel feel (1 lazy → 10 quicker); map brake to a lower MAX stays Accel 5; lead still owns follow. Removed from Map Speed Limit so it sits with Adaptive Accel. Mannerisms order is accel feel → follow → soft lat → Hypermile; Map Speed is mode/offset/lookahead → refresh/download → status. Shorter settings copy. Hypermile still snaps/restores Accel 1. No longitudinal math change.

NAP post-engage overlay stack revert (2026-09-13)
========================
* Removed the whole post-engage overlay stack for driveability: climb-to-MAX / climb-sustain (#144, #146), last-pressed pedal hold (#142), and the 1 s speed-coast (#136). Those paths added delay and an intrusive climb/pedal/`ENABLE` rewrite that surged / pulsed. Handoff is stock OP long again (no `post_engage_coast.py` / `preap_post_engage_hold.py`). Will revisit later with a lighter approach. Locationd safety unchanged.

NAP Follow Distance HUD hold (2026-09-13)
========================
* Stalk Follow Distance toast is held **1.5 s** (WARNING + PERMANENT) and a tip already at **1 or 7** still shows **Follow Distance: N** (`NAPFollowHudPending`). Fixes intermittent missing HUD when the one-frame param poll lost to another alert or a no-op write.

NAP Simulate Look full wipe restore (2026-09-13)
========================
* **Simulate Look On** is again the pre–False Alert Ignore full looking-path wipe on the existing **1–3 s** cadence (no-face / uncertain / phone / pose / eye). The FAI split had narrowed it to a no-face glance that still let phone/pose/eye nag. **False Alert Ignore On** (Sim Look Off) stays phone-only soft-clear; pose/eye still drain. Mutex unchanged. Hard cancels (hands-on ≥ 2 / stalk / door / reverse) unchanged. nap-dev defaults stay Simulate Look **On** / FAI **Off**. Same behavior as nap-release (defaults differ).

NAP Simulate Look / False Alert Ignore exclusive (2026-09-13)
========================
* Triple-tap **Simulate Look** and **False Alert Ignore** are **mutually exclusive** — only one may be On (both Off is allowed). Enabling one in the 3X / mici popup clears the other live. Turning FAI On aborts an in-flight glance hold; turning Simulate Look On stops the phone soft-clear path. DM/policy also resolve a stale both-On (old installs) on first read: **Simulate Look On / FAI Off**. nap-dev defaults: Simulate Look **On**, FAI **Off**. nap-release: both **Off**. Pose/eye timers and yaw thresholds unchanged.

NAP False Alert Ignore (2026-09-13)
========================
* Triple-tap NAP popup is now **Force Offroad**, **Simulate Look**, **False Alert Ignore** (third item). New `NAPDmFalseAlertIgnore` (default **Off** on nap-dev so it is not On with Simulate Look; persistent like Simulate Look). While engaged, false **phone/device** distraction (`phoneProb` / phone bit) soft-clears on the same random **(1.0 s, 3.0 s]** cadence so a false “Driver Distracted” can recover without a real glance. **Pose** and **eye** still drain and alert — no hold / no awareness reset while those are alarming. Simulate Look no longer full-wipes `driver_distracted` (that was masking pose/eye); it only injects a no-face / uncertain glance. Simulate Look On + False Alert Ignore Off = stock phone detection. Hands-on ≥ 2 / stalk / door / reverse unchanged. Not a nap-release default-On change.

NAP Force Offroad stock-CC handoff (2026-09-13)
========================
* Pre-AP pedal / software long (`enableLongControl`): Force Offroad first shows a big on-road **Yes / No** (**Ready to resume steering control?**). No cancels the toggle and leaves assist as-is. Yes then holds `started` until stock Tesla CC is **ENABLED** at current speed (CANCEL toward STANDBY, drop OP long, SET_ACCEL). Avoids the hard regen bite when OP long dropped with nothing holding speed. Lat-only / stock-CC / not engaged: Yes still required on-road, then today's immediate offroad. Parked: no popup. On OP long engage, stock CC is canceled off if it was ENABLED or STANDBY so it does not fight the pedal. 2.5 s card / 3.0 s hardwared fallback if DI never takes SET (timeout starts only after Yes).

NAP lead-follow comfort (2026-09-13)
========================
* Pre-AP lead-approach enter is a bit higher so occasional **bump-pull** (regen bite → Accel rematch → bite) at the follow gap is less chatty. Enter `v_rel` **0.55** m/s (~1.2 mph); exit stays **0.20**. A far/gentle overlay nibble (|a| < **0.15**) must not steal Accel-1 catch-up / Follow Distance close; real ease and MPC 0/−a still use `min()`. Clear-close skip is **2.5** m/s (~5.6 mph). Comfort peak **0.55**, onset slew **0.05**/frame, slack gates, and immediate release stay. Still closes onto Follow Distance 1–7. MPC / FCW / danger braking unchanged.

NAP Follow stalk tip vs full press (2026-09-13)
========================
* Behind a radar lead, a Pre-AP full stalk press (through 1st detent to 2nd / **5 mph**) no longer also steps Follow Distance. Follow commits only when the lever returns to **IDLE** after a first-detent **tip** (1 mph) that never hit 2nd detent; that tip frame’s MAX is still undone. Raw `SpdCtrlLvr_Stat` / CruiseButtons distinguish UP_1ST vs UP_2ND (and DN). `buttonEvents` alone are not a tip — Pre-AP maps both detents to the same accel/decelCruise. No lead: stalk is still MAX only. Same tip/hold Follow rule as nap-release; Hypermile eco / Hill Climb / Step Down unchanged.

NAP lead-approach earlier ease (2026-09-13)
========================
* Pre-AP comfort ease now starts as soon as radar has **reasonable feedback** on a closing lead (`leadOne` valid and closing), not only near Follow Distance. Start ceiling **200 m** (usable Bosch; far tracks need `radar` + `modelProb` ≥ 0.5 — LeadData has no track age). Head-start **24 s**. Clearly closing (`v_rel` ≥ 1.0 m/s) eases even with large slack, still capped at **0.55** and slewed. Hysteresis from #121 stays. Catch-up +a cap stays at 140 m. MPC / FCW / map climb vs lead (#118) unchanged. No brake-light feature.

NAP lead-follow comfort (2026-09-13)
========================
* Pre-AP lead-approach overlay is smoother on a slight grade: enter/exit **hysteresis** on radar `v_rel` / slack so regen does not chatter on/off around the follow gap, comfort peak |a| is **0.55** (Early map, was Normal 0.80), and more-negative overlay `a` slews at **0.05**/frame. Off / milder `a` is immediate. MPC danger / hard brake / FCW unchanged. Map climb still does not replace MPC when a lead is present (#118). Grade-hold for no-lead uphills is unchanged. Not a nap-release change.

NAP Follow Distance stalk tip vs hold (2026-09-13)
========================
* Behind a radar lead, a Tesla stalk **tip / bump** (1 mph / 1 kph cruise step) adjusts stock Follow Distance 1–7 only and undoes that frame’s MAX / `pedal_speed` step (HUD **Follow Distance: N**). A **full press** (5 mph / 5 kph) keeps MAX +5/−5 and does not remap Follow Distance. No lead: tip and hold both still step MAX as stock. Pedal delta magnitude is the source of truth; a button-only edge without a clear 5 mph delta still counts as a tip. Hypermile does not own follow.

NAP map climb vs lead + Follow HUD (2026-09-13)
========================
* Under HUD MAX, map climb (`a_up` Accel 1–10) no longer **replaces** a non-negative MPC `aTarget` when a radar lead is valid. That overwrite pulled ego toward MAX through a slower/matched lead (hard punch while closing, then drop to ~48 and refuse to rematch 53–55). `map_track_decel` above MAX still mins in. Grade-hold `+g·sin` extras are also skipped while a lead constrains. Follow Distance HUD toast restored: stalk/settings 1–7 change shows **Follow Distance: N** for 1.5 s (`hypermileFollowChanged`, same WARNING+permanent affordance as personality). Grade-hold itself stays. Not a nap-release change.

NAP Follow Distance (2026-09-13)
========================
* Unified stock **Follow Distance 1–7** (`NAPFollowDistance`) for stalk adjustments — same whether Hypermile is On or Off. Behind a radar lead, stalk up/down writes that param so the Driving Mannerisms slider updates live, and that frame’s MAX / `pedal_speed` step is undone. No lead: stalk still steps MAX. Full stock seven including closest **1**. Removed the Hypermile-only 1–5 band, the “never stock 1” floor, the ≤50 mph forced far-gap (stock 7) override, and `NAPHypermileFollowLevel`. Follow Distance stays visible while Hypermile is On. HUD shows **Follow Distance: N**. Eco / Step Down / Adaptive Accel / Early / Accel 1 snaps stay; they no longer force follow. Grade-hold behavior is unchanged. Intentional product change: Hypermile no longer owns follow levels. Not a nap-release change.

NAP Hypermile Hill Climb gate (2026-09-13)
========================
* Hill Climb no longer treats `TRACK_TAPER` (~4.5 mph under MAX) as “near MAX”. Crest / downhill ease only when ego is **at or above MAX** (cruise − deadband). Grade hold (`+g·sin`) only when clearly under MAX — deadband / at-MAX leave the map hold (0) so we do not punch past MAX and hunt with Accel-5. Fixes Hypermile hunting ~4 mph under MAX (47–51 under a 54 MAX) with regen pulses instead of settling. **Maps-elevation lookahead is NOT included.** Never raises MAX. Lead / MPC still win. Not a nap-release change.

NAP Simulate Look label (2026-09-13)
========================
* Triple-tap NAP popup title is **Simulate Look** (was **Simulate Look-at-Road**). Same `NAPDmSimulateLooking` param and behavior. Force Offroad label unchanged. Not a nap-release change.

NAP hidden settings (2026-09-13)
========================
* **Simulate Look** and **Force Offroad / Go Offline** are no longer on the normal NAP or Driving Mannerisms lists. Triple-tap **NAP** in Settings (3 taps in a 1.0 s sliding window) opens a side popup with those two toggles only. Tap outside the card to dismiss (X / tap NAP again also work). Same params and Reset-All defaults. Soft-lat, Hypermile, Hill Climb, Step Down unchanged. mici: triple-tap the **nap** button. Not a nap-release change.

NAP Hypermile Hill Climb (2026-09-12)
========================
* Settings → NAP → Driving Mannerisms → **Hill Climb** (default **On**, hidden/inert unless Hypermile is On). IMU pitch (`orientationNED[1]`) raises Accel 1 climb authority on a real uphill so Hypermile does not sag under HUD MAX, and eases lightly on a flattening crest / downhill. **Maps-elevation lookahead is NOT included.** Never raises MAX. Lead / MPC hard brake still win. Soft-lat, blinker, sticky MAX, Step Down, lead-close, DM unchanged. Not a nap-release change.

NAP curve MAX (2026-09-12)
========================
* Sharp curve: still slow for a comfortable corner (`limit_accel_in_turns` + a temporary lat-accel MAX cap). Snapshot HUD MAX / sticky at entry; after the bend (lat accel / steer straight-ish) restore that pre-curve set. Do not permanently bounce MAX down through a turn — Hypermile eco −5 (posted 60 → steady 55) must not replace a pre-curve MAX of 60. Step Down, lead-close, soft-lat, and blinker-keep-long unchanged.

NAP Hypermile (2026-09-12)
========================
* Hypermile On + Step Down Off no longer snaps Map Speed Offset to a flat **−5** (that dropped town posted **30 → 25**). Eco offset is live from the posted/OSM limit: **0** at/under 50 mph so 30 stays 30 and 50 stays 50; linear 0 → −8 from 50 to 80 (65 → 61); **−8** at 80 (→72) and capped −8 above (90 → 82). **Step Down On** uses the same 50→80 scale, just larger: **0** at/under 50 (town 30 stays 30), **−15** at 80 (→65), cap −15 above (90 → 75). Replaces eco, no stack, no flat −15 on town limits. **Maps-only:** eco / Step Down apply only with a known OSM/posted limit; maps off, no match, or unknown posted → no invented drop (same as sticky MAX). Off still restores the saved offset slider. Early lookahead / Accel 1 / stalk 1–5 / 50 mph follow split / curve hold / lead-close / soft-lat unchanged. Not a nap-release change.

NAP blinker (2026-09-12)
========================
* A latched driver-turn blinker no longer drops / pauses longitudinal. `enableLongControl` stays true through a held stalk / flash-latched turn; lead/map braking and accel still apply. Soft-lat On/Off lat behavior, brake silent long pause + sticky MAX + one SET / double SET, ALC tip/keep-alive, reverse hard-cancel, standstill one-SET gas gate, Hypermile, and lead-close accel unchanged.

NAP lead follow (2026-09-12)
========================
* Pre-AP: closing on / coming up behind a radar lead no longer uses the cruise **1.6–0.6 m/s²** punch (Adaptive Accel used the full profile on a large gap; map Accel 1–10 only gated MAX-rise climb). Catch-up **+a** is now `lead_close_accel_ms2`: **0.20** at Accel 1, **0.30** at 5, **0.50** at 10, inside ~140 m. Still closes onto the selected Follow Distance. MPC danger / hard brake, Hypermile, sticky MAX, soft-lat, and DM unchanged. Settings → NAP → Map Speed Limit → Acceleration.

NAP Hypermile (2026-09-12)
========================
* Settings → NAP → Driving Mannerisms → **Hypermile** (default Off, nap-dev only). Comfort-biased efficiency: On snaps Adaptive Accel + Cap/Follow / −5 mph / **Early** lookahead / **Accel 1** lazy climb (early light ease, not max regen bite), then restores those knobs when Off. Soft-lat / DM / blinker unchanged. Below ~50 mph follow sits far back (stock 7 / 1.9 s); above 50 mph uses a stalk 1–5 draft band (1=closest safe 0.9 s, 5=1.7 s). With a lead, stalk up=closer and down=farther (MAX unchanged). No lead: stalk still adjusts MAX. HUD shows **Hypermile: Follow N** when the level changes. Opt-in **Step Down Speed** (default Off, inert unless Hypermile is On): Cap/Follow targets **15 mph under posted** (75→60, 55→40; no stack with eco −5). Follow stalk SET can still hold above the step-down until posted changes. Not a nap-release change.

NAP driver monitoring (2026-09-12)
========================
* Pre-AP engaged: **Simulate Look** (default On) injects a simulated glance on the **stock vision looking-path** (`face_detected` + low pose std + `driver_distraction_filter.x < 0.37`) and **holds** that attentive state until stock gradual recovery returns awareness to **1.0** (not a one-frame pulse / mute). After drain starts, wait **past 1.0 s**, then fire at a **random time in the next 2.0 s** — fire is uniform in **(1.0 s, 3.0 s]** of that countdown. After a full reset the same rule applies to the next countdown. Orange / red stay stock **5 / 11 s** if the toggle is Off. Hands-on ≥ 2 / stalk / door / reverse hard cancels unchanged. Replaces the #103 hands-on-only first-band reset. Settings → triple-tap **NAP** (not Driving Mannerisms). Not a nap-release change.

NAP settings (2026-09-12)
========================
* Settings → NAP: Adaptive Accel Limits, Follow Distance, and Soft Lateral Handoff move into **Driving Mannerisms**. Same params and behavior. Back returns to NAP (not the side Settings list). Map Speed Limit, Radar, pedal, beams, speed-sign, and EPAS stay on the main NAP list.

NAP Pre-AP reverse / gear / standstill SET (2026-09-12)
========================
* Reverse (and any gear out of Drive / door) is a **full hard cancel**: session down, sticky MAX forgotten, soft-lat reset, CANCEL spoof so panda can re-arm `controls_allowed`. After Drive returns, a normal double SET engages without Controls Mismatch or a prior disable dance. Soft-lat Off blinker pause and sticky-MAX brake/turn pause unchanged.
* One SET after a silent long pause **at a stop** does not take long / creep from 0. SET still keeps held MAX (“I want resume”); a light throttle touch then resumes at that MAX. Rolling one-SET resume and double SET / forget-sticky unchanged.

NAP driver lat handoff (2026-09-12)
========================
* Soft-lat **On**: a driver-turn blinker no longer strips lateral. Lamp latch does not clear `latActive` / force EPS free — keep control if we still have it. Soft-lat may still yield if the driver pushes. While the driver-turn blinker is latched, do not re-enable (stay yielded / do not finish a take-back blend). After it clears, resume goes through soft yield + the normal **0.15 s** hands-off confirm + **1 s** blend — no dedicated blinker rising-edge blend. Soft-lat **Off** keeps today’s blinker lat-pause so a held turn still frees the wheel. ALC tip/keep-alive, long sticky-MAX turn pause, yield thresholds, emergency hard-brake cancel, and hazards unchanged.

NAP driver lat handoff (2026-09-11)
========================
* Soft-lat hands-off confirm before take-back shortened from **0.25 s → 0.15 s** (`HANDS_OFF_CONFIRM_S`). Free-wheel yield, 1 s blend, blinker re-entry, and emergency cancel unchanged. Renewed hands-on or a firm push during the 0.15 s wait still re-yields and resets.

NAP map speed (2026-09-11)
========================
* Map MAX short-zone ignore raised from ~50 ft (15 m) to **~250 ft (76 m)** along heading. Cross-street / bleed flashes that lasted past 50 ft are now ignored. Real on-route drops that continue for hundreds of meters (US 12 tagged 50 ~760 m, DeGraff 30) still ease. Off-route bearing/class filter unchanged.

NAP driver lat handoff (2026-09-11)
========================
* Soft lateral handoff is **default On**. After a free-wheel yield, stay free while `handsOnLevel >= 1` or a renewed firm push. When hands go to **0**, require **~0.25 s** hands-off confirm (`HANDS_OFF_CONFIRM_S`) before the **1 s** smoothstep blend — not the old 80 ms confirm. A brief hands-off or right-then-left crossover during the dodge must not snatch. Renewed hands-on or ≥ 0.55 Nm during that wait or the blend re-yields (EPS free) and resets the delay. `QUIET_WAIT_S` stays 0 (torsion-quiet used to blend on mid-dodge dips). Entry ~0.55 Nm / 90 ms + hands, blinker 1 s soft re-entry, emergency hard-brake cancel, sticky MAX / light brake, gravel spike rejection unchanged. Settings → NAP → Soft Lateral Handoff default **On**. Panda / hands-on ≥ 2 unchanged.

NAP map speed (2026-09-11)
========================
* Map MAX ignores a lower OSM limit that only lasts ~50 ft (15 m) along heading — cross-street bleed / intersection stubs. Real on-route drops that continue past that still ease. Off-route bearing/class filter unchanged.

NAP driver lat handoff (2026-09-11)
========================
* Soft lateral handoff is **default On**. **Yield = free the EPS**, not follow-measured angle control. A light purposeful push + hands (`|torsion| >= 0.55 Nm` for ~90 ms consecutive, `handsOnLevel >= 1`) drops `latActive` so Pre-AP sends `DAS_steeringControlType=0` — same release as blinker lat-pause. #79 still wrestled because OP kept full authority until the 0.70 Nm / 140 ms debounce finished, then kept `latActive` and commanded measured angle (closed-loop hold). Near 1.0 Nm the consecutive bar drops toward 60 ms so the soft path still beats hands-on ≥ 2. Hands-on hold + pin-to-wheel + **1 s blend** on return unchanged (parking-lot blinker soft re-entry kept: pin while lat down; 1 s blend on blinker rising edge; yield / stay free if hands still on). Gravel spike trains still gap-reset; rumble at ~0.50 Nm or hands-off must not free-yield. **Emergency / hard brake** while yielded or within 2 s of yield entry (digital Applied + `aEgo <= −3.5 m/s²` for 80 ms) fully cancels OP. Light brake is still sticky-MAX silent pause + one SET. Settings → NAP → Soft Lateral Handoff can turn **Off**. Panda / hands-on ≥ 2 unchanged.

NAP driver lat handoff (2026-09-11)
========================
* Soft lateral handoff is **default On**. Yield on a **sustained driver push + hands** (`|torsion| >= 0.70 Nm` for ~140 ms consecutive, `handsOnLevel >= 1`). Steer rate is **not** required — an isometric fight (low rate, high tracking error) soft-yields instead of holding until hands-on ≥ 2 / `STEER_THRESHOLD` hard cancel. Near 1.0 Nm the consecutive bar drops toward 80 ms so the soft path wins first. Disturbance veto only when torsion is **below** 0.70 Nm (wind / crown). Gravel spike trains still gap-reset. Hands-on hold + 1 s hands-off blend + curvature pin unchanged. **Emergency / hard brake** while yielded or within 2 s of yield entry (digital Applied + `aEgo <= −3.5 m/s²` for 80 ms) fully cancels OP. Light brake is still sticky-MAX silent pause + one SET. Settings → NAP → Soft Lateral Handoff can turn **Off**. Panda / hands-on ≥ 2 unchanged.

NAP driver lat handoff (2026-09-11)
========================
* Soft lateral handoff is **default On** and now yields only on **driver intent** to turn the wheel: sustained torsion (≥ 0.70 Nm / 140 ms) **and** matching steer rate **and** hands on the rim. Gravel spikes, opposite-rate pressure, and high tracking error without matching torsion (wind / crown) do not yield. Hands-on hold + 1 s hands-off blend unchanged. **Emergency / hard brake** while yielded or within 2 s of yield entry (digital Applied + `aEgo <= −3.5 m/s²` for 80 ms) fully cancels OP with the normal disengage chime — not the silent long pause. Light brake is still sticky-MAX silent pause + one SET. Settings → NAP → Soft Lateral Handoff can turn **Off**. Panda / hands-on ≥ 2 unchanged.

NAP driver lat handoff (2026-09-11)
========================
* Soft lateral handoff is **default On**. Yield stays ~0.70 Nm held 0.14 s consecutive (gap reset; not 0.5 Nm / 80 ms rumble). Stay yielded while `EPAS_handsOnLevel >= 1` so a mid-dodge torsion dip does not start hand-back. 1 s blend starts only after hands are truly off the rim for ~80 ms. Hands back on or a firm push cancels the blend and re-yields. Planner still pins to the wheel while yielded / lat down so resume tracks the path. Blinker-turn re-entry is still the same 1 s blend from the wheel. Settings → NAP → Soft Lateral Handoff: turn **Off** if gravel or wind still gray the chrome. Brake silent long-pause + one SET is unchanged. Panda / hands-on ≥ 2 hard cancel unchanged.

NAP driver lat handoff (2026-09-11)
========================
* Soft lateral handoff is **default On**. Firmer yield: ~0.85 Nm held 0.25 s consecutive (not 0.5 Nm rumble). After a yield, planner curvature stays on the wheel so resume actually tracks the path (green HUD + on-screen path was not enough). Settings → NAP → Soft Lateral Handoff: turn **Off** if gravel or wind still gray the chrome. Brake silent long-pause + one SET is unchanged.

NAP driver lat handoff (2026-09-11)
========================
* Light wheel input (about half the usual override effort) now yields steering without canceling openpilot. Speed control stays on. After 0.25 s of quiet the wheel blends back over 1 s. Gray HUD until ~70% lateral is back; no disengage chime. A hard yank / cancel still fully disengages. No panda flash.

NAP sticky MAX (2026-09-11)
========================
* Pedal mode: brake or a held/latched driver-turn blinker pauses longitudinal only. Held MAX is remembered. That pause is **silent** — no “Pedal Cruise Disengaged” / disengage chime. One SET resumes long at that MAX (already rebased if posted changed) without a full-stack engage fanfare. Double SET forgets sticky: maps on + posted known → current posted; maps off / unknown → current traveled speed. Same double SET is initial engage. Maps never overwrite sticky every frame; GPS drop does not invent or wipe posted. Tip ALC does not use this pause. Hard cancel (stalk / door / gear / steer fault / hands-on ≥ 2) still fully disengages and still plays the disengage prompt. No-pedal stock CC is unchanged. Soft lat handoff unchanged.

NAP speed-sign log (2026-09-11)
========================
* SIGN WAIT is only when openpilot is actively controlling. A 1 Hz SubMaster used to treat unknown / stale `selfdriveState` as engaged for ~10 s after cancel (`recv_frame <= 0` + 100 ms alive window). Poll at 20 Hz; gate on `active`; abandon in-flight ONNX on re-engage so the loop never blocks. If TAKE CONTROL / Communication Issue comes back, turn Logger Off.

NAP speed-sign log (2026-09-10)
========================
* Speed Sign Logger On no longer runs YOLO while openpilot is engaged (SIGN shows WAIT). Manual driving — moving OK — still logs signs at 1 Hz. If TAKE CONTROL comes back, turn Logger Off.

NAP blinker (2026-09-10)
========================
* A held/latched driver turn blinker now drops longitudinal as well as pausing lat. After the lamps/latch end, one stalk SET restores long (lat still paused or already active). ALC tip/keep-alive does not drop long. Stalk cancel still fully disengages; brake can still drop long. No panda flash required.

NAP blinker (2026-09-09)
========================
* Panda tesla_preap now keeps controls_allowed on a driver blinker-held turn (one lamp or LEFT/RIGHT, flash-latched ~1s) so hands-on ≥ 2 does not controlsMismatch after the turn. **Flash the panda after this update** (reboot so pandad reflashes; Python-only pull is not enough). Stalk cancel / doors / gear still drop.
* Held-blinker turns no longer full-cancel mid-corner. Higher steering torque / EPAS hands-on ≥ 2 while lat is paused stays engaged (lat pause only). The 3X "Steering Disengaged" HUD is EventName.pcmDisable (cruiseEnabled fell) — keep cruise up on lamp or held stalk, and do not treat that torque as controlsMismatch / steerDisengage. Stalk cancel / doors / gear / permanent steer fault still fully disengage.

NAP Force Offroad (2026-09-09)
========================
* Settings → NAP → Force Offroad / Go Offline. Temporarily drops the device into offroad (started=false) while the car is still moving so Download US Maps, Refresh maps, and software install unlock. Stops openpilot — drive manually. Default Off. Clears on toggle Off, Reset to Defaults, reboot, or the next ignition ON.

NAP speed-sign log (2026-09-09)
========================
* Optional on-drive MUTCD speed-sign logger (default off). ROAD camera + GPS, JSONL under /data. On-road SIGN plate shows the live mph for 1.5s. Does not change cruise or maps.

NAP blinker (2026-09-09)
========================
* During ALC (or leftover keep-alive), holding the physical stalk the same direction for more than 1.0s cancels the lane change and pauses steering as a driver turn. A shorter same-direction press does not force a turn. Tip-to-ALC from idle is still LEFT/RIGHT then IDLE within 0.40s.
* Tip vs turn no longer depends on speed. A partial stalk push (LEFT/RIGHT then IDLE within 0.40s) is automatic lane change at any speed; a full held stalk is a driver turn and releases steering. Lane change still will not *start* below 20 mph until a wheel nudge at speed.

NAP map speed (2026-09-08)
========================
* Refresh maps queries live OSM within 100 miles of the car and merges into the installed US sqlite. Download US Maps remains the first-install of the published pack.

NAP map speed (2026-09-07)
========================
* OSM map speed can drive HUD MAX (pedal mode). Off by default. Download US maps from Settings → NAP → Map Speed Limit (ODbL). Map speed sets the cruise ceiling; radar still follows a slower lead.

NAP 2026-09-02
========================
* Fix card crash on clean Comma install (Tesla radar DBC)

Version 0.11.1 (2026-05-18)
========================
* New driver monitoring model
* Improved image processing pipeline for driver camera
* Improved thermal policy for comma four
* Acura MDX 2022-24 support thanks to mvl-boston!
* Rivian R1S and R1T 2025 support thanks to lukasloetkolben!

Version 0.11.0 (2026-03-17)
========================
* New driving model #36798
  * Fully trained using a learned simulator
  * Improved longitudinal performance in Experimental mode
* Reduce comma four standby power usage by 77% to 52 mW
* Kia K7 2017 support thanks to royjr!
* Lexus LS 2018 support thanks to Hacheoy!

Version 0.10.3 (2025-12-17)
========================
* New driving model #36249
  * New temporal policy architecture
  * New on-policy training physics noise model
* New driver monitoring model #36409
  * Trained on a new dataset, including comma four data
* Improved inter-process communication memory efficiency

Version 0.10.2 (2025-11-19)
========================
* comma four support

Version 0.10.1 (2025-09-08)
========================
* New driving model #36276
  * World Model: removed global localization inputs
  * World Model: 2x the number of parameters
  * World Model: trained on 4x the number of segments
  * VAE Compression Model: new architecture and training objective
  * Driving Vision Model: trained on 4x the number of segments
* New Driver Monitoring model #36198
* Acura TLX 2021 support thanks to MVL!
* Honda City 2023 support thanks to vanillagorillaa and drFritz!
* Honda N-Box 2018 support thanks to miettal!
* Honda Odyssey 2021-25 support thanks to csouers and MVL!
* Honda Passport 2026 support thanks to vanillagorillaa and MVL!

Version 0.10.0 (2025-08-05)
========================
* New driving model
  * New training architecture
     * Described in our CVPR paper: "Learning to Drive from a World Model"
     * Longitudinal MPC replaced by E2E planning from World Model in Experimental Mode
     * Action from lateral MPC as training objective replaced by E2E planning from World Model
  * Low-speed lead car ground-truth fixes
* Enable live-learned steering actuation delay
* Opt-in audio recording for dashcam video
* Acura MDX 2025 support thanks to vanillagorillaa and MVL!
* Honda Accord 2023-25 support thanks to vanillagorillaa and MVL!
* Honda CR-V 2023-25 support thanks to vanillagorillaa and MVL!
* Honda Pilot 2023-25 support thanks to vanillagorillaa and MVL!

Version 0.9.9 (2025-05-23)
========================
* New driving model
  * New training architecture using parts from MLSIM
* Steering actuation delay is now learned online
* Ford Escape 2023-24 support thanks to incognitojam!
* Ford Kuga 2024 support thanks to incognitojam!
* Hyundai Nexo 2021 support thanks to sunnyhaibin!
* Tesla Model 3 and Y support thanks to lukasloetkolben!
* Lexus RC 2023 support thanks to nelsonjchen!

Version 0.9.8 (2025-02-28)
========================
* New driving model
  * Model now gates applying positive acceleration in Chill mode
* New driver monitoring model
  * Reduced false positives related to passengers
* Image processing pipeline moved to the ISP
  * More GPU time for bigger driving models
  * Power draw reduced 0.5W, which means your device runs cooler
* Added toggle to enable driver monitoring even when openpilot is not engaged
* Localizer rewritten to remove GPS dependency at runtime
* Firehose Mode for maximizing your training data uploads
* Enable openpilot longitudinal control for Ford Q3 vehicles
* New Toyota TSS2 longitudinal tune
* Rivian R1S and R1T support thanks to lukasloetkolben!
* Ford F-150, F-150 Hybrid, Mach-E, and Ranger support

Version 0.9.7 (2024-06-13)
========================
* New driving model
  * Inputs the past curvature for smoother and more accurate lateral control
  * Simplified neural network architecture in the model's last layers
  * Minor fixes to desire augmentation and weight decay
* New driver monitoring model
  * Improved end-to-end bit for phone detection
* Adjust driving personality with the follow distance button
* Support for hybrid variants of supported Ford models
* Fingerprinting without the OBD-II port on all cars
* Improved fuzzy fingerprinting for Ford and Volkswagen

Version 0.9.6 (2024-02-27)
========================
* New driving model
  * Vision model trained on more data
  * Improved driving performance
  * Directly outputs curvature for lateral control
* New driver monitoring model
  * Trained on larger dataset
* Model path UI
  * Shows where driving model wants to be
  * Shows what model is seeing more clearly, but more jittery
* AGNOS 9
* comma body streaming and controls over WebRTC
* Improved fuzzy fingerprinting for many makes and models
* Alpha longitudinal support for new Toyota models
* Chevrolet Equinox 2019-22 support thanks to JasonJShuler and nworb-cire!
* Dodge Durango 2020-21 support
* Hyundai Staria 2023 support thanks to sunnyhaibin!
* Kia Niro Plug-in Hybrid 2022 support thanks to sunnyhaibin!
* Lexus LC 2024 support thanks to nelsonjchen!
* Toyota RAV4 2023-24 support
* Toyota RAV4 Hybrid 2023-24 support

Version 0.9.5 (2023-11-17)
========================
* New driving model
  * Improved navigate on openpilot performance using navigation instructions as an additional model input
  * Do lateral planning inside the model
  * New vision transformer architecture
* Cadillac Escalade ESV 2019 support thanks to twilsonco!
* Hyundai Azera 2022 support thanks to sunnyhaibin!
* Hyundai Azera Hybrid 2020 support thanks to chanhojung and haram-KONA!
* Hyundai Custin 2023 support thanks to sunnyhaibin and Saber422!
* Hyundai Ioniq 6 2023 support thanks to sunnyhaibin and alamo3!
* Hyundai Kona Electric 2023 (Korean version) support thanks to sunnyhaibin and haram-KONA!
* Kia K8 Hybrid (with HDA II) 2023 support thanks to sunnyhaibin!
* Kia Optima Hybrid 2019 support
* Kia Sorento Hybrid 2023 support thanks to sunnyhaibin!
* Lexus GS F 2016 support thanks to snyperifle!
* Lexus IS 2023 support thanks to L3R5!

Version 0.9.4 (2023-07-27)
========================
* comma 3X support
* Navigate on openpilot in Experimental mode
  * When navigation has a destination, openpilot will input the map information into the model, which provides useful context to help the model understand the scene
  * When navigating on openpilot, openpilot will keep left or right appropriately at forks and exits
  * When navigating on openpilot, lane change behavior is unchanged and still activated by the driver
  * When navigate on openpilot is active, the path on the map is green
* UI updates
  * Navigation settings moved to home screen and map
  * Border color always shows engagement status. Blue means disengaged, green means engaged, and grey means engaged with human overriding
  * Alerts are shown inside the border. Black means info, orange means warning, and red means critical alert
* Bookmarked segments are preserved on the device's storage
* Ford Focus 2018 support
* Kia Carnival 2023 support thanks to sunnyhaibin!

Version 0.9.3 (2023-06-29)
========================
* New driving model
  * Improved height estimation and added height tracking in liveCalibration
  * Model inputs refactor
* New driving personality setting
  * Three settings: aggressive, standard, and relaxed
  * Standard is recommended and the default
  * In aggressive mode, lead follow distance is shorter and acceleration response is quicker
  * In relaxed mode, lead follow distance is longer
* Improved fuzzy fingerprinting for Hyundai, Kia, and Genesis
* Improved thermal management logic

Version 0.9.2 (2023-05-22)
========================
* New driving model
  * Reduced turn diving
  * Trained on a new dataset
* UI updates
  * New experimental mode visualization
  * Draw MPC path instead of model-predicted path
* AGNOS 7
  * Faster boot time
  * Fixes rare no sounds bug
  * Fixes bootsplash bug at extreme temperatures
* Buick LaCrosse 2017-19 support thanks to koch-cf!
* Chevrolet Trailblazer 2021-22 support thanks to TurboCE!
* Ford Bronco Sport 2021-22 support
* Ford Escape 2020-22 support
* Ford Explorer 2020-22 support
* Ford Kuga 2020-22 support
* Ford Maverick 2022-23 support
* Genesis GV80 2023 support thanks to JWingate80!
* Honda HR-V 2023 support thanks to AlexandreSato and galegozi!
* Kia Niro EV 2023 support thanks to JosselinLecocq!
* Lexus ES 2017-18 support
* Lincoln Aviator 2021 support
* Škoda Fabia 2022-23 support thanks to jyoung8607!


Version 0.9.1 (2023-02-28)
========================
* New driving model
  * 30% improved height estimation resulting in better driving performance for tall cars
* Driver monitoring: removed timer resetting on user interaction if distracted
* UI updates
  * Adjust alert volume using ambient noise level
  * Driver monitoring icon shows driver's head pose
  * German translation thanks to Vrabetz and CzokNorris!
* Cadillac Escalade 2017 support thanks to rickygilleland!
* Chevrolet Bolt EV 2022-23 support thanks to JasonJShuler!
* Genesis GV60 2023 support thanks to sunnyhaibin!
* Hyundai Tucson 2022-23 support
* Kia K5 Hybrid 2020 support thanks to sunnyhaibin!
* Kia Niro Hybrid 2023 support thanks to sunnyhaibin!
* Kia Sorento 2022-23 support thanks to sunnyhaibin!
* Kia Sorento Plug-in Hybrid 2022 support thanks to sunnyhaibin!
* Toyota C-HR 2021 support thanks to eFiniLan!
* Toyota C-HR Hybrid 2022 support thanks to Korben00!
* Volkswagen Crafter and MAN TGE 2017-23 support thanks to jyoung8607!

Version 0.9.0 (2022-11-21)
========================
* New driving model
  * Internal feature space information content increased tenfold during training to ~700 bits, which makes the model dramatically more accurate
  * Less reliance on previous frames makes model more reactive and snappy
  * Trained in new reprojective simulator
  * Trained in 36 hours from scratch, compared to one week for previous releases
  * Training now simulates both lateral and longitudinal behavior, which allows openpilot to slow down for turns, stop at traffic lights, and more in experimental mode
* Experimental driving mode
  * End-to-end longitudinal control
  * Stops for traffic lights and stop signs
  * Slows down for turns
  * openpilot defaults to chill mode, enable experimental mode in settings
* Driver monitoring updates
  * New bigger model with added end-to-end distracted trigger
  * Reduced false positives during driver calibration
* Self-tuning torque controller: learns parameters live for each car
* Torque controller used on all Toyota, Lexus, Hyundai, Kia, and Genesis models
* UI updates
  * Matched speeds shown on car's dash
  * Multi-language in navigation
  * Improved update experience
  * Border turns grey while overriding steering
  * Bookmark events while driving; view them in comma connect
  * New onroad visualization for experimental mode
* tools: new and improved cabana thanks to deanlee!
* Experimental longitudinal support for Volkswagen, CAN-FD Hyundai, and new GM models
* Genesis GV70 2022-23 support thanks to zunichky and sunnyhaibin!
* Hyundai Santa Cruz 2021-22 support thanks to sunnyhaibin!
* Kia Sportage 2023 support thanks to sunnyhaibin!
* Kia Sportage Hybrid 2023 support thanks to sunnyhaibin!
* Kia Stinger 2022 support thanks to sunnyhaibin!

Version 0.8.16 (2022-08-26)
========================
* New driving model
  * Reduced turn cutting
* Auto-detect right hand drive setting with driver monitoring model
* Improved fan controller for comma three
* New translations
  * Japanese thanks to cydia2020!
  * Brazilian Portuguese thanks to AlexandreSato!
* Chevrolet Bolt EUV 2022-23 support thanks to JasonJShuler!
* Chevrolet Silverado 1500 2020-21 support thanks to JasonJShuler!
* GMC Sierra 1500 2020-21 support thanks to JasonJShuler!
* Hyundai Ioniq 5 2022 support thanks to sunnyhaibin!
* Hyundai Kona Electric 2022 support thanks to sunnyhaibin!
* Hyundai Tucson Hybrid 2022 support thanks to sunnyhaibin!
* Subaru Legacy 2020-22 support thanks to martinl!
* Subaru Outback 2020-22 support

Version 0.8.15 (2022-07-20)
========================
* New driving model
  * Path planning uses end-to-end output instead of lane lines at all times
  * Reduced ping pong
  * Improved lane centering
* New lateral controller based on physical wheel torque model
  * Much smoother control that's consistent across the speed range
  * Effective feedforward that uses road roll
  * Simplified tuning, all car-specific parameters can be derived from data
  * Used on select Toyota and Hyundai models at first
  * Significantly improved control on TSS-P Prius
* New driver monitoring model
  * Bigger model, covering full interior view from driver camera
  * Works with a wider variety of mounting angles
  * 3x more unique comma three training data than previous
* Navigation improvements
  * Speed limits shown while navigating
  * Faster position fix by using raw GPS measurements
* UI updates
  * Multilanguage support for settings and home screen
  * New font
  * Refreshed max speed design
  * More consistent camera view perspective across cars
* Reduced power usage: device runs cooler and fan spins less
* AGNOS 5
  * Support VSCode remote SSH target
  * Support for delta updates to reduce data usage on future OS updates
* Chrysler ECU firmware fingerprinting thanks to realfast!
* Honda Civic 2022 support
* Hyundai Tucson 2021 support thanks to bluesforte!
* Kia EV6 2022 support
* Lexus NX Hybrid 2020 support thanks to AlexandreSato!
* Ram 1500 2019-21 support thanks to realfast!

Version 0.8.14 (2022-06-01)
========================
 * New driving model
   * Bigger model, using both of comma three's road-facing cameras
   * Better at cut-in detection and tight turns
 * New driver monitoring model
   * Tweaked network structure to improve output resolution for DSP
   * Fixed bug in quantization aware training to reduce quantizing errors
   * Resulted in 7x less MSE and no more random biases at runtime
 * Added toggle to disable disengaging on the accelerator pedal
 * comma body support
 * Audi RS3 support thanks to jyoung8607!
 * Hyundai Ioniq Plug-in Hybrid 2019 support thanks to sunnyhaibin!
 * Hyundai Tucson Diesel 2019 support thanks to sunnyhaibin!
 * Toyota Alphard Hybrid 2021 support
 * Toyota Avalon Hybrid 2022 support
 * Toyota RAV4 2022 support
 * Toyota RAV4 Hybrid 2022 support

Version 0.8.13 (2022-02-18)
========================
 * Improved driver monitoring
   * Re-tuned driver pose learner for relaxed driving positions
   * Added reliance on driving model to be more scene adaptive
   * Matched strictness between comma two and comma three
 * Improved performance in turns by compensating for the road bank angle
 * Improved camera focus on the comma two
 * AGNOS 4
   * ADB support
   * improved cell auto configuration
 * NEOS 19
   * package updates
   * stability improvements
 * Subaru ECU firmware fingerprinting thanks to martinl!
 * Hyundai Santa Fe Plug-in Hybrid 2022 support thanks to sunnyhaibin!
 * Mazda CX-5 2022 support thanks to Jafaral!
 * Subaru Impreza 2020 support thanks to martinl!
 * Toyota Avalon 2022 support thanks to sshane!
 * Toyota Prius v 2017 support thanks to CT921!
 * Volkswagen Caravelle 2020 support thanks to jyoung8607!

Version 0.8.12 (2021-12-15)
========================
 * New driving model
   * Improved behavior around exits
   * Better pose accuracy at high speeds, allowing max speed of 90mph
   * Fully incorporated comma three data into all parts of training stack
 * Improved follow distance
 * Better longitudinal policy, especially in low speed traffic
 * New alert sounds
 * AGNOS 3
   * Display burn in mitigation
   * Improved audio amplifier configuration
   * System reliability improvements
   * Update Python to 3.8.10
 * Raw logs upload moved to connect.comma.ai
 * Fixed HUD alerts on newer Honda Bosch thanks to csouers!
 * Audi Q3 2020-21 support thanks to jyoung8607!
 * Lexus RC 2020 support thanks to ErichMoraga!

Version 0.8.11 (2021-11-29)
========================
 * Support for CAN FD on the red panda
 * Support for an external panda on the comma three
 * Navigation: Show more detailed instructions when approaching maneuver
 * Fixed occasional steering faults on GM cars thanks to jyoung8607!
 * Nissan ECU firmware fingerprinting thanks to robin-reckmann, martinl, and razem-io!
 * Cadillac Escalade ESV 2016 support thanks to Gibby!
 * Genesis G70 2020 support thanks to tecandrew!
 * Hyundai Santa Fe Hybrid 2022 support thanks to sunnyhaibin!
 * Mazda CX-9 2021 support thanks to Jacar!
 * Volkswagen Polo 2020 support thanks to jyoung8607!
 * Volkswagen T-Roc 2021 support thanks to jyoung8607!

Version 0.8.10 (2021-11-01)
========================
 * New driving model
   * Trained on one million minutes!!!
   * Fixed lead training making lead predictions significantly more accurate
   * Fixed several localizer dataset bugs and loss function bugs, overall improved accuracy
 * New driver monitoring model
   * Trained on latest data from both comma two and comma three
   * Increased model field of view by 40% on comma three
   * Improved model stability on masked users
   * Improved pose prediction with reworked ground-truth stack
 * Lateral and longitudinal planning MPCs now in ACADOS
 * Combined longitudinal MPCs
   * All longitudinal planning now happens in a single MPC system
   * Fixed instability in MPC problem to prevent sporadic CPU usage
 * AGNOS 2: minor stability improvements and builder repo open sourced
 * tools: new and improved replay thanks to deanlee!
 * Moved community-supported cars outside of the Community Features toggle
 * Improved FW fingerprinting reliability for Hyundai/Kia/Genesis
 * Added prerequisites for longitudinal control on Hyundai/Kia/Genesis and Honda Bosch
 * Audi S3 2015 support thanks to jyoung8607!
 * Honda Freed 2020 support thanks to belm0!
 * Hyundai Ioniq Hybrid 2020-2022 support thanks to sunnyhaibin!
 * Hyundai Santa Fe 2022 support thanks to sunnyhaibin!
 * Kia K5 2021 support thanks to sunnyhaibin!
 * Škoda Kamiq 2021 support thanks to jyoung8607!
 * Škoda Karoq 2019 support thanks to jyoung8607!
 * Volkswagen Arteon 2021 support thanks to jyoung8607!
 * Volkswagen California 2021 support thanks to jyoung8607!
 * Volkswagen Taos 2022 support thanks to jyoung8607!

Version 0.8.9 (2021-09-14)
========================
 * Improved fan control on comma three
 * AGNOS 1.5: improved stability
 * Honda e 2020 support

Version 0.8.8 (2021-08-27)
========================
 * New driving model with improved laneless performance
   * Trained on 5000+ hours of diverse driving data from 3000+ users in 40+ countries
   * Better anti-cheating methods during simulator training ensure the model hugs less when in laneless mode
   * All new desire ground-truthing stack makes the model better at lane changes
 * New driver monitoring model: improved performance on comma three
 * NEOS 18 for comma two: update packages
 * AGNOS 1.3 for comma three: fix display init at high temperatures
 * Improved auto-exposure on comma three
 * Improved longitudinal control on Honda Nidec cars
 * Hyundai Kona Hybrid 2020 support thanks to haram-KONA!
 * Hyundai Sonata Hybrid 2021 support thanks to Matt-Wash-Burn!
 * Kia Niro Hybrid 2021 support thanks to tetious!

Version 0.8.7 (2021-07-31)
========================
 * comma three support!
 * Navigation alpha for the comma three!
 * Volkswagen T-Cross 2021 support thanks to jyoung8607!

Version 0.8.6 (2021-07-21)
========================
 * Revamp lateral and longitudinal planners
   * Refactor planner output API to be more readable and verbose
   * Planners now output desired trajectories for speed, acceleration, curvature, and curvature rate
   * Use MPC for longitudinal planning when no lead car is present, makes accel and decel smoother
 * Remove "CHECK DRIVER FACE VISIBILITY" warning
 * Fixed cruise fault on some TSS2.5 Camrys and international Toyotas
 * Hyundai Elantra Hybrid 2021 support thanks to tecandrew!
 * Hyundai Ioniq PHEV 2020 support thanks to YawWashout!
 * Kia Niro Hybrid 2019 support thanks to jyoung8607!
 * Škoda Octavia RS 2016 support thanks to jyoung8607!
 * Toyota Alphard 2020 support thanks to belm0!
 * Volkswagen Golf SportWagen 2015 support thanks to jona96!
 * Volkswagen Touran 2017 support thanks to jyoung8607!

Version 0.8.5 (2021-06-11)
========================
 * NEOS update: improved reliability and stability with better voltage regulator configuration
 * Smart model-based Forward Collision Warning
 * CAN-based fingerprinting moved behind community features toggle
 * Improved longitudinal control on Toyotas with a comma pedal
 * Improved auto-brightness using road-facing camera
 * Added "Software" settings page with updater controls
 * Audi Q2 2018 support thanks to jyoung8607!
 * Hyundai Elantra 2021 support thanks to CruiseBrantley!
 * Lexus UX Hybrid 2019-2020 support thanks to brianhaugen2!
 * Toyota Avalon Hybrid 2019 support thanks to jbates9011!
 * SEAT Leon 2017 & 2020 support thanks to jyoung8607!
 * Škoda Octavia 2015 & 2019 support thanks to jyoung8607!

Version 0.8.4 (2021-05-17)
========================
 * Delay controls start until system is ready
 * Fuzzy car identification, enabled with Community Features toggle
 * Localizer optimized for increased precision and less CPU usage
 * Re-tuned lateral control to be more aggressive when model is confident
 * Toyota Mirai 2021 support
 * Lexus NX 300 2020 support thanks to goesreallyfast!
 * Volkswagen Atlas 2018-19 support thanks to jyoung8607!

Version 0.8.3 (2021-04-01)
========================
 * New model
   * Trained on new diverse dataset from 2000+ users from 30+ countries
   * Trained with improved segnet from the comma-pencil community project
   * 🥬 Dramatically improved end-to-end lateral performance 🥬
 * Toggle added to disable the use of lanelines
 * NEOS update: update packages and support for new UI
 * New offroad UI based on Qt
 * Default SSH key only used for setup
 * Kia Ceed 2019 support thanks to ZanZaD13!
 * Kia Seltos 2021 support thanks to speedking456!
 * Added support for many Volkswagen and Škoda models thanks to jyoung8607!

Version 0.8.2 (2021-02-26)
========================
 * Use model points directly in MPC (no more polyfits), making lateral planning more accurate
 * Use model heading prediction for smoother lateral control
 * Smarter actuator delay compensation
 * Improve qcamera resolution for improved video in explorer and connect
 * Adjust maximum engagement speed to better fit the model's training distribution
 * New driver monitoring model trained with 3x more diverse data
 * Improved face detection with masks
 * More predictable DM alerts when visibility is bad
 * Rewritten video streaming between openpilot processes
 * Improved longitudinal tuning on TSS2 Corolla and Rav4 thanks to briskspirit!
 * Audi A3 2015 and 2017 support thanks to keeleysam!
 * Nissan Altima 2020 support thanks to avolmensky!
 * Lexus ES Hybrid 2018 support thanks to TheInventorMan!
 * Toyota Camry Hybrid 2021 support thanks to alancyau!

Version 0.8.1 (2020-12-21)
========================
 * Original EON is deprecated, upgrade to comma two
 * Better model performance in heavy rain
 * Better lane positioning in turns
 * Fixed bug where model would cut turns on empty roads at night
 * Fixed issue where some Toyotas would not completely stop thanks to briskspirit!
 * Toyota Camry 2021 with TSS2.5 support
 * Hyundai Ioniq Electric 2020 support thanks to baldwalker!

Version 0.8.0 (2020-11-30)
========================
 * New driving model: fully 3D and improved cut-in detection
 * UI draws 2 road edges, 4 lanelines and paths in 3D
 * Major fixes to cut-in detection for openpilot longitudinal
 * Grey panda is no longer supported, upgrade to comma two or black panda
 * Lexus NX 2018 support thanks to matt12eagles!
 * Kia Niro EV 2020 support thanks to nickn17!
 * Toyota Prius 2021 support thanks to rav4kumar!
 * Improved lane positioning with uncertain lanelines, wide lanes and exits
 * Improved lateral control for Prius and Subaru

Version 0.7.10 (2020-10-29)
========================
 * Grey panda is deprecated, upgrade to comma two or black panda
 * NEOS update: update to Python 3.8.2 and lower CPU frequency
 * Improved thermals due to reduced CPU frequency
 * Update SNPE to 1.41.0
 * Reduced offroad power consumption
 * Various system stability improvements
 * Acura RDX 2020 support thanks to csouers!

Version 0.7.9 (2020-10-09)
========================
 * Improved car battery power management
 * Improved updater robustness
 * Improved realtime performance
 * Reduced UI and modeld lags
 * Increased torque on 2020 Hyundai Sonata and Palisade

Version 0.7.8 (2020-08-19)
========================
 * New driver monitoring model: improved face detection and better compatibility with sunglasses
 * Download NEOS operating system updates in the background
 * Improved updater reliability and responsiveness
 * Hyundai Kona 2020, Veloster 2019, and Genesis G70 2018 support thanks to xps-genesis!

Version 0.7.7 (2020-07-20)
========================
 * White panda is no longer supported, upgrade to comma two or black panda
 * Improved vehicle model estimation using high precision localizer
 * Improved thermal management on comma two
 * Improved autofocus for road-facing camera
 * Improved noise performance for driver-facing camera
 * Block lane change start using blindspot monitor on select Toyota, Hyundai, and Subaru
 * Fix GM ignition detection
 * Code cleanup and smaller release sizes
 * Hyundai Sonata 2020 promoted to officially supported car
 * Hyundai Ioniq Electric Limited 2019 and Ioniq SE 2020 support thanks to baldwalker!
 * Subaru Forester 2019 and Ascent 2019 support thanks to martinl!

Version 0.7.6.1 (2020-06-16)
========================
 * Hotfix: update kernel on some comma twos (orders #8570-#8680)

Version 0.7.6 (2020-06-05)
========================
 * White panda is deprecated, upgrade to comma two or black panda
 * 2017 Nissan X-Trail, 2018-19 Leaf and 2019 Rogue support thanks to avolmensky!
 * 2017 Mazda CX-5 support in dashcam mode thanks to Jafaral!
 * Huge CPU savings in modeld by using thneed!
 * Lots of code cleanup and refactors

Version 0.7.5 (2020-05-13)
========================
 * Right-Hand Drive support for both driving and driver monitoring!
 * New driving model: improved at sharp turns and lead speed estimation
 * New driver monitoring model: overall improvement on comma two
 * Driver camera preview in settings to improve mounting position
 * Added support for many Hyundai, Kia, Genesis models thanks to xx979xx!
 * Improved lateral tuning for 2020 Toyota Rav 4 (hybrid)

Version 0.7.4 (2020-03-20)
========================
 * New driving model: improved lane changes and lead car detection
 * Improved driver monitoring model: improve eye detection
 * Improved calibration stability
 * Improved lateral control on some 2019 and 2020 Toyota Prius
 * Improved lateral control on VW Golf: 20% more steering torque
 * Fixed bug where some 2017 and 2018 Toyota C-HR would use the wrong steering angle sensor
 * Support for Honda Insight thanks to theantihero!
 * Code cleanup in car abstraction layers and ui

Version 0.7.3 (2020-02-21)
========================
 * Support for 2020 Highlander thanks to che220!
 * Support for 2018 Lexus NX 300h thanks to kengggg!
 * Speed up ECU firmware query
 * Fix bug where manager would sometimes hang after shutting down the car

Version 0.7.2 (2020-02-07)
========================
 * ECU firmware version based fingerprinting for Honda & Toyota
 * New driving model: improved path prediction during turns and lane changes and better lead speed tracking
 * Improve driver monitoring under extreme lighting and add low accuracy alert
 * Support for 2019 Rav4 Hybrid thanks to illumiN8i!
 * Support for 2016, 2017 and 2020 Lexus RX thanks to illumiN8i!
 * Support for 2020 Chrysler Pacifica Hybrid thanks to adhintz!

Version 0.7.1 (2020-01-20)
========================
 * comma two support!
 * Lane Change Assist above 45 mph!
 * Replace zmq with custom messaging library, msgq!
 * Supercombo model: calibration and driving models are combined for better lead estimate
 * More robust updater thanks to jyoung8607! Requires NEOS update
 * Improve low speed ACC tuning

Version 0.7 (2019-12-13)
========================
 * Move to SCons build system!
 * Add Lane Departure Warning (LDW) for all supported vehicles!
 * NEOS update: increase wifi speed thanks to jyoung8607!
 * Adaptive driver monitoring based on scene
 * New driving model trained end-to-end: improve lane lines and lead detection
 * Smarter torque limit alerts for all cars
 * Improve GM longitudinal control: proper computations for 15Hz radar
 * Move GM port, Toyota with DSU removed, comma pedal in community features; toggle switch required
 * Remove upload over cellular toggle: only upload qlog and qcamera files if not on wifi
 * Refactor Panda code towards ISO26262 and SIL2 compliance
 * Forward stock FCW for Honda Nidec
 * Volkswagen port now standard: comma Harness intercepts stock camera

Version 0.6.6 (2019-11-05)
========================
 * Volkswagen support thanks to jyoung8607!
 * Toyota Corolla Hybrid with TSS 2.0 support thanks to u8511049!
 * Lexus ES with TSS 2.0 support thanks to energee!
 * Fix GM ignition detection and lock safety mode not required anymore
 * Log panda firmware and dongle ID thanks to martinl!
 * New driving model: improve path prediction and lead detection
 * New driver monitoring model, 4x smaller and running on DSP
 * Display an alert and don't start openpilot if panda has wrong firmware
 * Fix bug preventing EON from terminating processes after a drive
 * Remove support for Toyota giraffe without the 120Ohm resistor

Version 0.6.5 (2019-10-07)
========================
 * NEOS update: upgrade to Python3 and new installer!
 * comma Harness support!
 * New driving model: improve path prediction
 * New driver monitoring model: more accurate face and eye detection
 * Redesign offroad screen to display updates and alerts
 * Increase maximum allowed acceleration
 * Prevent car 12V battery drain by cutting off EON charge after 3 days of no drive
 * Lexus CT Hybrid support thanks to thomaspich!
 * Louder chime for critical alerts
 * Add toggle to switch to dashcam mode
 * Fix "invalid vehicle params" error on DSU-less Toyota

Version 0.6.4 (2019-09-08)
========================
 * Forward stock AEB for Honda Nidec
 * Improve lane centering on banked roads
 * Always-on forward collision warning
 * Always-on driver monitoring, except for right hand drive countries
 * Driver monitoring learns the user's normal driving position
 * Honda Fit support thanks to energee!
 * Lexus IS support

Version 0.6.3 (2019-08-12)
========================
 * Alert sounds from EON: requires NEOS update
 * Improve driver monitoring: eye tracking and improved awareness logic
 * Improve path prediction with new driving model
 * Improve lane positioning with wide lanes and exits
 * Improve lateral control on RAV4
 * Slow down for turns using model
 * Open sourced regression test to verify outputs against reference logs
 * Open sourced regression test to sanity check all car models

Version 0.6.2 (2019-07-29)
========================
 * New driving model!
 * Improve lane tracking with double lines
 * Strongly improve stationary vehicle detection
 * Strongly reduce cases of braking due to false leads
 * Better lead tracking around turns
 * Improve cut-in prediction by using neural network
 * Improve lateral control on Toyota Camry and C-HR thanks to zorrobyte!
 * Fix unintended openpilot disengagements on Jeep thanks to adhintz!
 * Fix delayed transition to offroad when car is turned off

Version 0.6.1 (2019-07-21)
========================
 * Remote SSH with comma prime and [ssh.comma.ai](https://ssh.comma.ai)
 * Panda code Misra-c2012 compliance, tested against cppcheck coverage
 * Lockout openpilot after 3 terminal alerts for driver distracted or unresponsive
 * Toyota Sienna support thanks to wocsor!

Version 0.6 (2019-07-01)
========================
 * New model, with double the pixels and ten times the temporal context!
 * Car should not take exits when in the right lane
 * openpilot uses only ~65% of the CPU (down from 75%)
 * Routes visible in connect/explorer after only 0.2% is uploaded (qlogs)
 * loggerd and sensord are open source, every line of openpilot is now open
 * Panda safety code is MISRA compliant and ships with a signed version on release2
 * New NEOS is 500MB smaller and has a reproducible usr/pipenv
 * Lexus ES Hybrid support thanks to wocsor!
 * Improve tuning for supported Toyota with TSS 2.0
 * Various other stability improvements

Version 0.5.13 (2019-05-31)
==========================
 * Reduce panda power consumption by 70%, down to 80mW, when car is off (not for GM)
 * Reduce EON power consumption by 40%, down to 1100mW, when car is off
 * Reduce CPU utilization by 20% and improve stability
 * Temporarily remove mapd functionalities to improve stability
 * Add openpilot record-only mode for unsupported cars
 * Synchronize controlsd to pandad to reduce latency
 * Remove panda support for Subaru giraffe

Version 0.5.12 (2019-05-16)
==========================
 * Improve lateral control for the Prius and Prius Prime
 * Compress logs before writing to disk
 * Remove old driving data when storage reaches 90% full
 * Fix small offset in following distance
 * Various small CPU optimizations
 * Improve offroad power consumption: require NEOS Update
 * Add default speed limits for Estonia thanks to martinl!
 * Subaru Crosstrek support thanks to martinl!
 * Toyota Avalon support thanks to njbrown09!
 * Toyota Rav4 with TSS 2.0 support thanks to wocsor!
 * Toyota Corolla with TSS 2.0 support thanks to wocsor!

Version 0.5.11 (2019-04-17)
========================
 * Add support for Subaru
 * Reduce panda power consumption by 60% when car is off
 * Fix controlsd lag every 6 minutes. This would sometimes cause disengagements
 * Fix bug in controls with new angle-offset learner in MPC
 * Reduce cpu consumption of ubloxd by rewriting it in C++
 * Improve driver monitoring model and face detection
 * Improve performance of visiond and ui
 * Honda Passport 2019 support
 * Lexus RX Hybrid 2019 support thanks to schomems!
 * Improve road selection heuristic in mapd
 * Add Lane Departure Warning to dashboard for Toyota thanks to arne182

Version 0.5.10 (2019-03-19)
========================
 * Self-tuning vehicle parameters: steering offset, tire stiffness and steering ratio
 * Improve longitudinal control at low speed when lead vehicle harshly decelerates
 * Fix panda bug going unexpectedly in DCP mode when EON is connected
 * Reduce white panda power consumption by 500mW when EON is disconnected by turning off WIFI
 * New Driver Monitoring Model
 * Support QR codes for login using comma connect
 * Refactor comma pedal FW and use CRC-8 checksum algorithm for safety. Reflashing pedal is required.
   Please see `#hw-pedal` on [discord](discord.comma.ai) for assistance updating comma pedal.
 * Additional speed limit rules for Germany thanks to arne182
 * Allow negative speed limit offsets

Version 0.5.9 (2019-02-10)
========================
 * Improve calibration using a dedicated neural network
 * Abstract planner in its own process to remove lags in controls process
 * Improve speed limits with country/region defaults by road type
 * Reduce mapd data usage with gzip thanks to eFiniLan
 * Zip log files in the background to reduce disk usage
 * Kia Optima support thanks to emmertex!
 * Buick Regal 2018 support thanks to HOYS!
 * Comma pedal support for Toyota thanks to wocsor! Note: tuning needed and not maintained by comma
 * Chrysler Pacifica and Jeep Grand Cherokee support thanks to adhintz!

Version 0.5.8 (2019-01-17)
========================
 * Open sourced visiond
 * Auto-slowdown for upcoming turns
 * Chrysler/Jeep/Fiat support thanks to adhintz!
 * Honda Civic 2019 support thanks to csouers!
 * Improve use of car display in Toyota thanks to arne182!
 * No data upload when connected to Android or iOS hotspots and "Enable Upload Over Cellular" setting is off
 * EON stops charging when 12V battery drops below 11.8V

Version 0.5.7 (2018-12-06)
========================
 * Speed limit from OpenStreetMap added to UI
 * Highlight speed limit when speed exceeds road speed limit plus a delta
 * Option to limit openpilot max speed to road speed limit plus a delta
 * Cadillac ATS support thanks to vntarasov!
 * GMC Acadia support thanks to CryptoKylan!
 * Decrease GPU power consumption
 * NEOSv8 autoupdate

Version 0.5.6 (2018-11-16)
========================
 * Refresh settings layout and add feature descriptions
 * In Honda, keep stock camera on for logging and extra stock features; new openpilot giraffe setting is 0111!
 * In Toyota, option to keep stock camera on for logging and extra stock features (e.g. AHB); 120Ohm resistor required on giraffe.
 * Improve camera calibration stability
 * More tuning to Honda positive accelerations
 * Reduce brake pump use on Hondas
 * Chevrolet Malibu support thanks to tylergets!
 * Holden Astra support thanks to AlexHill!

Version 0.5.5 (2018-10-20)
========================
 * Increase allowed Honda positive accelerations
 * Fix sporadic unexpected braking when passing semi-trucks in Toyota
 * Fix gear reading bug in Hyundai Elantra thanks to emmertex!

Version 0.5.4 (2018-09-25)
========================
 * New Driving Model
 * New Driver Monitoring Model
 * Improve longitudinal mpc in mid-low speed braking
 * Honda Accord hybrid support thanks to energee!
 * Ship mpc binaries and sensibly reduce build time
 * Calibration more stable
 * More Hyundai and Kia cars supported thanks to emmertex!
 * Various GM Volt improvements thanks to vntarasov!

Version 0.5.3 (2018-09-03)
========================
 * Hyundai Santa Fe support!
 * Honda Pilot 2019 support thanks to energee!
 * Toyota Highlander support thanks to daehahn!
 * Improve steering tuning for Honda Odyssey

Version 0.5.2 (2018-08-16)
========================
 * New calibration: more accurate, a lot faster, open source!
 * Enable orbd
 * Add little endian support to CAN packer
 * Fix fingerprint for Honda Accord 1.5T
 * Improve driver monitoring model

Version 0.5.1 (2018-08-01)
========================
 * Fix radar error on Civic sedan 2018
 * Improve thermal management logic
 * Alpha Toyota C-HR and Camry support!
 * Auto-switch Driver Monitoring to 3 min counter when inaccurate

Version 0.5 (2018-07-11)
========================
 * Driver Monitoring (beta) option in settings!
 * Make visiond, loggerd and UI use less resources
 * 60 FPS UI
 * Better car parameters for most cars
 * New sidebar with stats
 * Remove Waze and Spotify to free up system resources
 * Remove rear view mirror option
 * Calibration 3x faster

Version 0.4.7.2 (2018-06-25)
==========================
 * Fix loggerd lag issue
 * No longer prompt for updates
 * Mitigate right lane hugging for properly mounted EON (procedure on wiki)

Version 0.4.7.1 (2018-06-18)
==========================
 * Fix Acura ILX steer faults
 * Fix bug in mock car

Version 0.4.7 (2018-06-15)
==========================
 * New model!
 * GM Volt (and CT6 lateral) support!
 * Honda Bosch lateral support!
 * Improve actuator modeling to reduce lateral wobble
 * Minor refactor of car abstraction layer
 * Hack around orbd startup issue

Version 0.4.6 (2018-05-18)
==========================
 * NEOSv6 required! Will autoupdate
 * Stability improvements
 * Fix all memory leaks
 * Update C++ compiler to clang6
 * Improve front camera exposure

Version 0.4.5 (2018-04-27)
==========================
 * Release notes added to the update popup
 * Improve auto shut-off logic to disallow empty battery
 * Added onboarding instructions
 * Include orbd, the first piece of new calibration algorithm
 * Show remaining upload data instead of file numbers
 * Fix UI bugs
 * Fix memory leaks

Version 0.4.4 (2018-04-13)
==========================
 * EON are flipped! Flip your EON's mount!
 * Alpha Honda Ridgeline support thanks to energee!
 * Support optional front camera recording
 * Upload over cellular toggle now applies to all files, not just video
 * Increase acceleration when closing lead gap
 * User now prompted for future updates
 * NEO no longer supported :(

Version 0.4.3.2 (2018-03-29)
============================
 * Improve autofocus
 * Improve driving when only one lane line is detected
 * Added fingerprint for Toyota Corolla LE
 * Fixed Toyota Corolla steer error
 * Full-screen driving UI
 * Improved path drawing

Version 0.4.3.1 (2018-03-19)
============================
 * Improve autofocus
 * Add check for MPC solution error
 * Make first distracted warning visual only

Version 0.4.3 (2018-03-13)
==========================
 * Add HDR and autofocus
 * Update UI aesthetic
 * Grey panda works in Waze
 * Add alpha support for 2017 Honda Pilot
 * Slight increase in acceleration response from stop
 * Switch CAN sending to use CANPacker
 * Fix pulsing acceleration regression on Honda
 * Fix openpilot bugs when stock system is in use
 * Change starting logic for chffrplus to use battery voltage

Version 0.4.2 (2018-02-05)
==========================
 * Add alpha support for 2017 Lexus RX Hybrid
 * Add alpha support for 2018 ACURA RDX
 * Updated fingerprint to include Toyota Rav4 SE and Prius Prime
 * Bugfixes for Acura ILX and Honda Odyssey

Version 0.4.1 (2018-01-30)
==========================
 * Add alpha support for 2017 Toyota Corolla
 * Add alpha support for 2018 Honda Odyssey with Honda Sensing
 * Add alpha support for Grey Panda
 * Refactored car abstraction layer to make car ports easier
 * Increased steering torque limit on Honda CR-V by 30%

Version 0.4.0.2 (2018-01-18)
==========================
 * Add focus adjustment slider
 * Minor bugfixes

Version 0.4.0.1 (2017-12-21)
==========================
 * New UI to match chffrplus
 * Improved lateral control tuning to fix oscillations on Civic
 * Add alpha support for 2017 Toyota Rav4 Hybrid
 * Reduced CPU usage
 * Removed unnecessary utilization of fan at max speed
 * Minor bug fixes

Version 0.3.9 (2017-11-21)
==========================
 * Add alpha support for 2017 Toyota Prius
 * Improved longitudinal control using model predictive control
 * Enable Forward Collision Warning
 * Acura ILX now maintains openpilot engaged at standstill when brakes are applied

Version 0.3.8.2 (2017-10-30)
==========================
 * Add alpha support for 2017 Toyota RAV4
 * Smoother lateral control
 * Stay silent if stock system is connected through giraffe
 * Minor bug fixes

Version 0.3.7 (2017-09-30)
==========================
 * Improved lateral control using model predictive control
 * Improved lane centering
 * Improved GPS
 * Reduced tendency of path deviation near right side exits
 * Enable engagement while the accelerator pedal is pressed
 * Enable engagement while the brake pedal is pressed, when stationary and with lead vehicle within 5m
 * Disable engagement when park brake or brake hold are active
 * Fixed sporadic longitudinal pulsing in Civic
 * Cleanups to vehicle interface

Version 0.3.6.1 (2017-08-15)
============================
 * Mitigate low speed steering oscillations on some vehicles
 * Include board steering check for CR-V

Version 0.3.6 (2017-08-08)
==========================
 * Fix alpha CR-V support
 * Improved GPS
 * Fix display of target speed not always matching HUD
 * Increased acceleration after stop
 * Mitigated some vehicles driving too close to the right line

Version 0.3.5 (2017-07-30)
==========================
 * Fix bug where new devices would not begin calibration
 * Minor robustness improvements

Version 0.3.4 (2017-07-28)
==========================
 * Improved model trained on more data
 * Much improved controls tuning
 * Performance improvements
 * Bugfixes and improvements to calibration
 * Driving log can play back video
 * Acura only: system now stays engaged below 25mph as long as brakes are applied

Version 0.3.3  (2017-06-28)
===========================
 * Improved model trained on more data
 * Alpha CR-V support thanks to energee and johnnwvs!
 * Using the opendbc project for DBC files
 * Minor performance improvements
 * UI update thanks to pjlao307
 * Power off button
 * 6% more torque on the Civic

Version 0.3.2  (2017-05-22)
===========================
 * Minor stability bugfixes
 * Added metrics and rear view mirror disable to settings
 * Update model with more crowdsourced data

Version 0.3.1  (2017-05-17)
===========================
 * visiond stability bugfix
 * Add logging for angle and flashing

Version 0.3.0  (2017-05-12)
===========================
 * Add CarParams struct to improve the abstraction layer
 * Refactor visiond IPC to support multiple clients
 * Add raw GPS and beginning support for navigation
 * Improve model in visiond using crowdsourced data
 * Add improved system logging to diagnose instability
 * Rewrite baseui in React Native
 * Moved calibration to the cloud

Version 0.2.9  (2017-03-01)
===========================
 * Retain compatibility with NEOS v1

Version 0.2.8  (2017-02-27)
===========================
 * Fix bug where frames were being dropped in minute 71

Version 0.2.7  (2017-02-08)
===========================
 * Better performance and pictures at night
 * Fix ptr alignment issue in pandad
 * Fix brake error light, fix crash if too cold

Version 0.2.6  (2017-01-31)
===========================
 * Fix bug in visiond model execution

Version 0.2.5  (2017-01-30)
===========================
 * Fix race condition in manager

Version 0.2.4  (2017-01-27)
===========================
 * OnePlus 3T support
 * Enable installation as NEOS app
 * Various minor bugfixes

Version 0.2.3  (2017-01-11)
===========================
 * Reduce space usage by 80%
 * Add better logging
 * Add Travis CI

Version 0.2.2  (2017-01-10)
===========================
 * Board triggers started signal on CAN messages
 * Improved autoexposure
 * Handle out of space, improve upload status

Version 0.2.1  (2016-12-14)
===========================
 * Performance improvements, removal of more numpy
 * Fix pandad process priority
 * Make counter timer reset on use of steering wheel

Version 0.2  (2016-12-12)
=========================
 * Car/Radar abstraction layers have shipped, see cereal/car.capnp
 * controlsd has been refactored
 * Shipped plant model and testing maneuvers
 * visiond exits more gracefully now
 * Hardware encoder in visiond should always init
 * ui now turns off the screen after 30 seconds
 * Switch to openpilot release branch for future releases
 * Added preliminary Docker container to run tests on PC

Version 0.1  (2016-11-29)
=========================
 * Initial release of openpilot
 * Adaptive cruise control is working
 * Lane keep assist is working
 * Support for Acura ILX 2016 with AcuraWatch Plus
 * Support for Honda Civic 2016 Touring Edition
