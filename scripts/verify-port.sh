#!/usr/bin/env bash
# Single-source-of-truth gate for naponfp-dev port completion.
# Mirrors verify-port-sp.sh but runs from worktrees/fp and validates against
# frogpilot's test surface instead of sunnypilot's.

set -e
set -o pipefail   # don't let `cmd | tail` mask a failing cmd's exit code

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

# Pick whatever GNU timeout we can find (bsd timeout doesn't exist on macOS;
# coreutils via brew installs it as gtimeout).
TIMEOUT="$(command -v timeout 2>/dev/null || command -v gtimeout 2>/dev/null || true)"
if [ -z "$TIMEOUT" ]; then
  TIMEOUT="python3 -c 'import subprocess,sys; sys.exit(subprocess.run(sys.argv[2:], timeout=int(sys.argv[1])).returncode)'"
fi

# shellcheck disable=SC1091
source .venv/bin/activate 2>/dev/null || { echo "FAIL: no .venv"; exit 1; }

red()   { printf '\033[31m%s\033[0m\n' "$*"; }
green() { printf '\033[32m%s\033[0m\n' "$*"; }

step() { printf '\n=== %s ===\n' "$*"; }
fail() { red "FAIL: $*"; exit 1; }

step "1. preap python tests"
python -m pytest -W ignore opendbc_repo/opendbc/car/tesla/preap/tests/ -q || fail "preap tests"

step "2. panda safety tests (preap)"
python -m pytest -W ignore opendbc_repo/opendbc/safety/tests/test_tesla_preap.py -q \
  || fail "safety tests"

step "3. selfdrived alerts"
python -m pytest -W ignore selfdrive/selfdrived/tests/test_alerts.py -q || fail "alerts tests"

step "4. car_specific tesla_preap tests"
python -m pytest -W ignore selfdrive/car/tests/test_car_specific_tesla_preap.py -q \
  || fail "car_specific tests"

step "5. STW_ACTN_RQ byte-layout invariant"
python -m pytest -W ignore opendbc_repo/opendbc/car/tesla/preap/tests/test_stw_actn_rq_spoof.py -q \
  || fail "stw_actn_rq spoof tests"

step "6. nap_conf calibration sanity"
python -m pytest -W ignore opendbc_repo/opendbc/car/tesla/preap/tests/test_nap_conf_calib_sanity.py -q \
  || fail "nap_conf calib tests"

step "7. ruff on every touched file"
TOUCHED="$(git diff --name-only origin/nap-vdas-dev...HEAD -- '*.py' 2>/dev/null \
  ; git -C opendbc_repo diff --name-only origin/nap-vdas-dev...HEAD -- '*.py' 2>/dev/null \
    | sed 's|^|opendbc_repo/|' )"
if [ -n "$TOUCHED" ]; then
  # shellcheck disable=SC2086
  ruff check $TOUCHED || fail "ruff"
fi

step "8. macOS scons clean"
grep -q "done building targets" <(scons cereal selfdrive -j1 -u 2>&1 | tail -2) \
  || fail "scons not clean"

step "9. Docker Linux scons"
# The Dockerfile's final RUN is `uv run scons --cache-readonly -j$(nproc)`,
# so a clean `docker build` exit *is* a clean Linux scons. The follow-up
# `docker run scons` re-verifies inside the built image; venv lives at
# $VIRTUAL_ENV (=/home/batman/.venv), not relative to the working dir.
docker build -t naponfp-ci -f Dockerfile.openpilot . >/tmp/naponfp-docker-build.log 2>&1 \
  || fail "docker build (see /tmp/naponfp-docker-build.log)"
docker run --rm naponfp-ci bash -c 'source $VIRTUAL_ENV/bin/activate && scons -j$(nproc)' \
  >/tmp/naponfp-docker.log 2>&1 || fail "docker scons (see /tmp/naponfp-docker.log)"
grep -q "done building targets" /tmp/naponfp-docker.log || fail "docker did not finish cleanly"

step "10. frogpilot's own test suite"
python -m pytest -W ignore selfdrive/test/ -q >/tmp/naponfp-pytest.log 2>&1 \
  || { tail -20 /tmp/naponfp-pytest.log; fail "selfdrive/test (see /tmp/naponfp-pytest.log)"; }

step "11. drive-3 replay"
python scripts/replay_stock_cc.py >/tmp/naponfp-replay.log 2>&1 \
  || fail "replay_stock_cc (see /tmp/naponfp-replay.log)"
python scripts/compare_stalk_frames.py >/dev/null 2>&1 \
  || fail "compare_stalk_frames"
# byte0 invariant: replay_stock_cc exercises the *current* StockCCSpoofer +
# real CANPacker, so its byte0 column reflects a regression in our tree.
# compare_stalk_frames diffs historical sendcan and can't tell the difference.
grep -E '^\s+t\+.*NAP' /tmp/naponfp-replay.log \
  | awk '{print $4}' | grep -qE 'byte0=0x[57]' \
  || fail "spoof byte0 wrong — VSL_Enbl_Rq regression?"

step "12. process_replay"
$TIMEOUT 600 python selfdrive/test/process_replay/test_processes.py -j2 \
  >/tmp/naponfp-procreplay.log 2>&1 \
  || { tail -20 /tmp/naponfp-procreplay.log; fail "process_replay (see /tmp/naponfp-procreplay.log)"; }

step "13. github CI workflow check"
RUNS_JSON="$(gh run list --branch naponfp-dev --limit 5 --json conclusion,status,name 2>/dev/null)"
[ -n "$RUNS_JSON" ] || fail "gh run list returned nothing"
echo "$RUNS_JSON" | python -c "import json, sys; runs = json.load(sys.stdin); \
  sys.exit(0 if runs else 1)" \
  || fail "no CI runs found for naponfp-dev — has it been pushed to origin?"
echo "$RUNS_JSON" | python -c "import json, sys; runs = json.load(sys.stdin); \
  incomplete = [r for r in runs if r['status'] != 'completed']; \
  failed = [r for r in runs if r['conclusion'] not in ('success', 'skipped', None)]; \
  sys.exit(0 if not incomplete and not failed else 1)" \
  || fail "github CI: incomplete or failing runs on naponfp-dev"

green ""
green "ALL CHECKS PASSED — naponfp-dev is ready"
exit 0
