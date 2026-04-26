#!/usr/bin/env bash
# Single-source-of-truth gate for naponfp-dev port completion.
# Mirrors verify-port-sp.sh but runs from worktrees/fp and validates against
# frogpilot's test surface instead of sunnypilot's.

set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

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
python -m pytest -W ignore selfdrive/test/ -q 2>&1 | tail -10 || fail "selfdrive/test"

step "11. drive-3 replay"
python scripts/replay_stock_cc.py 2>&1 | tail -5 || fail "replay_stock_cc"
python scripts/compare_stalk_frames.py 2>&1 | tail -5 || fail "compare_stalk_frames"
python scripts/compare_stalk_frames.py 2>&1 | grep -E '^\s+t\+.*NAP' \
  | awk '{print $4}' | grep -qE 'byte0=0x[57]' \
  || fail "spoof byte0 wrong — VSL_Enbl_Rq regression?"

step "12. process_replay"
timeout 600 python selfdrive/test/process_replay/test_processes.py -j2 2>&1 | tail -20 \
  || fail "process_replay"

step "13. github CI workflow check"
gh run list --branch naponfp-dev --limit 5 --json conclusion,status,name 2>/dev/null \
  | python -c "import json, sys; runs = json.load(sys.stdin); \
    incomplete = [r for r in runs if r['status'] != 'completed']; \
    failed = [r for r in runs if r['conclusion'] not in ('success', 'skipped', None)]; \
    sys.exit(0 if not incomplete and not failed else 1)" \
  || fail "github CI: incomplete or failing runs on naponfp-dev"

green ""
green "ALL CHECKS PASSED — naponfp-dev is ready"
exit 0
