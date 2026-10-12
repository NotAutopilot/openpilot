"""Read/write comma Software params for MCU/phone. No SSH. No engage."""
from __future__ import annotations

import datetime
import re
import subprocess

BRANCH_OK = re.compile(r"^[A-Za-z0-9._/-]+$")
BLOCKED_ACTIONS = {
    "engage",
    "disengage",
    "uninstall",
    "reset_calibration",
    "reset_lateral",
    "reset_longitudinal",
    "force_onroad",
}
SOFTWARE_ACTIONS = {"set_branch", "fetch", "download", "install"}
UPDATED_PATTERNS = (
    "openpilot.system.updated.updated",
    "system.updated.updated",
)


class SoftwareError(ValueError):
    pass


def _as_str(value) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    return str(value)


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        folded = value.strip().lower()
        if folded in ("1", "true", "yes", "on"):
            return True
        if folded in ("0", "false", "no", "off", ""):
            return False
    raise SoftwareError(f"invalid bool: {value!r}")


def sanitize_branch(raw: str | None) -> str:
    branch = (raw or "").strip()
    if not branch or len(branch) > 80:
        raise SoftwareError("invalid branch")
    if ".." in branch or branch.startswith(("/", "-")):
        raise SoftwareError("invalid branch")
    # Exact action names only: a branch name cannot engage anything, so
    # names like cursor/speedsignd-engaged-read-spacing-3353 are fine.
    key = branch.lower().replace("-", "_")
    if key in BLOCKED_ACTIONS:
        raise SoftwareError("action not allowed")
    if not BRANCH_OK.match(branch):
        raise SoftwareError("invalid branch")
    return branch


UPDATED_PGREP = "system.updated.updated"

# Same wording as the comma Software panel (selfdrive/ui/layouts/settings/software.py).
STATE_TEXT = {
    "checking...": "checking…",
    "downloading...": "downloading…",
    "finalizing update...": "finalizing update…",
}


PAUSED_TEXT = "Updates are paused (DisableUpdates is on), so Check and Download are blocked."
STOPPED_TEXT = (
    "The updater is not running. It starts when the car turns off; " +
    "if updates were just re-enabled, reboot the comma or cycle the car to start it."
)


def _as_int(value) -> int:
    if value is None:
        return 0
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, (int, float)):
        return int(value)
    try:
        return int(_as_str(value).strip() or 0)
    except ValueError:
        return 0


def _as_time(value) -> datetime.datetime | None:
    """LastUpdateTime is a TIME param (naive UTC datetime); older params give ISO text."""
    if value is None or value == "" or value == b"":
        return None
    if isinstance(value, datetime.datetime):
        when = value
    else:
        text = _as_str(value).strip().replace("Z", "")
        try:
            when = datetime.datetime.fromisoformat(text)
        except ValueError:
            return None
    if when.tzinfo is not None:
        when = when.astimezone(datetime.UTC).replace(tzinfo=None)
    return when


def time_ago(when: datetime.datetime | None, now: datetime.datetime | None = None) -> str:
    """Same buckets as time_ago() in the comma Software panel."""
    if when is None:
        return "never"
    now = now or datetime.datetime.now(datetime.UTC).replace(tzinfo=None)
    diff = int((now - when).total_seconds())
    if diff < 0 or now.year < 2024:
        return when.strftime("%a %b %d %Y")
    if diff < 60:
        return "now"
    if diff < 3600:
        n = diff // 60
        return f"{n} minute{'s' if n != 1 else ''} ago"
    if diff < 86400:
        n = diff // 3600
        return f"{n} hour{'s' if n != 1 else ''} ago"
    if diff < 604800:
        n = diff // 86400
        return f"{n} day{'s' if n != 1 else ''} ago"
    return when.strftime("%a %b %d %Y")


def updated_running() -> bool:
    """True when the updater process is alive. It only runs while the car is off."""
    try:
        out = subprocess.run(
            ["pgrep", "-f", UPDATED_PGREP],
            check=False,
            timeout=2,
            capture_output=True,
        )
        return out.returncode == 0
    except Exception:
        return False


def _flag(params, key: str) -> bool:
    try:
        return bool(params.get_bool(key))
    except Exception:
        return False


def _get(params, key: str):
    try:
        return params.get(key)
    except Exception:
        return None


def install_label(snap: dict) -> str:
    """'Install & reboot with <version / branch>' from UpdaterNewDescription."""
    desc = snap.get("new_description") or ""
    # get_description(): "<version> / <branch> / <commit> / <date>"; branches can contain "/".
    parts = [part.strip() for part in desc.split(" / ") if part.strip()]
    target = " / ".join(parts[:2]) if parts else (snap.get("target_branch") or "")
    return f"Install & reboot with {target}" if target else "Install & reboot"


# Native Software panel (selfdrive/ui/layouts/settings/software.py): the INSTALL
# button is shown and enabled whenever ui_state.is_offroad() and UpdateAvailable.
# It has no engaged or updater-busy check. While onroad it is hidden and the panel
# shows this line instead. is_offroad() is "not deviceState.started", which
# hardwared publishes as IsOnroad.
ONROAD_TEXT = "Updates are only downloaded while the car is off."


def install_blocked(snap: dict) -> str:
    """Same rule as the native INSTALL button: available only while offroad."""
    if snap.get("onroad"):
        return ONROAD_TEXT
    return ""


# Where the update flow is: Check -> Download -> Install & reboot.
STEP_LABEL = {
    "check": "Check",
    "checking": "Checking…",
    "download": "Download",
    "downloading": "Downloading…",
}


def next_step(snap: dict) -> str:
    """Which step of the update flow the Software tab is on."""
    state = snap["updater_state"]
    if state == "checking...":
        return "checking"
    if state != "idle":
        return "downloading"
    if snap["update_available"]:
        return "install"
    if snap["update_failed_count"] > 0:
        return "check"
    if snap["fetch_available"]:
        return "download"
    return "check"


def software_status(snap: dict) -> dict:
    """Status line plus Check/Download availability, mirroring the comma Software panel."""
    state = snap["updater_state"]
    busy = state != "idle"
    blocked = ""
    if snap["disable_updates"]:
        blocked = PAUSED_TEXT
    elif snap["onroad"]:
        blocked = ONROAD_TEXT
    elif not snap["updater_running"]:
        blocked = STOPPED_TEXT

    if busy:
        text = STATE_TEXT.get(state, state)
    elif snap["update_available"]:
        text = "update ready, reboot to install"
        if snap["new_description"]:
            text += f" ({snap['new_description']})"
    elif snap["update_failed_count"] > 0:
        text = "failed to check for update"
    elif snap["fetch_available"]:
        text = "update available, tap Download"
    else:
        text = "up to date"
    if not busy:
        text += ", last checked " + snap["last_checked"]
    action = "download" if snap["fetch_available"] and not snap["update_available"] else "check"
    step = next_step(snap)
    return {
        "status_text": text,
        "busy": busy,
        "blocked_reason": blocked,
        "next_action": action,
        "can_check": not busy and not blocked,
        # Download is offered only after a check found something new to fetch.
        "can_download": not busy and not blocked and snap["fetch_available"] and not snap["update_available"],
        "can_install": bool(snap["update_available"]) and not install_blocked(snap),
        "install_blocked_reason": install_blocked(snap),
        "install_label": install_label(snap),
        "step": step,
        "step_label": STEP_LABEL.get(step) or install_label(snap),
    }


def read_software(params, running=None) -> dict:
    branches_raw = _as_str(_get(params, "UpdaterAvailableBranches"))
    branches = [part for part in branches_raw.split(",") if part]
    offline = _flag(params, "DisableUpdates")
    last = _as_time(_get(params, "LastUpdateTime"))
    alive = updated_running() if running is None else bool(running() if callable(running) else running)
    snap = {
        "git_branch": _as_str(_get(params, "GitBranch")),
        "git_commit": _as_str(_get(params, "GitCommit")),
        "version": _as_str(_get(params, "Version")),
        "target_branch": _as_str(_get(params, "UpdaterTargetBranch")),
        "available_branches": branches,
        "updater_state": _as_str(_get(params, "UpdaterState")).strip() or "idle",
        "fetch_available": _flag(params, "UpdaterFetchAvailable"),
        "update_available": _flag(params, "UpdateAvailable"),
        "update_failed_count": _as_int(_get(params, "UpdateFailedCount")),
        "last_update_time": last.replace(microsecond=0).isoformat() + "Z" if last else "",
        "last_checked": time_ago(last),
        "last_update_exception": _as_str(_get(params, "LastUpdateException")).strip(),
        "current_description": _as_str(_get(params, "UpdaterCurrentDescription")).strip(),
        "new_description": _as_str(_get(params, "UpdaterNewDescription")).strip(),
        "onroad": _flag(params, "IsOnroad"),
        "engaged": _flag(params, "IsEngaged"),
        "updater_running": alive,
        "disable_updates": offline,
        "offline": offline,
    }
    snap.update(software_status(snap))
    return snap


def ping_updated(signal: str = "USR1") -> None:
    """Same wake-up the comma Software UI uses. Fail-soft if updated is idle."""
    for pattern in UPDATED_PATTERNS:
        try:
            subprocess.run(
                ["pkill", f"-SIG{signal}", "-f", pattern],
                check=False,
                timeout=2,
            )
        except Exception:
            pass


def _put(params, key: str, value: str) -> None:
    try:
        params.put(key, value, block=True)
    except TypeError:
        params.put(key, value)


def _put_bool(params, key: str, value: bool) -> None:
    try:
        try:
            # block so the snapshot we return right after already shows the new value
            params.put_bool(key, value, block=True)
        except TypeError:
            params.put_bool(key, value)
    except Exception:
        _put(params, key, "1" if value else "0")


def _as_confirm(value) -> bool:
    try:
        return _as_bool(value) if value is not None else False
    except SoftwareError:
        return False


def handle_software(payload: dict, params, pinger=ping_updated, running=None) -> dict:
    action = str(payload.get("action") or "").strip().lower().replace("-", "_")
    if not action:
        raise SoftwareError("missing action")
    if action in BLOCKED_ACTIONS or "engage" in action:
        raise SoftwareError("action not allowed")
    if action not in SOFTWARE_ACTIONS:
        raise SoftwareError(f"software action not allowed: {action}")
    if action == "set_branch":
        branch = sanitize_branch(str(payload.get("branch") or payload.get("name") or ""))
        _put(params, "UpdaterTargetBranch", branch)
        pinger("USR1")
    elif action == "install":
        # Native Software panel INSTALL: put DoReboot; the finalized update swaps in on boot.
        snap = read_software(params, running=running)
        if not snap["update_available"]:
            raise SoftwareError("No downloaded update is ready to install.")
        reason = install_blocked(snap)
        if reason:
            raise SoftwareError(reason)
        if not _as_confirm(payload.get("confirm")):
            raise SoftwareError("confirmation required to install and reboot")
        _put_bool(params, "DoReboot", True)
        snap["rebooting"] = True
        return snap
    else:
        # Same signals as the comma Software panel: SIGUSR1 checks, SIGHUP downloads.
        if _flag(params, "DisableUpdates"):
            raise SoftwareError("Updates are paused (DisableUpdates is on), so Check and Download are blocked.")
        if _flag(params, "IsOnroad"):
            raise SoftwareError(ONROAD_TEXT)
        pinger("HUP" if action == "download" else "USR1")
    return read_software(params, running=running)
