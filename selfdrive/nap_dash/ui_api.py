"""Read and write the controls discovered from the on-device settings UI.

No cereal subscriber. Onroad / engaged locks use the Params hardwared
already maintains (IsOnroad, IsEngaged).
"""
from __future__ import annotations

import subprocess
from typing import Any

from openpilot.selfdrive.nap_dash.discover import discover_manifest

_MANIFEST: dict[str, Any] | None = None

_PUBLIC = (
  "id", "panel", "section", "param", "title", "description", "kind",
  "confirm", "lock_onroad", "lock_engaged", "disabled", "needs_restart",
  "reboot_hint", "choices", "values", "release_hidden", "target_panel", "writable",
)


class SettingError(ValueError):
  """Invalid web setting name or value."""


def get_manifest(refresh: bool = False) -> dict[str, Any]:
  global _MANIFEST
  if _MANIFEST is None or refresh:
    _MANIFEST = discover_manifest()
  return _MANIFEST


def public_manifest(manifest: dict[str, Any] | None = None) -> dict[str, Any]:
  manifest = get_manifest() if manifest is None else manifest
  return {
    "hotspot": manifest["hotspot"],
    "panels": manifest["panels"],
    "controls": [{key: ctrl[key] for key in _PUBLIC} for ctrl in manifest["controls"]],
  }


def setting_catalog(manifest: dict[str, Any] | None = None) -> list[dict[str, Any]]:
  return public_manifest(manifest)["controls"]


def _flag(params, key: str) -> bool:
  try:
    return bool(params.get_bool(key))
  except Exception:
    return False


def _onroad(params) -> bool:
  return _flag(params, "IsOnroad")


def _engaged(params) -> bool:
  return _flag(params, "IsEngaged")


def _release(params) -> bool:
  return _flag(params, "IsReleaseBranch")


def _lock_reason(params, ctrl: dict[str, Any]) -> str:
  if ctrl.get("disabled"):
    return "not available"
  if ctrl.get("release_hidden") and _release(params):
    return "not available on this software"
  if ctrl.get("lock_onroad") and _onroad(params):
    return "locked while the car is on"
  if ctrl.get("lock_engaged") and _engaged(params):
    return "locked while openpilot is engaged"
  if ctrl.get("lock_param") and _flag(params, ctrl["lock_param"]):
    return "locked"
  return ""


def _locked_map(params, manifest: dict[str, Any]) -> dict[str, str]:
  locked = {}
  for ctrl in manifest["controls"]:
    if not ctrl.get("param"):
      continue
    reason = _lock_reason(params, ctrl)
    if reason:
      locked[ctrl["param"]] = reason
  return locked


def _read_one(params, ctrl: dict[str, Any], ptype: str) -> Any:
  try:
    if ctrl["kind"] in ("toggle", "action") or ptype == "bool":
      return bool(params.get_bool(ctrl["param"]))
    raw = params.get(ctrl["param"], return_default=True)
  except Exception:
    return None
  if raw is None or raw == "":
    return None
  if isinstance(raw, bytes):
    raw = raw.decode("utf-8", "replace")
  try:
    if ptype == "int":
      return int(raw)
    if ptype == "float":
      return float(raw)
  except (TypeError, ValueError):
    return raw
  return raw


def read_settings(params, manifest: dict[str, Any] | None = None) -> dict[str, Any]:
  """Snapshot of discovered params. Does not write."""
  manifest = get_manifest() if manifest is None else manifest
  types = manifest.get("param_types") or {}
  values: dict[str, Any] = {}
  for ctrl in manifest["controls"]:
    param = ctrl.get("param") or ""
    if not param or param in values:
      continue
    if ctrl["kind"] not in ("toggle", "choice", "text", "action"):
      continue
    values[param] = _read_one(params, ctrl, types.get(param, ""))
  return {
    "onroad": _onroad(params),
    "engaged": _engaged(params),
    "release": _release(params),
    "values": values,
    "locked": _locked_map(params, manifest),
  }


def _find(manifest: dict[str, Any], name: str) -> dict[str, Any] | None:
  key = str(name or "").strip()
  if not key:
    return None
  matches = [ctrl for ctrl in manifest["controls"] if ctrl.get("param") == key or ctrl.get("id") == key]
  for ctrl in matches:
    if ctrl.get("writable"):
      return ctrl
  return matches[0] if matches else None


def _as_bool(value: Any) -> bool:
  if isinstance(value, bool):
    return value
  if isinstance(value, (int, float)) and not isinstance(value, bool) and value in (0, 1):
    return bool(value)
  if isinstance(value, str):
    folded = value.strip().lower()
    if folded in ("1", "true", "yes", "on"):
      return True
    if folded in ("0", "false", "no", "off", ""):
      return False
  raise SettingError(f"invalid bool: {value!r}")


def _put_bool(params, key: str, value: bool) -> None:
  try:
    params.put_bool(key, bool(value), block=True)
  except TypeError:
    params.put_bool(key, bool(value))


def _put(params, key: str, value: Any) -> None:
  try:
    params.put(key, value, block=True)
  except TypeError:
    params.put(key, value)


def _match_choice(value: Any, allowed: list[Any]) -> Any:
  for item in allowed:
    if value == item:
      return item
    if isinstance(item, bool):
      continue
    if isinstance(item, (int, float)) and not isinstance(value, bool):
      try:
        if float(value) == float(item):
          return item
      except (TypeError, ValueError):
        continue
    if isinstance(item, str) and str(value) == item:
      return item
  raise SettingError("value not allowed")


def _needs_confirm(params, ctrl: dict[str, Any], turning_on: bool, confirm: bool) -> bool:
  # Force Offroad has no settings confirm on the comma either; its only prompt is
  # the on-road "Ready to resume steering control?" Yes/No (force_offroad_api).
  if ctrl.get("writer") == "force_offroad":
    return False
  if confirm or not ctrl.get("confirm"):
    return False
  if ctrl["kind"] == "action":
    return True
  if not turning_on:
    return False
  if ctrl.get("writer") == "experimental" and _flag(params, "ExperimentalModeConfirmed"):
    return False
  return True


def _apply_writer(params, ctrl: dict[str, Any], value: Any) -> None:
  writer = ctrl.get("writer") or ""
  param = ctrl["param"]
  if writer == "action":
    if _as_bool(value) is not True:
      raise SettingError("invalid value")
    _put_bool(params, param, True)
    return
  if writer == "choice" or ctrl["kind"] == "choice":
    chosen = _match_choice(value, list(ctrl.get("values") or []))
    _put(params, param, chosen)
    return
  on = _as_bool(value)
  if writer == "experimental":
    _put_bool(params, "ExperimentalMode", on)
    if on:
      _put_bool(params, "ExperimentalModeConfirmed", True)
  elif writer == "dm_sim":
    from openpilot.selfdrive.monitoring.dm_toggles import apply_dm_simulate_looking
    apply_dm_simulate_looking(params, on)
  elif writer == "dm_fai":
    from openpilot.selfdrive.monitoring.dm_toggles import apply_dm_false_alert_ignore
    apply_dm_false_alert_ignore(params, on)
  elif writer == "force_offroad":
    from openpilot.system.hardware.nap_force_offroad import apply_force_offroad_toggle
    apply_force_offroad_toggle(params, on, started=_onroad(params))
  elif writer == "hypermile":
    from openpilot.selfdrive.controls.lib.hypermile import apply_hypermile_toggle
    apply_hypermile_toggle(params, on)
  else:
    _put_bool(params, param, on)
    for other in ctrl.get("also_clear") or []:
      if other != param:
        _put_bool(params, other, False)
  if ctrl.get("needs_restart"):
    _put_bool(params, "OnroadCycleRequested", True)


def write_setting(params, name: str, value: Any, confirm: bool = False,
                  manifest: dict[str, Any] | None = None) -> dict[str, Any]:
  """Write one discovered control the same way the device UI does."""
  manifest = get_manifest() if manifest is None else manifest
  ctrl = _find(manifest, name)
  if ctrl is None:
    raise SettingError(f"unknown setting: {name}")
  param = str(ctrl.get("param") or "")
  folded = param.lower()
  if folded in {"engage", "disengage"} or folded.startswith("doengage") or "engage" == folded:
    raise SettingError("engaging openpilot from the web is not allowed")
  if not ctrl.get("writable"):
    raise SettingError("not available on the web")
  reason = _lock_reason(params, ctrl)
  if reason:
    raise SettingError(reason)
  turning_on = True
  if ctrl["kind"] == "toggle":
    turning_on = _as_bool(value)
  if _needs_confirm(params, ctrl, turning_on, bool(confirm)):
    raise SettingError("confirmation required")
  _apply_writer(params, ctrl, value)
  return read_settings(params, manifest)


def network_status(manifest: dict[str, Any] | None = None) -> dict[str, Any]:
  """Hotspot address plus local IPv4 addresses. No NetworkManager, no cereal."""
  manifest = get_manifest() if manifest is None else manifest
  hotspot = manifest["hotspot"]
  interfaces: list[dict[str, str]] = []
  try:
    proc = subprocess.run(["ip", "-4", "-o", "addr", "show"], capture_output=True, text=True, timeout=1, check=False)
    if proc.returncode == 0:
      for line in proc.stdout.splitlines():
        parts = line.split()
        if "inet" not in parts or len(parts) < 4:
          continue
        idx = parts.index("inet")
        interfaces.append({"name": parts[1], "addr": parts[idx + 1]})
  except Exception:
    interfaces = []
  return {"hotspot": hotspot, "interfaces": interfaces, "url": hotspot["url"]}
