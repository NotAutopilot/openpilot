"""Read device-UI settings source and build the web manifest.

The comma UI (selfdrive/ui/layouts/settings) stays the source of truth.
This module only parses those files. It does not import them, so pyray,
cereal, and the speed-limit processes are never started from here.
"""
from __future__ import annotations

import ast
import re
from pathlib import Path
from typing import Any

MISSING = object()

# Sidebar class -> manifest id. Titles still come from the PanelInfo strings.
_PANEL_CLASS = {
  "DeviceLayout": "device",
  "NetworkUI": "network",
  "TogglesLayout": "toggles",
  "SoftwareLayout": "software",
  "NAPLayout": "nap",
  "FirehoseLayout": "firehose",
  "DeveloperLayout": "developer",
}

_FILE_PANEL = {
  "toggles.py": "toggles",
  "developer.py": "developer",
  "device.py": "device",
  "hidden_toggles.py": "nap_hidden",
  "driving_mannerisms.py": "nap_mannerisms",
  "map_speed.py": "nap_map",
  "firehose.py": "firehose",
  "nap.py": "nap",
}

_PAGE_PANEL = {
  "driving_mannerisms": "nap_mannerisms",
  "map_speed": "nap_map",
  "radar": "nap_radar",
}

_BUILD_METHODS = ("_build_items", "_initialize_items", "__init__")
_MAX_MODULES = 250


def repo_root() -> Path:
  return Path(__file__).resolve().parents[2]


def _is_missing(value: Any) -> bool:
  return value is MISSING


def _call_name(node: ast.AST) -> str:
  if isinstance(node, ast.Name):
    return node.id
  if isinstance(node, ast.Attribute):
    return node.attr
  return ""


def _dotted(node: ast.AST) -> str:
  if isinstance(node, ast.Name):
    return node.id
  if isinstance(node, ast.Attribute):
    parent = _dotted(node.value)
    return f"{parent}.{node.attr}" if parent else node.attr
  return ""


class _Env(dict):
  """Constant table. Classes are nested dicts."""


def _eval(node: ast.AST | None, env: _Env) -> Any:
  if node is None:
    return MISSING
  if isinstance(node, ast.Constant):
    return node.value
  if isinstance(node, ast.Name):
    return env.get(node.id, MISSING)
  if isinstance(node, ast.Attribute):
    base = _eval(node.value, env)
    if isinstance(base, dict) and node.attr in base:
      return base[node.attr]
    return MISSING
  if isinstance(node, (ast.List, ast.Tuple)):
    out = []
    for elt in node.elts:
      value = _eval(elt, env)
      if _is_missing(value):
        return MISSING
      out.append(value)
    return out
  if isinstance(node, ast.Dict):
    result = {}
    for key, value in zip(node.keys, node.values, strict=True):
      k = _eval(key, env)
      v = _eval(value, env)
      if _is_missing(k) or _is_missing(v):
        return MISSING
      result[k] = v
    return result
  if isinstance(node, ast.Subscript):
    container = _eval(node.value, env)
    index = _eval(node.slice, env)
    if _is_missing(container) or _is_missing(index):
      return MISSING
    try:
      return container[index]
    except Exception:
      return MISSING
  if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
    value = _eval(node.operand, env)
    if isinstance(value, (int, float)) and not isinstance(value, bool):
      return -value
    return MISSING
  if isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Add, ast.Sub)):
    left = _eval(node.left, env)
    right = _eval(node.right, env)
    if _is_missing(left) or _is_missing(right):
      return MISSING
    try:
      return left + right if isinstance(node.op, ast.Add) else left - right
    except Exception:
      return MISSING
  if isinstance(node, ast.BoolOp):
    values = [_eval(v, env) for v in node.values]
    if any(_is_missing(v) for v in values):
      return MISSING
    if isinstance(node.op, ast.And):
      return all(values)
    return any(values)
  if isinstance(node, ast.IfExp):
    test = _eval(node.test, env)
    if _is_missing(test):
      return MISSING
    return _eval(node.body if test else node.orelse, env)
  if isinstance(node, ast.JoinedStr):
    return _eval_joined(node, env)
  if isinstance(node, ast.FormattedValue):
    value = _eval(node.value, env)
    if _is_missing(value):
      return MISSING
    spec = ""
    if node.format_spec is not None:
      spec = _eval(node.format_spec, env)
      if _is_missing(spec):
        return MISSING
    if spec == "+d" and isinstance(value, int) and not isinstance(value, bool):
      return f"{value:+d}"
    if spec == "" or spec is None:
      return str(value)
    return MISSING
  if isinstance(node, ast.Lambda):
    return _eval(node.body, env)
  if isinstance(node, ast.ListComp):
    return _eval_listcomp(node, env)
  if isinstance(node, ast.Call):
    return _eval_call(node, env)
  return MISSING


def _eval_joined(node: ast.JoinedStr, env: _Env) -> Any:
  parts: list[str] = []
  for value in node.values:
    piece = _eval(value, env)
    if _is_missing(piece):
      return MISSING
    parts.append(str(piece))
  return "".join(parts)


def _eval_listcomp(node: ast.ListComp, env: _Env) -> Any:
  if len(node.generators) != 1 or node.generators[0].ifs:
    return MISSING
  gen = node.generators[0]
  if not isinstance(gen.target, ast.Name):
    return MISSING
  iterable = _eval(gen.iter, env)
  if not isinstance(iterable, list):
    return MISSING
  out = []
  for item in iterable:
    local = _Env(env)
    local[gen.target.id] = item
    value = _eval(node.elt, local)
    if _is_missing(value):
      return MISSING
    out.append(value)
  return out


def _eval_call(node: ast.Call, env: _Env) -> Any:
  func = node.func
  if isinstance(func, ast.Name) and func.id in ("tr", "tr_noop"):
    return _eval(node.args[0], env) if node.args else MISSING
  if isinstance(func, ast.Name) and func.id == "str" and node.args:
    value = _eval(node.args[0], env)
    return MISSING if _is_missing(value) else str(value)
  if isinstance(func, ast.Name) and func.id in ("int", "float") and node.args:
    value = _eval(node.args[0], env)
    if _is_missing(value):
      return MISSING
    try:
      return int(value) if func.id == "int" else float(value)
    except Exception:
      return MISSING
  if isinstance(func, ast.Name) and func.id == "list" and len(node.args) == 1:
    value = _eval(node.args[0], env)
    return value if isinstance(value, list) else MISSING
  if isinstance(func, ast.Name) and func.id == "range":
    args = []
    for arg in node.args:
      value = _eval(arg, env)
      if not isinstance(value, int) or isinstance(value, bool):
        return MISSING
      args.append(value)
    try:
      return list(range(*args))
    except Exception:
      return MISSING
  if isinstance(func, ast.Attribute) and func.attr == "split" and node.args:
    base = _eval(func.value, env)
    sep = _eval(node.args[0], env)
    if isinstance(base, str) and isinstance(sep, str):
      maxsplit = -1
      if len(node.args) > 1:
        raw = _eval(node.args[1], env)
        if isinstance(raw, int):
          maxsplit = raw
      return base.split(sep, maxsplit)
  return MISSING


def _as_str(value: Any) -> str:
  return value if isinstance(value, str) else ""


def _kw(node: ast.Call, name: str) -> ast.AST | None:
  for keyword in node.keywords:
    if keyword.arg == name:
      return keyword.value
  return None


def _mentions(node: ast.AST, suffix: str) -> bool:
  for child in ast.walk(node):
    if _dotted(child).endswith(suffix):
      return True
  return False


def _lock_flags(node: ast.AST | None) -> dict[str, bool]:
  """How the device disables a control, from an enabled= expression."""
  flags = {"lock_onroad": False, "lock_engaged": False, "disabled": False}
  if node is None:
    return flags
  if isinstance(node, ast.Constant) and node.value is False:
    flags["disabled"] = True
    return flags
  dotted = _dotted(node)
  if dotted.endswith("is_offroad"):
    flags["lock_onroad"] = True
  if isinstance(node, ast.Lambda):
    body = node.body
    if isinstance(body, ast.UnaryOp) and isinstance(body.op, ast.Not):
      if _dotted(body.operand).endswith("engaged"):
        flags["lock_engaged"] = True
    if _mentions(body, "is_offroad"):
      flags["lock_onroad"] = True
    if _mentions(body, "engaged"):
      flags["lock_engaged"] = True
  if dotted.endswith("engaged"):
    flags["lock_engaged"] = True
  return flags


def _blank(panel: str, kind: str, title: str) -> dict[str, Any]:
  return {
    "id": "",
    "panel": panel,
    "section": "",
    "param": "",
    "title": title,
    "description": "",
    "kind": kind,
    "confirm": False,
    "lock_onroad": False,
    "lock_engaged": False,
    "disabled": False,
    "needs_restart": False,
    "reboot_hint": False,
    "writer": "",
    "choices": [],
    "values": [],
    "also_clear": [],
    "lock_param": "",
    "release_hidden": False,
    "hide_param": "",
    "target_panel": "",
    "writable": kind in ("toggle", "choice", "action"),
  }


def _finish(ctrl: dict[str, Any]) -> dict[str, Any]:
  if ctrl["disabled"]:
    ctrl["writable"] = False
  key = ctrl["param"] or ctrl["title"]
  ctrl["id"] = f"{ctrl['panel']}:{key}"
  return ctrl


class _Modules:
  def __init__(self, root: Path, extra_dirs: list[Path]):
    self.root = root
    self.extra_dirs = extra_dirs
    self.cache: dict[str, _Env] = {}
    self.loading: set[str] = set()

  def load(self, module: str) -> _Env:
    if module in self.cache:
      return self.cache[module]
    if module in self.loading or len(self.cache) >= _MAX_MODULES:
      return _Env()
    path = self._path(module)
    if path is None:
      self.cache[module] = _Env()
      return self.cache[module]
    self.loading.add(module)
    try:
      env = self._parse(path.read_text(encoding="utf-8"), path)
    except Exception:
      env = _Env()
    self.loading.discard(module)
    self.cache[module] = env
    return env

  def parse_file(self, path: Path, source: str | None = None) -> tuple[ast.AST, _Env]:
    text = source if source is not None else path.read_text(encoding="utf-8")
    tree = ast.parse(text, filename=str(path))
    env = self._module_env(tree)
    return tree, env

  def _path(self, module: str) -> Path | None:
    parts = module.split(".")
    rels = [Path(*parts)]
    if parts and parts[0] == "openpilot":
      rels.append(Path(*parts[1:]))
    for base in [self.root, *self.extra_dirs]:
      for rel in rels:
        candidate = (base / rel).with_suffix(".py")
        if candidate.is_file():
          return candidate
    return None

  def _parse(self, text: str, path: Path) -> _Env:
    tree = ast.parse(text, filename=str(path))
    return self._module_env(tree)

  def _module_env(self, tree: ast.AST) -> _Env:
    env = _Env()
    for node in tree.body if isinstance(tree, ast.Module) else []:
      if isinstance(node, ast.ImportFrom) and node.module:
        imported = self.load(node.module)
        for alias in node.names:
          if alias.name == "*":
            continue
          if alias.name in imported:
            env[alias.asname or alias.name] = imported[alias.name]
      elif isinstance(node, ast.ClassDef):
        env[node.name] = self._class_env(node, env)
      elif isinstance(node, ast.Assign):
        value = _eval(node.value, env)
        if _is_missing(value):
          continue
        for target in node.targets:
          if isinstance(target, ast.Name):
            env[target.id] = value
      elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.value is not None:
        value = _eval(node.value, env)
        if not _is_missing(value):
          env[node.target.id] = value
    return env

  def _class_env(self, node: ast.ClassDef, env: _Env) -> dict:
    body: dict[str, Any] = {}
    for stmt in node.body:
      if not isinstance(stmt, ast.Assign):
        continue
      value = _eval(stmt.value, _Env({**env, **body}))
      if _is_missing(value):
        continue
      for target in stmt.targets:
        if isinstance(target, ast.Name):
          body[target.id] = value
    return body


def _param_from_get(node: ast.AST, env: _Env) -> str:
  for child in ast.walk(node):
    if not isinstance(child, ast.Call):
      continue
    if _call_name(child.func) not in ("get_bool", "get"):
      continue
    if not child.args:
      continue
    value = _eval(child.args[0], env)
    if isinstance(value, str) and value:
      return value
  return ""


def _puts(fn: ast.AST) -> list[tuple[ast.AST, ast.AST, str]]:
  found = []
  for child in ast.walk(fn):
    if not isinstance(child, ast.Call):
      continue
    name = _call_name(child.func)
    if name not in ("put", "put_bool") or not child.args:
      continue
    found.append((child.args[0], child.args[1] if len(child.args) > 1 else ast.Constant(value=None), name))
  return found


def _analyze_callback(fn: ast.AST | None, env: _Env, n_choices: int) -> dict[str, Any]:
  info: dict[str, Any] = {
    "param": "",
    "writer": "",
    "confirm": False,
    "needs_restart": False,
    "reboot_hint": False,
    "also_clear": [],
    "values": [],
    "lock_engaged": False,
    "lock_onroad": False,
  }
  if fn is None:
    return info
  if _mentions(fn, "engaged"):
    info["lock_engaged"] = True
  if _mentions(fn, "is_offroad"):
    info["lock_onroad"] = True
  for child in ast.walk(fn):
    if isinstance(child, ast.Call) and _call_name(child.func) == "ConfirmDialog":
      info["confirm"] = True
    if isinstance(child, ast.Call) and _call_name(child.func) == "_show_reboot_modal":
      info["reboot_hint"] = True
    if isinstance(child, ast.Call) and _call_name(child.func) == "_handle_experimental_mode_toggle":
      info["writer"] = "experimental"
      info["confirm"] = True
  names = [_call_name(c.func) for c in ast.walk(fn) if isinstance(c, ast.Call)]
  if "apply_dm_simulate_looking" in names:
    info["writer"] = "dm_sim"
  elif "apply_dm_false_alert_ignore" in names:
    info["writer"] = "dm_fai"
  elif "apply_force_offroad_toggle" in names:
    info["writer"] = "force_offroad"
  elif "apply_hypermile_toggle" in names:
    info["writer"] = "hypermile"
  if info["writer"] == "" and _mentions(fn, "ExperimentalModeConfirmed"):
    info["writer"] = "experimental"
    info["confirm"] = True

  clears: list[str] = []
  for param_node, value_node, op in _puts(fn):
    param = _eval(param_node, env)
    if not isinstance(param, str) or not param:
      continue
    if param == "OnroadCycleRequested":
      info["needs_restart"] = True
      continue
    if param == "ExperimentalModeConfirmed":
      info["writer"] = "experimental"
      info["confirm"] = True
      continue
    value = _eval(value_node, env)
    if op == "put_bool" and value is False:
      clears.append(param)
      continue
    if not info["param"]:
      info["param"] = param
      info["values"] = _values_from_expr(value_node, env, n_choices)
      if op == "put_bool":
        info["writer"] = info["writer"] or "bool"
      else:
        info["writer"] = info["writer"] or "choice"
  if info["param"]:
    info["also_clear"] = [key for key in clears if key != info["param"]]
  return info


def _values_from_expr(node: ast.AST, env: _Env, n_choices: int) -> list[Any]:
  if isinstance(node, ast.Name) and node.id in ("index", "button_index", "state"):
    return list(range(n_choices)) if n_choices else []
  if isinstance(node, ast.Call) and _call_name(node.func) == "int" and node.args:
    inner = node.args[0]
    if isinstance(inner, ast.Name):
      return list(range(n_choices)) if n_choices else []
    if isinstance(inner, ast.Subscript):
      return _subscript_list(inner, env)
  if isinstance(node, ast.Call) and _call_name(node.func) == "float" and node.args:
    if isinstance(node.args[0], ast.Subscript):
      return _subscript_list(node.args[0], env)
  if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
    if isinstance(node.left, ast.Name) and isinstance(node.right, ast.Constant) and node.right.value == 1:
      return [i + 1 for i in range(n_choices)] if n_choices else []
  if isinstance(node, ast.Subscript):
    return _subscript_list(node, env)
  return []


def _subscript_list(node: ast.Subscript, env: _Env) -> list[Any]:
  values = _eval(node.value, env)
  return values if isinstance(values, list) else []


def _callback_fn(name: str, methods: dict[str, ast.AST], local: dict[str, ast.AST]) -> ast.AST | None:
  if not name:
    return None
  if name in local:
    return local[name]
  return methods.get(name)


def _fn_name(node: ast.AST | None) -> str:
  if isinstance(node, ast.Attribute):
    return node.attr
  if isinstance(node, ast.Name):
    return node.id
  return ""


def _page_of(fn: ast.AST | None) -> str:
  if fn is None:
    return ""
  for child in ast.walk(fn):
    if not isinstance(child, ast.Assign):
      continue
    for target in child.targets:
      if isinstance(target, ast.Attribute) and target.attr == "_page":
        value = child.value
        if isinstance(value, ast.Constant) and isinstance(value.value, str):
          return value.value
  return ""


def _self_attr(node: ast.AST) -> str:
  if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "self":
    return node.attr
  if isinstance(node, ast.Attribute):
    return _self_attr(node.value)
  return ""


class _Builder:
  def __init__(self, env: _Env, methods: dict[str, ast.FunctionDef], panel: str):
    self.env = env
    self.methods = methods
    self.panel = panel
    self.controls: list[dict[str, Any]] = []
    self.by_attr: dict[str, dict[str, Any]] = {}
    self.deferred: list[dict[str, Any]] = []
    self.section = ""
    self.pending_personality: dict[str, Any] | None = None
    self._defs: list[tuple[str, dict[str, Any]]] | None = None

  def build(self, fn: ast.FunctionDef, panel: str | None = None) -> None:
    previous = self.panel
    previous_deferred = self.deferred
    self.deferred = []
    if panel:
      self.panel = panel
      self.section = ""
    local = {node.name: node for node in fn.body if isinstance(node, ast.FunctionDef)}
    for stmt in fn.body:
      self._stmt(stmt, local)
    if self.pending_personality is not None:
      self._add(self.pending_personality)
      self.pending_personality = None
    for ctrl in self.deferred:
      self._add(ctrl)
    self.deferred = previous_deferred
    self.panel = previous

  def _add(self, ctrl: dict[str, Any]) -> None:
    if ctrl.get("_emitted"):
      return
    ctrl["_emitted"] = True
    ctrl["section"] = ctrl["section"] or self.section
    self.controls.append(_finish(ctrl))

  def _emit_node(self, node: ast.AST, local: dict[str, ast.FunctionDef]) -> None:
    if isinstance(node, (ast.List, ast.Tuple)):
      for elt in node.elts:
        self._emit_node(elt, local)
      return
    if isinstance(node, ast.Call) and _call_name(node.func) == "section_header_item" and node.args:
      title = _eval(node.args[0], self.env)
      if isinstance(title, str):
        self.section = title
      return
    if isinstance(node, ast.Call):
      ctrl = self._from_call(node, local)
      if ctrl is not None:
        self._add(ctrl)
      return
    attr = _self_attr(node)
    if attr and attr in self.by_attr:
      self._add(self.by_attr[attr])

  def _stmt(self, stmt: ast.AST, local: dict[str, ast.FunctionDef]) -> None:
    if isinstance(stmt, ast.Assign):
      self._assign(stmt, local)
    elif isinstance(stmt, ast.Expr) and isinstance(stmt.value, ast.Call):
      self._expr_call(stmt.value, local)
    elif isinstance(stmt, ast.For):
      self._for(stmt)
    elif isinstance(stmt, ast.If):
      return

  def _assign(self, stmt: ast.Assign, local: dict[str, ast.FunctionDef]) -> None:
    if len(stmt.targets) != 1:
      return
    target = stmt.targets[0]
    attr = _self_attr(target) if isinstance(target, ast.Attribute) else ""
    if attr == "_toggle_defs":
      self._capture_defs(stmt.value)
      return
    if isinstance(stmt.value, (ast.List, ast.Tuple)):
      self._emit_node(stmt.value, local)
      return
    if not isinstance(stmt.value, ast.Call):
      return
    # Scroller([self._toggle, ...]) — display order lives in the list arg.
    if stmt.value.args and isinstance(stmt.value.args[0], (ast.List, ast.Tuple)):
      if _call_name(stmt.value.func) in ("Scroller",):
        self._emit_node(stmt.value.args[0], local)
        return
    ctrl = self._from_call(stmt.value, local)
    if ctrl is None:
      return
    if attr == "_long_personality_setting":
      self.pending_personality = ctrl
      return
    if attr:
      self.by_attr[attr] = ctrl
      self.deferred.append(ctrl)
      return
    self._add(ctrl)

  def _expr_call(self, call: ast.Call, local: dict[str, ast.FunctionDef]) -> None:
    name = _call_name(call.func)
    if name == "section_header_item" and call.args:
      title = _eval(call.args[0], self.env)
      if isinstance(title, str):
        self.section = title
      return
    if name == "_build_speed_sign_items":
      # Tici Speed Sign Reader submenu: keep its rows on the phone dash NAP panel.
      fn = self.methods.get("_build_speed_sign_items")
      if fn is not None:
        self.build(fn)
      return
    if name == "_build_radar_items":
      fn = self.methods.get("_build_radar_items")
      if fn is not None:
        self.build(fn, "nap_radar")
      return
    if name in ("append",) and call.args:
      self._emit_node(call.args[0], local)
      return
    if name == "_add_toggle":
      ctrl = self._from_add_toggle(call)
      if ctrl is not None:
        self._add(ctrl)
      return
    ctrl = self._from_call(call, local)
    if ctrl is not None and name in ("toggle_item", "multiple_button_item", "button_item", "dual_button_item", "text_item"):
      self._add(ctrl)

  def _for(self, stmt: ast.For) -> None:
    iter_name = ""
    if isinstance(stmt.iter, ast.Call):
      iter_name = _dotted(stmt.iter.func)
    if self._defs is not None and iter_name.endswith("_toggle_defs.items"):
      insert_after = ""
      for child in ast.walk(stmt):
        if isinstance(child, ast.Compare) and child.ops and isinstance(child.ops[0], ast.Eq):
          if child.comparators and isinstance(child.comparators[0], ast.Constant):
            if isinstance(child.comparators[0].value, str):
              insert_after = child.comparators[0].value
      for param, ctrl in self._defs:
        self._add(ctrl)
        if param == insert_after and self.pending_personality is not None:
          self._add(self.pending_personality)
          self.pending_personality = None
      self._defs = None

  def _capture_defs(self, node: ast.AST) -> None:
    if not isinstance(node, ast.Dict):
      return
    defs = []
    for key, value in zip(node.keys, node.values, strict=True):
      param = _eval(key, self.env)
      if not isinstance(param, str) or not isinstance(value, ast.Tuple) or len(value.elts) < 2:
        continue
      title = _as_str(_eval(value.elts[0], self.env))
      desc = _as_str(_eval(value.elts[1], self.env))
      needs_restart = False
      if len(value.elts) >= 4:
        flag = _eval(value.elts[3], self.env)
        needs_restart = flag is True
      ctrl = _blank(self.panel, "toggle", title)
      ctrl["param"] = param
      ctrl["description"] = desc
      ctrl["needs_restart"] = needs_restart
      ctrl["lock_engaged"] = needs_restart
      ctrl["writer"] = "bool"
      ctrl["lock_param"] = param + "Lock"
      if param == "ExperimentalMode":
        ctrl["writer"] = "experimental"
        ctrl["confirm"] = True
        ctrl["description"] = ctrl["description"] or _experimental_blurb(self.methods.get("_update_toggles"), self.env)
      if needs_restart and "restart openpilot" not in ctrl["description"]:
        ctrl["description"] = (ctrl["description"] + " " if ctrl["description"] else "") + (
          "Changing this setting will restart openpilot if the car is powered on."
        )
      defs.append((param, ctrl))
    self._defs = defs

  def _from_add_toggle(self, call: ast.Call) -> dict[str, Any] | None:
    if len(call.args) < 2:
      return None
    param = _eval(call.args[0], self.env)
    title = _eval(call.args[1], self.env)
    if not isinstance(param, str) or not isinstance(title, str):
      return None
    desc = _as_str(_eval(call.args[2], self.env)) if len(call.args) > 2 else ""
    ctrl = _blank(self.panel, "toggle", title)
    ctrl["param"] = param
    ctrl["description"] = desc
    ctrl["writer"] = "bool"
    flags = _lock_flags(_kw(call, "enabled"))
    ctrl.update({k: ctrl[k] or flags[k] for k in ("lock_onroad", "lock_engaged", "disabled")})
    reboot = _eval(_kw(call, "needs_reboot"), self.env)
    if reboot is True:
      ctrl["reboot_hint"] = True
    return ctrl

  def _from_call(self, call: ast.Call, local: dict[str, ast.FunctionDef]) -> dict[str, Any] | None:
    name = _call_name(call.func)
    if name == "toggle_item":
      return self._toggle(call, local)
    if name == "multiple_button_item":
      return self._choice(call, local)
    if name == "button_item":
      return self._button(call, local)
    if name == "dual_button_item":
      return None
    if name == "text_item":
      return self._text(call)
    return None

  def _toggle(self, call: ast.Call, local: dict[str, ast.FunctionDef]) -> dict[str, Any] | None:
    title = _as_str(_eval(call.args[0], self.env)) if call.args else ""
    if not title:
      return None
    desc_node = _kw(call, "description")
    desc = _as_str(_eval(desc_node, self.env)) if desc_node is not None else ""
    param = ""
    state = _kw(call, "initial_state")
    if state is not None:
      param = _param_from_get(state, self.env)
    cb = _callback_fn(_fn_name(_kw(call, "callback")), self.methods, local)
    info = _analyze_callback(cb, self.env, 0)
    param = param or info["param"]
    if not param:
      return None
    ctrl = _blank(self.panel, "toggle", title)
    ctrl["param"] = param
    ctrl["description"] = desc
    ctrl["writer"] = info["writer"] or "bool"
    ctrl["confirm"] = info["confirm"]
    ctrl["needs_restart"] = info["needs_restart"]
    ctrl["reboot_hint"] = info["reboot_hint"]
    ctrl["also_clear"] = info["also_clear"]
    ctrl["lock_engaged"] = info["lock_engaged"]
    ctrl["lock_onroad"] = info["lock_onroad"]
    flags = _lock_flags(_kw(call, "enabled"))
    for key in ("lock_onroad", "lock_engaged", "disabled"):
      ctrl[key] = ctrl[key] or flags[key]
    if ctrl["needs_restart"]:
      ctrl["lock_engaged"] = True
    return ctrl

  def _choice(self, call: ast.Call, local: dict[str, ast.FunctionDef]) -> dict[str, Any] | None:
    title = _as_str(_eval(call.args[0], self.env)) if call.args else ""
    if not title:
      return None
    desc = _as_str(_eval(call.args[1], self.env)) if len(call.args) > 1 else ""
    if not desc:
      desc = _as_str(_eval(_kw(call, "description"), self.env))
    buttons = _eval(_kw(call, "buttons"), self.env)
    if not isinstance(buttons, list) or not buttons:
      buttons = []
    labels = [str(item) for item in buttons]
    cb = _callback_fn(_fn_name(_kw(call, "callback")), self.methods, local)
    info = _analyze_callback(cb, self.env, len(labels))
    if not info["param"]:
      return None
    values = info["values"] or list(range(len(labels)))
    if labels and values and len(labels) != len(values):
      labels = [str(v) for v in values]
    if not labels:
      labels = [str(v) for v in values]
    ctrl = _blank(self.panel, "choice", title)
    ctrl["param"] = info["param"]
    ctrl["description"] = desc
    ctrl["writer"] = "choice"
    ctrl["choices"] = labels
    ctrl["values"] = values
    ctrl["needs_restart"] = info["needs_restart"]
    ctrl["reboot_hint"] = info["reboot_hint"]
    ctrl["confirm"] = info["confirm"]
    ctrl["lock_engaged"] = info["lock_engaged"] or info["needs_restart"]
    ctrl["lock_onroad"] = info["lock_onroad"]
    return ctrl

  def _button(self, call: ast.Call, local: dict[str, ast.FunctionDef]) -> dict[str, Any] | None:
    title = _as_str(_eval(call.args[0], self.env)) if call.args else ""
    if not title or title == "Back To":
      return None
    desc = ""
    desc_node = _kw(call, "description")
    if desc_node is None and len(call.args) >= 3:
      desc_node = call.args[2]
    if desc_node is not None:
      desc = _as_str(_eval(desc_node, self.env))
    cb_name = _fn_name(_kw(call, "callback"))
    if not cb_name and len(call.args) >= 4:
      cb_name = _fn_name(call.args[3])
    cb = _callback_fn(cb_name, self.methods, local)
    page = _page_of(cb)
    if page in _PAGE_PANEL:
      ctrl = _blank(self.panel, "link", title)
      ctrl["description"] = desc
      ctrl["target_panel"] = _PAGE_PANEL[page]
      ctrl["writable"] = False
      return ctrl
    info = _analyze_callback(cb, self.env, 0)
    if info["param"] in ("DoReboot", "DoShutdown"):
      ctrl = _blank(self.panel, "action", title)
      ctrl["param"] = info["param"]
      ctrl["description"] = desc
      ctrl["writer"] = "action"
      ctrl["confirm"] = True
      ctrl["lock_engaged"] = True
      return ctrl
    ctrl = _blank(self.panel, "device_only", title)
    ctrl["description"] = desc or "Use this on the comma screen."
    ctrl["writable"] = False
    return ctrl

  def _text(self, call: ast.Call) -> dict[str, Any] | None:
    title = _as_str(_eval(call.args[0], self.env)) if call.args else ""
    if not title:
      return None
    param = ""
    if len(call.args) > 1:
      param = _param_from_get(call.args[1], self.env)
    if not param:
      return None
    ctrl = _blank(self.panel, "text", title)
    ctrl["param"] = param
    ctrl["writable"] = False
    ctrl["writer"] = ""
    return ctrl


def _experimental_blurb(fn: ast.AST | None, env: _Env) -> str:
  if fn is None:
    return ""
  for stmt in fn.body:
    if isinstance(stmt, ast.Assign):
      for target in stmt.targets:
        if isinstance(target, ast.Name) and target.id == "e2e_description":
          text = _eval(stmt.value, env)
          if isinstance(text, str):
            return text
  return ""


def _apply_later_locks(class_node: ast.ClassDef, by_attr: dict[str, dict[str, Any]]) -> None:
  for method in class_node.body:
    if not isinstance(method, ast.FunctionDef):
      continue
    names = _offroad_names(method)
    for stmt in method.body:
      _lock_stmt(stmt, by_attr, names, inside_if=False)


def _offroad_names(method: ast.FunctionDef) -> set[str]:
  found = set()
  for node in ast.walk(method):
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
      if _mentions(node.value, "is_offroad"):
        found.add(node.targets[0].id)
  return found


def _lock_stmt(stmt: ast.AST, by_attr: dict[str, dict], names: set[str], inside_if: bool) -> None:
  if isinstance(stmt, ast.If):
    for child in stmt.body + stmt.orelse:
      _lock_stmt(child, by_attr, names, inside_if=True)
    return
  if isinstance(stmt, (ast.For, ast.While, ast.With, ast.Try)):
    return
  call = stmt.value if isinstance(stmt, ast.Expr) and isinstance(stmt.value, ast.Call) else None
  if call is None or _call_name(call.func) != "set_enabled" or not call.args:
    return
  attr = _self_attr(call.func)
  ctrl = by_attr.get(attr)
  if ctrl is None:
    return
  arg = call.args[0]
  if isinstance(arg, ast.Constant) and arg.value is False and not inside_if:
    ctrl["disabled"] = True
    ctrl["writable"] = False
    return
  if _mentions(arg, "is_offroad") or (isinstance(arg, ast.Name) and arg.id in names):
    ctrl["lock_onroad"] = True
  flags = _lock_flags(arg)
  ctrl["lock_onroad"] = ctrl["lock_onroad"] or flags["lock_onroad"]
  ctrl["lock_engaged"] = ctrl["lock_engaged"] or flags["lock_engaged"]


def _release_hidden(class_node: ast.ClassDef, by_attr: dict[str, dict[str, Any]]) -> None:
  for method in class_node.body:
    if not isinstance(method, ast.FunctionDef):
      continue
    for node in ast.walk(method):
      if not isinstance(node, ast.For):
        continue
      if not isinstance(node.iter, (ast.Tuple, ast.List)):
        continue
      attrs = []
      for elt in node.iter.elts:
        if isinstance(elt, ast.Attribute) and isinstance(elt.value, ast.Name) and elt.value.id == "self":
          attrs.append(elt.attr)
      hides = False
      for child in ast.walk(node):
        if isinstance(child, ast.Call) and _call_name(child.func) == "set_visible":
          if child.args and _mentions(child.args[0], "_is_release"):
            hides = True
      if not hides:
        continue
      for attr in attrs:
        ctrl = by_attr.get(attr)
        if ctrl is not None:
          ctrl["release_hidden"] = True
          ctrl["hide_param"] = "IsReleaseBranch"


def _dual_buttons(class_node: ast.ClassDef, env: _Env, methods: dict[str, ast.FunctionDef], panel: str) -> list[dict]:
  found = []
  for method in class_node.body:
    if not isinstance(method, ast.FunctionDef):
      continue
    for node in ast.walk(method):
      if not isinstance(node, ast.Call) or _call_name(node.func) != "dual_button_item":
        continue
      if len(node.args) < 2:
        continue
      pairs = (
        (_as_str(_eval(node.args[0], env)), _fn_name(_kw(node, "left_callback"))),
        (_as_str(_eval(node.args[1], env)), _fn_name(_kw(node, "right_callback"))),
      )
      for title, cb_name in pairs:
        info = _analyze_callback(methods.get(cb_name), env, 0)
        if info["param"] not in ("DoReboot", "DoShutdown") or not title:
          continue
        ctrl = _blank(panel, "action", title)
        ctrl["param"] = info["param"]
        ctrl["writer"] = "action"
        ctrl["confirm"] = True
        ctrl["lock_engaged"] = True
        ctrl["description"] = "Confirmation required. Unavailable while openpilot is engaged."
        found.append(_finish(ctrl))
  return found


def _methods(class_node: ast.ClassDef) -> dict[str, ast.FunctionDef]:
  return {node.name: node for node in class_node.body if isinstance(node, ast.FunctionDef)}


def _entry(methods: dict[str, ast.FunctionDef]) -> ast.FunctionDef | None:
  for name in _BUILD_METHODS:
    if name in methods:
      return methods[name]
  return None


def _parse_sidebar(tree: ast.AST, env: _Env) -> list[dict[str, str]]:
  panels = []
  for node in ast.walk(tree):
    if not isinstance(node, ast.Call) or _call_name(node.func) != "PanelInfo":
      continue
    if len(node.args) < 2:
      continue
    title = _as_str(_eval(node.args[0], env))
    cls = _call_name(node.args[1]) if isinstance(node.args[1], ast.Call) else ""
    panel_id = _PANEL_CLASS.get(cls) or re.sub(r"[^a-z0-9]+", "_", title.lower()).strip("_")
    if title and panel_id:
      panels.append({"id": panel_id, "title": title, "parent": "", "layout": panel_id if panel_id in ("software", "network") else "controls"})
  return panels


def _hotspot(root: Path) -> dict[str, Any]:
  text = (root / "system" / "ui" / "lib" / "wifi_manager.py").read_text(encoding="utf-8")
  ip_match = re.search(r'^TETHERING_IP_ADDRESS\s*=\s*"([^"]+)"', text, re.M)
  prefix_match = re.search(r'^TETHERING_PREFIX\s*=\s*(\d+)', text, re.M)
  address = ip_match.group(1) if ip_match else "100.99.9.1"
  prefix = int(prefix_match.group(1)) if prefix_match else 24
  return {"address": address, "prefix": prefix, "url": f"http://{address}/"}


def _param_types(root: Path) -> dict[str, str]:
  path = root / "common" / "params_keys.h"
  if not path.is_file():
    return {}
  types = {}
  for name, body in re.findall(r'\{\s*"([A-Za-z0-9_]+)"\s*,\s*\{([^}]*)\}\s*\}', path.read_text(encoding="utf-8")):
    if re.search(r"\bFLOAT\b", body):
      types[name] = "float"
    elif re.search(r"\bBOOL\b", body):
      types[name] = "bool"
    elif re.search(r"\bINT\b", body):
      types[name] = "int"
    elif re.search(r"\b(STRING|JSON)\b", body):
      types[name] = "str"
    else:
      types[name] = "bytes"
  return types


def _panel_title_from_doc(path: Path) -> str:
  try:
    tree = ast.parse(path.read_text(encoding="utf-8"))
  except Exception:
    return ""
  doc = ast.get_docstring(tree) or ""
  match = re.search(r"submenu:\s*(.+?)\s+controls", doc)
  if match:
    return match.group(1).strip()
  return ""


def discover_manifest(root: Path | None = None, sources: dict[str, str] | None = None,
                      extra_module_dirs: list[Path] | None = None) -> dict[str, Any]:
  """Build panels + controls from the on-device settings sources."""
  root = repo_root() if root is None else root
  sources = sources or {}
  modules = _Modules(root, list(extra_module_dirs or []))
  settings_path = root / "selfdrive" / "ui" / "layouts" / "settings" / "settings.py"
  settings_tree, settings_env = modules.parse_file(settings_path, sources.get("settings.py"))
  panels = _parse_sidebar(settings_tree, settings_env)
  known = {panel["id"] for panel in panels}

  controls: list[dict[str, Any]] = []
  sub_titles: dict[str, str] = {}
  settings_dir = settings_path.parent
  for filename, panel in _FILE_PANEL.items():
    path = settings_dir / filename
    if not path.is_file() and filename not in sources:
      continue
    tree, env = modules.parse_file(path, sources.get(filename))
    if filename == "firehose.py":
      desc = env.get("DESCRIPTION") if isinstance(env.get("DESCRIPTION"), str) else ""
      extra = env.get("INSTRUCTIONS") if isinstance(env.get("INSTRUCTIONS"), str) else ""
      title = env.get("TITLE") if isinstance(env.get("TITLE"), str) else "Firehose Mode"
      if desc:
        ctrl = _blank("firehose", "info", title)
        ctrl["description"] = desc + ("\n\n" + extra if extra else "")
        ctrl["writable"] = False
        controls.append(_finish(ctrl))
      continue
    for node in tree.body:
      if not isinstance(node, ast.ClassDef):
        continue
      methods = _methods(node)
      entry = _entry(methods)
      if entry is None:
        continue
      builder = _Builder(env, methods, panel)
      builder.build(entry)
      _apply_later_locks(node, builder.by_attr)
      _release_hidden(node, builder.by_attr)
      controls.extend(builder.controls)
      if filename == "device.py":
        controls.extend(_dual_buttons(node, env, methods, panel))
    if panel not in known:
      sub_titles.setdefault(panel, _panel_title_from_doc(path))

  for ctrl in controls:
    if ctrl["kind"] == "link" and ctrl["target_panel"]:
      sub_titles[ctrl["target_panel"]] = ctrl["title"]

  children = [
    ("nap_mannerisms", sub_titles.get("nap_mannerisms") or "Driving Mannerisms"),
    ("nap_map", sub_titles.get("nap_map") or "Map Speed Limit"),
    ("nap_radar", sub_titles.get("nap_radar") or "Radar"),
    ("nap_hidden", "Hidden"),
  ]
  # Hidden stays reachable: on the comma it is a triple-tap, not a sidebar tab.
  existing = {panel["id"] for panel in panels}
  nap_at = next((i for i, panel in enumerate(panels) if panel["id"] == "nap"), len(panels) - 1)
  insert_at = nap_at + 1
  for panel_id, title in children:
    if panel_id in existing:
      continue
    if not any(ctrl["panel"] == panel_id for ctrl in controls):
      continue
    panels.insert(insert_at, {"id": panel_id, "title": title, "parent": "nap", "layout": "controls"})
    insert_at += 1

  for ctrl in controls:
    ctrl.pop("_emitted", None)

  hotspot = _hotspot(root)
  network = _blank("network", "info", "Comma hotspot")
  subnet = hotspot["address"].rsplit(".", 1)[0]
  network["description"] = " ".join([
    "Enable tethering on the comma (Settings, Network, Enable Tethering).",
    f"The hotspot hands out {subnet}.0/{hotspot['prefix']} with the comma at {hotspot['address']}.",
    f"Open {hotspot['url']}.",
    "Joining a home Wi-Fi network is unchanged; this page is also served there.",
    "A phone or the Tesla browser that uses the hotspot for internet is using the comma's LTE data.",
  ])
  network["writable"] = False
  controls.insert(0, _finish(network))

  # Drop device-only script buttons that are not settings. Keep reboot/power.
  # Speed-limit and every other real toggle stays, including ones we did not name here.
  return {
    "hotspot": hotspot,
    "panels": panels,
    "controls": controls,
    "param_types": _param_types(root),
  }
