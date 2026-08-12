#!/usr/bin/env python3
"""Build and audit small, deterministic public Pre-AP replay fixtures."""
from __future__ import annotations

import argparse
import base64
import binascii
import json
import os
import re
import sys
import tempfile
from collections import Counter
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import capnp
import zstandard
from cereal import car, log as capnp_log
from openpilot.tools.lib.logreader import LogReader

SCHEMA_VERSION = 1
PLACEHOLDER_VIN = "00000000000000000"
VIN_LENGTH = 17

EXPECTED_SAFETY_MODEL = car.CarParams.SafetyModel.teslaPreap
# Fixed Pre-AP replay params enable pedal + radar emulation + nosecone => 7 / 6.
PREAP_FLAG_ENABLE_PEDAL = 1
PREAP_FLAG_RADAR_EMULATION = 2
PREAP_FLAG_RADAR_BEHIND_NOSECONE = 4

CASE_MODES = frozenset({"pedal", "no-pedal"})
SAFE_LABEL_RE = re.compile(r"^[a-z][a-z0-9_]{0,31}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
HEX_VALUE_RE = re.compile(r"^[0-9a-f]*$")
DENY_ENCODINGS = frozenset({"utf8", "hex", "base64"})
VIN_FRAGMENT_MIN = 6
VIN_FRAGMENT_MAX = 17
CAN_OVERLAP = 16
MAX_COMPRESSED_INPUT_BYTES = 64 * 1024 * 1024
MAX_DECOMPRESSED_INPUT_BYTES = 512 * 1024 * 1024
MAX_LOG_MESSAGES = 2_000_000
MAX_CAN_CONCAT_BYTES = 4 * 1024 * 1024
VALID_SHARE_THRESHOLD = 0.9
ZSTD_CONTENTSIZE_UNKNOWN = (1 << 64) - 1
ZSTD_DECOMPRESS_CHUNK = 1024 * 1024
# Retained-as-is public Text constants. can[].dat is a scan sink only — never an
# inventory exemption for values that also originate from private/scrubbed leaves.
ATTESTED_PUBLIC_TEXT_DATA_LEAVES = frozenset({
  "carParams.brand",
  "carParams.carFingerprint",
})

SIX_PROCESS_OUTPUTS: dict[str, frozenset[str]] = {
  "card": frozenset({"sendcan", "carState", "carParams", "carOutput", "liveTracks"}),
  "controlsd": frozenset({"carControl", "controlsState"}),
  "selfdrived": frozenset({"selfdriveState", "onroadEvents"}),
  "radard": frozenset({"radarState"}),
  "plannerd": frozenset({"longitudinalPlan", "driverAssistance"}),
  "lagd": frozenset({"liveDelay"}),
}

POLICY_KEYS = frozenset({
  "schema_version", "allowed_services", "forbidden_services", "required_services",
  "selected_gps_service", "freshness_services", "pedal", "sensitive_zero_fields",
  "placeholder_vin", "cases",
})
PEDAL_KEYS = frozenset({
  "address", "bus", "data_length", "counter_index", "counter_mask",
  "checksum_index", "checksum", "minimum_frames", "minimum_counters",
})
CASE_KEYS = frozenset({"mode", "fingerprint", "safety_param"})
DENY_REQUIRED_KEYS = frozenset({"raw_input_sha256", "tokens"})
DENY_OPTIONAL_KEYS = frozenset({"sanitized_output_sha256", "input_sha256"})
# Attested release policy digest over POLICY_KEYS (canonical JSON). Weakened
# caller policies fail closed even when self-consistent.
RELEASE_POLICY_SHA256 = "8b37a7881af652246d5f7323dbd01f9e6c117440ee8d17fad77c88dfc3a1354f"
PEDAL_MIN_FRAMES = 10
PEDAL_MIN_COUNTERS = 8
ZSTD_MAGIC = b"\x28\xb5\x2f\xfd"


def safety_model_ordinal(value: Any) -> int:
  """SafetyModel ordinal. Static enums int(); DynamicEnum needs .raw (str() is "37")."""
  return int(getattr(value, "raw", value))


def safety_model_is_tesla_preap(value: Any) -> bool:
  return safety_model_ordinal(value) == int(EXPECTED_SAFETY_MODEL)


def expected_safety_param(mode: str) -> int:
  flags = PREAP_FLAG_RADAR_EMULATION | PREAP_FLAG_RADAR_BEHIND_NOSECONE
  if mode == "pedal":
    flags |= PREAP_FLAG_ENABLE_PEDAL
  return int(flags)


@dataclass(frozen=True)
class DenyManifest:
  tokens: tuple[tuple[str, bytes], ...]
  raw_input_sha256: str
  sanitized_output_sha256: str | None
  vin: bytes

MANDATORY_FORBIDDEN = frozenset({
  "clocks", "initData", "boot", "sentinel",
  "roadCameraState", "driverCameraState", "wideRoadCameraState",
  "logMessage", "errorLogMessage", "androidLog", "procLog",
  "roadEncodeIdx", "driverEncodeIdx", "wideRoadEncodeIdx", "qRoadEncodeIdx",
  "livestreamRoadEncodeIdx", "livestreamWideRoadEncodeIdx", "livestreamDriverEncodeIdx",
  "roadEncodeData", "driverEncodeData", "wideRoadEncodeData", "qRoadEncodeData",
  "livestreamRoadEncodeData", "livestreamWideRoadEncodeData", "livestreamDriverEncodeData",
  "touch",
  "qcomGnss", "ubloxRaw", "ubloxGnss", "gnssMeasurements", "gpsNMEA",
  "uiDebug", "navInstruction", "navRoute", "navThumbnail", "mapRenderState", "thumbnail",
  "managerState", "uploaderState",
  "rawAudioData", "soundPressure", "audioFeedback",
  "sendcan",
})

SENSITIVE_ZERO_BASELINE = frozenset({
  "gpsLocation.latitude", "gpsLocation.longitude", "gpsLocation.altitude",
  "gpsLocation.bearingDeg", "gpsLocation.unixTimestampMillis", "gpsLocation.vNED[]",
  "gpsLocationExternal.latitude", "gpsLocationExternal.longitude", "gpsLocationExternal.altitude",
  "gpsLocationExternal.bearingDeg", "gpsLocationExternal.unixTimestampMillis", "gpsLocationExternal.vNED[]",
})


class FixtureError(ValueError):
  pass


def digest(data: bytes) -> str:
  import hashlib
  return hashlib.sha256(data).hexdigest()


def field_parts(path: str) -> list[str]:
  parts: list[str] = []
  for part in path.split("."):
    if not part:
      continue
    if part == "[]":
      parts.append(part)
    elif part.endswith("[]"):
      parts.extend((part[:-2], "[]"))
    else:
      parts.append(part)
  return parts


def path_join(prefix: str, name: str) -> str:
  return name if not prefix else f"{prefix}.{name}"


def is_list_service(service: str) -> bool:
  field = capnp_log.Event.schema.fields[service]
  return field.proto.slot.type.which() == "list"


def event_service_names() -> set[str]:
  return set(capnp_log.Event.schema.union_fields)


def schema_for_service(service: str) -> Any:
  field = capnp_log.Event.schema.fields[service]
  t = field.proto.slot.type
  if t.which() == "list":
    return field.schema.elementType
  if t.which() == "struct":
    return field.schema
  raise FixtureError(f"unsupported service schema: {service}")


def resolve_policy_path(service: str, path: str) -> None:
  """Fail closed if a policy path cannot be resolved in the current Cap'n Proto schema."""
  parts = field_parts(path)
  if not parts:
    raise FixtureError(f"empty policy path for {service}")
  schema = schema_for_service(service)
  index = 0
  # Event-level list services use a leading [] marker before element fields.
  if is_list_service(service):
    if parts[0] != "[]":
      raise FixtureError(f"list service paths require leading []: {service}.{path}")
    index = 1
    if index == len(parts):
      raise FixtureError(f"list service path requires a field: {service}.{path}")
  while index < len(parts):
    part = parts[index]
    if part == "[]":
      raise FixtureError(f"invalid list marker in {service}.{path}")
    if part not in schema.fieldnames:
      raise FixtureError(f"policy field does not exist: {service}.{path}")
    field = schema.fields[part]
    index += 1
    if field.proto.which() == "group":
      if index == len(parts):
        raise FixtureError(f"path ends at group: {service}.{path}")
      schema = field.schema
      continue
    if field.proto.which() != "slot":
      raise FixtureError(f"unsupported field kind in {service}.{path}")
    twhich = field.proto.slot.type.which()
    if twhich == "list":
      if index >= len(parts) or parts[index] != "[]":
        raise FixtureError(f"list field requires []: {service}.{path}")
      index += 1
      elem = field.proto.slot.type.list.elementType
      if index == len(parts):
        if elem.which() == "struct":
          raise FixtureError(f"struct list requires subfield: {service}.{path}")
        return
      if elem.which() != "struct":
        raise FixtureError(f"cannot descend into scalar list: {service}.{path}")
      schema = field.schema.elementType
    elif twhich == "struct":
      if index == len(parts):
        raise FixtureError(f"path ends at struct: {service}.{path}")
      schema = field.schema
    else:
      if index != len(parts):
        raise FixtureError(f"extra path after leaf: {service}.{path}")
      return
  raise FixtureError(f"path ends at non-leaf: {service}.{path}")


def expand_allowed_paths(service: str, paths: list[str]) -> set[str]:
  """Canonical leaf path set for membership checks (with [] markers preserved)."""
  resolved: set[str] = set()
  for path in paths:
    resolve_policy_path(service, path)
    resolved.add(".".join(field_parts(path)))
  return resolved


def copy_path(source: Any, destination: Any, path: str) -> None:
  parts = field_parts(path)

  def walk(src: Any, dst: Any, remaining: list[str]) -> None:
    if not remaining:
      raise FixtureError("empty policy field path")
    part = remaining[0]
    if part == "[]":
      if len(remaining) == 1:
        for index, src_item in enumerate(src):
          dst[index] = src_item
        return
      for src_item, dst_item in zip(src, dst, strict=True):
        walk(src_item, dst_item, remaining[1:])
      return

    if hasattr(src, "schema") and part in src.schema.union_fields:
      try:
        active = src.which()
      except capnp.KjException:
        return
      if active != part:
        return
      try:
        dst_branch = dst.init(part)
      except (AttributeError, TypeError, capnp.KjException) as error:
        raise FixtureError(f"cannot initialize union branch: {path}") from error
      walk(getattr(src, part), dst_branch, remaining[1:])
      return

    try:
      src_value = getattr(src, part)
    except (AttributeError, TypeError, capnp.KjException) as error:
      raise FixtureError(f"policy field does not exist: {path}") from error

    if len(remaining) == 1:
      try:
        setattr(dst, part, src_value)
      except (AttributeError, TypeError, capnp.KjException) as error:
        raise FixtureError(f"cannot copy policy field: {path}") from error
      return

    try:
      if remaining[1] == "[]":
        src_list = src_value
        existing = getattr(dst, part)
        if len(existing) == len(src_list) and len(existing) > 0:
          dst_value = existing
        else:
          dst_value = dst.init(part, len(src_list))
      elif part in getattr(dst, "schema", type("x", (), {"union_fields": ()})).union_fields:
        # only init inactive/missing union branch
        try:
          if dst.which() == part:
            dst_value = getattr(dst, part)
          else:
            dst_value = dst.init(part)
        except capnp.KjException:
          dst_value = dst.init(part)
      else:
        dst_value = getattr(dst, part)
    except (AttributeError, TypeError, capnp.KjException) as error:
      raise FixtureError(f"policy field does not exist: {path}") from error
    walk(src_value, dst_value, remaining[1:])

  walk(source, destination, parts)


def copy_service(source: Any, destination: Any, service: str, paths: list[str]) -> None:
  try:
    source_value = getattr(source, service)
  except AttributeError as error:
    raise FixtureError(f"source service is unavailable: {service}") from error
  try:
    if is_list_service(service):
      destination_value = destination.init(service, len(source_value))
    else:
      destination_value = destination.init(service)
  except (AttributeError, TypeError, capnp.KjException) as error:
    raise FixtureError(f"cannot initialize service: {service}") from error
  for path in paths:
    copy_path(source_value, destination_value, path)


def zero_sensitive_location(location: Any) -> None:
  for field in ("latitude", "longitude", "altitude", "bearingDeg", "unixTimestampMillis"):
    if field in location.schema.fieldnames:
      setattr(location, field, 0)
  if "vNED" in location.schema.fieldnames:
    location.vNED = [0.0 for _ in range(len(location.vNED) or 3)]


def build_event(source: Any, policy: dict[str, Any]) -> Any | None:
  service = source.which()
  allowed = policy["allowed_services"]
  if service not in allowed:
    return None
  event = capnp_log.Event.new_message()
  event.logMonoTime = source.logMonoTime
  event.valid = source.valid
  paths = allowed[service]
  copy_service(source, event, service, paths)
  if service == "carParams":
    event.carParams.carVin = policy["placeholder_vin"]
    if len(event.carParams.carFw):
      event.carParams.carFw = []
  if service == policy["selected_gps_service"] or service in ("gpsLocation", "gpsLocationExternal"):
    zero_sensitive_location(getattr(event, service))
  return event.as_reader()


def default_equal(value: Any, default: Any) -> bool:
  if isinstance(value, (bytes, bytearray)) or isinstance(default, (bytes, bytearray)):
    return bytes(value or b"") == bytes(default or b"")
  if isinstance(value, str) or isinstance(default, str):
    return str(value or "") == str(default or "")
  try:
    if hasattr(value, "to_dict") and hasattr(default, "to_dict"):
      return value.to_dict() == default.to_dict()
  except Exception:
    pass
  try:
    return list(value) == list(default)
  except TypeError:
    return value == default


def populated_leaf_paths(struct: Any, prefix: str = "") -> dict[str, Any]:
  """Return non-default leaf paths using schema reflection."""
  result: dict[str, Any] = {}
  if struct is None:
    return result
  schema = struct.schema
  active_union = None
  if schema.union_fields:
    try:
      active_union = struct.which()
    except capnp.KjException:
      active_union = None

  for name in schema.fieldnames:
    if name in schema.union_fields and name != active_union:
      continue
    field = schema.fields[name]
    leaf = path_join(prefix, name)
    try:
      value = getattr(struct, name)
    except capnp.KjException:
      continue
    if field.proto.which() == "group":
      result.update(populated_leaf_paths(value, leaf))
      continue
    if field.proto.which() != "slot":
      continue
    twhich = field.proto.slot.type.which()
    if twhich == "struct":
      result.update(populated_leaf_paths(value, leaf))
    elif twhich == "list":
      elem = field.proto.slot.type.list.elementType
      list_leaf = f"{leaf}[]"
      if elem.which() == "struct":
        for item in value:
          result.update(populated_leaf_paths(item, list_leaf))
      else:
        if len(value) and any(bool(item) if not isinstance(item, (int, float)) else float(item) != 0 for item in value):
          result[list_leaf] = list(value)
    elif twhich == "text":
      if value:
        result[leaf] = value
    elif twhich == "data":
      if bytes(value):
        result[leaf] = bytes(value)
    elif twhich == "bool":
      if bool(value):
        result[leaf] = bool(value)
    elif twhich == "enum":
      if int(getattr(value, "raw", value)) != 0:
        result[leaf] = str(value)
    else:
      try:
        if float(value) != 0:
          result[leaf] = value
      except Exception:
        if value:
          result[leaf] = value
  return result


def path_matches_allowlist(leaf: str, allowed: set[str]) -> bool:
  if leaf in allowed:
    return True
  # allowlist stores paths with [] markers; leaf paths from populated_leaf_paths also use []
  return leaf in allowed


def _normalize(value: Any) -> Any:
  if isinstance(value, dict):
    return {key: _normalize(item) for key, item in value.items() if item not in (None, "", [], {}, b"")}
  if isinstance(value, list):
    return [_normalize(item) for item in value]
  return value


def _to_dict(value: Any) -> Any:
  if hasattr(value, "to_dict"):
    return _normalize(value.to_dict())
  try:
    return _normalize([item.to_dict() if hasattr(item, "to_dict") else item for item in value])
  except TypeError:
    return value


def validate_field_allowlist(messages: list[Any], policy: dict[str, Any]) -> None:
  allowed_map: dict[str, set[str]] = policy["_allowed_leaf_sets"]
  selected_gps = policy["selected_gps_service"]
  for event in messages:
    service = event.which()
    if service not in allowed_map:
      raise FixtureError(f"undeclared service in output: {service}")
    rebuilt = build_event(event, policy)
    if rebuilt is None:
      raise FixtureError(f"undeclared service in output: {service}")
    # Compare against a copy whose sensitive GPS zeros match sanitize semantics so
    # allowlisting detects undeclared leaves, not sensitive-zero violations.
    if service == selected_gps or service in ("gpsLocation", "gpsLocationExternal"):
      original_builder = event.as_builder()
      zero_sensitive_location(getattr(original_builder, service))
      original = _to_dict(getattr(original_builder, service))
      payload = getattr(original_builder, service)
    else:
      original = _to_dict(getattr(event, service))
      payload = getattr(event, service)
    canonical = _to_dict(getattr(rebuilt, service))
    if original == canonical:
      continue
    rebuilt_payload = getattr(rebuilt, service)
    if is_list_service(service):
      leaves: dict[str, Any] = {}
      canon_leaves: dict[str, Any] = {}
      for item in payload:
        leaves.update(populated_leaf_paths(item, "[]"))
      for item in rebuilt_payload:
        canon_leaves.update(populated_leaf_paths(item, "[]"))
    else:
      leaves = populated_leaf_paths(payload)
      canon_leaves = populated_leaf_paths(rebuilt_payload)
    allowed = allowed_map[service]
    for leaf in sorted(set(leaves) - set(canon_leaves)):
      if not path_matches_allowlist(leaf, allowed):
        raise FixtureError(f"undeclared populated field: {service}.{leaf}")
    raise FixtureError(f"undeclared populated field under service: {service}")


def iter_text_data_leaves(node: Any, prefix: str = "") -> list[tuple[str, bytes]]:
  leaves: list[tuple[str, bytes]] = []
  if node is None:
    return leaves
  if isinstance(node, list) or (hasattr(node, "__iter__") and not hasattr(node, "schema") and not isinstance(node, (str, bytes, bytearray))):
    try:
      for item in node:
        leaves.extend(iter_text_data_leaves(item, f"{prefix}[]" if prefix else "[]"))
      return leaves
    except TypeError:
      pass

  schema = getattr(node, "schema", None)
  if schema is None:
    return leaves

  active_union = None
  if schema.union_fields:
    try:
      active_union = node.which()
    except capnp.KjException:
      active_union = None

  for name in schema.fieldnames:
    if name in schema.union_fields and name != active_union:
      continue
    field = schema.fields[name]
    leaf = path_join(prefix, name)
    try:
      value = getattr(node, name)
    except capnp.KjException:
      continue
    if field.proto.which() == "group":
      leaves.extend(iter_text_data_leaves(value, leaf))
      continue
    if field.proto.which() != "slot":
      continue
    twhich = field.proto.slot.type.which()
    if twhich == "text":
      leaves.append((leaf, str(value).encode()))
    elif twhich == "data":
      leaves.append((leaf, bytes(value)))
    elif twhich == "struct":
      leaves.extend(iter_text_data_leaves(value, leaf))
    elif twhich == "list":
      elem = field.proto.slot.type.list.elementType
      list_prefix = f"{leaf}[]"
      if elem.which() == "struct":
        for item in value:
          leaves.extend(iter_text_data_leaves(item, list_prefix))
      elif elem.which() == "text":
        for item in value:
          leaves.append((list_prefix, str(item).encode()))
      elif elem.which() == "data":
        for item in value:
          leaves.append((list_prefix, bytes(item)))
  return leaves


def can_frames(message: Any) -> list[tuple[int, bytes, int]]:
  return [(int(frame.address), bytes(frame.dat), int(frame.src)) for frame in message]


def vin_fragments(vin: bytes) -> list[bytes]:
  fragments: list[bytes] = []
  upper = min(VIN_FRAGMENT_MAX, len(vin))
  for size in range(VIN_FRAGMENT_MIN, upper + 1):
    for offset in range(len(vin) - size + 1):
      fragments.append(vin[offset:offset + size])
  return fragments


def scan_buffer(matches: Counter[tuple[str, int, str]], source: str, buffer: bytes,
                tokens: list[tuple[str, bytes]], vin: bytes | None) -> None:
  for label, value in tokens:
    if value and value in buffer:
      matches[(label, len(value), source)] += buffer.count(value)
  if vin:
    for fragment in vin_fragments(vin):
      if fragment and fragment in buffer:
        matches[("vin", len(fragment), source)] += buffer.count(fragment)


def _append_capped(buffer: bytes, payload: bytes, limit: int = MAX_CAN_CONCAT_BYTES) -> bytes:
  combined = buffer + payload
  if len(combined) <= limit:
    return combined
  return combined[-limit:]


def token_scan(messages: list[Any], tokens: list[tuple[str, bytes]], vin: bytes | None) -> list[dict[str, Any]]:
  matches: Counter[tuple[str, int, str]] = Counter()
  scan_vin = vin if vin is not None else None
  # Exact Text/Data leaves via schema reflection
  for event in messages:
    service = event.which()
    payload = getattr(event, service)
    for leaf, data in iter_text_data_leaves(payload, service):
      scan_buffer(matches, leaf, data, tokens, scan_vin)

  # CAN per-frame, short rolling windows, full concatenations, and VIN-substring
  # reassembly that skips unrelated interleaved payloads.
  bus_suffix: dict[int, bytes] = {}
  bus_concat: dict[int, bytes] = {}
  addr_concat: dict[tuple[int, int], bytes] = {}
  vin_bus_concat: dict[int, bytes] = {}
  vin_global_concat = b""
  global_suffix = b""
  global_concat = b""
  for event in messages:
    if event.which() != "can":
      continue
    for address, payload, bus in can_frames(event.can):
      source = f"can[{bus}]@0x{address:x}"
      scan_buffer(matches, source, payload, tokens, scan_vin)
      prev = bus_suffix.get(bus, b"")
      window = prev[-CAN_OVERLAP:] + payload
      if prev:
        scan_buffer(matches, f"can[{bus}].rolling", window, tokens, scan_vin)
      bus_suffix[bus] = window[-CAN_OVERLAP:]
      gwindow = global_suffix[-CAN_OVERLAP:] + payload
      if global_suffix:
        scan_buffer(matches, "can.global.rolling", gwindow, tokens, scan_vin)
      global_suffix = gwindow[-CAN_OVERLAP:]

      bus_concat[bus] = _append_capped(bus_concat.get(bus, b""), payload)
      addr_key = (bus, address)
      addr_concat[addr_key] = _append_capped(addr_concat.get(addr_key, b""), payload)
      global_concat = _append_capped(global_concat, payload)

      if scan_vin and payload and payload in scan_vin:
        vin_bus_concat[bus] = _append_capped(vin_bus_concat.get(bus, b""), payload)
        vin_global_concat = _append_capped(vin_global_concat, payload)

  for bus, buffer in bus_concat.items():
    scan_buffer(matches, f"can[{bus}].concat", buffer, tokens, scan_vin)
  for (bus, address), buffer in addr_concat.items():
    scan_buffer(matches, f"can[{bus}]@0x{address:x}.concat", buffer, tokens, scan_vin)
  if global_concat:
    scan_buffer(matches, "can.global.concat", global_concat, tokens, scan_vin)
  for bus, buffer in vin_bus_concat.items():
    scan_buffer(matches, f"can[{bus}].vin_reassembly", buffer, tokens, scan_vin)
  if vin_global_concat:
    scan_buffer(matches, "can.global.vin_reassembly", vin_global_concat, tokens, scan_vin)

  return [
    {"label": label, "length": length, "count": count, "source": source}
    for (label, length, source), count in sorted(matches.items())
  ]


def zero_match_accounting(tokens: list[tuple[str, bytes]]) -> list[dict[str, Any]]:
  rows: list[dict[str, Any]] = []
  seen: set[tuple[str, int]] = set()
  for label, value in tokens:
    key = (label, len(value))
    if key in seen:
      continue
    seen.add(key)
    rows.append({"label": label, "length": len(value), "count": 0, "source": ""})
    if label == "vin":
      for size in range(VIN_FRAGMENT_MIN, min(VIN_FRAGMENT_MAX, len(value)) + 1):
        frag_key = (label, size)
        if frag_key in seen:
          continue
        seen.add(frag_key)
        rows.append({"label": label, "length": size, "count": 0, "source": ""})
  return rows


def validate_can(messages: list[Any], policy: dict[str, Any], mode: str) -> dict[str, Any]:
  rule = policy["pedal"]
  frames: list[tuple[int, bytes, int]] = []
  for event in messages:
    if event.which() == "can":
      frames.extend(can_frames(event.can))
  pedal_frames = [(data, bus) for address, data, bus in frames if address == rule["address"]]
  if mode == "no-pedal":
    if pedal_frames:
      raise FixtureError(f"no-pedal fixture contains pedal sensor frames: count={len(pedal_frames)}")
    return {"address": rule["address"], "count": 0, "valid_count": 0, "counter_count": 0, "bus": rule["bus"]}
  valid: list[bytes] = []
  counters: set[int] = set()
  for data, bus in pedal_frames:
    if bus != rule["bus"] or len(data) != rule["data_length"]:
      continue
    checksum = (sum(data[:rule["checksum_index"]]) + (rule["address"] & 0xff) + (rule["address"] >> 8)) & 0xff
    if data[rule["checksum_index"]] == checksum:
      valid.append(data)
      counters.add(data[rule["counter_index"]] & rule["counter_mask"])
  if len(valid) < rule["minimum_frames"] or len(counters) < rule["minimum_counters"]:
    raise FixtureError(f"pedal evidence insufficient: valid={len(valid)} counters={len(counters)}")
  return {
    "address": rule["address"], "count": len(pedal_frames), "valid_count": len(valid),
    "counter_count": len(counters), "bus": rule["bus"],
  }


def structural_checks(messages: list[Any], policy: dict[str, Any], case: str) -> dict[str, Any]:
  if not messages:
    raise FixtureError("fixture is empty")
  counts = Counter(event.which() for event in messages)
  valid_counts = Counter(event.which() for event in messages if event.valid)
  valid_count = sum(valid_counts.values())
  valid_share = valid_count / len(messages)
  if valid_share < VALID_SHARE_THRESHOLD:
    raise FixtureError(f"valid-message share below 90 percent: {valid_share:.3f}")

  allowed = set(policy["allowed_services"])
  for service in counts:
    if service not in allowed:
      raise FixtureError(f"undeclared service in output: {service}")

  for service, minimum in policy["required_services"].items():
    if valid_counts.get(service, 0) < minimum:
      raise FixtureError(f"required valid service count unmet: {service}")

  params = [event.carParams for event in messages if event.which() == "carParams"]
  if not params:
    raise FixtureError("carParams is required")
  expected = policy["cases"][case]
  pedal = expected["mode"] == "pedal"
  expected_param = int(expected["safety_param"])
  if expected_param != expected_safety_param(expected["mode"]):
    raise FixtureError("policy safety_param does not match fixed Pre-AP flags")
  for cp in params:
    if cp.carVin != policy["placeholder_vin"]:
      raise FixtureError("carParams.carVin is not the public placeholder")
    if len(cp.carFw) != 0:
      raise FixtureError("carParams.carFw must be empty")
    if cp.brand != "tesla":
      raise FixtureError("carParams.brand must be tesla")
    if cp.carFingerprint != expected["fingerprint"]:
      raise FixtureError("unexpected car fingerprint")
    if len(cp.safetyConfigs) != 1:
      raise FixtureError("carParams must contain exactly one safetyConfigs entry")
    safety = cp.safetyConfigs[0]
    if not safety_model_is_tesla_preap(safety.safetyModel):
      raise FixtureError("carParams safetyModel must be teslaPreap")
    if int(safety.safetyParam) != expected_param:
      raise FixtureError(f"carParams safetyParam must be {expected_param}")
    if bool(cp.openpilotLongitudinalControl) != pedal or bool(cp.pcmCruise) == pedal:
      raise FixtureError("Pre-AP pedal/longitudinal assertions failed")

  # Sensitive zeros must fail closed before allowlist rebuild compare, which also
  # zeroes GPS and would otherwise mislabel vNED/location leaks as undeclared fields.
  for field in policy["sensitive_zero_fields"]:
    parts = field_parts(field)
    service = parts[0]
    for event in messages:
      if event.which() != service:
        continue
      node: Any = getattr(event, service)
      index = 1
      while index < len(parts):
        part = parts[index]
        if part == "[]":
          if index == len(parts) - 1:
            if any(float(item) != 0 for item in node):
              raise FixtureError(f"sensitive field is not zero: {field}")
            break
          raise FixtureError(f"invalid sensitive path: {field}")
        node = getattr(node, part)
        index += 1
      else:
        if hasattr(node, "__iter__") and not isinstance(node, (str, bytes, bytearray)):
          if any(float(item) != 0 for item in node):
            raise FixtureError(f"sensitive field is not zero: {field}")
        elif float(node) != 0:
          raise FixtureError(f"sensitive field is not zero: {field}")

  validate_field_allowlist(messages, policy)

  return {
    "message_counts": dict(sorted(counts.items())),
    "valid_message_counts": dict(sorted(valid_counts.items())),
    "valid_message_share": valid_share,
  }


def canonical_policy_digest(raw: dict[str, Any]) -> str:
  material = {key: raw[key] for key in sorted(POLICY_KEYS)}
  return digest(json.dumps(material, sort_keys=True, separators=(",", ":")).encode())


def parse_policy(path: str) -> dict[str, Any]:
  try:
    with open(path, encoding="utf-8") as stream:
      raw = json.load(stream)
  except json.JSONDecodeError as error:
    raise FixtureError("policy is malformed JSON") from error
  except OSError as error:
    raise FixtureError("policy cannot be read") from error
  if not isinstance(raw, dict):
    raise FixtureError("policy root must be an object")
  if set(raw) != POLICY_KEYS:
    raise FixtureError("policy keys are invalid")
  if raw["schema_version"] != SCHEMA_VERSION:
    raise FixtureError("unsupported policy schema_version")
  if raw["placeholder_vin"] != PLACEHOLDER_VIN:
    raise FixtureError("placeholder_vin must be the fixed public value")
  if canonical_policy_digest(raw) != RELEASE_POLICY_SHA256:
    raise FixtureError("policy is not the attested release contract")

  allowed = raw["allowed_services"]
  forbidden = raw["forbidden_services"]
  required = raw["required_services"]
  freshness = raw["freshness_services"]
  if not isinstance(allowed, dict) or not allowed:
    raise FixtureError("allowed_services must be a nonempty object")
  if not isinstance(forbidden, list) or not all(isinstance(item, str) for item in forbidden):
    raise FixtureError("forbidden_services must be a string list")
  if not isinstance(required, dict) or not required:
    raise FixtureError("required_services must be a nonempty object")
  if not isinstance(freshness, list) or not all(isinstance(item, str) for item in freshness):
    raise FixtureError("freshness_services must be a string list")
  if not isinstance(raw["selected_gps_service"], str):
    raise FixtureError("selected_gps_service must be a string")

  known = event_service_names()
  for service in list(allowed) + forbidden + list(required) + freshness + [raw["selected_gps_service"]]:
    if service not in known:
      raise FixtureError(f"unknown Event service in policy: {service}")

  if set(allowed) & set(forbidden):
    raise FixtureError("allowed_services and forbidden_services must be disjoint")
  missing_forbidden = MANDATORY_FORBIDDEN - set(forbidden)
  if missing_forbidden:
    raise FixtureError("forbidden_services missing mandatory baseline")
  if raw["selected_gps_service"] not in allowed:
    raise FixtureError("selected_gps_service must be allowed")
  other_gps = {"gpsLocation", "gpsLocationExternal"} - {raw["selected_gps_service"]}
  if other_gps & set(allowed):
    raise FixtureError("only the selected GPS service may be allowed")
  for service in freshness:
    if service not in allowed:
      raise FixtureError(f"freshness service must be allowed: {service}")

  leaf_sets: dict[str, set[str]] = {}
  for service, paths in allowed.items():
    if not isinstance(service, str) or not isinstance(paths, list) or not all(isinstance(item, str) for item in paths):
      raise FixtureError("allowed_services entries must map to string lists")
    if service in freshness and paths:
      # freshness-only services must not copy identifying leaves
      raise FixtureError(f"freshness service must not declare identifying leaves: {service}")
    leaf_sets[service] = expand_allowed_paths(service, paths) if paths else set()

  for service, minimum in required.items():
    if service not in allowed:
      raise FixtureError(f"required service is not allowed: {service}")
    if not isinstance(minimum, int) or isinstance(minimum, bool) or minimum < 1:
      raise FixtureError(f"required service minimum must be a positive int: {service}")

  pedal = raw["pedal"]
  if not isinstance(pedal, dict) or set(pedal) != PEDAL_KEYS:
    raise FixtureError("pedal policy keys are invalid")
  for key in ("address", "bus", "data_length", "counter_index", "counter_mask", "checksum_index",
              "minimum_frames", "minimum_counters"):
    value = pedal[key]
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
      raise FixtureError(f"pedal.{key} must be a non-negative int")
  if pedal["checksum"] != "tesla_sum8_address":
    raise FixtureError("unsupported pedal checksum")
  if pedal["checksum_index"] >= pedal["data_length"] or pedal["counter_index"] >= pedal["data_length"]:
    raise FixtureError("pedal indices out of range")
  if pedal["data_length"] < 1 or pedal["bus"] > 255:
    raise FixtureError("pedal ranges are invalid")
  if pedal["minimum_frames"] < PEDAL_MIN_FRAMES or pedal["minimum_counters"] < PEDAL_MIN_COUNTERS:
    raise FixtureError("pedal evidence minima below release floor")

  zeros = raw["sensitive_zero_fields"]
  if not isinstance(zeros, list) or not all(isinstance(item, str) for item in zeros):
    raise FixtureError("sensitive_zero_fields must be a string list")
  selected = raw["selected_gps_service"]
  required_zeros = {item for item in SENSITIVE_ZERO_BASELINE if item.startswith(selected + ".")}
  if not required_zeros.issubset(set(zeros)):
    raise FixtureError("sensitive_zero_fields missing mandatory GPS zeros including vNED")
  for item in zeros:
    service = item.split(".", 1)[0]
    if service not in allowed and service not in ("gpsLocation", "gpsLocationExternal"):
      raise FixtureError(f"sensitive zero service not allowed: {service}")
    # validate path shape against schema when service allowed
    if service in allowed:
      rest = item.split(".", 1)[1]
      resolve_policy_path(service, rest)

  cases = raw["cases"]
  if not isinstance(cases, dict) or not cases:
    raise FixtureError("cases must be a nonempty object")
  for name, case in cases.items():
    if not isinstance(name, str) or not isinstance(case, dict) or set(case) != CASE_KEYS:
      raise FixtureError("case entries are invalid")
    if case["mode"] not in CASE_MODES or not isinstance(case["fingerprint"], str) or not case["fingerprint"]:
      raise FixtureError(f"case mode/fingerprint invalid: {name}")
    if case["fingerprint"] != "TESLA_MODEL_S_PREAP":
      raise FixtureError("only TESLA_MODEL_S_PREAP cases are supported")
    safety_param = case["safety_param"]
    if not isinstance(safety_param, int) or isinstance(safety_param, bool):
      raise FixtureError(f"case safety_param must be an int: {name}")
    if safety_param != expected_safety_param(case["mode"]):
      raise FixtureError(f"case safety_param must be {expected_safety_param(case['mode'])}: {name}")

  raw["_allowed_leaf_sets"] = leaf_sets
  return raw


def decode_deny_token_value(encoding: str, value: str) -> bytes:
  """Decode a deny token payload with strict, canonical binary-safe encodings."""
  if encoding not in DENY_ENCODINGS:
    raise FixtureError("deny token encoding must be utf8, hex, or base64")
  if not isinstance(value, str):
    raise FixtureError("deny token value must be a string")
  if encoding == "utf8":
    try:
      return value.encode("utf-8")
    except UnicodeEncodeError as error:
      raise FixtureError("deny token utf8 value is invalid") from error
  if encoding == "hex":
    if len(value) % 2 != 0 or not HEX_VALUE_RE.fullmatch(value):
      raise FixtureError("deny token hex value must be lowercase hexadecimal")
    return bytes.fromhex(value)
  try:
    decoded = base64.b64decode(value, validate=True)
  except binascii.Error as error:
    raise FixtureError("deny token base64 value is invalid") from error
  if base64.b64encode(decoded).decode("ascii") != value:
    raise FixtureError("deny token base64 value must be canonical")
  return decoded


def load_deny(path: str) -> DenyManifest:
  try:
    with open(path, encoding="utf-8") as stream:
      raw = json.load(stream)
  except json.JSONDecodeError as error:
    raise FixtureError("deny file is malformed JSON") from error
  if not isinstance(raw, dict):
    raise FixtureError("deny file root must be an object")
  unknown = set(raw) - DENY_REQUIRED_KEYS - DENY_OPTIONAL_KEYS
  if unknown:
    raise FixtureError("deny file contains unknown keys")
  if "raw_input_sha256" not in raw and "input_sha256" not in raw:
    raise FixtureError("deny raw_input_sha256 must be lowercase hexadecimal SHA-256")
  tokens_raw = raw.get("tokens")
  if not isinstance(tokens_raw, list) or not tokens_raw:
    raise FixtureError("deny file must contain a nonempty tokens list")
  expected = raw.get("raw_input_sha256", raw.get("input_sha256"))
  if not isinstance(expected, str) or not SHA256_RE.fullmatch(expected):
    raise FixtureError("deny raw_input_sha256 must be lowercase hexadecimal SHA-256")
  sanitized = raw.get("sanitized_output_sha256")
  if sanitized is not None and (not isinstance(sanitized, str) or not SHA256_RE.fullmatch(sanitized)):
    raise FixtureError("deny sanitized_output_sha256 must be lowercase hexadecimal SHA-256")

  tokens: list[tuple[str, bytes]] = []
  labels: set[str] = set()
  values: set[bytes] = set()
  for entry in tokens_raw:
    if not isinstance(entry, dict) or set(entry) != {"label", "encoding", "value"}:
      raise FixtureError("deny token entries require exactly label, encoding, and value")
    label = entry["label"]
    encoding = entry["encoding"]
    value = entry["value"]
    if not isinstance(label, str) or not SAFE_LABEL_RE.fullmatch(label):
      raise FixtureError("deny token label is not a safe identifier")
    if not isinstance(encoding, str):
      raise FixtureError("deny token encoding must be utf8, hex, or base64")
    encoded = decode_deny_token_value(encoding, value)
    if len(encoded) < VIN_FRAGMENT_MIN:
      raise FixtureError("deny token is shorter than six bytes")
    if label in labels:
      raise FixtureError("deny token labels must be unique")
    if encoded in values:
      raise FixtureError("deny token values must be unique")
    labels.add(label)
    values.add(encoded)
    tokens.append((label, encoded))

  vin_tokens = [(label, value) for label, value in tokens if label == "vin"]
  if len(vin_tokens) != 1:
    raise FixtureError("deny file must contain exactly one vin token")
  vin = vin_tokens[0][1]
  if len(vin) != VIN_LENGTH:
    raise FixtureError("vin token must be exactly 17 bytes")
  return DenyManifest(
    tokens=tuple(tokens),
    raw_input_sha256=expected,
    sanitized_output_sha256=sanitized,
    vin=vin,
  )


def extract_source_vins(messages: list[Any]) -> set[str]:
  vins = {str(event.carParams.carVin) for event in messages if event.which() == "carParams"}
  if not vins:
    raise FixtureError("source carParams.carVin is required")
  return vins


def verify_deny_vin_against_source(messages: list[Any], vin: bytes) -> None:
  vins = extract_source_vins(messages)
  if len(vins) != 1:
    raise FixtureError("source carParams.carVin must be unique")
  source_vin = next(iter(vins))
  if len(source_vin.encode()) != VIN_LENGTH:
    raise FixtureError("source carParams.carVin must be exactly 17 bytes")
  if source_vin.encode() != vin:
    raise FixtureError("deny vin token does not match source carParams.carVin")


def relative_text_data_leaf(service: str, leaf: str) -> str:
  if leaf == service:
    return ""
  if leaf.startswith(service) and len(leaf) > len(service) and leaf[len(service)] in ".[":
    rest = leaf[len(service):]
    return rest[1:] if rest.startswith(".") else rest
  raise FixtureError(f"text/data leaf provenance is malformed: {leaf}")


def is_scrubbed_private_leaf(leaf: str) -> bool:
  # carVin is allowlisted but replaced; carFw is stripped even if nested Data appears.
  return leaf == "carParams.carVin" or leaf.startswith("carParams.carFw[]")


def is_private_text_data_leaf(service: str, leaf: str, policy: dict[str, Any]) -> bool:
  """True when a raw Text/Data leaf must be covered by deny tokens (provenance-based)."""
  if is_scrubbed_private_leaf(leaf):
    return True
  if leaf in ATTESTED_PUBLIC_TEXT_DATA_LEAVES:
    return False
  allowed = policy["_allowed_leaf_sets"].get(service)
  if allowed is None:
    return True
  rel = relative_text_data_leaf(service, leaf)
  # Allowlisted retained channels (including can[].dat) are not inventory sources.
  # They remain token-scan sinks and never exempt private-provenance duplicates.
  if path_matches_allowlist(rel, allowed):
    return False
  return True


def derive_private_text_data_inventory(messages: list[Any], policy: dict[str, Any]) -> set[bytes]:
  """Inventory private Text/Data by raw leaf provenance, not sanitized set-subtraction."""
  inventory: set[bytes] = set()
  for event in messages:
    service = event.which()
    payload = getattr(event, service)
    for leaf, data in iter_text_data_leaves(payload, service):
      if len(data) < VIN_FRAGMENT_MIN:
        continue
      if is_private_text_data_leaf(service, leaf, policy):
        inventory.add(data)
  return inventory


def verify_deny_inventory_completeness(
  source_messages: list[Any],
  tokens: list[tuple[str, bytes]],
  vin: bytes,
  policy: dict[str, Any],
) -> None:
  verify_deny_vin_against_source(source_messages, vin)
  private_inventory = derive_private_text_data_inventory(source_messages, policy)
  covered = {value for _label, value in tokens}
  missing = private_inventory - covered
  if missing:
    raise FixtureError("deny tokens incomplete for decoded private Text/Data inventory")


def lstat_path(path: Path) -> os.stat_result:
  try:
    return os.lstat(path)
  except OSError as error:
    raise FixtureError("path is inaccessible") from error


def assert_not_symlink(path: Path, label: str) -> None:
  if path.exists() and path.is_symlink():
    raise FixtureError(f"{label} must not be a symlink")


def same_file(stat_a: os.stat_result, stat_b: os.stat_result) -> bool:
  return stat_a.st_ino == stat_b.st_ino and stat_a.st_dev == stat_b.st_dev


def reject_symlink_parents(path: Path, label: str) -> None:
  current = path if path.is_absolute() else Path.cwd() / path
  for parent in [current, *current.parents]:
    try:
      if parent.is_symlink():
        raise FixtureError(f"{label} parent must not be a symlink")
    except OSError as error:
      raise FixtureError(f"{label} path is inaccessible") from error


def canonicalize_endpoint(path: Path, label: str) -> Path:
  reject_symlink_parents(path, label)
  absolute = path if path.is_absolute() else Path.cwd() / path
  parent = absolute.parent
  try:
    resolved_parent = parent.resolve(strict=False)
  except OSError as error:
    raise FixtureError(f"{label} path is inaccessible") from error
  reject_symlink_parents(resolved_parent / absolute.name, label)
  canonical = resolved_parent / absolute.name
  if canonical.exists() or canonical.is_symlink():
    assert_not_symlink(canonical, label)
  return canonical


def validate_path_separation(paths: dict[str, Path]) -> dict[str, Path]:
  resolved: dict[str, Path] = {}
  for label, path in paths.items():
    resolved[label] = canonicalize_endpoint(path, label)
  items = list(resolved.items())
  for index, (label_a, path_a) in enumerate(items):
    for label_b, path_b in items[index + 1:]:
      if path_a == path_b:
        raise FixtureError(f"path collision between {label_a} and {label_b}")
      if path_a.exists() and path_b.exists():
        if same_file(lstat_path(path_a), lstat_path(path_b)):
          raise FixtureError(f"inode alias between {label_a} and {label_b}")
  return resolved


def open_private_temp(destination_dir: Path, suffix: str) -> tuple[int, Path]:
  destination_dir.mkdir(parents=True, exist_ok=True)
  fd, name = tempfile.mkstemp(prefix=".fixture-", suffix=suffix, dir=str(destination_dir))
  try:
    os.fchmod(fd, 0o600)
  except OSError:
    os.close(fd)
    Path(name).unlink(missing_ok=True)
    raise
  path = Path(name)
  if path.is_symlink():
    os.close(fd)
    path.unlink(missing_ok=True)
    raise FixtureError("temporary path must not be a symlink")
  return fd, path


def write_bytes_atomic(destination: Path, data: bytes) -> None:
  fd, tmp = open_private_temp(destination.parent, suffix=".tmp")
  try:
    with os.fdopen(fd, "wb") as stream:
      stream.write(data)
      stream.flush()
      os.fsync(stream.fileno())
    os.replace(tmp, destination)
  except Exception:
    try:
      os.close(fd)
    except Exception:
      pass
    tmp.unlink(missing_ok=True)
    raise


def write_json_atomic(destination: Path, payload: dict[str, Any]) -> None:
  data = (json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n").encode()
  write_bytes_atomic(destination, data)


def serialize_log_bytes(messages: list[Any], *, compress: bool = True) -> bytes:
  data = b"".join(message.as_builder().to_bytes() for message in messages)
  if compress:
    return zstandard.ZstdCompressor(level=10).compress(data)
  return data


def write_fd_bytes(fd: int, data: bytes) -> None:
  view = memoryview(data)
  while view:
    written = os.write(fd, view)
    if written <= 0:
      raise FixtureError("failed to write temporary fixture bytes")
    view = view[written:]
  os.fsync(fd)


def public_status(*, case: str, scope: str, output_sha256: str | None, output_bytes: int | None,
                  performed: bool | None = None) -> dict[str, Any]:
  report: dict[str, Any] = {"status": "ok", "case": case, "scope": scope}
  if output_sha256 is not None:
    report["sanitized_output_sha256" if scope != "public" else "fixture_sha256"] = output_sha256
  if output_bytes is not None:
    report["output_bytes"] = output_bytes
  if performed is not None:
    report["private_token_scan"] = {"performed": performed}
  return report


def build_validation_report(
  *,
  policy: dict[str, Any],
  case: str,
  messages: list[Any],
  fixture_bytes: bytes,
  scope: str,
  tokens: list[tuple[str, bytes]] | None,
  vin: bytes | None,
  source_message_counts: dict[str, int] | None,
) -> dict[str, Any]:
  report: dict[str, Any] = {
    "schema_version": policy["schema_version"],
    "case": case,
    "scope": scope,
    "fixture_sha256": digest(fixture_bytes),
    "output_bytes": len(fixture_bytes),
  }
  if source_message_counts is not None:
    report["source_message_counts"] = source_message_counts
  report.update(structural_checks(messages, policy, case))
  report["can_evidence"] = validate_can(messages, policy, policy["cases"][case]["mode"])

  if scope == "public":
    report["private_token_scan"] = {"performed": False}
  else:
    assert tokens is not None and vin is not None
    matches = token_scan(messages, tokens, vin)
    accounting = zero_match_accounting(list(tokens))
    if matches:
      by_key = {(row["label"], row["length"]): row for row in accounting}
      for match in matches:
        key = (match["label"], match["length"])
        row = by_key.get(key)
        if row is None:
          accounting.append(dict(match))
        else:
          row["count"] = match["count"]
          row["source"] = match["source"]
      report["private_token_scan"] = {"performed": True, "token_matches": matches}
      report["token_match_accounting"] = accounting
      report["_rejected_private_token"] = True
    else:
      report["private_token_scan"] = {"performed": True, "token_matches": accounting}
  return report


def accept_six_process_outputs(
  *,
  case: str,
  policy: dict[str, Any],
  process_outputs: Mapping[str, list[Any]],
  case_params: Mapping[str, Any],
  valid_share_threshold: float = VALID_SHARE_THRESHOLD,
) -> dict[str, Any]:
  """Publication gate over fixed typed case params and six-process replay outputs.

  Does not itself execute process replay. Callers supply the per-process output
  streams produced under ``case_params`` (for example NAP_PREAP_*_PARAMS).
  """
  if case not in policy["cases"]:
    raise FixtureError("unknown fixture case")
  expected = policy["cases"][case]
  mode = expected["mode"]
  if not isinstance(case_params.get("NAPPedalEnabled"), bool):
    raise FixtureError("case_params.NAPPedalEnabled must be a bool")
  if bool(case_params["NAPPedalEnabled"]) != (mode == "pedal"):
    raise FixtureError("case_params.NAPPedalEnabled does not match case mode")
  if case_params.get("NAPRadarEnabled") is not True or case_params.get("NAPRadarBehindNosecone") is not True:
    raise FixtureError("case_params must enable radar emulation and nosecone flags")

  missing = [name for name in SIX_PROCESS_OUTPUTS if name not in process_outputs]
  if missing:
    raise FixtureError(f"missing process outputs: {','.join(missing)}")

  per_process: dict[str, Any] = {}
  for proc_name, required_services in SIX_PROCESS_OUTPUTS.items():
    messages = list(process_outputs[proc_name])
    if not messages:
      raise FixtureError(f"process produced no outputs: {proc_name}")
    counts = Counter(message.which() for message in messages)
    valid_counts = Counter(message.which() for message in messages if message.valid)
    present = set(counts)
    if not required_services.issubset(present):
      raise FixtureError(f"process missing declared outputs: {proc_name}")
    shares: dict[str, float] = {}
    for service in required_services:
      total = counts[service]
      share = valid_counts.get(service, 0) / total
      shares[service] = share
      if share < valid_share_threshold:
        raise FixtureError(f"process valid-share below threshold: {proc_name}.{service}")
    per_process[proc_name] = {
      "message_counts": dict(sorted(counts.items())),
      "valid_shares": dict(sorted(shares.items())),
    }

  card_params = [message.carParams for message in process_outputs["card"] if message.which() == "carParams"]
  if not card_params:
    raise FixtureError("card did not emit carParams")
  expected_param = int(expected["safety_param"])
  for cp in card_params:
    if cp.carFingerprint != expected["fingerprint"]:
      raise FixtureError("card carParams fingerprint mismatch")
    if len(cp.safetyConfigs) != 1 or not safety_model_is_tesla_preap(cp.safetyConfigs[0].safetyModel):
      raise FixtureError("card carParams safety config mismatch")
    if int(cp.safetyConfigs[0].safetyParam) != expected_param:
      raise FixtureError("card carParams safetyParam mismatch")
    pedal = mode == "pedal"
    if bool(cp.openpilotLongitudinalControl) != pedal or bool(cp.pcmCruise) == pedal:
      raise FixtureError("card carParams mode mismatch")

  return {
    "status": "ok",
    "case": case,
    "processes": list(SIX_PROCESS_OUTPUTS),
    "process_reports": per_process,
    "card_mode": mode,
    "card_fingerprint": expected["fingerprint"],
    "card_safety_param": expected_param,
  }


def decompress_zstd_bounded(raw: bytes) -> bytes:
  """Bounded single-frame zstd decompress with exact EOF and trailing-data rejection."""
  try:
    params = zstandard.get_frame_parameters(raw)
  except zstandard.ZstdError as error:
    raise FixtureError("input is malformed") from error
  declared = params.content_size
  if declared != ZSTD_CONTENTSIZE_UNKNOWN and declared > MAX_DECOMPRESSED_INPUT_BYTES:
    raise FixtureError("input exceeds decompressed size limit")

  decompressor = zstandard.ZstdDecompressor().decompressobj()
  chunks: list[bytes] = []
  total = 0
  offset = 0
  try:
    while offset < len(raw):
      end = min(offset + ZSTD_DECOMPRESS_CHUNK, len(raw))
      piece = decompressor.decompress(raw[offset:end])
      offset = end
      if piece:
        total += len(piece)
        if total > MAX_DECOMPRESSED_INPUT_BYTES:
          raise FixtureError("input exceeds decompressed size limit")
        chunks.append(piece)
      if decompressor.eof:
        trailing = decompressor.unused_data + raw[offset:]
        if trailing:
          raise FixtureError("input is malformed")
        break
    else:
      if not decompressor.eof:
        raise FixtureError("input is malformed")
  except FixtureError:
    raise
  except zstandard.ZstdError as error:
    message = str(error).lower()
    if "maximum" in message or "too large" in message or "max" in message:
      raise FixtureError("input exceeds decompressed size limit") from error
    raise FixtureError("input is malformed") from error
  return b"".join(chunks)


def load_source_messages(raw: bytes) -> list[Any]:
  if len(raw) > MAX_COMPRESSED_INPUT_BYTES:
    raise FixtureError("input exceeds compressed size limit")
  data = raw
  try:
    if raw.startswith(ZSTD_MAGIC):
      data = decompress_zstd_bounded(raw)
    elif raw.startswith(b"BZh"):
      data = __import__("bz2").decompress(raw)
      if len(data) > MAX_DECOMPRESSED_INPUT_BYTES:
        raise FixtureError("input exceeds decompressed size limit")
  except FixtureError:
    raise
  except MemoryError as error:
    raise FixtureError("input exceeds decompressed size limit") from error
  except (zstandard.ZstdError, OSError, ValueError) as error:
    message = str(error).lower()
    if "maximum" in message or "too large" in message or "max" in message:
      raise FixtureError("input exceeds decompressed size limit") from error
    raise FixtureError("input is malformed") from error

  try:
    import warnings
    with warnings.catch_warnings(record=True) as caught:
      warnings.simplefilter("always", RuntimeWarning)
      messages = list(LogReader.from_bytes(data))
    if any(issubclass(item.category, RuntimeWarning) for item in caught) or (len(data) > 0 and not messages):
      raise FixtureError("input is malformed")
  except MemoryError as error:
    raise FixtureError("input exceeds decompressed size limit") from error
  except FixtureError:
    raise
  except (capnp.KjException, zstandard.ZstdError, ValueError, OSError, RuntimeError) as error:
    raise FixtureError("input is malformed") from error

  if len(messages) > MAX_LOG_MESSAGES:
    raise FixtureError("input exceeds message count limit")
  return messages


def process(
  input_path: str,
  output_path: str | None,
  report_path: str,
  policy_path: str,
  case: str,
  deny_path: str | None,
  sanitize: bool,
  input_sha256: str | None = None,
) -> dict[str, Any]:
  policy = parse_policy(policy_path)
  if case not in policy["cases"]:
    raise FixtureError("unknown fixture case")

  input_p = Path(input_path)
  policy_p = Path(policy_path)
  report_p = Path(report_path)
  output_p = Path(output_path) if output_path else None
  deny_p = Path(deny_path) if deny_path else None

  path_map = {"input": input_p, "policy": policy_p, "report": report_p}
  if output_p is not None:
    path_map["output"] = output_p
  if deny_p is not None:
    path_map["deny"] = deny_p
  if sanitize:
    if output_p is None:
      raise FixtureError("sanitize requires --output")
    if output_p == report_p:
      raise FixtureError("output and report must be distinct")
  resolved = validate_path_separation(path_map)
  input_p = resolved["input"]
  policy_p = resolved["policy"]
  report_p = resolved["report"]
  output_p = resolved.get("output")
  deny_p = resolved.get("deny")

  deny: DenyManifest | None = None
  if sanitize:
    if deny_p is None:
      raise FixtureError("sanitize requires a deny token file")
    deny = load_deny(str(deny_p))
  elif deny_p is not None:
    deny = load_deny(str(deny_p))

  if input_sha256 is not None:
    if not SHA256_RE.fullmatch(input_sha256):
      raise FixtureError("input_sha256 must be lowercase hexadecimal SHA-256")

  raw = input_p.read_bytes()
  if len(raw) > MAX_COMPRESSED_INPUT_BYTES:
    raise FixtureError("input exceeds compressed size limit")
  raw_hash = digest(raw)

  if sanitize:
    assert deny is not None
    if raw_hash != deny.raw_input_sha256:
      raise FixtureError("input SHA-256 does not match deny raw_input_sha256")
    if input_sha256 is not None and input_sha256 != deny.raw_input_sha256:
      raise FixtureError("input_sha256 conflicts with deny raw_input_sha256")
  else:
    if deny is not None:
      expected_fixture = deny.sanitized_output_sha256
      if expected_fixture is None:
        raise FixtureError("private validate requires deny sanitized_output_sha256")
      if raw_hash != expected_fixture:
        raise FixtureError("input SHA-256 does not match deny sanitized_output_sha256")
      if input_sha256 is not None and input_sha256 != expected_fixture:
        raise FixtureError("input_sha256 conflicts with deny sanitized_output_sha256")
    elif input_sha256 is not None and raw_hash != input_sha256:
      raise FixtureError("input SHA-256 does not match --input-sha256")

  source_messages = load_source_messages(raw)

  temp_output: Path | None = None
  try:
    if sanitize:
      assert output_p is not None and deny is not None
      # Fail closed for stale destinations: clear any preexisting output before work.
      if output_p.exists() or output_p.is_symlink():
        assert_not_symlink(output_p, "output")
        output_p.unlink()
      messages = [built for event in source_messages if (built := build_event(event, policy)) is not None]
      verify_deny_inventory_completeness(source_messages, list(deny.tokens), deny.vin, policy)
      fd, temp_output = open_private_temp(output_p.parent, suffix=".zst")
      try:
        output_bytes = serialize_log_bytes(messages)
        write_fd_bytes(fd, output_bytes)
      finally:
        os.close(fd)
      messages = list(LogReader.from_bytes(output_bytes))
      report = build_validation_report(
        policy=policy,
        case=case,
        messages=messages,
        fixture_bytes=output_bytes,
        scope="private",
        tokens=list(deny.tokens),
        vin=deny.vin,
        source_message_counts=dict(sorted(Counter(event.which() for event in source_messages).items())),
      )
      output_digest = digest(output_bytes)
      report["sanitized_output_sha256"] = output_digest
      if deny.sanitized_output_sha256 is not None and deny.sanitized_output_sha256 != output_digest:
        raise FixtureError("sanitized output SHA-256 does not match deny sanitized_output_sha256")
      rejected = report.pop("_rejected_private_token", False)
      write_json_atomic(report_p, report)
      if rejected:
        temp_output.unlink(missing_ok=True)
        temp_output = None
        output_p.unlink(missing_ok=True)
        raise FixtureError("private token detected in retained output")
      os.replace(temp_output, output_p)
      temp_output = None
      return public_status(
        case=case, scope="private", output_sha256=output_digest,
        output_bytes=len(output_bytes), performed=True,
      )

    scope = "private" if deny is not None else "public"
    if scope == "private":
      assert deny is not None
      # Private validate of a sanitized fixture still requires the original VIN token.
      if deny.vin.decode() == PLACEHOLDER_VIN:
        raise FixtureError("private validate vin token must be the original private VIN")
    report = build_validation_report(
      policy=policy,
      case=case,
      messages=source_messages,
      fixture_bytes=raw,
      scope=scope,
      tokens=list(deny.tokens) if deny is not None else None,
      vin=deny.vin if deny is not None else None,
      source_message_counts=None,
    )
    rejected = report.pop("_rejected_private_token", False)
    write_json_atomic(report_p, report)
    if rejected:
      raise FixtureError("private token detected in retained output")
    return public_status(
      case=case,
      scope=scope,
      output_sha256=digest(raw),
      output_bytes=len(raw),
      performed=(scope == "private"),
    )
  except Exception:
    if temp_output is not None:
      temp_output.unlink(missing_ok=True)
    if sanitize and output_p is not None:
      output_p.unlink(missing_ok=True)
    raise


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser()
  subparsers = parser.add_subparsers(dest="command", required=True)
  for command in ("sanitize", "validate"):
    sub = subparsers.add_parser(command)
    sub.add_argument("--policy", required=True)
    sub.add_argument("--input-sha256")
    sub.add_argument("--case", required=True)
    sub.add_argument("--input", required=True)
    sub.add_argument("--output", required=command == "sanitize")
    sub.add_argument("--report", required=True)
    sub.add_argument("--deny-token-file")
  args = parser.parse_args(argv)
  try:
    status = process(
      args.input,
      args.output,
      args.report,
      args.policy,
      args.case,
      args.deny_token_file,
      args.command == "sanitize",
      args.input_sha256,
    )
    print(json.dumps(status, sort_keys=True, separators=(",", ":")))
    return 0
  except (FixtureError, OSError, capnp.KjException, zstandard.ZstdError, json.JSONDecodeError, TypeError, ValueError, KeyError):
    print("fixture validation failed", file=sys.stderr)
    return 2


if __name__ == "__main__":
  raise SystemExit(main())
