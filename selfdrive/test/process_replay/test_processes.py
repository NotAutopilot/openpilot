#!/usr/bin/env python3
import argparse
import concurrent.futures
import hashlib
import json
import math
import os
import re
import shutil
import stat
import sys
import time
import traceback
from collections import Counter, defaultdict
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any, Literal
from urllib.request import Request, urlopen

import zstandard as zstd
from tqdm import tqdm

from opendbc.car.car_helpers import interface_names
from openpilot.common.git import get_commit
from openpilot.common.params import ParamKeyType, Params
from openpilot.common.utils import atomic_write
from openpilot.selfdrive.test.process_replay.compare_logs import compare_logs, format_diff
from openpilot.selfdrive.test.process_replay.process_replay import CONFIGS, FAKEDATA, PROC_REPLAY_DIR, check_most_messages_valid, \
                                                                   replay_process
from openpilot.tools.lib.filereader import FileReader
from openpilot.tools.lib.logreader import LogReader, save_log
from openpilot.tools.lib.openpilotci import get_url
from openpilot.tools.lib.url_file import URLFile, URLFileException

RESULTS_SCHEMA_VERSION = 1
CASE_ID_RE = re.compile(r"^[a-z0-9][a-z0-9-]*$")
EXCLUDED_PROCS = frozenset({"modeld", "dmonitoringmodeld"})
ALWAYS_PROCS = ("card", "controlsd", "lagd")
EXTRA_PROCS = (
  "selfdrived", "radard", "plannerd", "calibrationd", "dmonitoringd",
  "locationd", "paramsd", "ubloxd", "torqued",
)
PREAP_PROCS = ("card", "controlsd", "selfdrived", "radard", "plannerd", "lagd")
ALL_REPLAY_PROCS = tuple(cfg.proc_name for cfg in CONFIGS if cfg.proc_name not in EXCLUDED_PROCS)
PROC_BY_NAME = {cfg.proc_name: cfg for cfg in CONFIGS if cfg.proc_name not in EXCLUDED_PROCS}

ARTIFACTS_REPO = "NotAutopilot/ci-artifacts"
ARTIFACTS_BRANCH = "process-replay"
ARTIFACTS_RAW = f"https://raw.githubusercontent.com/{ARTIFACTS_REPO}"

# Keep this list compatible with regen_all.py (imports source_segments as segments).
source_segments = [
  ("HYUNDAI", "02c45f73a2e5c6e9|2021-01-01--19-08-22--1"),     # HYUNDAI.HYUNDAI_SONATA
  ("HYUNDAI2", "d545129f3ca90f28|2022-11-07--20-43-08--3"),    # HYUNDAI.HYUNDAI_KIA_EV6 (+ QCOM GPS)
  ("TOYOTA", "0982d79ebb0de295|2021-01-04--17-13-21--13"),     # TOYOTA.TOYOTA_PRIUS
  ("TOYOTA2", "0982d79ebb0de295|2021-01-03--20-03-36--6"),     # TOYOTA.TOYOTA_RAV4
  ("TOYOTA3", "8011d605be1cbb77|000000cc--8e8d8ec716--6"),     # TOYOTA.TOYOTA_COROLLA_TSS2
  ("HONDA", "eb140f119469d9ab|2021-06-12--10-46-24--27"),      # HONDA.HONDA_CIVIC (NIDEC)
  ("HONDA2", "7d2244f34d1bbcda|2021-06-25--12-25-37--26"),     # HONDA.HONDA_ACCORD (BOSCH)
  ("CHRYSLER", "4deb27de11bee626|2021-02-20--11-28-55--8"),    # CHRYSLER.CHRYSLER_PACIFICA_2018_HYBRID
  ("RAM", "17fc16d840fe9d21|2023-04-26--13-28-44--5"),         # CHRYSLER.RAM_1500_5TH_GEN
  ("SUBARU", "341dccd5359e3c97|2022-09-12--10-35-33--3"),      # SUBARU.SUBARU_OUTBACK
  ("GM", "376bf99325883932|2022-10-27--13-41-22--1"),         # GM.CHEVROLET_BOLT_EUV
  ("NISSAN", "35336926920f3571|2021-02-12--18-38-48--46"),     # NISSAN.NISSAN_XTRAIL
  ("VOLKSWAGEN", "de9592456ad7d144|2021-06-29--11-00-15--6"),  # VOLKSWAGEN.VOLKSWAGEN_GOLF
  # FIXME the sensor timings are bad in mazda segment, we're not fully testing it, but it should be replaced
  ("MAZDA", "bd6a637565e91581|2021-10-30--15-14-53--4"),       # MAZDA.MAZDA_CX9_2021
  ("FORD", "54827bf84c38b14f|2023-01-26--21-59-07--4"),        # FORD.FORD_BRONCO_SPORT_MK1
  ("RIVIAN", "bc095dc92e101734|000000db--ee9fe46e57--1"),      # RIVIAN.RIVIAN_R1_GEN1
  ("TESLA", "2c912ca5de3b1ee9|0000025d--6eb6bcbca4--4"),       # TESLA.TESLA_MODEL_Y

  # Enable when port is tested and dashcamOnly is no longer set
  #("VOLKSWAGEN2", "3cfdec54aa035f3f|2022-07-19--23-45-10--2"),  # VOLKSWAGEN.VOLKSWAGEN_PASSAT_NMS
]

# dashcamOnly makes don't need to be tested until a full port is done.
# "mg" is also excluded because xnor-tech's MG ZS EV port doesn't have test
# routes in routes.py yet — inherited from the upstream merge and not used by NAP.
excluded_interfaces = ["mock", "body", "psa", "mg"]

REF_COMMIT_FN = os.path.join(PROC_REPLAY_DIR, "ref_commit")
REF_COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")

# Audited pre-manifest artifact tree SHA. Remove after the first manifest-backed release lands.
LEGACY_ARTIFACTS_COMMITS: frozenset[str] = frozenset({"2c9cdb739cd53df058130e480fd6a3a23e6c4387"})

MAX_GITHUB_REF_BODY_BYTES = 64 * 1024
MAX_REF_COMMIT_FILE_BYTES = 41  # 40 ASCII hex + optional LF
MAX_MANIFEST_BYTES = 2 * 1024 * 1024
MAX_SOURCE_COMPRESSED_BYTES = 64 * 1024 * 1024
MAX_REFERENCE_COMPRESSED_BYTES = 64 * 1024 * 1024
MAX_LOG_DECOMPRESSED_BYTES = 512 * 1024 * 1024
MAX_LOG_MESSAGES = 2_000_000

_OPEN_NOFOLLOW_FLAGS = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)

ParamValue = bool | int | float
TaskStatus = Literal["pass", "diff", "error"]


class ManifestUnavailable(Exception):
  """Exact missing/404 for a pinned artifacts object."""


def _freeze_params(params: Mapping[str, ParamValue] | None) -> Mapping[str, ParamValue]:
  return MappingProxyType(dict(params or {}))


def validate_case_id(case_id: str) -> str:
  if not CASE_ID_RE.match(case_id):
    raise ValueError(f"invalid case_id {case_id!r}; expected {CASE_ID_RE.pattern}")
  return case_id


def validate_ref_commit(value: str) -> str:
  if not isinstance(value, str) or REF_COMMIT_RE.fullmatch(value) is None:
    raise ValueError(f"ref_commit must be a full lowercase SHA1, got {value!r}")
  return value


def decode_ref_commit_bytes(raw: bytes) -> str:
  text = raw.decode("utf-8")
  if text.endswith("\n"):
    text = text[:-1]
  if "\n" in text or "\r" in text:
    raise ValueError("ref_commit must be a single line")
  return validate_ref_commit(text)


def validate_source_metadata(
  case_id: str,
  executable: bool,
  source_bytes: int | None,
  source_sha256: str | None,
) -> None:
  has_bytes = source_bytes is not None
  has_sha = source_sha256 is not None
  if has_bytes ^ has_sha:
    raise ValueError(
      f"{case_id}: partial source metadata is rejected; provide both source_bytes and source_sha256, or neither"
    )
  if executable:
    if not (has_bytes and has_sha):
      raise ValueError(f"{case_id}: executable ReplayCase requires both source_bytes and source_sha256")
    if not isinstance(source_bytes, int) or isinstance(source_bytes, bool) or source_bytes <= 0:
      raise ValueError(f"{case_id}: source_bytes must be a positive int")
    if source_bytes > MAX_SOURCE_COMPRESSED_BYTES:
      raise ValueError(f"{case_id}: source_bytes exceeds MAX_SOURCE_COMPRESSED_BYTES")
    if not isinstance(source_sha256, str) or not re.fullmatch(r"[0-9a-f]{64}", source_sha256):
      raise ValueError(f"{case_id}: source_sha256 must be 64 lowercase hex")
  elif has_bytes or has_sha:
    raise ValueError(f"{case_id}: pending non-executable descriptors must omit source_bytes and source_sha256")


def resolve_regular_member(root: str | os.PathLike[str], filename: str) -> Path:
  if filename != os.path.basename(filename) or not filename or filename in {".", ".."}:
    raise ValueError(f"ref member filename must be a basename, got {filename!r}")
  root_path = Path(root).resolve(strict=True)
  if not root_path.is_dir():
    raise ValueError(f"reference root is not a directory: {root_path}")
  candidate = root_path / filename
  st = os.lstat(candidate)
  if stat.S_ISLNK(st.st_mode):
    raise ValueError(f"symlink forbidden in reference tree: {candidate}")
  if not stat.S_ISREG(st.st_mode):
    raise ValueError(f"reference member must be a regular file: {candidate}")
  resolved = candidate.resolve(strict=True)
  try:
    resolved.relative_to(root_path)
  except ValueError as exc:
    raise ValueError(f"reference member escapes root {root_path}: {resolved}") from exc
  return resolved


def read_regular_member_bytes(root: str | os.PathLike[str], filename: str, *, max_bytes: int) -> bytes:
  path = resolve_regular_member(root, filename)
  fd = os.open(path, _OPEN_NOFOLLOW_FLAGS)
  try:
    st = os.fstat(fd)
    if stat.S_ISLNK(st.st_mode) or not stat.S_ISREG(st.st_mode):
      raise ValueError(f"reference member must stay a regular file: {path}")
    data = os.read(fd, max_bytes + 1)
  finally:
    os.close(fd)
  if len(data) > max_bytes:
    raise ValueError(f"reference member exceeds {max_bytes} bytes: {path}")
  return data


def _is_missing_artifact_error(exc: BaseException) -> bool:
  if isinstance(exc, ManifestUnavailable):
    return True
  if isinstance(exc, (URLFileException, AssertionError, FileNotFoundError)):
    msg = str(exc).lower()
    return any(token in msg for token in ("doesn't exist", "does not exist", "not found", "404"))
  return False


def read_url_bytes_bounded(url: str, *, max_bytes: int, url_file_cls: type = URLFile) -> bytes:
  try:
    handle = url_file_cls(url, cache=False)
    data = handle.read(max_bytes + 1)
  except Exception as exc:
    if _is_missing_artifact_error(exc):
      raise ManifestUnavailable(str(exc)) from exc
    raise
  if len(data) > max_bytes:
    raise ValueError(f"remote artifact exceeds {max_bytes} bytes: {url}")
  return data


def decompress_log_bytes(data: bytes, *, max_output_bytes: int = MAX_LOG_DECOMPRESSED_BYTES) -> bytes:
  if data.startswith(b"\x28\xb5\x2f\xfd"):
    dctx = zstd.ZstdDecompressor()
    chunks: list[bytes] = []
    total = 0
    with dctx.stream_reader(data) as reader:
      while True:
        chunk = reader.read(1024 * 1024)
        if not chunk:
          break
        total += len(chunk)
        if total > max_output_bytes:
          raise ValueError(f"decompressed log exceeds {max_output_bytes} bytes")
        chunks.append(chunk)
    return b"".join(chunks)
  if data.startswith(b"BZh9"):
    import bz2
    if len(data) > MAX_REFERENCE_COMPRESSED_BYTES:
      raise ValueError("bz2 payload exceeds compressed bound")
    out = bz2.decompress(data)
    if len(out) > max_output_bytes:
      raise ValueError(f"decompressed log exceeds {max_output_bytes} bytes")
    return out
  if len(data) > max_output_bytes:
    raise ValueError(f"uncompressed log exceeds {max_output_bytes} bytes")
  return data


def read_reference_bytes(locator: str, *, max_bytes: int = MAX_REFERENCE_COMPRESSED_BYTES,
                         url_file_cls: type = URLFile) -> bytes:
  if locator.startswith(("http://", "https://")):
    return read_url_bytes_bounded(locator, max_bytes=max_bytes, url_file_cls=url_file_cls)
  root = os.path.dirname(locator) or "."
  filename = os.path.basename(locator)
  return read_regular_member_bytes(root, filename, max_bytes=max_bytes)


def load_ref_log_messages(locator: str, *, expected_bytes: int | None = None,
                          expected_sha256: str | None = None,
                          url_file_cls: type = URLFile) -> list[Any]:
  max_bytes = expected_bytes if expected_bytes is not None else MAX_REFERENCE_COMPRESSED_BYTES
  if expected_bytes is not None and expected_bytes > MAX_REFERENCE_COMPRESSED_BYTES:
    raise ValueError("reference declared size exceeds MAX_REFERENCE_COMPRESSED_BYTES")
  raw = read_reference_bytes(locator, max_bytes=max_bytes, url_file_cls=url_file_cls)
  if expected_bytes is not None and len(raw) != expected_bytes:
    raise ValueError(f"reference size mismatch expected={expected_bytes} actual={len(raw)}")
  if expected_sha256 is not None:
    digest = hashlib.sha256(raw).hexdigest()
    if digest != expected_sha256:
      raise ValueError(f"reference sha256 mismatch expected={expected_sha256} actual={digest}")
  # Always decompress under a hard ceiling, then parse the bounded payload.
  if raw.startswith((b"\x28\xb5\x2f\xfd", b"BZh9")):
    payload = decompress_log_bytes(raw)
  else:
    payload = decompress_log_bytes(raw)
  lr = LogReader.from_bytes(payload)
  out: list[Any] = []
  for msg in lr:
    out.append(msg)
    if len(out) > MAX_LOG_MESSAGES:
      raise ValueError(f"reference log exceeds MAX_LOG_MESSAGES ({MAX_LOG_MESSAGES})")
  return out


@dataclass(frozen=True)
class ReplayCase:
  __hash__ = None

  case_id: str
  car_brand: str
  source: str
  processes: tuple[str, ...]
  custom_params: Mapping[str, ParamValue] = field(default_factory=dict)
  source_sha256: str | None = None
  source_bytes: int | None = None

  def __post_init__(self):
    object.__setattr__(self, "custom_params", _freeze_params(self.custom_params))
    object.__setattr__(self, "processes", tuple(self.processes))
    validate_case_id(self.case_id)
    unknown = [p for p in self.processes if p not in PROC_BY_NAME]
    if unknown:
      raise ValueError(f"{self.case_id}: unknown processes {unknown}")
    if len(set(self.processes)) != len(self.processes):
      raise ValueError(f"{self.case_id}: duplicate processes")
    validate_source_metadata(self.case_id, bool(self.source), self.source_bytes, self.source_sha256)

  @property
  def executable(self) -> bool:
    return bool(self.source)

  def source_url(self) -> str:
    if not self.executable:
      raise ValueError(f"{self.case_id}: source is not executable")
    if self.source.startswith(("http://", "https://")):
      return self.source
    route, seg_num = self.source.rsplit("--", 1)
    return get_url(route, seg_num, "rlog.zst")

  def to_record(self) -> dict[str, Any]:
    return {
      "case_id": self.case_id,
      "car_brand": self.car_brand,
      "source": self.source,
      "processes": list(self.processes),
      "custom_params": dict(self.custom_params),
      "source_sha256": self.source_sha256,
      "source_bytes": self.source_bytes,
      "executable": self.executable,
      "params_digest": params_digest(self.custom_params),
    }


@dataclass(frozen=True)
class ReplayTask:
  case: ReplayCase
  process: str

  def __post_init__(self):
    if self.process not in self.case.processes:
      raise ValueError(f"{self.process} not in case {self.case.case_id}")
    if self.process not in PROC_BY_NAME:
      raise ValueError(f"unknown process {self.process}")

  @property
  def task_id(self) -> str:
    return f"{self.case.case_id}:{self.process}"


@dataclass(frozen=True)
class TaskResult:
  status: TaskStatus
  task_id: str
  elapsed_s: float
  diff_or_traceback: str
  ref_path: str
  new_path: str
  message_counts: Mapping[str, int] = field(default_factory=dict)

  def __post_init__(self):
    object.__setattr__(self, "message_counts", dict(self.message_counts))

  def to_record(self) -> dict[str, Any]:
    return {
      "status": self.status,
      "task_id": self.task_id,
      "elapsed_s": self.elapsed_s,
      "diff_or_traceback": self.diff_or_traceback,
      "ref_path": self.ref_path,
      "new_path": self.new_path,
      "message_counts": dict(self.message_counts),
    }


# Exact active-source digests measured from the current public route sources.
ACTIVE_SOURCE_DIGESTS: dict[str, dict[str, int | str]] = {
  "chrysler": {"bytes": 11951101, "sha256": "a30f65eb1033afbd0cff70d965aa9c0a276cc5ceb73d28baa817430df178009a"},
  "ford": {"bytes": 11315727, "sha256": "786dfef08325d24e8205320862adc940706ff3475ae082ea104127fb5da62823"},
  "gm": {"bytes": 9846869, "sha256": "6c10ce899270e4fc8350730e44cb39dfbda451670dd598798207ffc929baa69a"},
  "honda": {"bytes": 10090147, "sha256": "8ea4ab5412a99bfc65fbe95cd169a54edb6bcb68f6fa20c97f2bc58ed4bba538"},
  "honda2": {"bytes": 10419375, "sha256": "fc9ab2b4fd9702ceb8e3342186076dc6a85dbd6a3bd859ac6b965234bcc65c69"},
  "hyundai": {"bytes": 11873030, "sha256": "0b6734026f7c775f7fd648ef76130f9b9a672c5236c6a328eff31ec9cc3c5343"},
  "hyundai2": {"bytes": 12527194, "sha256": "4cc51a77bf8d128d5720fd692e5a0994c887ee35fdd949cc0ce54d4215e2b7a2"},
  "mazda": {"bytes": 12309353, "sha256": "a84e9f84c39802f58a64031e7c56b362c0e745b5c216848ca2d561f5ae6d440a"},
  "nissan": {"bytes": 11775886, "sha256": "58f3822418e86d8c4afca79ef9058602c8825fadaedf8b63477ed53a2b9ba748"},
  "ram": {"bytes": 11420074, "sha256": "cc1d8bb9729a71c3166e04343e436a67fbc1ff7107a30201a5d13ea9b45ea143"},
  "rivian": {"bytes": 11022997, "sha256": "2f2574c057b7cb55fbb2e05c99c5f1b1d6d894222422c4b51d4363ad66e6c840"},
  "subaru": {"bytes": 9393878, "sha256": "f501c1c99bae27ca710f40f723a1960ea4e898836d7edfb57ea6397591e5aff2"},
  "tesla": {"bytes": 8287366, "sha256": "ca3bfc37b118f0e1987c2177b77b7446d44fbc7806cc5fae71c2ed79d4776fff"},
  "toyota": {"bytes": 10846201, "sha256": "6c028c0d40328fc00fa12acf91dba30d9f152e89d99a7089aadac62f6c7d5d64"},
  "toyota3": {"bytes": 9272538, "sha256": "876100021b13ddd1a39efad5fdb4e7376ad5b298f64bf390b5f58742ffc6e429"},
  "volkswagen": {"bytes": 10593610, "sha256": "bca512375f61ade1af793227f4aa9137ba1cbd0067d3b92f1242f5907f84a0af"},
}


def _legacy_case(case_id: str, car_brand: str, segment: str, full: bool) -> ReplayCase:
  processes = ALWAYS_PROCS + (EXTRA_PROCS if full else ())
  digest = ACTIVE_SOURCE_DIGESTS[case_id]
  return ReplayCase(
    case_id=case_id,
    car_brand=car_brand,
    source=segment,
    processes=processes,
    source_bytes=int(digest["bytes"]),
    source_sha256=str(digest["sha256"]),
  )


ACTIVE_CASES: tuple[ReplayCase, ...] = (
  _legacy_case("hyundai", "HYUNDAI", "regenAA0FC4ED71E|2025-04-08--22-57-50--0", True),
  _legacy_case("hyundai2", "HYUNDAI2", "regenAFB9780D823|2025-04-08--23-00-34--0", False),
  _legacy_case("toyota", "TOYOTA", "regen218A4DCFAA1|2025-04-08--22-57-51--0", True),
  # TODO: get new RAV4 route without enableDsu
  # _legacy_case("toyota2", "TOYOTA2", "regen107352E20EB|2025-04-08--22-57-46--0", False),
  _legacy_case("toyota3", "TOYOTA3", "regen1455E3B4BDF|2025-04-09--03-26-06--0", False),
  _legacy_case("honda", "HONDA", "regenB328FF8BA0A|2025-04-08--22-57-45--0", False),
  _legacy_case("honda2", "HONDA2", "regen6170C8C9A35|2025-04-08--22-57-46--0", False),
  _legacy_case("chrysler", "CHRYSLER", "regen5B28FC2A437|2025-04-08--23-04-24--0", False),
  _legacy_case("ram", "RAM", "regenBF81EA96E08|2025-04-08--23-06-54--0", False),
  _legacy_case("subaru", "SUBARU", "regen7366F13F6A1|2025-04-08--23-07-07--0", False),
  _legacy_case("gm", "GM", "regen1271097D038|2025-04-09--03-26-00--0", False),
  _legacy_case("nissan", "NISSAN", "regen15D60604EAB|2025-04-08--23-06-59--0", False),
  _legacy_case("volkswagen", "VOLKSWAGEN", "regen0F2F06C9539|2025-04-08--23-06-56--0", False),
  _legacy_case("mazda", "MAZDA", "regenACF84CCF482|2024-08-30--03-21-55--0", False),
  _legacy_case("ford", "FORD", "regen755D8CB1E1F|2025-04-08--23-13-43--0", False),
  _legacy_case("rivian", "RIVIAN", "regen5FCAC896BBE|2025-04-08--23-13-35--0", False),
  _legacy_case("tesla", "TESLA", "2c912ca5de3b1ee9|0000025d--6eb6bcbca4--4", False),
)

# Compatibility alias for anything still iterating brand/segment pairs.
segments = [(c.car_brand, c.source) for c in ACTIVE_CASES]

NAP_PREAP_PEDAL_PARAMS: dict[str, ParamValue] = {
  "NAPForcePreAP": True,
  "NAPAdaptiveAccel": True,
  "NAPPedalEnabled": True,
  "NAPFollowDistance": 7,
  "NAPPedalProfile": 4,
  "NAPPedalCanBus": 2,
  "NAPPedalCalibDone": True,
  "NAPPedalCalibMin": 0.73299810159,
  "NAPPedalCalibMax": 110.23299810159,
  "NAPPedalCalibFactor": 0.9478672985781991,
  "NAPPedalCalibZero": 4.732998101589998,
  "NAPRadarEnabled": True,
  "NAPRadarBehindNosecone": True,
  "NAPRadarOffset": -0.5,
  "NAPiBoosterEnabled": False,
  "NAPBrakeFactor": 1.0,
}

NAP_PREAP_NO_PEDAL_PARAMS: dict[str, ParamValue] = {
  **NAP_PREAP_PEDAL_PARAMS,
  "NAPPedalEnabled": False,
  "NAPFollowDistance": 5,
  "NAPPedalCalibMin": 0.9382172261100007,
  "NAPPedalCalibMax": 110.43821722611,
  "NAPPedalCalibFactor": 0.946969696969697,
  "NAPPedalCalibZero": 4.838217226109999,
  "NAPRadarOffset": -0.25,
}

# Temporary staged descriptors for rollout PR A. Sources stay non-executable until fixture publish.
PENDING_CASES: tuple[ReplayCase, ...] = (
  ReplayCase(
    case_id="nap-preap-pedal-v1",
    car_brand="TESLA",
    source="",
    processes=PREAP_PROCS,
    custom_params=NAP_PREAP_PEDAL_PARAMS,
  ),
  ReplayCase(
    case_id="nap-preap-no-pedal-v1",
    car_brand="TESLA",
    source="",
    processes=PREAP_PROCS,
    custom_params=NAP_PREAP_NO_PEDAL_PARAMS,
  ),
)

EMPTY_PARAMS_DIGEST = "37517e5f3dc66819f61f5a7bb8ace1921282415f10551d2defa5c3eb0985b570"
NAP_PREAP_PEDAL_PARAMS_DIGEST = "b5217b239c759d5a960872a99f2e9ff6cacdb37aff6e99e1ee1bd0c20dead809"
NAP_PREAP_NO_PEDAL_PARAMS_DIGEST = "96b5f9f76bdf20f24bf2a29d408d87e60ebf377eccc3858b4f0c0de7c3c2e786"


def params_type_name(value: ParamValue) -> str:
  if type(value) is bool:
    return "bool"
  if type(value) is int:
    return "int"
  if type(value) is float:
    return "float"
  raise TypeError(f"unsupported param value type {type(value)!r}")


def canonical_params_manifest(params: Mapping[str, ParamValue]) -> bytes:
  """Canonical JSON array sorted by key: key/type/value with fixed separators and trailing LF."""
  parts: list[str] = []
  for key in sorted(params):
    value = params[key]
    type_name = params_type_name(value)
    if type_name == "bool":
      encoded = "true" if value else "false"
    elif type_name == "int":
      encoded = str(value)
    else:
      if not isinstance(value, float) or not math.isfinite(value):
        raise ValueError(f"param {key} must be a finite float")
      encoded = repr(value)
    parts.append(f'{{"key":"{key}","type":"{type_name}","value":{encoded}}}')
  return ("[" + ",".join(parts) + "]\n").encode("utf-8")


def params_digest(params: Mapping[str, ParamValue]) -> str:
  return hashlib.sha256(canonical_params_manifest(params)).hexdigest()


def expected_param_key_type(value: ParamValue) -> ParamKeyType:
  return {
    "bool": ParamKeyType.BOOL,
    "int": ParamKeyType.INT,
    "float": ParamKeyType.FLOAT,
  }[params_type_name(value)]


def validate_params_against_header(params: Mapping[str, ParamValue], params_obj: Params | None = None) -> None:
  checker = params_obj or Params()
  for key, value in params.items():
    declared = checker.get_type(key)
    expected = expected_param_key_type(value)
    if declared != expected:
      raise TypeError(f"param {key}: value type {type(value)!r} does not match header type {declared!r}")


def build_tasks(cases: Iterable[ReplayCase]) -> list[ReplayTask]:
  return [ReplayTask(case=case, process=proc) for case in cases for proc in case.processes]


def select_cases(inventory: Literal["active", "staged"] = "active") -> tuple[ReplayCase, ...]:
  if inventory == "active":
    return ACTIVE_CASES
  if inventory == "staged":
    return ACTIVE_CASES + PENDING_CASES
  raise ValueError(f"unknown inventory {inventory!r}")


def assert_inventory_invariants(cases: Iterable[ReplayCase]) -> None:
  cases = tuple(cases)
  case_ids = [c.case_id for c in cases]
  if len(case_ids) != len(set(case_ids)):
    raise AssertionError(f"duplicate case_ids: {case_ids}")
  tasks = build_tasks(cases)
  task_ids = [t.task_id for t in tasks]
  if len(task_ids) != len(set(task_ids)):
    raise AssertionError(f"duplicate task_ids: {task_ids}")
  for case in cases:
    validate_source_metadata(case.case_id, case.executable, case.source_bytes, case.source_sha256)
    validate_params_against_header(case.custom_params)
    for key in case.custom_params:
      if not key:
        raise AssertionError("empty param key")


def unique_cases_by_id(tasks: Iterable[ReplayTask]) -> dict[str, ReplayCase]:
  """Deterministic case_id deduplication; never hash ReplayCase (custom_params is unhashable)."""
  out: dict[str, ReplayCase] = {}
  for task in tasks:
    existing = out.get(task.case.case_id)
    if existing is None:
      out[task.case.case_id] = task.case
    elif existing != task.case:
      raise AssertionError(f"conflicting ReplayCase records for case_id={task.case.case_id}")
  return out


def filter_tasks(
  tasks: Iterable[ReplayTask],
  *,
  whitelist_procs: Iterable[str] | None = None,
  blacklist_procs: Iterable[str] | None = None,
  whitelist_cars: Iterable[str] | None = None,
  blacklist_cars: Iterable[str] | None = None,
) -> list[ReplayTask]:
  all_procs = {t.process for t in tasks} or set(ALL_REPLAY_PROCS)
  all_cars = {t.case.car_brand for t in tasks}
  tested_procs = set(whitelist_procs if whitelist_procs is not None else all_procs) - set(blacklist_procs or [])
  tested_cars = {c.upper() for c in (whitelist_cars if whitelist_cars is not None else all_cars)} - {c.upper() for c in (blacklist_cars or [])}
  return [t for t in tasks if t.process in tested_procs and t.case.car_brand.upper() in tested_cars]


def shard_task_ids(tasks: Iterable[ReplayTask], shard: Literal["card", "controlsd", "lagd", "other"]) -> list[str]:
  if shard == "other":
    blocked = {"card", "controlsd", "lagd"}
    return [t.task_id for t in tasks if t.process not in blocked]
  return [t.task_id for t in tasks if t.process == shard]


def ref_filename(case_id: str, process: str, ref_commit: str) -> str:
  validate_case_id(case_id)
  validate_ref_commit(ref_commit)
  return f"{case_id}__{process}__{ref_commit}.zst"


def legacy_ref_filename(segment: str, process: str, ref_commit: str) -> str:
  validate_ref_commit(ref_commit)
  return f"{segment}_{process}_{ref_commit}.zst".replace("|", "_")


def pin_artifacts_commit(branch: str = ARTIFACTS_BRANCH, opener: Callable[..., Any] = urlopen) -> str:
  req = Request(f"https://api.github.com/repos/{ARTIFACTS_REPO}/git/ref/heads/{branch}", headers={"Accept": "application/vnd.github+json"})
  with opener(req, timeout=30) as resp:
    body = resp.read(MAX_GITHUB_REF_BODY_BYTES + 1)
  if len(body) > MAX_GITHUB_REF_BODY_BYTES:
    raise ValueError("GitHub ref response exceeds MAX_GITHUB_REF_BODY_BYTES")
  payload = json.loads(body.decode("utf-8"))
  sha = payload["object"]["sha"]
  return validate_ref_commit(sha)


def artifacts_file_url(artifacts_commit: str, filename: str) -> str:
  if not re.fullmatch(r"[0-9a-f]{40}", artifacts_commit):
    raise ValueError(f"artifacts commit must be a full SHA, got {artifacts_commit!r}")
  return f"{ARTIFACTS_RAW}/{artifacts_commit}/{filename}"


def resolve_ref_commit(
  artifacts_commit: str | None = None,
  ref_commit_path: str = REF_COMMIT_FN,
  *,
  reference_dir: str | None = None,
  pin_fn: Callable[..., str] = pin_artifacts_commit,
  url_file_cls: type = URLFile,
) -> tuple[str, str | None]:
  if reference_dir is not None:
    raw = read_regular_member_bytes(reference_dir, "ref_commit", max_bytes=MAX_REF_COMMIT_FILE_BYTES)
    return decode_ref_commit_bytes(raw), artifacts_commit

  if os.path.lexists(ref_commit_path):
    root = os.path.dirname(ref_commit_path) or "."
    raw = read_regular_member_bytes(root, os.path.basename(ref_commit_path), max_bytes=MAX_REF_COMMIT_FILE_BYTES)
    return decode_ref_commit_bytes(raw), artifacts_commit

  if artifacts_commit is None:
    artifacts_commit = pin_fn()
  validate_ref_commit(artifacts_commit)
  raw = read_url_bytes_bounded(
    artifacts_file_url(artifacts_commit, "ref_commit"),
    max_bytes=MAX_REF_COMMIT_FILE_BYTES,
    url_file_cls=url_file_cls,
  )
  return decode_ref_commit_bytes(raw), artifacts_commit


def write_candidate_ref_commit(ref_commit: str, candidate_dir: str = FAKEDATA) -> str:
  """Write candidate ref_commit beside generated refs so a copied tree is reusable with --reference-dir."""
  validate_ref_commit(ref_commit)
  os.makedirs(candidate_dir, exist_ok=True)
  path = os.path.join(candidate_dir, "ref_commit")
  with atomic_write(path, mode="w", encoding="utf-8", overwrite=True) as f:
    f.write(ref_commit)
    f.write("\n")
  return path


def parse_artifacts_manifest(
  raw: bytes,
  *,
  expected_ref_commit: str,
  selected_task_ids: Iterable[str] | None = None,
) -> dict[str, Any]:
  if not raw:
    raise ValueError("malformed artifacts manifest: empty body")
  try:
    payload = json.loads(raw.decode("utf-8"))
  except (UnicodeDecodeError, json.JSONDecodeError) as exc:
    raise ValueError(f"malformed artifacts manifest: {exc}") from exc
  if not isinstance(payload, dict):
    raise ValueError("malformed artifacts manifest: expected object")
  schema_version = payload.get("schema_version")
  if schema_version != 1:
    raise ValueError(f"unsupported artifacts manifest schema_version: {schema_version!r}")
  manifest_ref = payload.get("ref_commit", payload.get("openpilot_sha"))
  if manifest_ref != expected_ref_commit:
    raise ValueError(
      f"artifacts manifest ref identity mismatch: expected {expected_ref_commit}, got {manifest_ref!r}"
    )
  refs = payload.get("refs")
  if not isinstance(refs, dict):
    raise ValueError("malformed artifacts manifest: refs must be an object")
  for task_id, meta in refs.items():
    if not isinstance(task_id, str) or ":" not in task_id:
      raise ValueError(f"malformed artifacts manifest task id: {task_id!r}")
    if not isinstance(meta, dict):
      raise ValueError(f"malformed artifacts manifest refs[{task_id!r}]")
    filename = meta.get("filename")
    size = meta.get("size", meta.get("bytes"))
    sha256 = meta.get("sha256")
    if not isinstance(filename, str) or filename != os.path.basename(filename):
      raise ValueError(f"malformed artifacts manifest filename for {task_id}")
    if size is not None:
      if not isinstance(size, int) or isinstance(size, bool) or size <= 0 or size > MAX_REFERENCE_COMPRESSED_BYTES:
        raise ValueError(f"malformed artifacts manifest size for {task_id}")
    if sha256 is not None and (not isinstance(sha256, str) or not re.fullmatch(r"[0-9a-f]{64}", sha256)):
      raise ValueError(f"malformed artifacts manifest sha256 for {task_id}")
  if selected_task_ids is not None:
    missing = [tid for tid in selected_task_ids if tid not in refs]
    if missing:
      raise ValueError(f"artifacts manifest missing task metadata for {missing[0]}")
  return payload


def load_artifacts_manifest(
  artifacts_commit: str,
  *,
  expected_ref_commit: str,
  selected_task_ids: Iterable[str] | None = None,
  url_file_cls: type = URLFile,
) -> dict[str, Any] | None:
  """Load pinned manifest.json once.

  Returns None only for an exact missing manifest on an allowlisted pre-bootstrap commit.
  """
  validate_ref_commit(artifacts_commit)
  validate_ref_commit(expected_ref_commit)
  url = artifacts_file_url(artifacts_commit, "manifest.json")
  try:
    raw = read_url_bytes_bounded(url, max_bytes=MAX_MANIFEST_BYTES, url_file_cls=url_file_cls)
  except ManifestUnavailable as error:
    if artifacts_commit in LEGACY_ARTIFACTS_COMMITS:
      return None
    raise ValueError(f"artifacts manifest missing for non-legacy commit {artifacts_commit}") from error
  return parse_artifacts_manifest(
    raw,
    expected_ref_commit=expected_ref_commit,
    selected_task_ids=selected_task_ids,
  )


def resolve_ref_path(
  task: ReplayTask,
  *,
  ref_commit: str,
  artifacts_commit: str | None,
  reference_dir: str | None,
  update_refs: bool,
  artifacts_manifest: dict[str, Any] | None = None,
  pin_fn: Callable[..., str] = pin_artifacts_commit,
) -> str:
  if update_refs:
    # Candidate generation does not read an existing reference.
    return ""

  validate_ref_commit(ref_commit)
  new_name = ref_filename(task.case.case_id, task.process, ref_commit)
  legacy_name = None
  if task.case.source and not task.case.source.startswith(("http://", "https://")):
    legacy_name = legacy_ref_filename(task.case.source, task.process, ref_commit)

  def _local_regular(root: str, filename: str) -> str | None:
    try:
      return str(resolve_regular_member(root, filename))
    except FileNotFoundError:
      return None

  # A local candidate tree is authoritative. In particular, do not consult
  # FAKEDATA when --reference-dir points at a copied candidate tree.
  if reference_dir is not None:
    Path(reference_dir).resolve(strict=True)  # must exist as a real directory
    found = _local_regular(reference_dir, new_name)
    if found is None and legacy_name is not None:
      found = _local_regular(reference_dir, legacy_name)
    if found is not None:
      return found
    # Caller asked for a local candidate tree; do not silently fall back to remote.
    return str(Path(reference_dir).resolve(strict=True) / new_name)

  # Default workspace: only accept regular non-symlink members under FAKEDATA.
  if os.path.isdir(FAKEDATA):
    found = _local_regular(FAKEDATA, new_name)
    if found is None and legacy_name is not None:
      found = _local_regular(FAKEDATA, legacy_name)
    if found is not None:
      return found

  if artifacts_commit is None:
    artifacts_commit = pin_fn()
  validate_ref_commit(artifacts_commit)

  if artifacts_manifest is not None:
    refs = artifacts_manifest.get("refs")
    if not isinstance(refs, dict):
      raise ValueError("malformed artifacts manifest: refs must be an object")
    meta = refs.get(task.task_id)
    if not isinstance(meta, dict):
      raise ValueError(f"artifacts manifest missing task metadata for {task.task_id}")
    filename = meta.get("filename")
    if filename != new_name:
      raise ValueError(
        f"artifacts manifest has non-canonical filename for {task.task_id}: expected {new_name!r}, got {filename!r}"
      )
    return artifacts_file_url(artifacts_commit, new_name)

  # Pre-bootstrap trees have no manifest. Only allowlisted commits may use
  # the historical segment filename; once a manifest exists it is canonical.
  if artifacts_commit not in LEGACY_ARTIFACTS_COMMITS:
    raise ValueError(
      f"artifacts commit {artifacts_commit} has no manifest and is not in LEGACY_ARTIFACTS_COMMITS"
    )
  if legacy_name is not None:
    return artifacts_file_url(artifacts_commit, legacy_name)
  return artifacts_file_url(artifacts_commit, new_name)


def new_log_path(task: ReplayTask, tested_commit: str, output_dir: str | None = None) -> str:
  # output_dir is accepted for tests only; normal main keeps generated logs in FAKEDATA.
  validate_ref_commit(tested_commit)
  base = output_dir or FAKEDATA
  os.makedirs(base, exist_ok=True)
  return os.path.join(base, ref_filename(task.case.case_id, task.process, tested_commit))


def clear_stale_fakedata(path: str = FAKEDATA) -> None:
  if os.path.isdir(path):
    shutil.rmtree(path)
  os.makedirs(path, exist_ok=True)


def download_case_bytes(case: ReplayCase) -> bytes:
  if not case.executable:
    raise ValueError(f"{case.case_id}: refusing to download non-executable pending source")
  if case.source_bytes is None or case.source_sha256 is None:
    raise ValueError(f"{case.case_id}: executable source requires size and sha256")
  if case.source_bytes > MAX_SOURCE_COMPRESSED_BYTES:
    raise ValueError(f"{case.case_id}: source_bytes exceeds MAX_SOURCE_COMPRESSED_BYTES")
  with FileReader(case.source_url()) as f:
    data = f.read(case.source_bytes + 1)
  if len(data) != case.source_bytes:
    raise ValueError(f"{case.case_id}: size mismatch expected={case.source_bytes} actual={len(data)}")
  digest = hashlib.sha256(data).hexdigest()
  if digest != case.source_sha256:
    raise ValueError(f"{case.case_id}: sha256 mismatch expected={case.source_sha256} actual={digest}")
  return data


def download_case_by_id(case_payload: dict[str, Any]) -> tuple[str, bytes | None, str | None]:
  case_id = case_payload["case_id"]
  try:
    case = ReplayCase(
      case_id=case_payload["case_id"],
      car_brand=case_payload["car_brand"],
      source=case_payload["source"],
      processes=tuple(case_payload["processes"]),
      custom_params=case_payload["custom_params"],
      source_sha256=case_payload.get("source_sha256"),
      source_bytes=case_payload.get("source_bytes"),
    )
    return case_id, download_case_bytes(case), None
  except Exception:
    return case_id, None, traceback.format_exc()


def message_counts(log_msgs: Iterable[Any]) -> dict[str, int]:
  return dict(Counter(m.which() for m in log_msgs))


def test_process(cfg, lr, task: ReplayTask, ref_log_path: str, new_log_path: str, ignore_fields=None, ignore_msgs=None,
                 update_refs: bool = False) -> tuple[TaskStatus, str, list[Any]]:
  if ignore_fields is None:
    ignore_fields = []
  if ignore_msgs is None:
    ignore_msgs = []

  log_msgs = replay_process(cfg, lr, disable_progress=True, custom_params=dict(task.case.custom_params))
  # Persist immediately so missing/corrupt refs still retain the generated log.
  save_log(new_log_path, log_msgs)

  if not check_most_messages_valid(log_msgs):
    return "error", f"Route did not have enough valid messages: {new_log_path}", log_msgs

  if cfg.proc_name != "ubloxd" or any(m.which() in cfg.pubs for m in lr):
    seen_msgs = {m.which() for m in log_msgs}
    expected_msgs = set(cfg.subs)
    if seen_msgs != expected_msgs:
      return "error", f"Expected messages: {expected_msgs}, but got: {seen_msgs}", log_msgs

  if update_refs or not ref_log_path:
    return "pass", "", log_msgs

  try:
    ref_log_msgs = load_ref_log_messages(ref_log_path)
    diff = compare_logs(ref_log_msgs, log_msgs, ignore_fields + cfg.ignore, ignore_msgs, cfg.tolerance)
  except Exception as e:
    return "error", str(e), log_msgs

  if isinstance(diff, str) or len(diff):
    return "diff", diff if isinstance(diff, str) else json.dumps([list(x) if not isinstance(x, str) else x for x in diff], default=str), log_msgs
  return "pass", "", log_msgs


def run_test_process(data: dict[str, Any]) -> TaskResult:
  task_id = data["task_id"]
  ref_path = data.get("ref_log_path", "")
  new_path = data.get("new_log_path", "")
  started = time.perf_counter()
  try:
    case = ReplayCase(
      case_id=data["case_id"],
      car_brand=data["car_brand"],
      source=data["source"],
      processes=tuple(data["processes"]),
      custom_params=data["custom_params"],
      source_sha256=data.get("source_sha256"),
      source_bytes=data.get("source_bytes"),
    )
    task = ReplayTask(case=case, process=data["process"])
    cfg = PROC_BY_NAME[task.process]
    raw = data["lr_dat"]
    if not isinstance(raw, (bytes, bytearray)):
      raise TypeError("lr_dat must be bytes")
    if len(raw) > MAX_SOURCE_COMPRESSED_BYTES:
      raise ValueError("source payload exceeds MAX_SOURCE_COMPRESSED_BYTES")
    if raw.startswith((b"\x28\xb5\x2f\xfd", b"BZh9")):
      lr = LogReader.from_bytes(decompress_log_bytes(bytes(raw)))
    else:
      if len(raw) > MAX_LOG_DECOMPRESSED_BYTES:
        raise ValueError("uncompressed source exceeds MAX_LOG_DECOMPRESSED_BYTES")
      lr = LogReader.from_bytes(bytes(raw))
    status, detail, log_msgs = test_process(
      cfg, lr, task, ref_path, new_path,
      ignore_fields=data.get("ignore_fields") or [],
      ignore_msgs=data.get("ignore_msgs") or [],
      update_refs=bool(data.get("update_refs")),
    )
    if status == "diff" and not isinstance(detail, str):
      detail = str(detail)
    return TaskResult(
      status=status,
      task_id=task_id,
      elapsed_s=time.perf_counter() - started,
      diff_or_traceback=detail if isinstance(detail, str) else str(detail),
      ref_path=ref_path,
      new_path=new_path,
      message_counts=message_counts(log_msgs),
    )
  except Exception:
    return TaskResult(
      status="error",
      task_id=task_id,
      elapsed_s=time.perf_counter() - started,
      diff_or_traceback=traceback.format_exc(),
      ref_path=ref_path,
      new_path=new_path,
      message_counts={},
    )


def make_task_payload(task: ReplayTask, *, lr_dat: bytes, ref_log_path: str, new_log_path: str,
                      ignore_fields: list[str], ignore_msgs: list[str], update_refs: bool) -> dict[str, Any]:
  return {
    "task_id": task.task_id,
    "case_id": task.case.case_id,
    "car_brand": task.case.car_brand,
    "source": task.case.source,
    "processes": list(task.case.processes),
    "custom_params": dict(task.case.custom_params),
    "source_sha256": task.case.source_sha256,
    "source_bytes": task.case.source_bytes,
    "process": task.process,
    "lr_dat": lr_dat,
    "ref_log_path": ref_log_path,
    "new_log_path": new_log_path,
    "ignore_fields": ignore_fields,
    "ignore_msgs": ignore_msgs,
    "update_refs": update_refs,
  }


def format_diff_text(results: list[TaskResult], ref_commit: str) -> tuple[str, str, bool]:
  legacy_results: dict[str, dict[str, Any]] = defaultdict(dict)
  log_paths: dict[str, dict[str, dict[str, str]]] = defaultdict(lambda: defaultdict(dict))
  for result in results:
    case_id, proc = result.task_id.split(":", 1)
    if result.status == "pass":
      legacy_results[case_id][proc] = []
    elif result.status == "diff":
      legacy_results[case_id][proc] = result.diff_or_traceback
    else:
      legacy_results[case_id][proc] = result.diff_or_traceback
    log_paths[case_id][proc]["ref"] = result.ref_path
    log_paths[case_id][proc]["new"] = result.new_path
  return format_diff(legacy_results, log_paths, ref_commit)


def build_results_document(
  *,
  tested_commit: str,
  ref_commit: str,
  artifacts_commit: str | None,
  inventory: str,
  cases: Iterable[ReplayCase],
  expected_task_ids: Iterable[str],
  results: Mapping[str, TaskResult],
) -> dict[str, Any]:
  expected = sorted(expected_task_ids)
  settled = [results[tid] for tid in expected if tid in results]
  status_counts = Counter(r.status for r in settled)
  incomplete = [tid for tid in expected if tid not in results]
  return {
    "schema_version": RESULTS_SCHEMA_VERSION,
    "tested_commit": tested_commit,
    "ref_commit": ref_commit,
    "artifacts_commit": artifacts_commit,
    "inventory": inventory,
    "cases": [c.to_record() for c in cases],
    "expected_task_ids": expected,
    "completed_task_ids": sorted(results),
    "incomplete_task_ids": incomplete,
    "status_counts": {
      "pass": status_counts.get("pass", 0),
      "diff": status_counts.get("diff", 0),
      "error": status_counts.get("error", 0),
      "incomplete": len(incomplete),
    },
    "results": [results[tid].to_record() for tid in sorted(results)],
  }


def write_diagnostics(output_dir: str, document: dict[str, Any], results: list[TaskResult], ref_commit: str) -> None:
  os.makedirs(output_dir, exist_ok=True)
  results_path = os.path.join(output_dir, "results.json")
  diff_path = os.path.join(output_dir, "diff.txt")
  with atomic_write(results_path, mode="w", encoding="utf-8", overwrite=True) as f:
    json.dump(document, f, indent=2, sort_keys=True)
    f.write("\n")
  diff_short, diff_long, _ = format_diff_text(results, ref_commit)
  with atomic_write(diff_path, mode="w", encoding="utf-8", overwrite=True) as f:
    f.write(diff_long)
    if not diff_long.endswith("\n"):
      f.write("\n")
  return


def retain_failed_logs(results: Iterable[TaskResult]) -> None:
  for result in results:
    if not result.new_path:
      continue
    if result.status == "pass":
      if os.path.exists(result.new_path):
        os.remove(result.new_path)
    # failed/diff logs are retained


def run_task_pool(
  payloads: list[dict[str, Any]],
  *,
  jobs: int,
  worker: Callable[[dict[str, Any]], TaskResult] = run_test_process,
  on_settled: Callable[[TaskResult], None] | None = None,
  executor_factory=concurrent.futures.ProcessPoolExecutor,
) -> list[TaskResult]:
  results: list[TaskResult] = []
  if not payloads:
    return results

  with executor_factory(max_workers=max(jobs, 1)) as pool:
    future_map = {pool.submit(worker, payload): payload for payload in payloads}
    for fut in concurrent.futures.as_completed(future_map):
      payload = future_map[fut]
      try:
        result = fut.result()
      except Exception:
        result = TaskResult(
          status="error",
          task_id=payload["task_id"],
          elapsed_s=0.0,
          diff_or_traceback=traceback.format_exc(),
          ref_path=payload.get("ref_log_path", ""),
          new_path=payload.get("new_log_path", ""),
        )
      results.append(result)
      if on_settled is not None:
        on_settled(result)
  return results


def exit_code_for(document: dict[str, Any]) -> int:
  counts = document["status_counts"]
  if counts["diff"] or counts["error"] or counts["incomplete"]:
    return 1
  return 0


def main(
  argv: list[str] | None = None,
  *,
  get_commit_fn: Callable[[], str | None] = get_commit,
  pin_artifacts_commit_fn: Callable[..., str] = pin_artifacts_commit,
  resolve_ref_commit_fn: Callable[..., tuple[str, str | None]] = resolve_ref_commit,
  load_artifacts_manifest_fn: Callable[..., dict[str, Any] | None] = load_artifacts_manifest,
  download_case_by_id_fn: Callable[[dict[str, Any]], tuple[str, bytes | None, str | None]] = download_case_by_id,
  run_task_pool_fn: Callable[..., list[TaskResult]] = run_task_pool,
  clear_stale_fakedata_fn: Callable[..., None] = clear_stale_fakedata,
  write_candidate_ref_commit_fn: Callable[..., str] = write_candidate_ref_commit,
  interface_names_fn: Callable[[], set[str]] | None = None,
) -> int:
  all_cars = {c.car_brand for c in ACTIVE_CASES}
  all_procs = set(ALL_REPLAY_PROCS)
  cpu_count = os.cpu_count() or 1

  parser = argparse.ArgumentParser(description="Regression test to identify changes in a process's output")
  parser.add_argument("--whitelist-procs", type=str, nargs="*", default=None,
                      help="Whitelist given processes from the test (e.g. controlsd)")
  parser.add_argument("--whitelist-cars", type=str, nargs="*", default=None,
                      help="Whitelist given cars from the test (e.g. HONDA)")
  parser.add_argument("--blacklist-procs", type=str, nargs="*", default=[],
                      help="Blacklist given processes from the test (e.g. controlsd)")
  parser.add_argument("--blacklist-cars", type=str, nargs="*", default=[],
                      help="Blacklist given cars from the test (e.g. HONDA)")
  parser.add_argument("--ignore-fields", type=str, nargs="*", default=[],
                      help="Extra fields or msgs to ignore (e.g. driverMonitoringState.events)")
  parser.add_argument("--ignore-msgs", type=str, nargs="*", default=[],
                      help="Msgs to ignore (e.g. carEvents)")
  parser.add_argument("--update-refs", action="store_true",
                      help="Write candidate reference logs for the selected tasks")
  parser.add_argument("--inventory", choices=("active", "staged"), default="active",
                      help="Case inventory to use (staged includes PENDING_CASES)")
  parser.add_argument("--list-cases", action="store_true", help="Print selected source case records as JSON and exit")
  parser.add_argument("--list-tasks", action="store_true", help="Print selected canonical task IDs and exit")
  parser.add_argument("--output-dir", type=str, default=PROC_REPLAY_DIR,
                      help="Directory for diff.txt and results.json")
  parser.add_argument("--reference-dir", type=str, default=None,
                      help="Local directory of already-generated candidate refs")
  parser.add_argument("-j", "--jobs", type=int, default=max(cpu_count - 2, 1),
                      help="Max amount of parallel jobs")
  args = parser.parse_args(argv)

  cases = select_cases(args.inventory)
  assert_inventory_invariants(cases)
  tasks = filter_tasks(
    build_tasks(cases),
    whitelist_procs=args.whitelist_procs if args.whitelist_procs is not None else all_procs,
    blacklist_procs=args.blacklist_procs,
    whitelist_cars=args.whitelist_cars if args.whitelist_cars is not None else all_cars,
    blacklist_cars=args.blacklist_cars,
  )

  if args.list_cases:
    print(json.dumps([c.to_record() for c in cases], indent=2, sort_keys=True))
    return 0
  if args.list_tasks:
    for task_id in sorted(t.task_id for t in tasks):
      print(task_id)
    return 0

  tested_procs = {t.process for t in tasks}
  tested_cars = {t.case.car_brand for t in tasks}
  full_test = (
    args.inventory == "active"
    and tested_procs == all_procs
    and tested_cars == all_cars
    and not args.ignore_fields
    and not args.ignore_msgs
    and args.whitelist_procs is None
    and args.whitelist_cars is None
    and not args.blacklist_procs
    and not args.blacklist_cars
  )
  if args.update_refs and args.reference_dir is not None:
    raise SystemExit("--update-refs cannot use --reference-dir")

  expected_task_ids = sorted(t.task_id for t in tasks)
  settled: dict[str, TaskResult] = {}
  cur_commit = ""
  ref_commit = ""
  artifacts_commit: str | None = None
  artifacts_manifest: dict[str, Any] | None = None

  # Diagnostics are an artifact in their own right. Create them before any
  # commit lookup, cleanup, or network operation so setup failures are visible.
  os.makedirs(args.output_dir, exist_ok=True)

  def flush() -> dict[str, Any]:
    doc = build_results_document(
      tested_commit=cur_commit,
      ref_commit=ref_commit,
      artifacts_commit=artifacts_commit,
      inventory=args.inventory,
      cases=cases,
      expected_task_ids=expected_task_ids,
      results=settled,
    )
    write_diagnostics(args.output_dir, doc, list(settled.values()), ref_commit)
    return doc

  def settle(result: TaskResult) -> dict[str, Any]:
    settled[result.task_id] = result
    return flush()

  def settle_all(message: str) -> dict[str, Any]:
    doc = None
    for task in tasks:
      if task.task_id in settled:
        continue
      doc = settle(TaskResult(
        status="error",
        task_id=task.task_id,
        elapsed_s=0.0,
        diff_or_traceback=message,
        ref_path="",
        new_path="",
      ))
    return doc if doc is not None else flush()

  document = flush()

  try:
    cur_commit = validate_ref_commit(get_commit_fn() or "")
    # A selected reference directory owns its files and metadata. Never clear
    # it as part of preparing the default candidate workspace.
    if args.reference_dir is None:
      clear_stale_fakedata_fn(FAKEDATA)
  except Exception:
    document = settle_all(traceback.format_exc())
    print("TEST FAILED")
    return exit_code_for(document)

  try:
    if args.update_refs:
      ref_commit = cur_commit
      write_candidate_ref_commit_fn(ref_commit, FAKEDATA)
    elif args.reference_dir is not None:
      ref_commit, artifacts_commit = resolve_ref_commit_fn(
        artifacts_commit,
        reference_dir=args.reference_dir,
        pin_fn=pin_artifacts_commit_fn,
      )
    else:
      artifacts_commit = pin_artifacts_commit_fn()
      ref_commit, artifacts_commit = resolve_ref_commit_fn(
        artifacts_commit,
        pin_fn=pin_artifacts_commit_fn,
      )
      artifacts_manifest = load_artifacts_manifest_fn(
        artifacts_commit,
        expected_ref_commit=ref_commit,
        selected_task_ids=expected_task_ids,
      )
  except Exception:
    document = settle_all(traceback.format_exc())
    print("TEST FAILED")
    return exit_code_for(document)

  document = flush()


  print(f"***** testing against commit {ref_commit} *****")
  if artifacts_commit:
    print(f"***** artifacts pinned at {artifacts_commit} *****")

  try:
    if full_test:
      names = interface_names_fn() if interface_names_fn is not None else set(interface_names)
      untested = (names - set(excluded_interfaces)) - {c.lower() for c in tested_cars}
      if untested:
        raise AssertionError(f"Cars missing routes: {str(untested)}")
  except Exception:
    document = settle_all(traceback.format_exc())
    print("TEST FAILED")
    return exit_code_for(document)
  # Download each selected executable case once (case_id dedup; never hash ReplayCase).
  case_by_id = unique_cases_by_id(tasks)
  download_errors: dict[str, str] = {}
  log_data: dict[str, bytes] = {}

  download_payloads = []
  for case in case_by_id.values():
    if not case.executable:
      download_errors[case.case_id] = f"{case.case_id}: pending case has no executable source"
      continue
    download_payloads.append({
      "case_id": case.case_id,
      "car_brand": case.car_brand,
      "source": case.source,
      "processes": list(case.processes),
      "custom_params": dict(case.custom_params),
      "source_sha256": case.source_sha256,
      "source_bytes": case.source_bytes,
    })

  if download_payloads:
    try:
      with concurrent.futures.ProcessPoolExecutor(max_workers=args.jobs) as pool:
        future_map = {pool.submit(download_case_by_id_fn, payload): payload for payload in download_payloads}
        for fut in tqdm(concurrent.futures.as_completed(future_map), desc="Getting Logs", total=len(future_map)):
          payload = future_map[fut]
          try:
            case_id, data, err = fut.result()
          except Exception:
            case_id, data, err = payload["case_id"], None, traceback.format_exc()
          if err is not None or data is None:
            download_errors[case_id] = err or "download failed"
          else:
            log_data[case_id] = data
    except Exception:
      setup_error = traceback.format_exc()
      for payload in download_payloads:
        download_errors.setdefault(payload["case_id"], setup_error)

  # Materialize download failures as settled task errors before workers run.
  for task in tasks:
    if task.case.case_id in download_errors:
      try:
        failed_new_path = new_log_path(task, cur_commit)
      except Exception:
        failed_new_path = ""
      document = settle(TaskResult(
        status="error",
        task_id=task.task_id,
        elapsed_s=0.0,
        diff_or_traceback=download_errors[task.case.case_id],
        ref_path="",
        new_path=failed_new_path,
      ))


  payloads: list[dict[str, Any]] = []
  for task in tasks:
    if task.task_id in settled:
      continue
    ref_path = ""
    new_path = ""
    try:
      ref_path = resolve_ref_path(
        task,
        ref_commit=ref_commit,
        artifacts_commit=artifacts_commit,
        reference_dir=args.reference_dir,
        update_refs=args.update_refs,
        artifacts_manifest=artifacts_manifest,
        pin_fn=pin_artifacts_commit_fn,
      )
      new_path = new_log_path(task, cur_commit)
      payloads.append(make_task_payload(
        task,
        lr_dat=log_data[task.case.case_id],
        ref_log_path=ref_path,
        new_log_path=new_path,
        ignore_fields=args.ignore_fields,
        ignore_msgs=args.ignore_msgs,
        update_refs=args.update_refs,
      ))
    except Exception:
      document = settle(TaskResult(
        status="error",
        task_id=task.task_id,
        elapsed_s=0.0,
        diff_or_traceback=traceback.format_exc(),
        ref_path=ref_path,
        new_path=new_path,
      ))

  def on_settled(result: TaskResult) -> None:
    settle(result)
  try:
    pooled_results = run_task_pool_fn(payloads, jobs=args.jobs, on_settled=on_settled)
    for result in pooled_results:
      settled.setdefault(result.task_id, result)
    missing_after_pool = set(expected_task_ids) - set(settled)
    for task_id in missing_after_pool:
      settled[task_id] = TaskResult(
        status="error",
        task_id=task_id,
        elapsed_s=0.0,
        diff_or_traceback="worker pool completed without a result",
        ref_path="",
        new_path="",
      )
  except Exception:
    document = settle_all(traceback.format_exc())
    print("TEST FAILED")
    return exit_code_for(document)

  document = flush()

  if not args.update_refs:
    retain_failed_logs(settled.values())
    diff_short, _, failed = format_diff_text(list(settled.values()), ref_commit)
    print(diff_short)
    if exit_code_for(document):
      print("TEST FAILED")
    else:
      print("TEST SUCCEEDED")
  else:
    # Ensure candidate tree remains reusable even if workers already wrote logs.
    write_candidate_ref_commit_fn(cur_commit, FAKEDATA)
    print(f"\n\nWrote candidate reference logs for commit: {cur_commit}")
    # Candidate-only: do not claim acceptance by rewriting REF_COMMIT_FN.

  return exit_code_for(document)


if __name__ == "__main__":
  sys.exit(main())
