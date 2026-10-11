"""Download US Maps: retry + HTTP Range resume, and hash-mismatch cleanup.

A local HTTP server stands in for GitHub Releases (which answers Range with
206 after the 302 to release-assets.githubusercontent.com).
"""
import hashlib
import http.server
import os
import threading
import time

import pytest

import openpilot.selfdrive.mapd.fetch_maps as fm
from openpilot.selfdrive.mapd.fetch_maps import HashMismatchError, download_url, fetch_and_install, staging_dir
from openpilot.selfdrive.mapd.tests.test_fetch_maps import _make_zst


class _Server:
  """Serves `data`. The first `stall` GETs send `cut` bytes, then hang past the client timeout."""

  def __init__(self, data: bytes, *, stall: int = 0, cut: int = 0, stall_s: float = 1.0):
    self.data, self.stall, self.cut, self.stall_s = data, stall, cut, stall_s
    self.ranges: list[str | None] = []
    outer = self

    class H(http.server.BaseHTTPRequestHandler):
      def log_message(self, *a):
        pass

      def do_GET(self):
        rng = self.headers.get("Range")
        outer.ranges.append(rng)
        start = int(rng.split("=")[1].split("-")[0]) if rng else 0
        body = outer.data[start:]
        if rng:
          if start >= len(outer.data):
            self.send_response(416)
            self.send_header("Content-Range", f"bytes */{len(outer.data)}")
            self.end_headers()
            return
          self.send_response(206)
          self.send_header("Content-Range", f"bytes {start}-{len(outer.data) - 1}/{len(outer.data)}")
        else:
          self.send_response(200)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if outer.stall > 0:
          outer.stall -= 1
          self.wfile.write(body[:outer.cut])
          self.wfile.flush()
          time.sleep(outer.stall_s)
          return
        self.wfile.write(body)

    self.httpd = http.server.ThreadingHTTPServer(("127.0.0.1", 0), H)
    self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}/speed_limits_us.sqlite.zst"
    threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

  def close(self):
    self.httpd.shutdown()
    self.httpd.server_close()


@pytest.fixture
def small_budget(monkeypatch):
  monkeypatch.setattr(fm, "required_free_bytes", lambda *a, **k: 0)
  monkeypatch.setattr(fm, "ASSET_SQLITE_BYTES", 0)
  monkeypatch.setattr(fm, "FREE_MARGIN_BYTES", 0)
  monkeypatch.setattr(fm, "CHUNK", 1024)


def test_timeout_mid_stream_retries_and_resumes_with_range(tmp_path, small_budget, capsys):
  data = os.urandom(300_000)
  srv = _Server(data, stall=1, cut=100_000)
  try:
    sleeps: list[float] = []
    dest = str(tmp_path / "a.zst")
    n = download_url(srv.url, dest, timeout=0.3, sleep=sleeps.append)
  finally:
    srv.close()
  assert n == len(data)
  assert open(dest, "rb").read() == data
  assert srv.ranges[0] is None and srv.ranges[1] == "bytes=100000-"
  assert len(sleeps) == 1 and sleeps[0] > 0
  out = capsys.readouterr().out
  assert "Retrying… (1)" in out and "ERROR" not in out
  assert not os.path.exists(dest + ".partial")


def test_error_only_after_all_retries_and_partial_kept(tmp_path, small_budget, capsys):
  data = os.urandom(200_000)
  srv = _Server(data, stall=99, cut=0)
  srv.cut = 50_000
  try:
    dest = str(tmp_path / "a.zst")
    with pytest.raises(RuntimeError, match="after 3 attempts"):
      download_url(srv.url, dest, attempts=3, timeout=0.2, sleep=lambda s: None)
  finally:
    srv.close()
  out = capsys.readouterr().out
  assert "Retrying… (1)" in out and "Retrying… (2)" in out and "Retrying… (3)" not in out
  # Each attempt resumed where the last left off; nothing thrown away.
  assert os.path.getsize(dest + ".partial") == 150_000


def test_second_tap_resumes_from_partial_and_installs(tmp_path, small_budget):
  _src, zst = _make_zst(tmp_path)
  data = open(zst, "rb").read()
  sha = hashlib.sha256(data).hexdigest()
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  cut = len(data) // 2
  srv = _Server(data, stall=1, cut=cut, stall_s=1.0)
  try:
    first = fm.download_url

    def one_attempt(url, d, progress=True, **kw):
      return first(url, d, progress, attempts=1, timeout=0.3)
    fm.download_url = one_attempt
    try:
      with pytest.raises(RuntimeError, match="after 1 attempts"):
        fetch_and_install(dest=dest, url=srv.url, sha256=sha, sqlite_sha256="")
      partial = os.path.join(staging_dir(dest), "speed_limits_us.sqlite.zst.partial")
      assert os.path.getsize(partial) == cut  # kept for the next tap
      fetch_and_install(dest=dest, url=srv.url, sha256=sha, sqlite_sha256="")
    finally:
      fm.download_url = first
  finally:
    srv.close()
  assert srv.ranges[-1] == f"bytes={cut}-"
  assert fm.sqlite_ok(dest)
  assert not os.path.isdir(staging_dir(dest)) or not os.listdir(staging_dir(dest))


def test_hash_mismatch_deletes_staged_download(tmp_path, small_budget):
  _src, zst = _make_zst(tmp_path)
  data = open(zst, "rb").read()
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  srv = _Server(data)
  try:
    with pytest.raises(HashMismatchError):
      fetch_and_install(dest=dest, url=srv.url, sha256="0" * 64, sqlite_sha256="")
  finally:
    srv.close()
  assert not os.path.exists(dest)
  stage = staging_dir(dest)
  assert not os.path.isdir(stage) or not os.listdir(stage)


def test_partial_for_a_different_asset_is_not_resumed(tmp_path, small_budget):
  _src, zst = _make_zst(tmp_path)
  data = open(zst, "rb").read()
  sha = hashlib.sha256(data).hexdigest()
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  stage = staging_dir(dest)
  os.makedirs(stage)
  with open(os.path.join(stage, "speed_limits_us.sqlite.zst.partial"), "wb") as f:
    f.write(b"stale bytes from an older pack")
  with open(os.path.join(stage, "speed_limits_us.sqlite.zst.expect"), "w") as f:
    f.write("old-url\nold-sha\n")
  srv = _Server(data)
  try:
    fetch_and_install(dest=dest, url=srv.url, sha256=sha, sqlite_sha256="")
  finally:
    srv.close()
  assert srv.ranges == [None]
  assert fm.sqlite_ok(dest)
