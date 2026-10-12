import sqlite3

from openpilot.selfdrive.mapd import rail_crossings as rc


def _db():
  con = sqlite3.connect(":memory:")
  rc.ensure_table(con)
  # northbound road at lon -97.1; crossings ahead, behind, and off to the side
  rc.insert(con, 1, 44.9020, -97.1, 10, "level_crossing", barrier="full", light="yes")
  rc.insert(con, 2, 44.9040, -97.1, 10, "level_crossing")
  rc.insert(con, 3, 44.8980, -97.1, 10, "level_crossing")   # behind
  rc.insert(con, 4, 44.9020, -97.095, 11, "crossing")       # ~400 m east
  return con


def test_ahead_nearest_first_with_tags():
  got = rc.crossings_ahead(_db(), 44.9, -97.1, 0.0, horizon_m=500)
  assert [c.node_id for c in got] == [1, 2]
  assert 210 < got[0].distance_m < 235 and abs(got[0].lateral_m) < 1
  assert got[0].barrier == "full" and got[0].light == "yes" and got[1].barrier == ""


def test_heading_and_horizon():
  con = _db()
  assert [c.node_id for c in rc.crossings_ahead(con, 44.9, -97.1, 180.0)] == [3]
  assert [c.node_id for c in rc.crossings_ahead(con, 44.9, -97.1, 0.0, horizon_m=300)] == [1]
  assert [c.node_id for c in rc.crossings_ahead(con, 44.9, -97.1, 0.0, lateral_m=500)] == [1, 4, 2]  # by distance along heading


def test_old_pack_without_table():
  assert rc.crossings_ahead(sqlite3.connect(":memory:"), 44.9, -97.1, 0.0) == []
