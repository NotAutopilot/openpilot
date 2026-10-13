from types import SimpleNamespace

from openpilot.selfdrive.mapd.speed_overrides import MPH_TO_MS, PENDING_MAX_S, SpeedOverrides, parse_confirmed


def m(way):
  return SimpleNamespace(way_id=way, road_name="CR 31")


def test_parse():
  assert parse_confirmed(None) is None
  assert parse_confirmed({"mph": 33, "seq": 1}) is None
  assert parse_confirmed('{"mph": 55, "seq": 2, "lat": 45.1}') == {"mph": 55, "seq": 2.0, "lat": 45.1}


def test_yes_keyed_by_way_and_direction_and_persists(tmp_path):
  p = str(tmp_path / "o.json")
  o = SpeedOverrides(p)
  e = o.consume({"mph": 55, "seq": 1.0}, m(111), 90.0, 0.0)
  assert e["way_id"] == 111 and e["mph"] == 55
  assert abs(o.limit_ms(111, 100.0) - 55 * MPH_TO_MS) < 1e-6
  assert o.limit_ms(111, 270.0) is None      # opposite direction keeps OSM
  assert o.limit_ms(222, 90.0) is None
  assert o.consume({"mph": 55, "seq": 1.0}, m(222), 90.0, 1.0) is None  # same seq once
  o2 = SpeedOverrides(p)                        # future drive
  assert abs(o2.limit_ms(111, 80.0) - 55 * MPH_TO_MS) < 1e-6
  o2.consume({"mph": 45, "seq": 2.0}, m(111), 95.0, 2.0)   # newer YES replaces
  assert len(o2.entries) == 1 and o2.find(111, 90.0)["mph"] == 45


def test_pending_until_match_then_expires(tmp_path):
  o = SpeedOverrides(str(tmp_path / "o.json"))
  assert o.consume({"mph": 30, "seq": 1.0}, None, 10.0, 0.0) is None
  assert o.consume(None, m(5), 10.0, 3.0)["way_id"] == 5
  assert o.consume({"mph": 30, "seq": 2.0}, None, 10.0, 10.0) is None
  assert o.consume(None, m(6), 10.0, 10.0 + PENDING_MAX_S + 1) is None
  assert o.find(6, 10.0) is None
