from cereal import messaging

from openpilot.selfdrive.test.process_replay.compare_logs import _diff_capnp_values


PATH = ("modelV2", "position", "x")


def _position_x_reader(values):
  message = messaging.new_message("modelV2")
  message.modelV2.position.x = values
  return message.as_reader().modelV2.position.x


def test_equal_position_x_lists_have_no_diff():
  first = _position_x_reader([1, 2])
  second = _position_x_reader([1, 2])

  assert list(_diff_capnp_values(first, second, PATH, 0)) == []


def test_changed_position_x_element_reports_indexed_change():
  first = _position_x_reader([1, 2])
  second = _position_x_reader([1, 9])

  assert list(_diff_capnp_values(first, second, PATH, 0)) == [
    ("change", "modelV2.position.x.1", (2, 9)),
  ]


def test_grown_position_x_list_reports_indexed_additions():
  first = _position_x_reader([1, 2])
  second = _position_x_reader([1, 2, 3, 4])

  assert list(_diff_capnp_values(first, second, PATH, 0)) == [
    ("add", "modelV2.position.x", [(2, 3), (3, 4)]),
  ]


def test_shrunken_position_x_list_reports_reverse_indexed_removals():
  first = _position_x_reader([1, 2, 3, 4])
  second = _position_x_reader([1, 2])

  assert list(_diff_capnp_values(first, second, PATH, 0)) == [
    ("remove", "modelV2.position.x", [(3, 4), (2, 3)]),
  ]
