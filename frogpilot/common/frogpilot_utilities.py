#!/usr/bin/env python3
import json
import math
import os
import requests
import subprocess
import threading
import zipfile

from pathlib import Path

import openpilot.system.sentry as sentry

from cereal import messaging

from openpilot.frogpilot.common.frogpilot_variables import EARTH_RADIUS, FROGS_GO_MOO_PATH

class ThreadManager:
  def __init__(self):
    self.thread_lock = threading.Lock()

    self.running_threads = {}

  def run_with_lock(self, target, args=(), report=True):
    name = target.__name__

    if not isinstance(args, (tuple, list)):
      args = (args,)

    with self.thread_lock:
      dead_threads = [key for key, thread in self.running_threads.items() if not thread.is_alive()]
      for key in dead_threads:
        del self.running_threads[key]

      if name in self.running_threads and self.running_threads[name].is_alive():
        return

      def wrapped_target(*t_args):
        try:
          target(*t_args)
        except Exception as exception:
          print(f"Error in thread '{name}': {exception}")
          if report:
            sentry.capture_exception(exception)

      thread = threading.Thread(args=args, daemon=True, target=wrapped_target)
      thread.start()
      self.running_threads[name] = thread

  def is_thread_alive(self, name):
    with self.thread_lock:
      thread = self.running_threads.get(name)
      return thread is not None and thread.is_alive()


def calculate_bearing_offset(latitude, longitude, current_bearing, distance):
  bearing = math.radians(current_bearing)
  lat_rad = math.radians(latitude)
  lon_rad = math.radians(longitude)

  delta = distance / EARTH_RADIUS

  new_lat = math.asin(math.sin(lat_rad) * math.cos(delta) + math.cos(lat_rad) * math.sin(delta) * math.cos(bearing))
  new_lon = lon_rad + math.atan2(math.sin(bearing) * math.sin(delta) * math.cos(lat_rad),  math.cos(delta) - math.sin(lat_rad) * math.sin(new_lat))
  return math.degrees(new_lat), math.degrees(new_lon)


def calculate_distance_to_point(lat1, lon1, lat2, lon2):
  lat1_rad = math.radians(lat1)
  lon1_rad = math.radians(lon1)
  lat2_rad = math.radians(lat2)
  lon2_rad = math.radians(lon2)

  delta_lat = lat2_rad - lat1_rad
  delta_lon = lon2_rad - lon1_rad

  a = (math.sin(delta_lat / 2) ** 2) + math.cos(lat1_rad) * math.cos(lat2_rad) * (math.sin(delta_lon / 2) ** 2)
  c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

  return EARTH_RADIUS * c


def contains_event_type(events, frogpilot_events, *event_types):
  return any(events.contains(event_type) or frogpilot_events.contains(event_type) for event_type in event_types)


def delete_file(path, print_error=True, report=True):
  path = Path(path)
  if path.is_file() or path.is_symlink():
    run_cmd(["sudo", "rm", "-f", str(path)], f"Deleted file: {path}", f"Failed to delete file: {path}", report=report)
  elif path.is_dir():
    run_cmd(["sudo", "rm", "-rf", str(path)], f"Deleted directory: {path}", f"Failed to delete directory: {path}", report=report)
  elif print_error:
    print(f"File not found: {path}")


def extract_zip(zip_file, extract_path):
  with zipfile.ZipFile(zip_file, "r") as zip:
    print(f"Extracting {zip_file} to {extract_path}")
    zip.extractall(extract_path)

  zip_file.unlink()
  print(f"Extraction completed!")


@cache
def is_FrogsGoMoo():
  return FROGS_GO_MOO_PATH.is_file()


def is_url_pingable(url):
  if not url:
    return False

  if not hasattr(is_url_pingable, "session"):
    is_url_pingable.session = requests.Session()
    is_url_pingable.session.headers.update({"User-Agent": "frogpilot-ping-test/1.0 (https://github.com/FrogAi/FrogPilot)"})

  try:
    response = is_url_pingable.session.head(url, timeout=10, allow_redirects=True)
    if response.status_code in (405, 501):
      response = is_url_pingable.session.get(url, timeout=10, allow_redirects=True, stream=True)

    is_accessible = response.ok
    response.close()
    return is_accessible

  except (requests.exceptions.ConnectionError, requests.exceptions.SSLError):
    return False
  except requests.exceptions.RequestException as error:
    print(f"{error.__class__.__name__} while pinging {url}: {error}")
    return False
  except Exception as exception:
    print(f"Unexpected error while pinging {url}: {exception}")
    return False


def load_json_file(path):
  if path.is_file():
    try:
      with open(path) as file:
        return json.load(file)
    except json.JSONDecodeError:
      print(f"Failed to load JSON file: {path}")
      return {}
  return {}


def run_cmd(cmd, success_message, fail_message, env=None, report=True):
  try:
    result = subprocess.run(cmd, capture_output=True, check=True, env=env, text=True)
    print(success_message)
    return result.stdout.strip()
  except subprocess.CalledProcessError as exception:
    print(f"Command failed with error: {exception.stderr}")
    print(fail_message)
    if report:
      sentry.capture_exception(exception.stderr)
    return None
  except Exception as exception:
    print(f"Unexpected error occurred: {exception}")
    print(fail_message)
    if report:
      sentry.capture_exception(exception)
    return None


def update_can_parser(can_parser, can_sock):
  can_msgs = messaging.drain_sock(can_sock, wait_for_one=True)
  can_parser.update([(msg.logMonoTime, [[frame.address, frame.dat, frame.src] for frame in msg.can]) for msg in can_msgs if msg.which() == "can"])


def update_json_file(path, data):
  temp_path = f"{path}.tmp"
  with open(temp_path, "w") as file:
    json.dump(data, file, indent=2, sort_keys=True)
    file.flush()
    os.fsync(file.fileno())

  os.replace(temp_path, path)
