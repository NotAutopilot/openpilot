import json
import os
import random
import requests

from collections import Counter
from datetime import datetime, timezone
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

from cereal import car, custom
from openpilot.system.hardware import HARDWARE
from openpilot.system.version import get_build_metadata

from openpilot.frogpilot.common.frogpilot_download_utilities import github_rate_limited
from openpilot.frogpilot.common.frogpilot_utilities import clean_model_name, is_url_pingable

BUCKET = os.environ.get("STATS_BUCKET", "")
ORG_ID = os.environ.get("STATS_ORG_ID", "")
TOKEN = os.environ.get("STATS_TOKEN", "")
STATS_URL = os.environ.get("STATS_URL", "")

BASE_URL = "https://nominatim.openstreetmap.org"
GITHUB_API_URL = "https://api.github.com/repos/FrogAi/FrogPilot/commits"

MINIMUM_POPULATION = 100_000

TRACKED_BRANCHES = ["FrogPilot", "FrogPilot-Staging", "FrogPilot-Testing"]

def get_branch_commits(now):
  commits = []

  with requests.Session() as session:
    session.headers.update({
      "Accept": "application/vnd.github.v3+json",
      "Accept-Language": "en",
      "User-Agent": "frogpilot-branch-commits-checker/1.0 (https://github.com/FrogAi/FrogPilot)"
     })

    if github_rate_limited(session):
      print("Skipping commit check due to rate limits.")
      return []

    for branch in TRACKED_BRANCHES:
      try:
        response = session.get(f"{GITHUB_API_URL}/{branch}", timeout=10)
        response.raise_for_status()

        sha = response.json().get("sha")
        if sha:
          commits.append(Point("branch_commits").field("commit", sha).tag("branch", branch).time(now))
      except requests.exceptions.RequestException as exception:
        print(f"Failed to get commit for {branch}: {exception}")

  return commits

def get_city_center(latitude, longitude):
  try:
    with requests.Session() as session:
      session.headers.update({
        "Accept-Language": "en",
        "User-Agent": "frogpilot-city-center-checker/1.0 (https://github.com/FrogAi/FrogPilot)"
      })

      location_params = {
        "addressdetails": 1, "format": "jsonv2",
        "lat": latitude, "lon": longitude, "zoom": 13
      }
      response = session.get(f"{BASE_URL}/reverse", params=location_params, timeout=10)
      response.raise_for_status()
      address = response.json().get("address", {})

      city_name = address.get("city") or address.get("town") or address.get("village") or address.get("hamlet")
      state_name = address.get("province") or address.get("region") or address.get("state") or address.get("state_district") or "N/A"
      country_name = address.get("country", "N/A")
      country_code = (address.get("country_code") or "").lower()

      if city_name:
        city_query_params = {
          "q": f"{city_name}, {state_name}, {country_name}",
          "addressdetails": 1, "extratags": 1,
          "format": "jsonv2", "limit": 1
        }
        response = session.get(f"{BASE_URL}/search", params=city_query_params, timeout=10)
        response.raise_for_status()
        city_results = response.json()

        if city_results:
          city_result = city_results[0]
          population = int(str(city_result.get("extratags", {}).get("population", "0")).replace(",", "").replace(" ", "").split(";")[0])

          if population >= MINIMUM_POPULATION:
            city_address = city_result.get("address", {})
            selected_city_name = city_address.get("city") or city_address.get("town") or city_name
            return (float(city_result["lat"]), float(city_result["lon"]), selected_city_name, state_name, country_name)

      capital_query = (f"{state_name} state capital" if country_code == "us" else f"capital of {state_name}, {country_name}")
      capital_query_params = {
        "q": capital_query,
        "addressdetails": 1, "extratags": 1,
        "format": "jsonv2", "limit": 5
      }
      response = session.get(f"{BASE_URL}/search", params=capital_query_params, timeout=10)
      response.raise_for_status()
      capital_results = response.json()

      selected_capital = None
      for capital_result in capital_results:
        if capital_result is None:
          continue

        capital_address = capital_result.get("address", {})
        capital_state = (capital_address.get("province") or capital_address.get("region") or capital_address.get("state") or capital_address.get("state_district"))
        capital_country = capital_address.get("country")

        if capital_country != country_name:
          continue
        if state_name != "N/A" and capital_state != state_name:
          continue

        is_tagged_capital = (capital_result.get("extratags") or {}).get("capital") in ("administrative", "state", "yes")
        if is_tagged_capital:
          selected_capital = capital_result
          break

        if selected_capital is None:
          selected_capital = capital_result

      if selected_capital:
        selected_capital_address = selected_capital.get("address", {})
        selected_city_name = (selected_capital_address.get("city") or selected_capital_address.get("town") or selected_capital.get("display_name", "").split(",")[0])
        return (float(selected_capital["lat"]), float(selected_capital["lon"]), selected_city_name, state_name, country_name)

  except (requests.exceptions.ConnectionError, requests.exceptions.Timeout):
    pass

  return (0.0, 0.0, "N/A", "N/A", "N/A")

def send_stats(params, frogpilot_toggles):
  if not is_url_pingable(os.environ.get("STATS_URL", "")):
    return

  build_metadata = get_build_metadata()

  car_params = "{}"
  msg_bytes = params.get("CarParamsPersistent")
  if msg_bytes:
    with car.CarParams.from_bytes(msg_bytes) as CP:
      cp_dict = CP.to_dict()
      cp_dict.pop("carFw", None)
      cp_dict.pop("carVin", None)
      car_params = json.dumps(cp_dict)

  frogpilot_car_params = "{}"
  frogpilot_msg_bytes = params.get("FrogPilotCarParamsPersistent")
  if frogpilot_msg_bytes:
    with custom.FrogPilotCarParams.from_bytes(frogpilot_msg_bytes) as FPCP:
      fpcp_dict = FPCP.to_dict()
      fpcp_dict.pop("carFw", None)
      fpcp_dict.pop("carVin", None)
      frogpilot_car_params = json.dumps(fpcp_dict)

  dongle_id = params.get("FrogPilotDongleId")
  frogpilot_stats = params.get("FrogPilotStats")

  location = json.loads(params.get("LastGPSPosition") or "{}") or {}
  original_latitude = location.get("latitude", 0.0)
  original_longitude = location.get("longitude", 0.0)
  latitude, longitude, city, state, country = get_city_center(original_latitude, original_longitude)

  now = datetime.now(timezone.utc)

  theme_attributes = sorted(["color_scheme", "distance_icons", "icon_pack", "signal_icons", "sound_pack"])
  theme_counts = Counter(getattr(frogpilot_toggles, attribute).replace("-animated", "") for attribute in theme_attributes)
  winners = [theme for theme, count in theme_counts.items() if count == max(theme_counts.values(), default=0)]
  if len(winners) > 1 and "stock" in winners:
    winners.remove("stock")
  selected_theme = random.choice(winners).replace("-user_created", "").replace("_", " ") if winners else "stock"

  user_point = (
    Point("user_stats")
    .field("calibrated_lateral_acceleration", params.get("CalibratedLateralAcceleration"))
    .field("calibration_progress", params.get("CalibrationProgress"))
    .field("car_params", car_params)
    .field("city", city)
    .field("commit", build_metadata.openpilot.git_commit)
    .field("country", country)
    .field("device", HARDWARE.get_device_type())
    .field("event", 1)
    .field("frogpilot_car_params", frogpilot_car_params)
    .field("frogpilot_stats", json.dumps(frogpilot_stats))
    .field("latitude", latitude)
    .field("longitude", longitude)
    .field("state", state)
    .field("stats", json.dumps(frogpilot_stats))  # Remove in the future
    .field("theme", selected_theme.title())
    .field("toggles", json.dumps(frogpilot_toggles.__dict__))
    .field("tuning_level", params.get("TuningLevel") + 1 if params.get_bool("TuningLevelConfirmed") else 0)
    .field("using_default_model", params.get("DrivingModel").endswith("_default"))
    .tag("branch", build_metadata.channel)
    .tag("dongle_id", dongle_id)
    .time(now)
  )

  model_points = []
  for model_name, data in sorted(params.get("ModelDrivesAndScores").items()):
    drives = data.get("Drives", 0)
    score = data.get("Score", 0)

    if drives > 0:
      point = (
        Point("model_scores")
        .field("drives", int(drives))
        .field("score", int(score))
        .tag("dongle_id", dongle_id)
        .tag("model_name", clean_model_name(model_name))
        .time(now)
      )
      model_points.append(point)

  all_points = get_branch_commits(now) + model_points + [user_point]

  client = InfluxDBClient(org=ORG_ID, timeout=60000, token=TOKEN, url=STATS_URL)
  try:
    client.write_api(write_options=SYNCHRONOUS).write(bucket=BUCKET, record=all_points)
    print("Successfully sent FrogPilot stats!")
  except Exception as error:
    print(f"Failed to send stats: {error}")
