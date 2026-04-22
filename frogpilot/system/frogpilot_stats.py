import json
import requests

from cereal import car, custom

from openpilot.frogpilot.common.frogpilot_download_utilities import github_rate_limited
from openpilot.frogpilot.common.frogpilot_utilities import clean_model_name, get_frogpilot_api_info
from openpilot.frogpilot.common.frogpilot_variables import FROGPILOT_API

BASE_URL = "https://nominatim.openstreetmap.org"
GITHUB_API_URL = "https://api.github.com/repos/FrogAi/FrogPilot/commits"

MINIMUM_POPULATION = 100_000

TRACKED_BRANCHES = ["FrogPilot", "FrogPilot-Staging", "FrogPilot-Testing"]

def get_branch_commits():
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
          commits.append({"branch": branch, "commit": sha})
      except requests.exceptions.RequestException as exception:
        print(f"Failed to get commit for {branch}: {exception}")

  return commits

def get_city_center(latitude, longitude):
  if latitude == 0 and longitude == 0:
    return (0.0, 0.0, "N/A", "N/A", "N/A")

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
  api_token, build_metadata, device_type, dongle_id = get_frogpilot_api_info()

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

  frogpilot_stats = params.get("FrogPilotStats")

  location = json.loads(params.get("LastGPSPosition") or "{}") or {}
  original_latitude = location.get("latitude", 0.0)
  original_longitude = location.get("longitude", 0.0)
  latitude, longitude, city, state, country = get_city_center(original_latitude, original_longitude)

  payload = {
    "api_token": api_token,
    "branch_commits": get_branch_commits(),
    "build_metadata": build_metadata,
    "model_scores": [],
    "user_stats": {
      "calibrated_lateral_acceleration": params.get("CalibratedLateralAcceleration"),
      "calibration_progress": params.get("CalibrationProgress"),
      "car_params": car_params,
      "city": city,
      "country": country,
      "device": device_type,
      "frogpilot_car_params": frogpilot_car_params,
      "frogpilot_dongle_id": dongle_id,
      "frogpilot_stats": json.dumps(frogpilot_stats),
      "latitude": latitude,
      "longitude": longitude,
      "state": state,
      "toggles": json.dumps(frogpilot_toggles.__dict__),
      "using_default_model": params.get("DrivingModel").endswith("_default"),
    },
  }

  for model_name, data in sorted(params.get("ModelDrivesAndScores").items()):
    drives = data.get("Drives", 0)
    score = data.get("Score", 0)

    if drives > 0:
      payload["model_scores"].append({
        "model_name": clean_model_name(model_name),
        "drives": int(drives),
        "score": int(score),
      })

  try:
    response = requests.post(f"{FROGPILOT_API}/stats", json=payload, headers={"Content-Type": "application/json", "User-Agent": "frogpilot-api/1.0"}, timeout=30)
    response.raise_for_status()
    print("Successfully sent FrogPilot stats!")
  except requests.exceptions.RequestException as error:
    print(f"Failed to send stats: {error}")
