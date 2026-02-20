#!/usr/bin/env python3
"""
Tesla Pre-AP EPAS Firmware Flasher (GUI version)
Extracts, patches, and flashes the EPAS (Electric Power Assisted Steering)
firmware to enable openpilot steering control on pre-AP Tesla vehicles.

Firmware modifications:
  - Patches EPB_epasEACAllow to always return 1
  - Patches GTW_epasControlType to always return 1
  - Patches GTW_epasLDWEnable to always return 1
  (The actual CAN messages must still be present on the bus)

Based on the Tinkla EPAS flasher by the openpilot Tesla community.

CRITICAL: This permanently modifies EPAS firmware. Ensure you have a backup.
Uses direct Panda access with UDS protocol (stops pandad via ScriptRunner).
"""

import argparse
import os
import sys
import time
import struct
import hashlib
import binascii

# ============================================
# Firmware Constants
# ============================================
FW_MD5SUM = "9e51ddd80606fbdaaf604c73c8dde0d1"
FW_START_ADDR = 0x7000
FW_END_ADDR = 0x45FFF
FW_SIZE = FW_END_ADDR - FW_START_ADDR + 1
BOOTLOADER_ADDR = 0x3ff7000

# UDS defaults
DEFAULT_CAN_ADDR = 0x730
DEFAULT_CAN_BUS = 0
DEFAULT_BOOTLOADER_FILE = "epas-bootloader-0x3ff7000-0x3ffacbd.bin"
BOOTLOADER_MD5SUM = "09cdd03705692725290942f4065c2706"
BOOTLOADER_SIZE = 15550
RISK_ACK_PARAM_KEY = "NAPEpasRiskAccepted"

# Panda connection
PANDA_CONNECT_RETRIES = 5
PANDA_CONNECT_DELAY = 2.0


def p(msg="", end="\n"):
  """Print with flush for real-time ScriptRunner display."""
  print(msg, end=end, flush=True)


def progress_bar(current, total, prefix="", width=40):
  """Simple text progress bar."""
  pct = current / total if total > 0 else 0
  filled = int(width * pct)
  bar = "#" * filled + "-" * (width - filled)
  p(f"  {prefix}[{bar}] {pct*100:.1f}%")


def _consume_ui_risk_ack() -> bool:
  """
  Consume one-time EPAS risk acknowledgment set by UI.

  Returns True when a valid acknowledgment token was present.
  """
  try:
    from openpilot.common.params import Params
    params = Params()
    accepted = params.get_bool(RISK_ACK_PARAM_KEY)
    if accepted:
      params.put_bool(RISK_ACK_PARAM_KEY, False)
    return accepted
  except Exception:
    return False


def _ensure_risk_ack(args) -> bool:
  """
  Require explicit risk acknowledgment for operations that write EPAS firmware.

  When launched from the ScriptRunner UI, risk is acknowledged by the user
  pressing Start after reading the on-screen warnings.  CLI users must pass
  --accept-risk.
  """
  if args.extract_only:
    return True

  if args.accept_risk:
    return True

  if _consume_ui_risk_ack():
    return True

  # When launched from ScriptRunner the NAPScriptRunning param is set,
  # meaning the user already navigated through the UI warnings.
  try:
    from openpilot.common.params import Params
    if Params().get_bool("NAPScriptRunning"):
      return True
  except Exception:
    pass

  p("")
  p("RISK ACKNOWLEDGMENT REQUIRED")
  p("- Flashing/restoring EPAS firmware modifies a safety-critical steering ECU.")
  p("- While unlikely, interruption/power loss or wrong image can brick EPAS.")
  p("- By proceeding, you assume this risk.")
  p("")
  p("Re-run with --accept-risk to continue from CLI,")
  p("or start from UI and press the explicit risk agreement prompt.")
  return False


# ============================================
# Security Access Algorithm (Tesla-specific)
# ============================================
def get_security_access_key(seed):
  key = 0xc541a9

  mask = struct.unpack('<I', seed.ljust(4, b'\x00'))[0] | 0x20000000
  for _i in range(32):
    msb = key & 1 ^ mask & 1
    mask = mask >> 1
    key = key >> 1
    if msb != 0:
      key = (key | msb << 0x17) ^ 0x109028

  mask = 0x55f222f9
  for _i in range(32):
    msb = key & 1 ^ mask & 1
    mask = mask >> 1
    key = key >> 1
    if msb != 0:
      key = (key | msb << 0x17) ^ 0x109028

  key = bytes([
    (key & 0xff0) >> 4,
    (key & 0xf000) >> 8 | (key & 0xf00000) >> 20,
    (key & 0xf0000) >> 16 | (key & 0xf) << 4,
  ])

  return key


# ============================================
# UDS Helper Functions
# ============================================
def wait_for_ecu(uds_client):
  """Wait for ECU to respond after reboot."""
  from opendbc.car.uds import MessageTimeoutError

  p("  Waiting for ECU .", end="")
  prev_timeout = uds_client.timeout
  uds_client.timeout = 0.1
  for _ in range(10):
    try:
      uds_client.tester_present()
      uds_client.timeout = prev_timeout
      p("")
      return
    except MessageTimeoutError:
      p(".", end="")
  uds_client.timeout = prev_timeout
  raise Exception("ECU did not respond after reboot!")


def extract_firmware(uds_client, start_addr, end_addr):
  """Extract firmware from EPAS ECU via UDS."""
  from opendbc.car.uds import SESSION_TYPE, ACCESS_TYPE

  p("Starting extended diagnostic session...")
  uds_client.diagnostic_session_control(SESSION_TYPE.EXTENDED_DIAGNOSTIC)

  p("Requesting security access seed...")
  seed = uds_client.security_access(ACCESS_TYPE.REQUEST_SEED)
  p(f"  Seed: 0x{seed.hex()}")

  p("Sending security access key...")
  key = get_security_access_key(seed)
  p(f"  Key: 0x{key.hex()}")
  uds_client.security_access(ACCESS_TYPE.SEND_KEY, key)

  p("Extracting firmware...")
  p(f"  Start addr: {hex(start_addr)}")
  p(f"  End addr:   {hex(end_addr)}")
  total = end_addr - start_addr + 1
  fw = b""
  chunk_size = 128
  chunks_done = 0
  total_chunks = (total + chunk_size - 1) // chunk_size
  report_every = max(1, total_chunks // 20)  # report ~20 times

  for addr in range(start_addr, end_addr + 1, chunk_size):
    dat = uds_client.read_memory_by_address(addr, chunk_size)
    if len(dat) != chunk_size:
      raise Exception(f"Expected {chunk_size} bytes but got {len(dat)} at addr {hex(addr)}")
    fw += dat
    chunks_done += 1
    if chunks_done % report_every == 0 or chunks_done == total_chunks:
      progress_bar(chunks_done, total_chunks, "Extract: ")

  p(f"  Extracted {len(fw)} bytes")
  return fw


def update_checksums(fw, offset, restore=False):
  """Recalculate firmware checksums after patching."""
  for addr in [0x79c0, 0x79d0]:
    idx = addr - offset
    if idx < 0:
      raise Exception(f"Checksum address {hex(addr)} is before firmware start")

    start = struct.unpack('<I', fw[idx:idx + 4])[0]
    end = struct.unpack('<I', fw[idx + 4:idx + 8])[0]

    if start < offset or end < start:
      raise Exception(f"Invalid checksum range {hex(start)}-{hex(end)}")
    if end - offset >= len(fw):
      raise Exception(f"Checksum end addr {hex(end)} is outside firmware range")

    crc32 = binascii.crc32(fw[start - offset:end - offset + 1])
    cksum = struct.pack("<I", crc32)

    if not restore:
      changed = "CHANGED" if cksum != fw[idx + 8:idx + 12] else "unchanged"
      p(f"  {hex(start)}-{hex(end)}: CRC32={hex(crc32)} ({changed})")

    fw = fw[:idx + 8] + cksum + fw[idx + 12:]

  return fw


def patch_firmware(fw, offset, restore=False):
  """Apply or revert EPAS firmware patches."""
  mods = [
    # load 1 instead of extracting EPB_epasEACAllow
    [0x031750, b"\x80\xff\x74\x2b", b"\x20\x56\x01\x00"],
    # load 1 instead of extracting GTW_epasControlType
    [0x031892, b"\x80\xff\x32\x2a", b"\x20\x56\x01\x00"],
    # load 1 instead of extracting GTW_epasLDWEnable
    [0x031974, b"\x80\xff\x50\x29", b"\x20\x56\x01\x00"],
  ]

  for addr, old_val, new_val in mods:
    idx = addr - offset
    if idx < 0:
      raise Exception(f"Patch address {hex(addr)} is before firmware start")

    if restore:
      if fw[idx:idx + len(old_val)] != new_val:
        continue
      tmp = old_val
      old_val = new_val
      new_val = tmp
    else:
      p(f"  {hex(addr)}: 0x{fw[idx:idx + len(old_val)].hex()} -> 0x{new_val.hex()}")
      if len(old_val) != len(new_val):
        raise Exception(f"Patch size mismatch at {hex(addr)}")
      if fw[idx:idx + len(old_val)] != old_val:
        raise Exception(f"Unexpected firmware content at {hex(addr)}: "
                        f"0x{fw[idx:idx + len(old_val)].hex()} != 0x{old_val.hex()}")

    fw = fw[:idx] + new_val + fw[idx + len(new_val):]

  return fw


def flash_bootloader(uds_client, bootloader_path, start_addr):
  """Flash bootloader to EPAS ECU."""
  from opendbc.car.uds import SESSION_TYPE, ACCESS_TYPE, ROUTINE_CONTROL_TYPE

  p("Reading bootloader...")
  with open(bootloader_path, "rb") as f:
    fw = f.read()
  fw_len = len(fw)
  end_addr = start_addr + fw_len - 1
  p(f"  Size: {fw_len} bytes")

  bl_md5 = hashlib.md5(fw).hexdigest()
  p(f"  MD5:  {bl_md5}")
  if fw_len != BOOTLOADER_SIZE or bl_md5 != BOOTLOADER_MD5SUM:
    raise Exception(f"Bootloader integrity check failed! "
                    f"Expected md5={BOOTLOADER_MD5SUM} size={BOOTLOADER_SIZE}, "
                    f"got md5={bl_md5} size={fw_len}")

  p("Starting programming session...")
  uds_client.diagnostic_session_control(SESSION_TYPE.PROGRAMMING)
  wait_for_ecu(uds_client)

  p("Requesting security access seed...")
  seed = uds_client.security_access(ACCESS_TYPE.REQUEST_SEED)
  p(f"  Seed: 0x{seed.hex()}")

  p("Sending security access key...")
  key = get_security_access_key(seed)
  p(f"  Key: 0x{key.hex()}")
  uds_client.security_access(ACCESS_TYPE.SEND_KEY, key)

  p("Requesting download...")
  p(f"  Start addr: {hex(start_addr)}")
  p(f"  End addr:   {hex(end_addr)}")
  p(f"  Size:       {hex(fw_len)}")
  block_size = uds_client.request_download(start_addr, fw_len)

  p("Transferring bootloader...")
  p(f"  Block size: {block_size}")
  chunk_size = block_size - 2
  if chunk_size <= 0:
    raise Exception(f"Invalid block size {block_size} returned by ECU")
  cnt = 0
  total_chunks = (fw_len + chunk_size - 1) // chunk_size
  report_every = max(1, total_chunks // 10)

  for i in range(0, fw_len, chunk_size):
    cnt += 1
    uds_client.transfer_data(cnt & 0xFF, fw[i:i + chunk_size])
    if cnt % report_every == 0 or i + chunk_size >= fw_len:
      progress_bar(cnt, total_chunks, "Bootloader: ")

  p("Requesting transfer exit...")
  uds_client.request_transfer_exit()

  p("Entering bootloader...")
  uds_client.routine_control(ROUTINE_CONTROL_TYPE.START, 0x0301,
                             struct.pack(">I", start_addr))
  p("  Bootloader flashed successfully")


def flash_firmware(uds_client, fw_slice, start_addr, end_addr):
  """Flash modified firmware to EPAS ECU."""
  from opendbc.car.uds import (SESSION_TYPE, ACCESS_TYPE, ROUTINE_CONTROL_TYPE,
                                ROUTINE_IDENTIFIER_TYPE, RESET_TYPE, MessageTimeoutError)

  slice_len = end_addr - start_addr + 1
  if slice_len != len(fw_slice):
    raise Exception(f"Firmware size mismatch: expected {slice_len}, got {len(fw_slice)}")

  start_and_length = struct.pack('>II', start_addr, slice_len)

  p("Erasing memory...")
  p(f"  Start addr: {hex(start_addr)}")
  p(f"  End addr:   {hex(end_addr)}")
  p(f"  Size:       {hex(slice_len)}")
  uds_client.routine_control(ROUTINE_CONTROL_TYPE.START,
                             ROUTINE_IDENTIFIER_TYPE.ERASE_MEMORY,
                             start_and_length)

  p("Requesting download...")
  block_size = uds_client.request_download(start_addr, slice_len)

  p("Transferring firmware...")
  p(f"  Block size: {block_size}")
  chunk_size = block_size - 2
  if chunk_size <= 0:
    raise Exception(f"Invalid block size {block_size} returned by ECU")
  cnt = 0
  total_chunks = (slice_len + chunk_size - 1) // chunk_size
  report_every = max(1, total_chunks // 20)

  for i in range(0, slice_len, chunk_size):
    cnt += 1
    uds_client.transfer_data(cnt & 0xFF, fw_slice[i:i + chunk_size])
    if cnt % report_every == 0 or i + chunk_size >= slice_len:
      progress_bar(cnt, total_chunks, "Firmware: ")

  p("Requesting transfer exit...")
  uds_client.request_transfer_exit()

  p("Resetting ECU...")
  wait_for_ecu(uds_client)
  try:
    uds_client.ecu_reset(RESET_TYPE.HARD | 0x80)
  except MessageTimeoutError:
    # suppress-response bit set, timeout expected (waits for reboot)
    pass
  wait_for_ecu(uds_client)

  p("Starting extended diagnostic session...")
  uds_client.diagnostic_session_control(SESSION_TYPE.EXTENDED_DIAGNOSTIC)

  p("Requesting security access seed...")
  seed = uds_client.security_access(ACCESS_TYPE.REQUEST_SEED)
  p(f"  Seed: 0x{seed.hex()}")

  p("Sending security access key...")
  key = get_security_access_key(seed)
  p(f"  Key: 0x{key.hex()}")
  uds_client.security_access(ACCESS_TYPE.SEND_KEY, key)

  p("Checking dependencies...")
  uds_client.routine_control(ROUTINE_CONTROL_TYPE.START, 0xDC03)

  p("  Firmware flashed successfully!")


def connect_panda():
  """Connect to Panda with retries."""
  from panda import Panda

  for attempt in range(PANDA_CONNECT_RETRIES):
    try:
      panda = Panda()
      return panda
    except Exception as e:
      if attempt < PANDA_CONNECT_RETRIES - 1:
        p(f"  Panda not ready ({e}), retrying in {PANDA_CONNECT_DELAY}s...")
        time.sleep(PANDA_CONNECT_DELAY)
      else:
        raise


def parse_args(cli_args=None):
  parser = argparse.ArgumentParser(
    description="Patch/restore Tesla Pre-AP EPAS firmware via Panda UDS",
  )
  parser.add_argument("--extract-only", action="store_true",
                      help="Extract firmware only; do not flash")
  parser.add_argument("--restore", action="store_true",
                      help="Flash stock firmware image (no patch modifications)")
  parser.add_argument("--debug", action="store_true",
                      help="Enable UDS debug logging")
  parser.add_argument("--accept-risk", action="store_true",
                      help="Acknowledge EPAS flashing risk and allow write operations")
  parser.add_argument("--bootloader", type=str, default=DEFAULT_BOOTLOADER_FILE,
                      help="Bootloader filename/path (relative paths are resolved under scripts/nap/firmware)")
  parser.add_argument("--can-addr", type=lambda x: int(x, 0), default=DEFAULT_CAN_ADDR,
                      help="UDS transmit CAN address (default: 0x730)")
  parser.add_argument("--can-bus", type=int, default=DEFAULT_CAN_BUS,
                      help="CAN bus number (default: 0)")
  return parser.parse_args(cli_args)


def main(cli_args=None):
  from panda import Panda
  from cereal import car
  from opendbc.car.uds import UdsClient
  SafetyModel = car.CarParams.SafetyModel
  args = parse_args(cli_args)

  p("=" * 60)
  p("EPAS FIRMWARE FLASHER")
  p("=" * 60)
  p("")
  p("This tool will:")
  p("  1. Extract the current EPAS firmware")
  p("  2. Verify the firmware checksum")
  if args.extract_only:
    p("  3. Save backup image and exit")
  elif args.restore:
    p("  3. Flash stock firmware image")
  else:
    p("  3. Apply steering control patches")
    p("  4. Flash the patched firmware")
  p("")
  p("WARNING: This permanently modifies the EPAS firmware.")
  p("A backup of the original firmware will be saved.")
  p("=" * 60)

  if not _ensure_risk_ack(args):
    return 1

  # Resolve paths relative to script location
  script_dir = os.path.dirname(os.path.realpath(__file__))
  firmware_dir = os.path.join(script_dir, "firmware")
  bootloader_path = args.bootloader
  if not os.path.isabs(bootloader_path):
    bootloader_path = os.path.join(firmware_dir, bootloader_path)
  fw_fn = os.path.join(firmware_dir, f"epas-firmware-{hex(FW_START_ADDR)}-{hex(FW_END_ADDR)}.bin")

  if not args.extract_only and not os.path.exists(bootloader_path):
    p(f"\nERROR: Bootloader not found at: {bootloader_path}")
    return 1

  panda = None
  try:
    # Allow time for manager to see NAPScriptRunning and stop pandad
    p("\nWaiting for pandad to release Panda USB...")
    time.sleep(2)

    p("Connecting to Panda...")
    panda = connect_panda()
    p("  Connected")

    panda.set_safety_mode(SafetyModel.elm327)
    p("  Safety mode: ELM327 (UDS)")

    # Flush stale CAN messages left by pandad / other processes
    p("  Flushing CAN buffers...")
    panda.can_clear(0xFFFF)
    time.sleep(0.1)
    panda.can_recv()

    uds_client = UdsClient(panda, args.can_addr, bus=args.can_bus, timeout=1)
    p(f"  UDS client: addr=0x{args.can_addr:03X}, bus={args.can_bus}")

    # Step 1: Extract firmware
    p("\n" + "=" * 40)
    p("STEP 1: Extract Firmware")
    p("=" * 40)

    fw_slice = None
    if args.extract_only or not os.path.exists(fw_fn):
      fw_slice = extract_firmware(uds_client, FW_START_ADDR, FW_END_ADDR)
      # Undo any previous patches to get clean firmware
      fw_slice = patch_firmware(fw_slice, FW_START_ADDR, restore=True)
      fw_slice = update_checksums(fw_slice, FW_START_ADDR, restore=True)
      p(f"  Saving backup: {fw_fn}")
      os.makedirs(os.path.dirname(fw_fn), exist_ok=True)
      with open(fw_fn, "wb") as f:
        f.write(fw_slice)
    else:
      p(f"  Loading cached firmware: {fw_fn}")
      with open(fw_fn, "rb") as f:
        fw_slice = f.read()
      md5 = hashlib.md5(fw_slice).hexdigest()
      if md5 != FW_MD5SUM or len(fw_slice) != FW_SIZE:
        p(f"  Cached firmware invalid (md5={md5}), re-extracting...")
        fw_slice = extract_firmware(uds_client, FW_START_ADDR, FW_END_ADDR)
        # Undo any previous patches to get clean firmware
        fw_slice = patch_firmware(fw_slice, FW_START_ADDR, restore=True)
        fw_slice = update_checksums(fw_slice, FW_START_ADDR, restore=True)
        p(f"  Saving backup: {fw_fn}")
        os.makedirs(os.path.dirname(fw_fn), exist_ok=True)
        with open(fw_fn, "wb") as f:
          f.write(fw_slice)

    # Step 2: Verify firmware
    p("\n" + "=" * 40)
    p("STEP 2: Verify Firmware")
    p("=" * 40)

    md5 = hashlib.md5(fw_slice).hexdigest()
    p(f"  Size: {len(fw_slice)} bytes (expected {FW_SIZE})")
    p(f"  MD5:  {md5}")

    if len(fw_slice) != FW_SIZE:
      p(f"\nERROR: Firmware size mismatch!")
      return 1

    if md5 != FW_MD5SUM:
      p(f"\nERROR: Firmware checksum mismatch!")
      p(f"  Expected: {FW_MD5SUM}")
      p(f"  Got:      {md5}")
      return 1

    p("  Firmware verified OK")

    if args.extract_only:
      p("\n" + "=" * 60)
      p("EXTRACT COMPLETE")
      p("=" * 60)
      p(f"Backup saved to: {fw_fn}")
      return 0

    # Step 3: Patch or Restore firmware payload
    p("\n" + "=" * 40)
    p("STEP 3: Prepare Firmware Payload")
    p("=" * 40)

    if args.restore:
      p("Restore mode enabled: flashing stock firmware image.")
    else:
      p("Applying patches...")
      fw_slice = patch_firmware(fw_slice, FW_START_ADDR)
      p("Updating checksums...")
      fw_slice = update_checksums(fw_slice, FW_START_ADDR)
      mod_fn = os.path.join(firmware_dir, f"{FW_MD5SUM}.modified.bin")
      p(f"  Saving modified firmware: {mod_fn}")
      with open(mod_fn, "wb") as f:
        f.write(fw_slice)

    # Step 4: Flash
    p("\n" + "=" * 40)
    p("STEP 4: Flash Bootloader")
    p("=" * 40)

    flash_bootloader(uds_client, bootloader_path, BOOTLOADER_ADDR)

    p("\n" + "=" * 40)
    p("STEP 5: Flash Firmware")
    p("=" * 40)

    flash_firmware(uds_client, fw_slice, FW_START_ADDR, FW_END_ADDR)

    p("\n" + "=" * 60)
    p("EPAS FLASH COMPLETE!")
    p("=" * 60)
    p("")
    if args.restore:
      p("The stock EPAS firmware image has been restored.")
    else:
      p("The EPAS firmware has been patched for openpilot steering control.")
    p("Please restart the vehicle to apply changes.")

    return 0

  except KeyboardInterrupt:
    p("\nFlashing interrupted!")
    p("WARNING: If interrupted during flash, the EPAS may need recovery.")
    return 1
  except Exception as e:
    p(f"\nERROR: {e}")
    import traceback
    traceback.print_exc()
    return 1
  finally:
    if panda is not None:
      try:
        panda.set_safety_mode(SafetyModel.silent)
        panda.close()
        p("\nPanda connection closed.")
      except Exception:
        pass


if __name__ == "__main__":
  sys.exit(main())
