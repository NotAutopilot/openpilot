#!/usr/bin/env python3
"""
Tesla Pre-AP Configuration Tool
================================
Interactive CLI for configuring Tesla Model S Pre-AP OpenPilot settings.

Run with: python tesla_setup.py

All settings are stored in OpenPilot's persistent Params database and
will survive reboots.
"""

import sys
import os
import subprocess

# Add opendbc to path for imports
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OPENDBC_PATH = os.path.join(SCRIPT_DIR, "opendbc_repo")
if OPENDBC_PATH not in sys.path:
  sys.path.insert(0, OPENDBC_PATH)

try:
  from opendbc.car.tesla.tinkla_conf import tinkla_conf, PEDAL_PROFILES
  CONFIG_AVAILABLE = True
except ImportError as e:
  print(f"Error: Could not import tinkla_conf: {e}")
  print("Make sure you're running from the openpilot directory.")
  CONFIG_AVAILABLE = False


# ANSI color codes for pretty output
class Colors:
  HEADER = '\033[95m'
  BLUE = '\033[94m'
  CYAN = '\033[96m'
  GREEN = '\033[92m'
  YELLOW = '\033[93m'
  RED = '\033[91m'
  BOLD = '\033[1m'
  DIM = '\033[2m'
  RESET = '\033[0m'


def clear_screen():
  """Clear the terminal screen."""
  os.system('clear' if os.name == 'posix' else 'cls')


def print_header():
  """Print the application header."""
  print(f"{Colors.CYAN}{Colors.BOLD}")
  print("╔══════════════════════════════════════════════════════════════╗")
  print("║           TESLA PRE-AP CONFIGURATION TOOL                    ║")
  print("║                     OpenPilot Port                           ║")
  print("╚══════════════════════════════════════════════════════════════╝")
  print(f"{Colors.RESET}")


def toggle_indicator(value: bool) -> str:
  """Return a nice toggle indicator."""
  if value:
    return f"{Colors.GREEN}[ON] {Colors.RESET}"
  else:
    return f"{Colors.RED}[OFF]{Colors.RESET}"


def print_main_menu():
  """Print the main menu."""
  print_header()
  
  print(f"{Colors.BOLD}  CURRENT CONFIGURATION{Colors.RESET}")
  print(f"  {'─' * 50}")
  
  # Control Modes
  print(f"\n  {Colors.YELLOW}CONTROL MODES{Colors.RESET}")
  print(f"       Double-Pull Mode:   {Colors.GREEN}[ALWAYS ON]{Colors.RESET} {Colors.DIM}(safety feature){Colors.RESET}")
  print(f"    1. HSO (Override)      {toggle_indicator(tinkla_conf.hso_enabled)}")
  print(f"    2. HSO Delay           {Colors.CYAN}{tinkla_conf.hso_numb_period:.1f}s{Colors.RESET}")
  
  # Longitudinal Control
  print(f"\n  {Colors.YELLOW}LONGITUDINAL CONTROL{Colors.RESET}")
  print(f"    3. Use Comma Pedal     {toggle_indicator(tinkla_conf.use_pedal)}")
  pedal_status = f"{Colors.GREEN}CALIBRATED{Colors.RESET}" if tinkla_conf.pedal_calibrated else f"{Colors.RED}NOT CALIBRATED{Colors.RESET}"
  print(f"       Pedal Status:       {pedal_status}")
  print(f"    4. ACC Stalk Spam      {toggle_indicator(tinkla_conf.acc_spam_enabled)}")
  
  # Pedal Hardware
  print(f"\n  {Colors.YELLOW}PEDAL HARDWARE{Colors.RESET}")
  print(f"    5. Pedal CAN Bus       {Colors.CYAN}Bus {tinkla_conf.pedal_can_bus}{Colors.RESET}")
  print(f"    6. Pedal Profile       {Colors.CYAN}{tinkla_conf.pedal_profile}{Colors.RESET}")
  
  # Radar
  print(f"\n  {Colors.YELLOW}RADAR{Colors.RESET}")
  print(f"    7. Radar Enabled       {toggle_indicator(tinkla_conf.radar_enabled)}")
  print(f"    8. Behind Nosecone     {toggle_indicator(tinkla_conf.radar_behind_nosecone)}")
  
  # Actions
  print(f"\n  {Colors.YELLOW}ACTIONS{Colors.RESET}")
  print(f"    C. {Colors.CYAN}Run Pedal Calibration{Colors.RESET}")
  print(f"    V. {Colors.CYAN}View Full Config{Colors.RESET}")
  print(f"    R. {Colors.RED}Reset to Defaults{Colors.RESET}")
  print(f"    Q. {Colors.DIM}Quit{Colors.RESET}")
  
  print(f"\n  {'─' * 50}")


def get_user_input(prompt: str = "Enter choice: ") -> str:
  """Get user input with nice formatting."""
  try:
    return input(f"  {Colors.BOLD}>{Colors.RESET} {prompt}").strip().upper()
  except (KeyboardInterrupt, EOFError):
    return 'Q'


def confirm_action(message: str) -> bool:
  """Ask for confirmation."""
  response = input(f"  {Colors.YELLOW}⚠ {message} (y/N): {Colors.RESET}").strip().lower()
  return response == 'y'


def print_success(message: str):
  """Print a success message."""
  print(f"  {Colors.GREEN}✓ {message}{Colors.RESET}")


def print_info(message: str):
  """Print an info message."""
  print(f"  {Colors.CYAN}ℹ {message}{Colors.RESET}")


def toggle_hso():
  """Toggle HSO (Human Steering Override)."""
  new_value = not tinkla_conf.hso_enabled
  tinkla_conf.hso_enabled = new_value
  if new_value:
    print_success("HSO ENABLED")
    print_info("  Turning the wheel will pause steering without disengaging")
    print_info("  Steering resumes automatically after release + delay")
  else:
    print_success("HSO DISABLED")
    print_info("  Warning: Turning the wheel will disengage OpenPilot completely")


def set_hso_delay():
  """Set HSO resume delay."""
  print(f"\n  {Colors.CYAN}Current HSO Delay: {tinkla_conf.hso_numb_period:.1f}s{Colors.RESET}")
  print("  Enter new delay in seconds (0.5 - 5.0):")
  try:
    value = float(input("  > "))
    tinkla_conf.hso_numb_period = value
    print_success(f"HSO Delay set to {tinkla_conf.hso_numb_period:.1f}s")
  except ValueError:
    print(f"  {Colors.RED}Invalid value. Please enter a number.{Colors.RESET}")


def toggle_pedal():
  """Toggle Comma Pedal usage."""
  if not tinkla_conf.use_pedal:
    # Turning ON - check calibration
    if not tinkla_conf.pedal_calibrated:
      print(f"  {Colors.YELLOW}⚠ Pedal is not calibrated!{Colors.RESET}")
      print("  You should run calibration first.")
      if not confirm_action("Enable pedal anyway?"):
        return
    tinkla_conf.use_pedal = True
    print_success("Comma Pedal ENABLED")
    print_info("  Longitudinal control via Pedal Interceptor")
  else:
    tinkla_conf.use_pedal = False
    print_success("Comma Pedal DISABLED")
    print_info("  Longitudinal control via ACC Stalk Spam (if enabled)")


def toggle_acc_spam():
  """Toggle ACC stalk spamming."""
  new_value = not tinkla_conf.acc_spam_enabled
  tinkla_conf.acc_spam_enabled = new_value
  if new_value:
    print_success("ACC Stalk Spam ENABLED")
    print_info("  Longitudinal control via stock cruise buttons")
    print_info("  Used when Comma Pedal is disabled or unavailable")
  else:
    print_success("ACC Stalk Spam DISABLED")
    print_info("  Warning: No longitudinal control unless Pedal is enabled!")


def toggle_pedal_can_bus():
  """Toggle pedal CAN bus between 0 and 2."""
  new_value = not tinkla_conf.pedal_can_zero  # Toggle
  tinkla_conf.pedal_can_zero = new_value
  bus = tinkla_conf.pedal_can_bus
  print_success(f"Pedal CAN Bus set to {bus}")
  if bus == 0:
    print_info("  Pedal connected to main CAN bus (Party)")
  else:
    print_info("  Pedal connected to auxiliary CAN bus 2")


def select_pedal_profile():
  """Select pedal profile."""
  print(f"\n  {Colors.CYAN}Current Profile: {tinkla_conf.pedal_profile}{Colors.RESET}")
  print("\n  Available Profiles:")
  profiles = list(PEDAL_PROFILES.keys())
  for i, profile in enumerate(profiles, 1):
    current = " (current)" if profile == tinkla_conf.pedal_profile else ""
    print(f"    {i}. {profile}{current}")
  
  print("\n  Enter profile number:")
  try:
    choice = int(input("  > "))
    if 1 <= choice <= len(profiles):
      tinkla_conf.pedal_profile = profiles[choice - 1]
      print_success(f"Pedal Profile set to {tinkla_conf.pedal_profile}")
    else:
      print(f"  {Colors.RED}Invalid choice.{Colors.RESET}")
  except ValueError:
    print(f"  {Colors.RED}Invalid input.{Colors.RESET}")


def toggle_radar():
  """Toggle radar."""
  new_value = not tinkla_conf.radar_enabled
  tinkla_conf.radar_enabled = new_value
  if new_value:
    print_success("Radar ENABLED")
    print_info("  Using Bosch radar for lead car detection")
  else:
    print_success("Radar DISABLED")
    print_info("  Using vision-only lead car detection")


def toggle_nosecone():
  """Toggle radar behind nosecone setting."""
  new_value = not tinkla_conf.radar_behind_nosecone
  tinkla_conf.radar_behind_nosecone = new_value
  if new_value:
    print_success("Radar Behind Nosecone: YES")
    print_info("  Signal attenuation compensation enabled")
  else:
    print_success("Radar Behind Nosecone: NO")


def run_pedal_calibration():
  """Run the pedal calibration tool."""
  print(f"\n  {Colors.YELLOW}Starting Pedal Calibration...{Colors.RESET}")
  print("  This will run the interactive calibration tool.")
  print()
  
  if not confirm_action("Make sure the car is ON and stationary. Continue?"):
    return
  
  calibrate_script = os.path.join(SCRIPT_DIR, "calibrate_pedal.py")
  if os.path.exists(calibrate_script):
    print()
    subprocess.run([sys.executable, calibrate_script])
    print()
    input("  Press Enter to continue...")
  else:
    print(f"  {Colors.RED}Calibration script not found at:{Colors.RESET}")
    print(f"    {calibrate_script}")


def view_full_config():
  """Display full configuration."""
  clear_screen()
  print_header()
  tinkla_conf.print_config()
  print()
  input("  Press Enter to continue...")


def reset_to_defaults():
  """Reset all settings to defaults."""
  if confirm_action("Reset ALL settings to defaults?"):
    tinkla_conf.reset_to_defaults()
    print_success("All settings reset to defaults!")


def main():
  """Main entry point."""
  if not CONFIG_AVAILABLE:
    sys.exit(1)
  
  while True:
    clear_screen()
    print_main_menu()
    
    choice = get_user_input()
    
    if choice == '1':
      toggle_hso()
    elif choice == '2':
      set_hso_delay()
    elif choice == '3':
      toggle_pedal()
    elif choice == '4':
      toggle_acc_spam()
    elif choice == '5':
      toggle_pedal_can_bus()
    elif choice == '6':
      select_pedal_profile()
    elif choice == '7':
      toggle_radar()
    elif choice == '8':
      toggle_nosecone()
    elif choice == 'C':
      run_pedal_calibration()
      continue  # Skip the pause
    elif choice == 'V':
      view_full_config()
      continue  # Skip the pause
    elif choice == 'R':
      reset_to_defaults()
    elif choice == 'Q':
      clear_screen()
      print(f"  {Colors.CYAN}Goodbye! Drive safe. 🚗{Colors.RESET}")
      print()
      break
    else:
      print(f"  {Colors.RED}Invalid choice. Please try again.{Colors.RESET}")
    
    print()
    input("  Press Enter to continue...")


if __name__ == "__main__":
  main()

