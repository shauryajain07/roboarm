#!/usr/bin/env bash
# One-time Raspberry Pi setup for the ROBOARM controller.
set -euo pipefail
sudo apt-get update
sudo apt-get install -y python3-serial python3-tk python3-evdev git
# serial port access without sudo (takes effect after reboot / re-login)
sudo usermod -aG dialout,input "$USER"   # input = game controller
[ -f arm_config.json ] || cp arm_config.example.json arm_config.json
echo
echo "Done. Reboot (or log out/in) for serial permissions, then:"
echo "  python3 -m unittest discover -s tests"
echo "  python3 record.py --mock"
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null && echo "^ serial port(s) found" || echo "(no USB serial port yet - plug the arm in)"
