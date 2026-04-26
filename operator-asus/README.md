# operator-asus/ — ASUS Tkinter GUI 操控面板

This directory will hold the Tkinter-based teleop GUI (`asus_teleop_gui.py` on ASUS host `edward@192.168.0.182` / Tailscale `100.103.0.5`). Sends chassis `cmd_vel` + `lift_cmd` to Orin via LiveKit data channel. To be imported in PR-4.

Keyboard layout: WASD (translate) + QE (rotate) + G/H (lift up/down).
