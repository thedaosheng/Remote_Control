#!/bin/bash
export MPLBACKEND=Agg
cd touch/mujoco_sim
python3 20260409-cc-mock_touch_data.py 2>&1
echo "SCRIPT1_DONE"
python3 20260409-cc-touch_probe_basic.py 2>&1
echo "SCRIPT2_DONE"
python3 20260409-cc-touch_probe_textures.py 2>&1
echo "SCRIPT3_DONE"
python3 20260409-cc-force_visualizer.py 2>&1
echo "SCRIPT4_DONE"
ls ../screenshots/*.png 2>/dev/null | wc -l
