# ---------------------------------------------------------------------------
# Quick script to play visual patterns on panel to see if they're working 
# ---------------------------------------------------------------------------
# Imports
import os
import sys
from ledpanels import display_ctrl
from scw_panel_lib import turn_off_panels, exc_visual_stim

# Params
stim_type_str = 'ol_loom0'  # pattern to test
MOTION_DURATION = 10  # seconds, how long to play pattern for
PLAYBACK_LEVEL = 30 # open loop playback gain(?) Hz = 90deg/sec

# -----------------------------------
# Main script
if __name__ == '__main__':
    script_path = os.path.realpath(sys.argv[0])
    script_dir = os.path.dirname(script_path)
    exp_dir = script_dir
    ctrl = display_ctrl.LedControler()
    ctrl.load_SD_inf(exp_dir + '/firmware/panel_controller/SD.mat')
    
    print('Playing %s'%(stim_type_str))
    exc_visual_stim(ctrl, stim_type_str, MOTION_DURATION, gain_x=PLAYBACK_LEVEL, gain_y=0, bias_x=0, bias_y=0, x_init=0)
    print('Visual stimulus ended')
    turn_off_panels(ctrl)
