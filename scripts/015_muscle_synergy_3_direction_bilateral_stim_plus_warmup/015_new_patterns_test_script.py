#####Start playing with the new patterns
#!/usr/bin/env python
#
### Exp module using panel_com
#from glob import glob

import time
import os
import sys
import roslib
import rospy
from std_msgs.msg import String
from exp_scripts.msg import MsgExpState
from exp_scripts.msg import MsgExpMetadata
from exp_scripts import git_tools
from ledpanels import display_ctrl
from muscle_imager.srv import SrvRefFrame
from muscle_imager.srv import SrvRefFrameRequest
import readline
import cPickle
import itertools


############################################################################
########################### Metadata Information ###########################
############################################################################

exp_description = \
"""
Examine correlations between wing muscles on left-vs-right wing under 
open and closed loop optomotor stablization conditions (left-right bias)
+/- vertical translation.
Practice experiments for Alysha.
"""

NUM_REPS = 20  #increase the reps per fly 10-20 #to 20 from 10 for original experiments was 3-5
CL_GAIN_X = -1

#no chrimson in this exp
#CHRIMSON_VOLTS = 5
#CHRIMSON_CHANNEL = 3

fly_dob = raw_input('fly DOB:')
genotype_nickname = '22H05-Gal4:UAS-GcAMP6f'
head_fixed = True

print genotype_nickname


############################################################################
########################### Initialize Experiment ##########################
############################################################################



script_path = os.path.realpath(sys.argv[0])
script_dir = os.path.dirname(script_path)
fly_genotype = """w[1118]/+[HCS];P{20XUAS-IVS-GCaMP6f}attP40/+;P{y[+t7.7]w[+mC]=GMR22H05-GAL4}attP2/+"""
#load the script to publish as message
with open(script_path,'rt') as f:
    script_code = f.read() 
#list of all git tracked repositories
with open(os.path.join(script_dir,'tracked_git_repos.txt')) as f:
    repo_dirs = f.readlines()
assert git_tools.check_git_status(repo_dirs)
git_SHA = git_tools.get_SHA_keys(repo_dirs)



#############################################################################
################################ Run experiment #############################
#############################################################################



if __name__ == '__main__':
    try:
        import numpy as np
        print 'start'
        rospy.init_node('exp_script')
        exp_dir = script_dir
        ctrl = display_ctrl.LedControler()
        ctrl.load_SD_inf(exp_dir + '/firmware/panel_controler/SD.mat')
        
        exp_pub = rospy.Publisher('/exp_scripts/exp_state', 
                                    String,
                                    queue_size = 10)
        meta_pub = rospy.Publisher('/exp_scripts/exp_metadata', 
                                    String,
                                    queue_size = 10)
        blk_pub = rospy.Publisher('/exp_scripts/exp_block',
                                    String,
                                    queue_size = 10)
        ch_pub = rospy.Publisher('/kinefly/flystate2phidgetsanalog/command',
                                    String,
                                    queue_size = 10)

        rospy.wait_for_service('/unmixer_left/RefFrameServer')
        get_ref_frame_left = rospy.ServiceProxy('/unmixer_left/RefFrameServer', SrvRefFrame)
        rospy.wait_for_service('/unmixer_right/RefFrameServer')
        get_ref_frame_right = rospy.ServiceProxy('/unmixer_right/RefFrameServer', SrvRefFrame)
        # init experiment
        time.sleep(5) # wait for all the publishers to come online
            

        def exc_cl_blocks(block_name,gain_x,gain_y,bias_x,bias_y,subpat):
            #pattern_name = 'Pattern_4x4_blocks_60.mat'
            pattern_name = 'Pattern_rotation_translation_48P_bd.mat'
            blk_pub.publish(block_name)
            print block_name

            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            #ctlr.SetMode(modex, 0) 
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = 0, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)

        def exc_ol_blocks(block_name,gain_x,gain_y,bias_x,bias_y, subpat):
            pattern_name = 'Pattern_rotation_translation_48P_bd.mat'
            blk_pub.publish(block_name)
            print block_name
            
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name('Pattern_4x4_blocks_60.mat')
            ctrl.set_position(np.random.randint(0,96),0)   ## zero mean first subpattern set_position(stanting_position, int(subpat))
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = CL_GAIN_X, gain_y = 0, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)

        def exc_ol_simple_blocks(block_name,gain_x,gain_y,bias_x,bias_y,subpat):
            pattern_name = 'Pattern_rotation_translation_48P_bd.mat'
            blk_pub.publish(block_name)
            print block_name
            
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name('Pattern_4x4_blocks_60.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = 0, gain_y = 0, bias_x = 0,bias_y = 0)  #static first 3 seconds
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(1)   #actual time of experiment
            #ch_pub.publish('set_a30 %s'%(ch))
            #time.sleep(2)
            ch_pub.publish('condition static for decay')
            ctrl.send_gain_bias(gain_x = 0, gain_y = 0, bias_x = 0,bias_y = 0)
            time.sleep(4)

        def exc_ol_stripe(block_name,gain_x,gain_y,bias_x,bias_y,subpat):
            pattern_name = 'Pattern_bd_stripe_solid_drum.mat'

            blk_pub.publish(block_name)
            print block_name
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name('Pattern_4x4_blocks_60.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = CL_GAIN_X, gain_y = 0, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_position(18,0)
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)


            def exc_cl_stripe(block_name,gain_x,gain_y,bias_x,bias_y,subpat):  ###make sure to change this
            pattern_name = 'Pattern_bd_stripe_solid_drum.mat'

            blk_pub.publish(block_name)
            print block_name
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name('Pattern_4x4_blocks_60.mat')
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = CL_GAIN_X, gain_y = 0, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_position(18,0)
            ctrl.set_mode('xrate=funcx','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)


        def exc_loom_blocks(block_name,gain_x,gain_y,bias_x,bias_y,subpat):    ####change this to closed loop stripe###
            pattern_name = 'Pattern_expansion_48.mat'
            blk_pub.publish(block_name)
            print block_name

            ctrl.stop()
            ctrl.set_position_function_by_name('X','default')
            ctrl.set_pattern_by_name(pattern_name)
            ctrl.set_position(np.random.randint(0,96),0)
            ctrl.set_mode('xrate=ch0','yrate=funcy')
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = 0, bias_x = 0,bias_y = 0)
            exp_pub.publish('condition=baseline')
            ctrl.start()
            time.sleep(3)
            ctrl.stop()
            ctrl.send_gain_bias(gain_x = gain_x, gain_y = gain_y, bias_x = bias_x, bias_y = bias_y)
            exp_pub.publish('condition=test')
            ctrl.start()
            time.sleep(3)
            ch_pub.publish('set_a30 %s'%(ch))
            time.sleep(2)
            ch_pub.publish('set_a30 0')
            time.sleep(2)




























def OpenLp(ctlr, gainOL, OLstim_dur_sec, subpat, slpT):
    time.sleep(slpT)
    modex = 0
    ctlr.SetMode(modex, 0)    
    #gainOLsigned = signDir * gainOL
    ctlr.SetGainOffset(gainOL, 0, 0, 0)
    time.sleep(slpT)
    # run stimulus
    ctlr.Start()
    time.sleep(OLstim_dur_sec)
    ctlr.Stop()
    time.sleep(slpT)
    
def ClsdLp(ctlr, gainCL, CLstim_dur_sec, subpat, slpT):
#    # initialize stimulus
    time.sleep(slpT)
    modex = 1 # closed loop
    ctlr.SetMode(modex, 0)
    time.sleep(slpT)
    ctlr.SetGainOffset(gainCL, 0, 0, 0)
    time.sleep(slpT)
    #ctlr.SetPositions(0,subpat) # SW grating
    #time.sleep(slpT)
    # run stimulus
    ctlr.Start()
    time.sleep(CLstim_dur_sec)
    ctlr.Stop()
    time.sleep(slpT)
    modex = 0 # back to open loop
    ctlr.SetMode(modex, 0)
    time.sleep(slpT)
    ctlr.SetGainOffset(gainOL, 0, 0, 0)
    time.sleep(slpT)

def StaticIm(Static_dur, slpT):
    # static frontal stripe 
    time.sleep(slpT)
    ctlr.SetGainOffset(0, 0, 0, 0)
    time.sleep(slpT)
    #ctlr.SetPositions(0,0)
    #time.sleep(slpT)
    # run stimulus
    ctlr.Start()
    time.sleep(Static_dur)
    ctlr.Stop()
    time.sleep(slpT)

    
## Generate array to determine Pat and Dirctn
patss = [[(0.,.25)],[(1.,.5)],[(2.,.75)],[(3.,1.)],[(4.,1.25)],[(5.,1.5)],[(0.,.25)],[(1.,.5)],[(2.,.75)],[(3.,1.)],[(4.,1.25)],[(5.,1.5)],[(0.,.25)],[(1.,.5)],[(2.,.75)],[(3.,1.)],[(4.,1.25)],[(5.,1.5)],[(0.,.25)],[(1.,.5)],[(2.,.75)],[(3.,1.)],[(4.,1.25)],[(5.,1.5)],[(0.,.25)],[(1.,.5)],[(2.,.75)],[(3.,1.)],[(4.,1.25)],[(5.,1.5)],[(0.,.25)],[(1.,.5)],[(2.,.75)],[(3.,1.)],[(4.,1.25)],[(5.,1.5)],[(0.,.25)],[(1.,.5)],[(2.,.75)],[(3.,1.)],[(4.,1.25)],[(5.,1.5)],[(0.,.25)],[(1.,.5)],[(2.,.75)],[(3.,1.)],[(4.,1.25)],[(5.,1.5)],[(0.,.25)],[(1.,.5)],[(2.,.75)],[(3.,1.)],[(4.,1.25)],[(5.,1.5)],[(0.,.25)],[(1.,.5)],[(2.,.75)],[(3.,1.)],[(4.,1.25)],[(5.,1.5)]]
pats = np.vstack(patss)
# Randomize
np.random.shuffle(pats)

## Prep AO

## AO channel for the SSR control voltage. to ai1
# OdrY = 3.
# OdrN = 0.
# firstAOCH = 0

## AO channel for the pattern and its direction. 'AuxPhidgAO1'
secondAOCH = 1

## AO channel for the Acq. Stimulus Trigger 
thirdAOCH = 2

analog = Phidgets.Devices.Analog.Analog()
analog.openPhidget(PhidSN)
analog.waitForAttach(10000)

## Channel for SSR control (and thus OdrY/OdrN)
#analog.setEnabled(firstAOCH, True)
#analog.setVoltage(firstAOCH, OdrN)

## Channel for the pattern ID voltage
NoPattV = 9.99
analog.setEnabled(secondAOCH, True)
analog.setVoltage(secondAOCH, NoPattV)


## set LEDpanel runtime params
#gainCL = -1.5
gainCL = -3.0
gainOL_ = 6. # 6. corresponds to 60Hz update rate. for 32Hz update rate (pattern frame rate), set to 3.2 (for 2 Hz; 72 deg/s pattern movement)
gainOL = gainOL_
pattern_id = 1
subpat = int(1) # for SquareWave (SW) grating 

CLstim_dur  = 2.
Static_dur = 1. #30.
OLstim_dur = 2. #30.

#CLstim_dur  = 20.
#Static_dur = 2. #30.
#OLstim_dur = 2. #30.

slpT = .05 # for LED panel controller to update

#NoPattV = 9.99
NoStimV = 9.5
CLVoltage = 0.
#LoomVoltage = 8.
# Graded Loom rates: PattIDV = LoomVoltage * signDir + signDir*(abs(dirctn)-1.)
#AllOnVoltage = 7.
StatImVoltage = 0. 
OLVoltage = 2.5 # pattern-dependent. 2.5v for yaw. - if to the left


## LED Panel Controller parameters. Details.
gain_x = gainOL_ # sets direction (+R / -L)
offset_x = 0 # sets speed
gain_y = 0 # keep 0
offset_y = 0 # always keep 0. has no meaning in open loop mode
modex = 0 #open loop
#modex = 1 #closed-loop
#modey = 0 #open-loop
#pattern_id = 1 
#xpos = 0 # sets pattern.x_num 
#ypos = 0 # sets pattern.y_num, the sub-pattern, if modey is open-loop (modey=0 / use by default)
#subpat = ypos # 0-based
## PanelCom.send commands
#ctlr.SetGainOffset(gain_x, offset_x, gain_y, offset_y)
#ctlr.SetMode(modex, modey)
#ctlr.SetPatternID(pattern_id)
#ctlr.SetPositions(xpos,ypos)

# Thor Stimulus Trigger to start acquiring images and signals
analog.setEnabled(thirdAOCH, True)
analog.setVoltage(thirdAOCH, 5.)

# Assign the LED-PCB to 'PanelCom', under 'ctlr'
#ctlr = PanelCom(userport='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A603XPGN-if00-port0')
#Metal Panel Display Controller box: In 2p-room with sabertooth comp.
ctlr = PanelCom(userport='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A400f6mE-if00-port0')
# plastic Panel Display Controller box. "From 2-Photon room" on tape.


  
#time.sleep(slpT)
#ctlr.AllOff()
time.sleep(3.0) # just some time to start the Scope
# set pattern.x_num to zero, to be clean in ThorSync upon initiation

print str(datetime.datetime.now())

#modex = 1 # closed loop
#ctlr.SetMode(modex, 0)
#time.sleep(slpT) 
#ctlr.SetGainOffset(gainCL, 0, 0, 0)  # for closed loop
#time.sleep(slpT)
pattern_id = 1 # 1-based
ctlr.SetPatternID(pattern_id)
time.sleep(slpT)
ctlr.SetPositions(0,subpat)
time.sleep(slpT)

# initial closed loop stimulation
print 'CL'
PattIDV = CLVoltage
analog.setVoltage(secondAOCH, PattIDV)
pattern_id = 1
ClsdLp(ctlr, gainCL, CLstim_dur, subpat, slpT)
print 'AllOff'
analog.setVoltage(secondAOCH, NoPattV)
ctlr.AllOff()
time.sleep(0.5)

k = -1
while (k <  pats.shape[0]-1 ):
    k = k+1
    L=k+1
    (subpats, dirctn) = pats[k]
#    signDir = np.sign(dirctn)
#    if signDir == 1:
#	pattDir = 'Right '
#    else:
#        pattDir = 'Left '

    print 'static image'
    PattIDV = StatImVoltage
    ctlr.SetPositions(0,int(subpats))
    analog.setVoltage(secondAOCH, PattIDV)
    pattern_id = 2
    StaticIm(Static_dur, slpT)

    print 'OL Stim; %d out of ' % L + str( pats.shape[0] )

    PattIDV = OLVoltage * dirctn
    analog.setVoltage(secondAOCH, PattIDV)
    OpenLp(ctlr, gainOL, OLstim_dur, subpats, slpT)

    print 'panels off'
    PattIDV = StatImVoltage
    analog.setVoltage(secondAOCH, PattIDV)
    ctlr.AllOff()
    time.sleep(1.)

    print 'CL'
    PattIDV = CLVoltage
    analog.setVoltage(secondAOCH, PattIDV)
    pattern_id = 1
    ClsdLp(ctlr, gainCL, CLstim_dur, subpat, slpT)

# To indicate the end in ThorSync
analog.setVoltage(secondAOCH, NoPattV)
print 'AllOff'

ctlr.AllOff()
time.sleep(0.5)

## Close Phidget AO
#analog.setVoltage(firstAOCH, 0.) 
#analog.setEnabled(firstAOCH, False)

# terminate PatternIDV
analog.setVoltage(secondAOCH, 0.)
analog.setEnabled(secondAOCH, False)

# End data acquisition
analog.setVoltage(thirdAOCH, 0.)
analog.setEnabled(thirdAOCH, False)

#time.sleep(.5)
analog.closePhidget()

print str(datetime.datetime.now())
