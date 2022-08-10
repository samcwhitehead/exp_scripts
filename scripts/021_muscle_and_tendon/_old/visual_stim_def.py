# !/usr/bin/env python
# work space for making a more efficient visual stim function

"""
Function to execute generic visual stimulus

INPUTS:
    - block_name: string describing the type of visual stimulus. should be of the form '(feedback type)_(rot axis)_(direction)'
        i.e. 'ol_yaw_left' = open loop yaw left or 'cl_stripe' = closed loop stripe
    - duration: time to show stimulus in seconds
    - gain_x, gain_y: gains in x, y directions
    - bias_x, bias_y: bias in x, y directions
    - ch: boolean for using a Chrimson stimulus or not (under construction!)
"""
def exc_visual_stim(block_name, duration, gain_x=0, gain_y=0, bias_x=0, bias_y=0, ch=0):
    
    # publish the type of stimulus to the blk ros channel
    blk_pub.publish(block_name)
    print block_name

    # try to extract some info from the string pattern
    block_name_split = block_name.split('_')

    # first see if we're doing open or closed loop 
    if (len(block_name_split)<3) or (block_name_split[0]=='cl'):
        open_loop_flag = False

    elif block_name_split[0]=='ol':
        open_loop_flag = True

    else:
        raise Exception("Could not determine feedback type (open vs closed loop)")

    # next determine rotational axis/visual pattern. 
    # NB: this determines what to load off of SD card
    pat_str = '_'.join(block_name_split[1:])
    pattern_dict = get_pattern_dict()
    pattern_name = pattern_dict[pat_str]
    elif pat_str == 'yaw_right'
        pattern_name = 'Pattern_rot_axis_5.mat'
    elif pat_str == 'yaw_left'
        pattern_name = 'Pattern_rot_axis_4.mat'
    elif pat_str == 'yaw_right'
        pattern_name = 'Pattern_rot_axis_5.mat'
        


"""
Quick function to define a dictionary that takes pattern strings as inputs and outputs SD card filenames
...could be improved a lot

"""
def get_pattern_dict():
    pattern_dict = {'stripe'     : 'Pattern_bar.mat',
                    'yaw_right'  : 'Pattern_rot_axis_5.mat',
                    'yaw_left'   : 'Pattern_rot_axis_4.mat',
                    'pitch_up'   : 'Pattern_rot_axis_0.mat',
                    'pitch_down' : 'Pattern_rot_axis_2.mat',
                    'roll_left'  : 'Pattern_rot_axis_3.mat',
                    'roll_right' : 'Pattern_rot_axis_1.mat',
                    }

    return pattern_dict
