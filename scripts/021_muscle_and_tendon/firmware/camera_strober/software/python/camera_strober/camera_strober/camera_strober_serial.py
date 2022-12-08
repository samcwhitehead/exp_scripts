"""
python serial interface for arduino nano board controlling pulse timing to muscle imaging cameras

closely following examples below:
    - https://github.com/iorodeo/nano_ssr_software/blob/master/basic/python_basic_ssr/basic_ssr/basic_ssr.py
    - https://github.com/willdickson/camera_trigger/blob/master/software/python/camera_trigger/camera_trigger/camera_trigger.py

"""
from __future__ import print_function
import serial
import sys  # want to be able to check version
import time
import json

# define class
class CameraStroberSerial(serial.Serial):
    
    
    # should ask will about this, but best to include command numbers here?
    CMD_SET_WB_PERCENT = 1
    CMD_GET_WB_PERCENT = 2
    CMD_CAM_ENABLED = 3
    CMD_CAM_DISABLED = 4
    CMD_GET_CAM_ENABLE = 5

    CAMERA_LEFT  = 2
    CAMERA_RIGHT = 1
    RESET_DT = 2.0 

    BAUDRATE = 115200

    CAM_TO_ID = {
            'left'  : CAMERA_LEFT,
            'right' : CAMERA_RIGHT,
            }


    def __init__(self, port='/dev/ttyUSB0', timeout=2.0):
        """
        initialize class
        """
        param = {'baudrate': self.BAUDRATE, 'timeout': timeout}
        super(CameraStroberSerial,self).__init__(port, **param)
        time.sleep(self.RESET_DT)
    
    
    def send_cmd(self, cmd_num, cmd_val):
        """
        general function to send command to board
        """
        # need to encode string as utf-8 if using python 3
        write_str = '[%d, %d]\n' %(cmd_num, cmd_val)
        if  sys.version_info[0] >= 3:
            self.write(write_str.encode())
        else:
            self.write(write_str)
        
        # try to read something out, in case this is a "get" command
        # may have to do a check on this throwing errors
        resp = self.readline()
        resp = json.loads(resp)
        return resp
   
   
    # not sure if this is best practice, but i feel like it will be more
    # user-friendly to define each function here (as opposed to JUST making a
    # general "set" function and then reading in a cmd_num)
    def set_wb_trig_percent(self, wb_trig_percent):
        """
        SET the wb trigger percent, i.e. the phase during the wingbeat in which
        we open camera shutter, expressed as percentage
        """
        # check input value
        if not wb_trig_percent in range(0, 101):
            #raise ValueError, 'wb_trig_percent must be between 0 and 100'
            raise ValueError('wb_trig_percent must be between 0 and 100')
            
        # if okay, write to board
        resp = self.send_cmd(self.CMD_SET_WB_PERCENT, wb_trig_percent)
        
        
    def get_wb_trig_percent(self):
        """
        GET the wb trigger percent, i.e. the phase during the wingbeat in which
        we open camera shutter, expressed as percentage
        """
        resp = self.send_cmd(self.CMD_GET_WB_PERCENT, 0)
        return resp['val']

    def set_cam_state(self,cam_side, state):
        if state:
            self.enable_cam(cam_side)
        else:
            self.disable_cam(cam_side)

           
    def enable_cam(self, cam_side):
        """
        ENABLE the selected camera (right or left) 
        """
        cmd_num = self.CMD_CAM_ENABLED
        cam_val = self.CAM_TO_ID[cam_side]
        try:
            resp = self.send_cmd(cmd_num, cam_val)
        except KeyError:
            #raise ValueError, 'camera side must be either right or left'
            raise ValueError('camera side must be either right or left')


    def disable_cam(self, cam_side):
        """
        DISABLE the selected camera (right or left) 
        """
        cmd_num = self.CMD_CAM_DISABLED
        cam_val = self.CAM_TO_ID[cam_side]
        try:
            resp = self.send_cmd(cmd_num, cam_val)
        except KeyError:
            #raise ValueError, 'camera side must be either right or left'
            raise ValueError('camera side must be either right or left')
         
            
    def get_cam_state(self, cam_side):
        """
        Check if selected camera (right or left) is enabled
        """
        cmd_num=self.CMD_GET_CAM_ENABLE
        cam_val = self.CAM_TO_ID[cam_side]
        try:
            resp = self.send_cmd(cmd_num, cam_val)
        except KeyError:
            #raise ValueError, 'camera side must be either right or left'
            raise ValueError('camera side must be either right or left')
        return bool(resp['val'])
        
        
# -----------------------------------------------------------------------------
if __name__ == '__main__':

    dev = CameraStroberSerial()

    resp = dev.get_wb_trig_percent()
    print('wb_trig_precent ', resp)

    dev.set_wb_trig_percent(25)
    resp = dev.get_wb_trig_percent()
    print('wb_trig_precent ', resp)


    for cam in ('left', 'right'):
        resp = dev.get_cam_state(cam)
        print('cam', cam, 'state = ', resp)

    dev.disable_cam('left') 

    for cam in ('left', 'right'):
        resp = dev.get_cam_state(cam)
        print('cam', cam, 'state = ', resp)
    
    dev.enable_cam('left') 

    for cam in ('left', 'right'):
        resp = dev.get_cam_state(cam)
        print('cam', cam, 'state = ', resp)


        
            
        
        

