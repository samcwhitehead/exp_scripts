"""
Python serial interface for Arduino Nano board controlling pulse timing to muscle imaging cameras

Closely following examples below:
    - https://github.com/iorodeo/nano_ssr_software/blob/master/basic/python_basic_ssr/basic_ssr/basic_ssr.py
    - https://github.com/willdickson/camera_trigger/blob/master/software/python/camera_trigger/camera_trigger/camera_trigger.py

"""
import serial
import sys  # want to be able to check version

# define class
class CameraStroberSerial(serial.Serial):
    
    
    # should ask Will about this, but best to include command numbers here?
    CMD_SET_WB_PERCENT = 1
    CMD_GET_WB_PERCENT = 2
    CMD_CAM_ENABLED = 3
    CMD_CAM_DISABLED = 4
    CMD_GET_CAM_ENABLE = 5

    CAMERA_LEFT  = 1
    CAMERA_RIGHT = 2


    def __init__(self,**kwargs):
        """
        Initialize class
        """
        super(CameraStroberSerial,self).__init__(**kwargs)
    
    
    def send_cmd(self, cmd_num, cmd_val):
        """
        General function to send command to board
        """
        # need to encode string as utf-8 if using Python 3
        write_str = '[%d, %d]\n' %(cmd_num, cmd_val)
        if  sys.version_info[0] >= 3:
            self.write(write_str.encode())
        else:
            self.write(write_str)
        
        # try to read something out, in case this is a "get" command
        # May have to do a check on this throwing errors
        resp = self.readline()
        return resp
   
   
    # not sure if this is best practice, but I feel like it will be more user-friendly to define each function here
    # (as opposed to JUST making a general "set" function and then reading in a cmd_num)
    def set_wb_trig_percent(self, wb_trig_percent=50, cmd_num=CMD_SET_WB_PERCENT):
        """
        SET the wb trigger percent, i.e. the phase during the wingbeat in which we open camera shutter, expressed as percentage
        """
        # check input value
        if not wb_trig_percent in range(0, 101):
            raise ValueError, 'wb_trig_percent must be between 0 and 100'
            
        # if okay, write to board
        self.send_cmd(cmd_num, wb_trig_percent)
        
        
    def get_wb_trig_percent(self, cmd_num=CMD_GET_WB_PERCENT):
        """
        GET the wb trigger percent, i.e. the phase during the wingbeat in which we open camera shutter, expressed as percentage
        """
        return self.send_cmd(cmd_num, 0)

           
    def enable_cam(self, cam_side, cmd_num=CMD_CAM_ENABLED):
        """
        ENABLE the selected camera (right or left) 
        """
        if cam_side == "left":
            self.send_cmd(cmd_num, CAMERA_LEFT)
        elif cam_side == "right"
            self.send_cmd(cmd_num, CAMERA_RIGHT)
        else:
            raise ValueError, 'camera side must be either right or left'


    def disable_cam(self, cam_side, cmd_num=CMD_CAM_DISABLED):
        """
        DISABLE the selected camera (right or left) 
        """
        if cam_side == "left":
            self.send_cmd(cmd_num, CAMERA_LEFT)
        elif cam_side == "right"
            self.send_cmd(cmd_num, CAMERA_RIGHT)
        else:
            raise ValueError, 'camera side must be either right or left'
         
            
    def get_cam_state(self, cam_side, cmd_num=CMD_GET_CAM_ENABLE):
        """
        Check if selected camera (right or left) is enabled
        """
        if cam_side == "left":
            cam_state = self.send_cmd(cmd_num, CAMERA_LEFT)
        elif cam_side == "right"
            cam_state = self.send_cmd(cmd_num, CAMERA_RIGHT)
        else:
            raise ValueError, 'camera side must be either right or left'
        
        return cam_state
       
        
        
            
        
        

