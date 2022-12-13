"""
Temp script to print rframe hdf5 contents

"""
import os 
import h5py

FLYDB_PATH = '/media/imager/DataExternal/FlyDB'
FLY_NUM = 24
SIDE = 'left'
GROUP_KEY = 'LogRefFrame'

def print_rframe():
    fly_path = os.path.join(FLYDB_PATH, 'Fly%04d'%(FLY_NUM))
    rframe_fn = 'live_viewer_%s_rframe_fits.hdf5'%(SIDE)
    
    with h5py.File(os.path.join(fly_path, rframe_fn), 'r') as h5f:
        for key in h5f[GROUP_KEY].keys():
            print(key, ' = ', h5f[GROUP_KEY][key][()])

if __name__ == '__main__':
	print_rframe()
