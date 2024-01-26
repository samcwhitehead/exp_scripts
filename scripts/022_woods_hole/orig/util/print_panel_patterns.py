import scipy.io

if __name__ == '__main__':
    path = '/home/imager/catkin/src/exp_scripts/scripts/021_muscle_and_tendon/firmware/panel_controller/SD.mat'
    matdata = scipy.io.loadmat(path)
    # print(matdata['SD'][0][0][0][0][0])
    try:
        funcstrings = [x[0] for x in matdata['SD'][0][0][0][0][0][1][0]]
        patstrings = [x[0] for x in  matdata['SD'][0][0][1][0][0][-1][0]]
    except IndexError:
        patstrings = [x[0] for x in matdata['SD'][0][0][0][0][0][-1][0]]
        funcstrings = [x[0] for x in matdata['SD'][0][0][1][0][0][1][0]]

    print(patstrings)
