'
##############################
Basic protocol
##############################

(1) Turn on led arena
(2) Wait for panels to turn on and show addresses
(3) Turn on panel controler
(4) Wait for panel controler to finish boot sequence
(5) Run each of the following commands in their own terminal window/tab
	roslaunch exp_scripts 011_main.launch
	roslaunch Kinefly main.launch
	rosrun muscle_imager config_cam.py
	roslaunch ledpanels main.launch
(6) In each of the MuscleVeiwer windows select the profile (GMR22H05) and adust the masks to overlay 
    muscle anatomy. Ensure that anatomy is evenly illuminated and the time-series signals look reasonable. 
(7) Before running any experiments test execute the experiment script:
	rosrun exp_scripts 011_bilateral_correlations_Q2_manafold.py
(8) If you get an assertion error stating that dependency repositories are not up to date, git commit any changes to the offending repository.
(9) When you are ready to run an experiment, in their own terminal windows execute: 
		roslaunch exp_scripts 011_record.launch
		rosrun exp_scripts 011_bilateral_correlations_Q2_manafold.py
(10) After completing the experiment control-c to stop the 0111_record.launch process.
(11) in any available terminal windown execute:
	save_bag
	this will copy the newly created bag file to a new fly directory in FlyDB.

##############################
Important directories:
##############################

############################
# src.git                  #
# general purpose codes    #
############################

muscle_model
 *** package for dealing with 3d model of the thorax anaotomy
 *** as well as the affine registration of the model data to
 *** an imaging frame. The confocal data are stored in this 
 *** repository using the git lfs file versioning system
 *** https://git-lfs.github.com/.
 *** this repository needs to be added to the $PYTHONPATH
 *** for planotaxis program as well as most of my experiment
 *** scripts to run. Sorry, I know we don't like to use 
 *** PYTHONPATH anymore but... hmf. 

planotaxis
 *** program that performs extraction of muscle signals using
 *** the functionality provided by the muscle_model package.
 *** execute viewer.py to launch the program. For convenience
 *** I have added an alias to ~/.bashrc to call this program from 
 *** the command line:
 *** alias planotaxis='python /home/imager/src.git/planotaxis/viewer.py'

flylib
 *** contains the NetFly class that I use in analysis as an interface
 *** to the data in the FlyDB folder, as well as the util module
 *** that contaions some usefull functions for analysis. 
 ***************************
 *** IMPORTANT NOTE ********
 ***************************
 *** I recently migrated the code into flylib from a package called
 *** thllib. If you run into an import error where thllib can't be 
 *** found then you might need to change the import calls to read:
 *** import flylib as flb
 *** or
 *** from flylib import util

############################
# ~/catkin                 #
# location of ros src code #
############################


*** structure of the catkin directory:

.
├── build
├── devel
├── src
│   ├── audio_common

		*** audio on ros - maybe a way of grabbing ephys data ***

│   ├── camera_aravis

		*** for basalar cameras, not needed on this system ***

│   ├── CMakeLists.txt -> /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake
│   ├── driver_common
		
		*** needed for legacy camera drivers ***

│   ├── exp_scripts
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── exp_scripts
│   │   ├── launch
│   │   ├── msg
│   │   │   ├── MsgExpMetadata.msg
│   │   │   └── MsgExpState.msg
│   │   ├── package.xml
│   │   ├── scripts
│   │   │   ├── 000_sys_test

		*** testing things out ***

│   │   │   ├── 001_landing_optogenetics

		*** optogenetic activation of dn044 trigging the landing reflex ***
		*** 002-004 iterations of this experiment ***

│   │   │   ├── 002_landing_optogenetics_NA=0.45
│   │   │   ├── 003_landing_optogenetics_two_stim
│   │   │   ├── 004_landing_optogenetics_higer_res
│   │   │   ├── 005_b1_silencing
│   │   │   ├── 006_b1_silencing2
│   │   │   ├── 007_dn_chrimson_screen

		*** tested optogenetic activation of dn lines ***

│   │   │   ├── 008_feedback_gain

		*** explored several levels of feedback gain ***

│   │   │   ├── 009_feedback_gain_contrast_test

		*** explored contrast and gain at the same time ***

│   │   │   ├── 010_bilateral_correlations_Q1_gain

		*** started looking at blateral correlations between wings***
		*** these experiments explored the level of feedback gain ***

│   │   │   ├── 011_bilateral_correlations_Q2_manafold

		*********************************************************************
		*** Basic protocol for exploring left vs right muscle activity    ***
		*** most of the data collected for 2018 janelia meeting collected ***
		*** using this protocol.                                          ***
		*********************************************************************

│   │   │   └── 012_bilateral_correlations_Q3_roll_vs_yaw

		*** itteration of 011 - tested a yaw vs roll stimulus ***

│   │   ├── setup.py
│   │   └── src
│   │       ├── exp_scripts
│   │			├── git_tools.py
│	│			└── __init__.py

		*** tools for working with git repositories ***
				
│   │       └── __init__.py
│   ├── Kinefly

		*** 'nuf said ***

│   ├── ledpanels

		*** nodes to contol led display from ros ***

│   ├── muscle_imager
│   │   ├── bin
│   │   ├── CMakeLists.txt
│   │   ├── firmware
│   │   │   └── cam_trigger

	    *** the code for the camera trigger ***

│   │   ├── include
│   │   │   └── muscle_imager
│   │   ├── launch
│   │   ├── models
│   │   │   └── thorax
│   │   ├── msg
│   │   │   ├── Msg2DAffineFrame.msg
│   │   │   ├── MsgArrayNumpyND.msg
│   │   │   └── MsgExtractedSignal.msg

		*** messages used to extract ca signals in real-time ***

│   │   ├── nodes
│   │   │   ├── accumulate_publisher.py

		************************************************************
		*** simple node to live-boxcar filter an image stream    ***
		*** place between camera node and kinefly if you need to ***
		*** run the behavior camera at a high frame rate and     ***
		*** you experience aliasing of the wingstroke            ***
		*** eg. to track fast leg extensions                     ***
		************************************************************

│   │   │   ├── config_cam.py

		************************************************************
		*** script to setup the setings on the point grey cameras **
		*** hack to overcome a bug where these settings cannot   ***
		*** be set in the launch file                            ***
		************************************************************

│   │   │   ├── imagingAnalysis.py

		*** used for development, probably can be deleted        ***

│   │   │   ├── live_viewer.py

		*** node that drives the viewer for the muscle imaging   ***

│   │   │   ├── live_viewer.ui

		*** qt ui template file for the live_viewer node         ***

│   │   │   ├── strip_chart.py

		*** node that drives the strip chart - still a little bug **
		*** with the wingstroke frequence signal                 ***

│   │   │   ├── strip_chart.ui

		*** ui file for the strip chart node                     ***

│   │   │   └── unmixer.py

		************************************************************
		*** node that unmixes calcium signals in realtime using  ***
		*** the manual regestration defined in the live_viewer   ***
		*** node                                                 ***
		************************************************************

│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── src
│   │   └── srv
│   │       └── SrvRefFrame.srv
	
		************************************************************
		*** service that serves a 'reference frame' allows the   ***
		*** unmixer node to update the reference frame only when ***
		*** the user changes the reference frame                 ***
		************************************************************

│   ├── phidgets

		************************************************************
		*** not used presently, the analog output signals        ***
		*** from kinefly are part of the kinefly package and,    ***
		*** data aquisition from the wingbeat analyzer is handled***
		*** by the phidgets_daq package.                         ***
		************************************************************

│   ├── phidgets_daq

		************************************************************
		*** a data aquisition node for the phidgets interfacekit ***
		************************************************************

│   └── pointgrey_camera_driver

		************************************************************
		*** fork of the pointgrey camera driver. modification made**
		*** to fix trigger mode problem and activate the format7 ***
		*** image mode. pull from psilentp/pointgrey_camera_driver**
		************************************************************