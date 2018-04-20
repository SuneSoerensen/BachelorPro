#pragma once

//=====================
//   General Purpose
//=====================
#define deg2rad 0.0174532925
#define rad2deg 57.2957795

//===========
//   TIAFC
//===========
#define TIAFC_MODE		1	// 0 = normal, 1 = info

//camera settings (taking an image):
#define CAMERA			1	// 0 = default camera
#define CAM_RES_WIDTH	1280
#define CAM_RES_HEIGHT	720
#define FRAMES_TO_GET	14	// min 7, according to initial tests

//crop settings (taking an image):
<<<<<<< HEAD
#define CROP_TOP		100
#define CROP_BOTTOM	695
#define CROP_LEFT		140
#define CROP_RIGHT	1000
=======
#define CROP_TOP		0
#define CROP_BOTTOM		600
#define CROP_LEFT		200
#define CROP_RIGHT		950
>>>>>>> e1089487b3e2f56699db2fb5a301ef389090f4cf

//for thresholding:
#define THRESHOLDING_THRES	200 	//should be between the intensities of the background and the object
#define THRESHOLDING_MODE	0	//0 = light object on dark background, 1 = dark object on light background

//============
//   Vision
//============
#define VISION_MODE		1	// 0 = normal, 1 = info

//for calibration (pixel-coords to real-coords):
#define CALC_FACTOR		1000	//determines precision in intermediate results and calculations
#define REAL_WIDTH		79.0	//mm
#define REAL_HEIGHT		79.0	//mm
#define GLOB_X_DIR		1	//global x-direction in relation to the cameras x-direction
#define GLOB_Y_DIR		-1	//global y-direction in relation to the cameras y-direction

//for the Hough-transform line detector (grasp regs):
//(for curvy objects: short lines (f.ex. 10-15) and large gap (f.ex. 3-5))
//(for edgy objects: long lines (f.ex. 40+) and small gap (f.ex. 0-2))
#define MIN_POINTS_IN_LINE	10
#define MIN_LINE_LENGTH		10
#define MAX_LINE_GAP		3

//=================
//   AnalytGrasp
//=================
#define ANALYT_GRASP_MODE	1	// 0 = normal, 1 = info

#define MIN_P1_ANG		0.17 //radians (~10 deg)
#define MAX_P1_ANG		2.62 //radians (~150 deg)
#define MAX_DEV_ANG		0.17 //radians (~10 deg)

#define MAX_P1_INTERSEC_DIST	10.0 //pixels

//===============
//   URControl
//===============
#define URCONTROL_MODE	1 //0=standard 1=debug

#define ACC			0.1
#define VEL			0.1
#define MOVTIME			5
#define BLENDR			0

#define UR_MAX_X		-0.15569 //m
#define UR_MIN_X		-0.54456 //m
#define UR_MAX_Y		-0.19335 //m
#define UR_MIN_Y		-0.47619 //m
#define UR_MAX_Z		 0.600 //m
#define UR_MIN_Z		 0.300 //m
#define R_SQUARED		 0.8*0.8

#define REAL_TO_UR_OFFSET_X	-367.8
#define REAL_TO_UR_OFFSET_Y	-334.8

#define OFFSET_ANGLE -45.0*deg2rad // = -45 deg (because of new setup)

#define INIT_POS_X  -0.3678 //m
#define INIT_POS_Y  -0.3348 //m
#define INIT_POS_Z   0.43302 //m
#define INIT_POS_RX -0.945 //rad
#define INIT_POS_RY -2.9945 //rad
#define INIT_POS_RZ  0.0 //rad

#define STATE_HOME  0
#define STATE_INIT  1
#define STATE_OTHER 2

//===============
//   SDHControl
//===============
#define SDHCONTROL_MODE 1 //0=standard 1=debug
#define GRASPSTARTHEIGHT 160 //5mm further than range of fingers
#define GRASPFINDSTEPSIZE 0.001 //Stepsize for finding finger-config
#define GRASPDISTLIM 155.0 //Max distance from finger-base to grasp-point
#define FINGEROFFSET 38.105 //Distance from center of SDH to base of fingers
#define SDH_HALF_FINGER_WIDTH 12.5 //mm
#define LENGTH1 86.5 //Length of finger from base to middle-joint
#define LENGTH2 68.5 //length of finger from middle-joint to tip
#define WRIST_HEIGHT 98.0 //mm
#define SEPERATOR_HEIGHT 23 //mm

#define TARGET_Z 80.0 //mm above plate

#define SDH_ANGLE_THRESH 10.0*deg2rad
#define SDH_MAX_ABS_ANGLE 90.0*deg2rad

#define SDH_PRECISION 10.0 //mm
#define SDH_DIST_INTO_OBJECT 0 //mm for each finger

#define SDH_FINGER_BASE_OFF_X 33.0 //mm
#define SDH_FINGER_BASE_OFF_Y 19.0525 //mm

#define SDH_ANGLE_DIFF_FIRST 10.0 //deg

#define PREGRASP_SCALE 30.0 //mm

