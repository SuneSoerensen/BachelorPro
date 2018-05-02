#pragma once

//=====================
//   General Purpose
//=====================
#define deg2rad 0.0174532925
#define rad2deg 57.2957795

//============
//   Vision
//============
#define VISION_DEV_MODE	1	//developer mode for Vision and all of its dependencies (0 = false, 1 = true)

//--------
// Vision
//--------
#define VISION_MODE		1	// 0 = normal, 1 = info

//for calibration (pixel-coords to real-coords):
//(for developer tests: 186.0, 141.0)
#define REAL_OBJ_WIDTH		186.0 //mm
#define REAL_OBJ_HEIGHT		141.0 //mm
#define GLOB_X_DIR		1 //global x-direction in relation to the cameras x-direction
#define GLOB_Y_DIR		-1 //global y-direction in relation to the cameras y-direction

//-------------
// AnalytGrasp
//-------------
#define ANALYT_GRASP_MODE	1	// 0 = normal, 1 = info

//for grasping:
#define GRASP_REG_WIDTH		30.0 //mm (width of a finger + a bit)
#define GRASP_REG_MAX_DEV		1.9 //mm

#define MAX_COM_FOCUS_DIST	50.0 //mm
#define MIN_GRASP_POINT_DIST	60.0 //mm (the absolutely minimum distance between two grasp points)

#define P1_MIN_ANG_AC		0.17 //radians (~10 deg) (the minimum angle between finger a and c for a p1 grasp)
#define P1_MAX_ANG_AC		2.26 //radians (~130 deg) (the maximum angle between finger a and c for a p1 grasp)
#define P1_MAX_ANG_AB_BC_DEV		0.17 //radians (~10 deg) (the maximum the angle between finger b and the others can deviate from the ideal in either direction)
#define P1_MAX_FOCUS_ANG_DEV		0.17 //radians (~10 deg)

#define P2_MAX_ANG_AC		0.17 //radians (~10 deg) (the maximum angle between finger a and c for a p2 grasp)
#define P2_MAX_ANG_AB_BC_DEV		0.17 //radians (~10 deg) (the maximum the angle between finger b and the others can deviate from the ideal in either direction)
#define P2_MAX_FOCUS_ANG_DEV		0.17 //radians (~10 deg)
#define P2_DIST_AC				66.0 //mm
#define P2_MAX_DIST_AC_DEV		1.0 //mm

#define P3_MAX_ANG_AC_DEV	0.17 //radians (~10 deg) (the maximum the angle between finger a and c can deviate from 0 in either direction)
#define P3_MAX_FOCUS_ANG_DEV		0.08 //radians (~5 deg)

//-------
// TIAFC
//-------
#define TIAFC_MODE		1	// 0 = normal, 1 = info

//camera settings:
#define CAMERA			0	// 0 = default camera
#define CAM_RES_WIDTH	1280
#define CAM_RES_HEIGHT	720
#define FRAMES_TO_GET	14	// min 7, according to initial tests

//crop settings:
//(for developer tests: 100, 500, 400, 1000)
#define CROP_TOP		100
#define CROP_BOTTOM		500
#define CROP_LEFT		400
#define CROP_RIGHT		1000

//for thresholding:
#define THRESHOLDING_THRES	110 //should be between the intensities of the background and the object
#define THRESHOLDING_MODE	0 //0 = light object on dark background, 1 = dark object on light background

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

#define MAX_WRIST_ANGLE 360.0*deg2rad

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
#define SDHCONTROL_MODE 0 //0=standard 1=debug
#define GRASPSTARTHEIGHT 160 //5mm further than range of fingers //TODO: delete if unused
#define GRASPFINDSTEPSIZE 0.001 //Stepsize for finding finger-config. in grasp()
#define GRASPDISTLIM 155.0 //Total length of a finger
#define FINGEROFFSET 38.105 //Distance from center of SDH to base of fingers
#define SDH_HALF_FINGER_WIDTH 12.5 //mm
#define LENGTH1 86.5 //Length of finger from base to middle-joint
#define LENGTH2 68.5 //length of finger from middle-joint to tip
#define WRIST_HEIGHT 98.0 //mm
#define SEPERATOR_HEIGHT 23 //mm

#define TARGET_Z 70.0 //mm Defines at which height the target object should be grasped

#define SDH_ANGLE_THRESH 20.0*deg2rad //Max angle of fingertips in a grasp
#define SDH_MAX_ABS_ANGLE 91.0*deg2rad //Max abs. angle of any SDH joint

#define SDH_PRECISION 10.0 //mm Defines how far a grasp can be from the intended, and still be valid
#define SDH_DIST_INTO_OBJECT 4 //mm for each finger

#define FINGERAC_ANGLE_THRESH 10*deg2rad //How close the angle of finger A and C should be to each other

#define SDH_FINGER_BASE_OFF_X 33.0 //mm
#define SDH_FINGER_BASE_OFF_Y 19.0525 //mm

//#define SDH_ANGLE_DIFF_FIRST 10.0 //deg //TODO: delete if still unused

#define PREGRASP_SCALE 30.0 //mm How far from the grasp-position each finger should be, in a pre-grasp

#define POSTION_TEST 1 //0 = Standard 1 = To run test
