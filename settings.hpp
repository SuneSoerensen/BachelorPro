#pragma once

//=====================
//   General Purpose
//=====================
#define deg2rad 0.0174532925
#define rad2deg 57.2957795

//===========
//   TIAFC
//===========
#define TIAFC_MODE		1	// 0 = normal, 1 = debug

//camera settings (taking an image):
#define CAMERA			0	// 0 = default camera
#define CAM_RES_WIDTH		1280
#define CAM_RES_HEIGHT		720
#define FRAMES_TO_GET		14	// min 7, according to initial tests

//crop settings (taking an image):
#define CROP_TOP		150
#define CROP_BOTTOM		500
#define CROP_LEFT		500
#define CROP_RIGHT		950

//for the Canny edge-detector (finding contours):
#define CANNY_THRES		50	// 0 <= THRESHOLD <= 255

//for removing too small contours:
#define MIN_POINTS_IN_CONTS	70

//============
//   Vision
//============
#define VISION_MODE		1	// 0 = normal, 1 = debug

//for calibration (pixel-coords to real-coords):
#define CALC_FACTOR		1000	//determines precision in intermediate results and calculations
#define REAL_WIDTH		48	//mm
#define REAL_HEIGHT		48	//mm
#define GLOB_X_DIR		1	//global x-direction in relation to the cameras x-direction
#define GLOB_Y_DIR		-1	//global y-direction in relation to the cameras y-direction

//for the Hough-transform line detector (grasp regs):
#define MIN_POINTS_IN_LINE	10 //for curvy objects: short lines (f.ex. 10-15) and large gap (f.ex. 3-5)
#define MIN_LINE_LENGTH		10 //for edgy objects: long lines (f.ex. 40+) and small gap (f.ex. 0-2)
#define MAX_LINE_GAP		3

//=================
//   AnalytGrasp
//=================
#define ANALYT_GRASP_MODE	1	// 0 = normal, 1 = debug

#define MAX_THREE_FING_ANG	2.35 //radians (~135 deg)
#define MAX_DEV_ANG		0.17 //radians (~10 deg)

//===============
//   URControl
//===============
#define URCONTROL_MODE		0 //0=standard 1=debug

#define ACC			0.1
#define VEL			0.1
#define MOVTIME			5
#define BLENDR			0

#define UR_MAX_X		0.400
#define UR_MIN_X		-0.308
#define UR_MAX_Y		-0.285
#define UR_MIN_Y		-0.785
#define UR_MAX_Z		0.700
#define UR_MIN_Z		0.305
#define R_SQUARED		0.8*0.8

#define REAL_TO_UR_OFFSET_X	-108.7
#define REAL_TO_UR_OFFSET_Y	-485.37

//===============
//   SDHControl
//===============
#define SDHCONTROL_MODE 1 //0=standard 1=debug
#define GRASPSTARTHEIGHT 160 //5mm further than range of fingers
#define GRASPFINDSTEPSIZE 0.001 //Stepsize for finding finger-config
#define GRASPDISTLIM 155 //Max distance from finger-base to grasp-point
#define FINGEROFFSET 38.105 //Distance from center of SDH to base of fingers
#define LENGTH1 86.5 //Length of finger from base to middle-joint
#define LENGTH2 68.5 //length of finger from middle-joint to tip

#define SDH_ANGLE_THRESH 10.0*deg2rad
#define SDH_MAX_ABS_ANGLE 90.0*deg2rad

#define SDH_PRECISION 1.0 //mm
#define SDH_DIST_INTO_OBJECT 4.0 //mm for each finger
