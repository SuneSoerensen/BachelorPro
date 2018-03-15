#pragma once



//===========
//   TIAFC
//===========
#define TIAFC_MODE		1	// 0 = normal, 1 = debug

//camera settings (taking an image):
#define CAMERA			0	// 0 = default camera
#define FRAMES_TO_GET		14	// min 7, according to initial tests

//crop settings (taking an image):
#define CROP_TOP		113
#define CROP_BOTTOM		330
#define CROP_LEFT		200
#define CROP_RIGHT		635

//for the Canny edge-detector (finding contours):
#define CANNY_THRES		100	// 0 <= THRESHOLD <= 255

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
#define MIN_POINTS_IN_LINE	10
#define MIN_LINE_LENGTH		10
#define MAX_LINE_GAP		3

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
#define UR_MIN_Z		0.283
#define R_SQUARED		0.8*0.8

#define REAL_TO_UR_OFFSET_X	-108.7
#define REAL_TO_UR_OFFSET_Y	-485.37
