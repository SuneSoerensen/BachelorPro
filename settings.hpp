#pragma once



//===========
//   TIAFC
//===========
#define TIAFC_MODE		1	// 0 = normal, 1 = debug

#define CAMERA			1	// 0 = default camera
#define FRAMES_TO_GET		42	// min 7, according to initial tests

#define CROP_TOP		  113
#define CROP_BOTTOM		330
#define CROP_LEFT		  200
#define CROP_RIGHT		635

#define CANNY_THRES		100	// 0 <= THRESHOLD <= 255

//==============
//   ContProc
//==============
#define CONTPROC_MODE		1	// 0 = normal, 1 = debug

#define CALC_FACTOR		1000
#define REAL_WIDTH		48	//mm
#define REAL_HEIGHT		48	//mm
#define GLOB_X_DIR		1
#define GLOB_Y_DIR		-1

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
