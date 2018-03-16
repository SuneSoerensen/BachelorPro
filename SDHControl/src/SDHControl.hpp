//
//  SDHControl.hpp
//  SDHControl
//
//  Created by <author> on 08/03/2018.
//
//

#ifndef SDHControl_hpp
#define SDHControl_hpp

//TODO: move to settings.hpp
#define GRASPSTARTHEIGHT 160 //5mm further than range of fingers
#define GRASPFINDSTEPSIZE 0.001 //Stepsize for finding finger-config
#define GRASPDISTLIM 155 //Max distance from finger-base to grasp-point
#define FINGEROFFSET 38.105 //Distance from center of SDH to base of fingers
#define DEBUG_MODE 1
#define LENGTH1 86.5 //Length of finger from base to middle-joint
#define LENGTH2 68.5 //length of finger from middle-joint to tip

#define deg2rad 0.0174532925
#define rad2deg 57.2957795

#include <stdio.h>
#include <vector>

#include <rwhw/sdh/SDHDriver.hpp>

//#include <rw/common/TimerUtil.hpp>
//#include <rw/common/Timer.hpp>
//#include <rw/common/macros.hpp>

#include <rw/math/Q.hpp>

using namespace rw::math;
using namespace rw::common;

using namespace rwhw;
//using namespace rw::common;

#include "sdh/sdh.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"
#include "sdhoptions.h"

USING_NAMESPACE_SDH

using namespace rw::math;
using namespace rw::common;

using namespace rwhw;
using namespace rw::common;
using namespace std;

class SDHControl
{
public:
    SDHControl();
    SDHControl(string canDev);

    void goToQ(Q aQ);
    void goToInit();

    bool isConnected();

    void grasp(double distA, double distB, double distC); //NOTE: Only concentric for now!!

    vector<double> calcFingerAngle(double x, double y);

    bool isThereAnan(double a,double b,double c,double d,double e,double f);

    ~SDHControl();

  private:
    SDHDriver *sdh;
    Q initQ = Q(7, -80*deg2rad, 80*deg2rad, 45*deg2rad, -80*deg2rad, 80*deg2rad, -80*deg2rad, 80*deg2rad);
    bool connected;


};


#endif /* SDHControl_hpp */
