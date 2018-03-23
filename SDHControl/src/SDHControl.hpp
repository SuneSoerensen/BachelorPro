#ifndef SDHControl_hpp
#define SDHControl_hpp

#include <stdio.h>
#include <vector>

#include <rwhw/sdh/SDHDriver.hpp>
#include <rw/math/Q.hpp>

using namespace rw::math;
using namespace rw::common;

using namespace rwhw;

#include "sdh/sdh.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"
#include "sdhoptions.h"

#include "settings.hpp"

USING_NAMESPACE_SDH

using namespace rw::math;
using namespace rw::common;
using namespace rwhw;
//using namespace rw::common;
using namespace std;

class SDHControl
{
public:
    SDHControl();

    //Constructor. Connects automatically:
    SDHControl(string canDev);

    //Go to any valid configuration:
    void goToQ(Q aQ);

    //Go to init-configuration:
    void goToInit();

    //Stop all motors and controllers:
    void fullStop();

    //Check if connected to hand:
    bool isConnected();

    //Grasp, based on distance for each finger (NOTE: joint #2 (btwn. finger A and C) is currently always 45 deg):
    void grasp(double distA, double distB, double distC);

    ~SDHControl();

  private:
    //Pointer to SDHDriver-object:
    SDHDriver *sdh;

    //Init-configuration:
    Q initQ = Q(7, -80*deg2rad, 80*deg2rad, 45*deg2rad, -80*deg2rad, 80*deg2rad, -80*deg2rad, 80*deg2rad);

    //Reflects whether connected to hand or not:
    bool connected = false;

    //Inverse kinematics for a finger (output is {finger_base, finger_tip}):
    vector<double> calcFingerAngle(double x, double y);

    //Misc. checks (NOTE: all returns TRUE if input is not applicable for finger):
    bool isThereAnan(double a,double b,double c,double d,double e,double f); //NOTE: doesn't check joint #2!
    bool areThereInvalidAngles(Q aQ);
    bool areThereInvalidAngles(vector<double> anglesA, vector<double> anglesB, vector<double> anglesC);
    bool isNotWithinThresh(double diffA, double diffB, double diffC);

    //Uses all above checks to verify if input angles are a valid configuration:
    bool checkSolution(vector<double> anglesA, vector<double> anglesB, vector<double> anglesC);
};


#endif
