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
    SDHControl(string canDev);

    void goToQ(Q aQ);
    void goToInit();
    void fullStop();

    bool isConnected();

    void grasp(double distA, double distB, double distC); //NOTE: Joint 2 (btw finger A and C) is currently always 45 deg

    vector<double> calcFingerAngle(double x, double y);

    ~SDHControl();

  private:
    SDHDriver *sdh;
    Q initQ = Q(7, -80*deg2rad, 80*deg2rad, 45*deg2rad, -80*deg2rad, 80*deg2rad, -80*deg2rad, 80*deg2rad);
    bool connected = false;

    bool isThereAnan(double a,double b,double c,double d,double e,double f);
    bool areThereInvalidAngles(double a,double b,double c,double d,double e,double f);
    bool isNotWithinThresh(double diffA, double diffB, double diffC);
    bool checkSolution(vector<double> anglesA, vector<double> anglesB, vector<double> anglesC);


};


#endif /* SDHControl_hpp */
