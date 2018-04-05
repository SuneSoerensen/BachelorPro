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

    //Constructor. Connects automatically via CAN:
    SDHControl(string canDev);

    //Constructor. Connects automatically via RS232:
    SDHControl(int port, unsigned long baudrate, double timeout);

    //Go to any valid configuration:
    void goToQ(Q aQ);

    //Go to init-configuration:
    void goToInit();

    //Stop all motors and controllers:
    void fullStop();

    //Check if connected to hand:
    bool isConnected();

    //Grasp, based on distance for each finger and an angle between finger a and c:
    void grasp(double fingerAX, double fingerAY, double fingerBX, double fingerBY, double fingerCX, double fingerCY);

    //Two-finger grasp (finger B moves to init and joint #2 is 90deg):
    //void grasp(double distA, double distC);

    //Adjust velocity for SDH joints:
    void adjustVel(double joint0, double joint1, double joint2, double joint3, double joint4, double joint5, double joint6);

    ~SDHControl();

private:
    //Pointer to SDHDriver-object:
    SDHDriver *sdh;

    //Init-configuration:
    Q initQ = Q(7, -80*deg2rad, 80*deg2rad, 60*deg2rad, -80*deg2rad, 80*deg2rad, -80*deg2rad, 80*deg2rad);

    //Reflects whether connected to hand or not:
    bool connected = false;

    //Sides of the triangle to control grasp {AB, BC, CA}:
    double goalSides[3] = {0.0, 0.0, 0.0};

    //Calculate fingertip-pos and finger angle in the plane, for finger A and C:
    //Output: {dist, angle}
    vector<double> calcFingertipPos(double visDist, double anAngle);

    //Inverse kinematics for a finger (output is {finger_base, finger_tip}):
    vector<double> calcFingerAngle(double x, double y);

    //Misc. checks (NOTE: all returns TRUE if input is not applicable for finger):
    bool isThereAnan(double a,double b,double c,double d,double e,double f); //NOTE: doesn't check joint #2!
    bool areThereInvalidAngles(Q aQ);
    bool areThereInvalidAngles(vector<double> anglesA, vector<double> anglesB, vector<double> anglesC);
    bool isNotWithinThresh(double diffA, double diffB, double diffC);

    //Uses all above checks to verify if input angles are a valid configuration:
    bool checkSolution(vector<double> anglesA, vector<double> anglesB, vector<double> anglesC);

    vector<double> calcFingerDist(double angleBase, double angleTop);
    bool controlGrasp(double goalDistA, double goalDistB, double goalDistC);
    bool controlGraspPlacment(double angleAC, double currDistA, double currDistB, double currDistC);

    //Same as above, but for two-finger grasps:
    bool checkSolution(vector<double> anglesA, vector<double> anglesC);
};


#endif
