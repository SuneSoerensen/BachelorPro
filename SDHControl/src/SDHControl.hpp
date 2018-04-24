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

    //Grasp, based on coordinates for each contact point OR pre-grasp (each finger is 30 mm further out than actual grasp):
    double grasp(double fingerAX, double fingerAY, double fingerBX, double fingerBY, double fingerCX, double fingerCY, bool isPreGrasp);

    //Two-finger grasp (finger B moves away and joint #2 is 90deg):
    double grasp(double fingerAX, double fingerAY, double fingerCX, double fingerCY, bool isPreGrasp);

    bool getIsValidGrasp();
    bool getIsExpectedGrasp();

    Q getQ();

    //Adjust velocity for SDH joints: TODO: delete this, as it is not used!
    void adjustVel(double joint0, double joint1, double joint2, double joint3, double joint4, double joint5, double joint6);

    ~SDHControl();

private:
    //Pointer to SDHDriver-object:
    SDHDriver *sdh;

    Q forTest;

    //Reflects whether connected to hand or not:
    bool connected = false;

    //Sides of the triangle to control grasp {AB, BC, CA}:
    double goalSides[3] = {0.0, 0.0, 0.0};

    //To control if the hand is grasping
    bool isValidGrasp= false;
    bool isExpectedGrasp = false;

    //Calculate fingertip-pos and finger angle in the plane, for finger A and C:
    //Output: {dist, angle}
    vector<double> calcFingertipPos(double visDist, double anAngle);

    //Inverse kinematics for a finger
    //Output: {finger_base, finger_tip}:
    vector<double> calcFingerAngle(double x, double y);

    //Misc. checks (NOTE: all returns TRUE if input is not applicable for finger):
    bool isThereAnan(double a,double b,double c,double d,double e,double f); //NOTE: doesn't check joint #2!
    bool areThereInvalidAngles(Q aQ);
    bool areThereInvalidAngles(vector<double> anglesA, vector<double> anglesB, vector<double> anglesC);
    bool isNotWithinThresh(double diffA, double diffB, double diffC);

    //Uses all above checks to verify if input angles are a valid configuration:
    bool checkSolution(vector<double> anglesA, vector<double> anglesB, vector<double> anglesC);

    //Kinematics for finger (seen as 2-joint "arm", from the side):
    vector<double> calcFingerDist(double angleBase, double angleTop);

    //Control if hand is grasping an object
    //Output: true if the grasp is valid
    void controlGrasp(double goalDistA, double goalDistB, double goalDistC);
    void controlGrasp(double goalDistA, double goalDistC);


    //Control if grasp is the intended grasp, using the triangle calculated in grasp():
    void controlGraspPlacement(double angleAC, double currDistA, double currDistB, double currDistC);

    //Same as above, but for two-finger grasps:
    bool checkSolution(vector<double> anglesA, vector<double> anglesC);
};


#endif
