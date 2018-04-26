#ifndef URControl_hpp
#define URControl_hpp

#include <stdio.h>
#include <string>
#include <rw/rw.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include "settings.hpp"


USE_ROBWORK_NAMESPACE

using namespace robwork;
using namespace rwhw;

using namespace std;

class URControl
{
public:
    URControl();
    URControl(string anIp, int aPort);

    void connect();
    void disconnect();
    void sendScript(string fileName);

    void moveToInit();
    void moveToHome();
    void moveRel(double anX, double aY, double aZ);

    void setWristAngle(double anAngle);

    bool checkBounds(double x, double y, double z);

    void printcurrToolPos();

    ~URControl();
private:
  URPrimaryInterface ur;
  int port;
  string ip;
  bool haveBeenToInit;
  double currToolPos[3] = {INIT_POS_X, INIT_POS_Y, INIT_POS_Z};
  int numOfMoves = 0; //DEBUG: for saving all move scripts

  int state = STATE_OTHER;

};
#endif
