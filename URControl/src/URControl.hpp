//
//  URControl.hpp
//  tutorial
//
//  Created by SLS on 21/02/2018.
//
//

#ifndef URControl_hpp
#define URControl_hpp

#include <stdio.h>
#include <string>
#include <rw/rw.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>


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
    //void moveAbs(double anX, double aY, double aZ); //TODO: delete if not used

    void setWristAngle(double anAngle);

    bool checkBounds(double x, double y, double z);

    //void updateCurrToolPos(); //TODO: delete eventually

    //Helpful debug function:
    void printcurrToolPos();

    ~URControl();
private:
  /*rwhw::*/URPrimaryInterface ur;
  int port;
  string ip;
  bool haveBeenToInit;
  double currToolPos[6] = {INIT_POS_X, INIT_POS_Y, INIT_POS_Z, INIT_POS_RX, INIT_POS_RY, INIT_POS_RZ};
  int numOfMoves = 0; //DEBUG: for saving all move scripts

  int state;

};


#endif /* URControl_hpp */
