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
    void moveAbs(double anX, double aY, double aZ);

    bool checkBounds(double x, double y, double z);

    void updateCurrToolPos();

    //Helpful debug function:
    void printcurrToolPos();

    ~URControl();
private:
  rwhw::URPrimaryInterface ur;
  int port;
  string ip;
  bool haveBeenToInit;
  double currToolPos[6] = {-0.1087, -0.48537, 0.43305, 0.0, -3.1409, 0.0};
  int numOfMoves = 0; //DEBUG: for saving all move scripts

};


#endif /* URControl_hpp */
