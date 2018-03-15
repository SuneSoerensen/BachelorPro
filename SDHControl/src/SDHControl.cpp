//
//  SDHControl.cpp
//  SDHControl
//
//  Created by <author> on 08/03/2018.
//
//

#define FINGERPAD_OFF_ANGLE 0*deg2rad

#include "SDHControl.hpp"

SDHControl::SDHControl()
{
}

SDHControl::SDHControl(string canDev)
{
  sdh = new SDHDriver();
  sdh->connect( canDev, 1000000 );

  if(!sdh->isConnected())
    throw("[SDHControl::SDHControl]: Could not connect to hardware!");

  goToInit();
}

void SDHControl::goToInit()
{
  sdh->moveCmd(initQ, true);
}

void SDHControl::grasp(double distA, double distB, double distB)
{

}

double SDHControl::getGraspHeight(double distA, double distB, double distB)
{

}

SDHControl::~SDHControl()
{
}
