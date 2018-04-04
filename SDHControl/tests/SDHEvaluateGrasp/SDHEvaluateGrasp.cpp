#include <iostream>
#include "SDHControl.hpp"
#include <string>

//#include <unistd.h>


using namespace std;

int main()
{
  cout << "welcome to test of graspcontrol" << '\n';
  SDHControl hand;

  double baseAng = -0*deg2rad;
  double topAng =  0*deg2rad;

  vector<double> dists = hand.calcFingerDist(baseAng,topAng);


  cout << "calc dists " << dists[0] << " ; " << dists[1]<< '\n';

  vector<double> angles = hand.calcFingerAngle(dists[0]-FINGEROFFSET,dists[1]);

  cout << "calc angles " << (angles[0]*rad2deg)-90 << " ; " << (angles[1]*rad2deg)<<  '\n';



  return 0;
}
