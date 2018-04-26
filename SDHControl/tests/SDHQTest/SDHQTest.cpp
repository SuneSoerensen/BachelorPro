#include <iostream>
#include "SDHControl.hpp"
#include <string>
#include <unistd.h>


using namespace std;

int main()
{
  Q handQ = Q(7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  Q actualQ = Q(7);

  cout << "Welcome to SDHQTest.cpp" << endl;
  try
  {
    SDHControl hand(0, 115200, 0.5);

    hand.goToInit();

    hand.goToQ(handQ);
    cout << "Target , " << handQ[0]*rad2deg << "," << handQ[1]*rad2deg << "," << handQ[2]*rad2deg << "," << handQ[3]*rad2deg << "," << handQ[4]*rad2deg << "," << handQ[5]*rad2deg << "," << handQ[6]*rad2deg << endl;
    actualQ = hand.getQ();
    cout << "Actual , " << actualQ[0]*rad2deg << "," << actualQ[1]*rad2deg << "," << actualQ[2]*rad2deg << "," << actualQ[3]*rad2deg << "," << actualQ[4]*rad2deg << "," << actualQ[5]*rad2deg << "," << actualQ[6]*rad2deg << endl;

    for(double i = 0.0; i >= -81.0*deg2rad; i -= 10.0*deg2rad)
    {
      handQ[0] = i;
      handQ[3] = i;
      handQ[5] = i;
      hand.goToQ(handQ);
      cout << "Target , " << handQ[0]*rad2deg << "," << handQ[1]*rad2deg << "," << handQ[2]*rad2deg << "," << handQ[3]*rad2deg << "," << handQ[4]*rad2deg << "," << handQ[5]*rad2deg << "," << handQ[6]*rad2deg << endl;
      actualQ = hand.getQ();
      cout << "Actual , " << actualQ[0]*rad2deg << "," << actualQ[1]*rad2deg << "," << actualQ[2]*rad2deg << "," << actualQ[3]*rad2deg << "," << actualQ[4]*rad2deg << "," << actualQ[5]*rad2deg << "," << actualQ[6]*rad2deg << endl;
      cout << "Diff , " << (actualQ[0]-handQ[0])*rad2deg << "," << (actualQ[1]-handQ[1])*rad2deg << "," << (actualQ[2]-handQ[2])*rad2deg << "," << (actualQ[3]-handQ[3])*rad2deg << ",";
      cout << (actualQ[4]-handQ[4])*rad2deg << "," << (actualQ[5]-handQ[5])*rad2deg << "," << (actualQ[6]-handQ[6])*rad2deg << endl;
    }


    for(double i = 80.0*deg2rad; i >= -81.0*deg2rad; i -= 10.0*deg2rad)
    {
      handQ[1] = i;
      handQ[4] = i;
      handQ[6] = i;
      hand.goToQ(handQ);
      cout << "Target , " << handQ[0]*rad2deg << "," << handQ[1]*rad2deg << "," << handQ[2]*rad2deg << "," << handQ[3]*rad2deg << "," << handQ[4]*rad2deg << "," << handQ[5]*rad2deg << "," << handQ[6]*rad2deg << endl;
      actualQ = hand.getQ();
      cout << "Actual , " << actualQ[0]*rad2deg << "," << actualQ[1]*rad2deg << "," << actualQ[2]*rad2deg << "," << actualQ[3]*rad2deg << "," << actualQ[4]*rad2deg << "," << actualQ[5]*rad2deg << "," << actualQ[6]*rad2deg << endl;
      cout << "Diff , " << (actualQ[0]-handQ[0])*rad2deg << "," << (actualQ[1]-handQ[1])*rad2deg << "," << (actualQ[2]-handQ[2])*rad2deg << "," << (actualQ[3]-handQ[3])*rad2deg << ",";
      cout << (actualQ[4]-handQ[4])*rad2deg << "," << (actualQ[5]-handQ[5])*rad2deg << "," << (actualQ[6]-handQ[6])*rad2deg << endl;
    }

    for(double i = 0.0; i <= 91.0*deg2rad; i += 10.0*deg2rad)
    {
      handQ[2] = i;
      hand.goToQ(handQ);
      cout << "Target , " << handQ[0]*rad2deg << "," << handQ[1]*rad2deg << "," << handQ[2]*rad2deg << "," << handQ[3]*rad2deg << "," << handQ[4]*rad2deg << "," << handQ[5]*rad2deg << "," << handQ[6]*rad2deg << endl;
      actualQ = hand.getQ();
      cout << "Actual , " << actualQ[0]*rad2deg << "," << actualQ[1]*rad2deg << "," << actualQ[2]*rad2deg << "," << actualQ[3]*rad2deg << "," << actualQ[4]*rad2deg << "," << actualQ[5]*rad2deg << "," << actualQ[6]*rad2deg << endl;
      cout << "Diff , " << (actualQ[0]-handQ[0])*rad2deg << "," << (actualQ[1]-handQ[1])*rad2deg << "," << (actualQ[2]-handQ[2])*rad2deg << "," << (actualQ[3]-handQ[3])*rad2deg << ",";
      cout << (actualQ[4]-handQ[4])*rad2deg << "," << (actualQ[5]-handQ[5])*rad2deg << "," << (actualQ[6]-handQ[6])*rad2deg << endl;
    }
  }
  catch(const char e[])
  {
    cout << e << endl;
  }




  return 0;
}
