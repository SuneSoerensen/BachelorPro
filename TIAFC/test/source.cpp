#include <iostream>
#include "Vision.hpp"
#include <string>

using namespace std;



int main()
{
	try
	{
		Vision test;
		string run = "y";
		string mode;

		while (run == "y")
		{
			cout << "select ''Calib'', ''GetCoords'' or ''GraspRegs'' (1, 2, 3): ";
			cin >> mode;

			if (mode == "1")
			{
				test.Calib();
			}
			if (mode == "2")
			{
				Coords objCoords = test.GetObjCoords();
				cout << "real coords: " << objCoords.x << ";" << objCoords.y << endl;
			}
			if (mode == "3")
			{
				test.FindGraspRegs(test.graspRegsList, test.contourImage, test.contourMatrix);
			}

			/*Coords vecA;
			cout << "vecA:" << endl;
			cout << "x = ";
			cin >> vecA.x;
			cout << "y = ";
			cin >> vecA.y;
			cout << endl;

			Coords vecB;
			cout << "vecB:" << endl;
			cout << "x = ";
			cin >> vecB.x;
			cout << "y = ";
			cin >> vecB.y;
			cout << endl;

			cout << test.CalcAngle(vecA, vecB) << endl;
			cout << endl;*/

			cout << "Do again?" << endl;
			cout << "(y / n): ";
			cin >> run;
		}
	}
	catch (const char errMess[])
	{
		cout << errMess << endl;
	}

	return 0;
}

