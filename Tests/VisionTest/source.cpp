#include <iostream>
#include "Vision.hpp"
#include <string>

using namespace std;

int main()
{
	try
	{
		cout << "================" << endl;
		cout << "   VisionTest" << endl;
		cout << "================" << endl;
		cout << endl;

		Vision test;
		string run = "y";
		string mode;

		while (1)
		{
			cout << "select between:" << endl;
			cout << "(0): misc." << endl;
			cout << "(1): Calib()" << endl;
			cout << "(2): GetObjCoords()" << endl;
			cout << "(3): CalcGrasp()" << endl;
			cout << "type input: ";
			cin >> mode;
			cout << endl;

			if (mode == "0")
			{
				
			}
			else if (mode == "1")
			{
				test.Calib();
			}
			else if (mode == "2")
			{
				Coords objCoords = test.GetObjCoords();
				cout << "real coords: " << objCoords.x << ";" << objCoords.y << endl;
				cout << endl;
			}
			else if (mode == "3")
			{
				test.CalcGrasp();
				Coords focus = test.GetGraspFocus();
				vector<Coords> points = test.GetGraspPoints();
				cout << "grasp focus: " << focus.x << ";" << focus.y << endl;
				cout << "grasp distance from COM: " << test.GetDistFromCOM() << endl;
				cout << "grasp points: ";
				for (int i = 0; i < points.size(); i++)
					cout << "(" << points[i].x << "; " << points[i].y << ")";
				cout << endl;
			}
			else
			{
				cout << "invalid input!" << endl;
			}

			cout << "Do again?" << endl;
			cout << "select between:" << endl;
			cout << "(y): yes" << endl;
			cout << "(whatever): no" << endl;
			cout << "type input: ";
			cin >> run;
			if (run != "y")
			{
				cout << "stopping..." << endl;
				break;
			}
			cout << endl;
		}
	}
	catch (const char errMess[])
	{
		cout << errMess << endl;
	}

	return 0;
}

