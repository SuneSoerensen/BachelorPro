#pragma once
#include <cmath>

using namespace std;



class Coords //Coordinates (2D)
{
public:
	//==================
	//   Constructors
	//==================
	Coords();
	Coords(int anX, int aY);
	Coords(int aScalar);

	//====================
	//   "set"-function
	//====================
	void Set(int anX, int aY);
	void Set(int aScalar);

	//=====================
	//   math-functions
	//=====================
	bool Eq(Coords aCoords);
	bool Eq(int aScalar);

	double Length();
	int Dot(Coords aCoords);

	//-------------
	// With Coords
	//-------------
	Coords Add(Coords aCoords);
	Coords Sub(Coords aCoords);
	Coords Mul(Coords aCoords);
	Coords Div(Coords aCoords);

	//-------------
	// With scalar
	//-------------
	Coords Add(int aScalar);
	Coords Sub(int aScalar);
	Coords Mul(int aScalar);
	Coords Div(int aScalar);

	//===================
	//   Deconstructor
	//===================
	~Coords();

	//================
	//   Attributes
	//================
	int x;
	int y;
};

