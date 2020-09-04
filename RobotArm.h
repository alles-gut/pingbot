#pragma once
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <string>
#include <algorithm>

using namespace std;

class RobotArm {
private:
	int ai = 170, bi = 32, ci = 261, di = 182, ei = 236;
	int set = 380;
public:
	float x = 0, y = 0, z = 0, vx = 0, vy = 0, vz = 0;
	float th1 = 0, th2 = 0, th3 = 0, th4 = 0;
	int linearval = 0;
	string com;
	char command[100];
	char base[100] = "j,-0.54,0.590,-0.04,0.000\n";
	char rotate[100] = "v,150,250\n";
	RobotArm();
	~RobotArm();
	//void motor(double Vx0, double Vy0, double Vz0, double Vx1, double Vz1);
	void inverse(int sx, int sy);
	void Makestring();
	void Putdata(float ix, float iy, float iz, float ivx, float ivy, float ivz);
	void Resetdata();
	void Calculate();
};