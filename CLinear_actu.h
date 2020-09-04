#pragma once
#include "AXL.h"
#include "AXM.h"
#include <iostream>
#include <conio.h>

class CLinear_actu
{
public:
	CLinear_actu();
	~CLinear_actu();
	void move_actu(int pos, int vel, int accel);
	double get_act_pose();
};

