#include "RobotArm.h"
#include <iostream>

RobotArm::RobotArm() {
}

RobotArm::~RobotArm() {
}
void RobotArm::Putdata(float ix, float iy, float iz, float ivx, float ivy, float ivz) {
	x = ix; y = iy; z = iz; vx = ivx; vy = ivy; vz = ivz;
}
void RobotArm::Resetdata() {
	x = 0, y = 0, z = 0, vx = 0, vy = 0, vz = 0;
	th1 = 0, th2 = 0, th3 = 0, th4 = 0;
}
void RobotArm::Calculate() {
	inverse((int)(x * 1000), (int)(z * 1000));
	//if (vz >= 0) {
		th4 = 0;
	//}
	//else {
		//th4 = (float)atan(-vz / vy);
	//}
	printf("theta 1 : %f\n", th1 * 180 / 3.14);
	printf("theta 2 : %f\n", th2 * 180 / 3.14);
	printf("theta 3 : %f\n", th3 * 180 / 3.14);
	printf("theta 4 : %f\n", th4 * 180 / 3.14);
}

void RobotArm::inverse(int sx, int sy)
{	//mm단위로 계산
	float k = 0; int k1 = 0, k2 = 0, xx = 0;
	sy = sy - ei + 80;
	cout << "sy" << sy << endl;
	if (sy > 400 || sy < 0) {
		th1 = -0.54;
		th2 = 0.590;
		th3 = -0.04;
		th4 = 0.000;
		return;
	}
	else if(sy<350){
		xx = 200;
		linearval = sx - 200 + xx;
		cout << "linearval" << linearval << endl;
		k = sqrt(pow((xx * ai - sy * bi), 2) + pow((sy * ai + xx * bi), 2));
		k1 = (sy * ai) + (xx * bi);
		k2 = ai * ai + bi * bi - ci * ci + pow(xx, 2) + pow(sy, 2);
		th1 = asin(k2 / (2 * k)) - asin(k1 / k);
		th2 = asin((ai * cos(th1) - bi * sin(th1) - sy) / ci) - th1;
		th3 = -th1 - th2;
		return;
	}
	else {
		xx = 150;
		linearval = sx - 200 + xx;
		cout << "linearval" << linearval << endl;
		k = sqrt(pow((xx * ai - sy * bi), 2) + pow((sy * ai + xx * bi), 2));
		k1 = (sy * ai) + (xx * bi);
		k2 = ai * ai + bi * bi - ci * ci + pow(xx, 2) + pow(sy, 2);
		th1 = asin(k2 / (2 * k)) - asin(k1 / k);
		th2 = asin((ai * cos(th1) - bi * sin(th1) - sy) / ci) - th1;
		th3 = -th1 - th2;
		return;
	}
}

void RobotArm::Makestring() {
	com = "j," + std::to_string(th1).substr(0, 5) + "," + std::to_string(th2).substr(0, 5) + "," + std::to_string(th3).substr(0, 5) + "," + std::to_string(th4).substr(0, 5) + "\n";
	//com = "j,-0.44,-0.19,0.628," + std::to_string(th4).substr(0, 5) + "\n";
	cout << com << endl;
	strcpy_s(command, com.c_str());
}

/*tuple<double, double, double> RobotArm::motor(double Vx0, double Vy0, double Vz0, double Vx1, double Vz1) {
	double Ke = 0.6, Kv = 0.1;
	double a = 0, b = 0, c = 0, d = 0, B = 0, D = 0;
	double Vy1 = 0, alpha = 0, V = 0, beta = 0;
	Vy1 = Vy0 * Vx1 / Vx0;
	alpha = atan(Vy0 / Vx0);
	a = cos(alpha) * (Vx1 + Ke * Vx0) + sin(alpha) * (Vy1 + Ke * Vy0);
	b = cos(alpha) * (Ke + 1);
	c = cos(alpha) * (Vx1 + (Kv - 1) * Vx0) + sin(alpha) * (Vy1 + (Kv - 1) * Vy0);
	d = cos(alpha) * Kv;
	B = Vz1 + Ke * Vz0;
	D = Vz1 + (Kv - 1) * Vz0;
	V = (b * c + a * d - pow((pow((b * c + a * d), 2) - 4 * b * d * (a * c + B * D)), 0.5)) / (2 * b * d);
	beta = atan((a - b * V) / B);
	printf("%f\n", V);
	printf("alpha : %f\n", alpha);
	printf("theta 4 : %f\n", beta);
	return tuple<double, double, double>(V, alpha, beta);
}*/


