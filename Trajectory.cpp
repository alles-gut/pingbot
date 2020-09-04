#include "Trajectory.h"
#include <iostream>
#include <math.h>

Trajectory::Trajectory() {
}

Trajectory::~Trajectory() {
}

void Trajectory::PutData(long t, double x, double y, double z) {

	if (count == 0) {
		time = t;
		T_raw(0, 0) = 0;
		X_raw(0, 0) = x; Y_raw(0, 0) = y; Z_raw(0, 0) = z;
		count++;
	}
	else if (count < num) {
		long rt = 0;
		rt = t - time;
		//rrt = (double)rt;
		T_raw(count, 0) = rt;
		X_raw(count, 0) = x; Y_raw(count, 0) = y; Z_raw(count, 0) = z;
		count++;
	}
	return;
}

void Trajectory::Filter() {
	long T_sum = 0;
	double X_sum = 0, Y_sum = 0, Z_sum = 0;
	for (int i = 0; i < num - stride_num + 1; i++) {
		for (int j = 0; j < stride_num; j++) {
			T_sum += T_raw(i + j, 0);
			X_sum += X_raw(i + j, 0);
			Y_sum += Y_raw(i + j, 0);
			Z_sum += Z_raw(i + j, 0);
		}
		TT(i, 0) = T_sum / stride_num;
		T(i, 1) = T_sum / stride_num; T(i, 0) = pow(T_sum / stride_num, 2);
		X(i,0) = X_sum / stride_num; Y(i, 0) = Y_sum / stride_num; Z(i, 0) = Z_sum / stride_num;
		T_sum = 0;
		X_sum = 0;
		Y_sum = 0;
		Z_sum = 0;
	}

}

void Trajectory::Calculate() {
	for (int i = 0; i < num - stride_num + 1; i++) {
		T(i, 2) = 1;
		TT(i, 1) = 1;
	}
	Eigen::MatrixXd pinvT = T.completeOrthogonalDecomposition().pseudoInverse();
	Eigen::MatrixXd pinvTT = TT.completeOrthogonalDecomposition().pseudoInverse();
	AX = pinvTT * X;
	AY = pinvTT * Y;
	AZ = pinvT * Z;
	X0 = AX(1, 0) / 1000; Y0 = AY(1, 0) / 1000; Z0 = AZ(2, 0) / 1000;
	vX0 = AX(0, 0); vY0 = AY(0, 0); vZ0 = AZ(1, 0);
	Ye = Yd;
	Te = (exp(Kd*(Ye - Y0)) - 1) / (Kd*vY0);
	//Te = (Ye - Y0) / vY0;
	//Th = (vZ0 + sqrt(vZ0 * vZ0 + 2 * g * Z0)) / g;
	//vh = Kc * (g * Th - vZ0);
	Xe = (X0 + log(Kd * vX0 * Te + 1) / Kd);
	//Xe = X0 + vX0 * Te;
	//Ze = vh * (Te - Th) - 0.5 * g * (Te-Th) * (Te-Th);
	vX = vX0 / (Kd*vX0*Te + 1);
	vY = vY0 / (Kd*vY0*Te + 1);
	//vX = vX0;
	//vY = vY0;
	//vZ = vh - g * (Te - Th);
	if (vZ0 > 0) {
		Kb = atan(vZ0 * sqrt(Kd / g));
		Th = Kb / Ka;
		Zh = Z0 - log(abs(cos(Kb))) / Kd;
		Tp = Th + acosh(exp(Kd * Zh)) / Ka;
		vZp = sqrt(g / Kd) * tanh((Tp - Th) * Ka);
		Kb2 = atan(Kc * vZp * sqrt(Kd / g));
		Th2 = Tp + Kb2 / Ka;

		if (Th2 > Te) {
			Ze = -log(abs(cos(Kb2)) / abs(cos(Kb2 - Ka * (Te - Tp)))) / Kd;
			vZ = sqrt(g / Kd) * tan(Kb2 - Ka * (Te - Tp));
		}
		else {
			Zh2 = -log(abs(cos(Kb2))) / Kd;
			Ze = Zh2 - log(cosh(Ka * (Te - Th2))) / Kd;
			vZ = -sqrt(g / Kd) * tanh(Ka * (Te - Th2));
		}

	}
	else {
		Kb = atanh(-vZ0 * sqrt(Kd / g));
		Tp = (acosh(cosh(Kb) * exp(Kd * Z0)) - Kb) / Ka;
		vZp = sqrt(g / Kb) * tanh(Ka * Tp + Kb);
		Kb2 = atan(Kc * vZp * sqrt(Kd / g));
		Th2 = Tp + Kb2 / Ka;

		if (Th2 > Te) {
			Ze = - log(abs(cos(Kb2) / cos(Kb2 - Ka * (Te - Tp)))) / Kd;
			vZ = sqrt(g / Kd) * tan(Kb2 - Ka * (Te - Tp));
		}
		else {
			Zh2 = - log(abs(cos(Kb2))) / Kd;
			Ze = Zh2 - log(cosh(Ka * (Te - Th2))) / Kd;
			vZ = -sqrt(g / Kd) * tanh(Ka * (Te - Th2));
		}
	}
	std::cout << "X0" << X0 << std::endl;
	std::cout << "vX0" << vX0 << std::endl;

}

void Trajectory::PrintMat(char ch) {
	if (ch == 'T') {
		std::cout <<  "T :" << std::endl << T << std::endl;
	}
	else if (ch == 'TT') {
		std::cout << "TT :" << std::endl << TT << std::endl;
	}
	else if (ch == 'X') {
		std::cout << "X :" << std::endl << X << std::endl;
	}
	else if (ch == 'Y') {
		std::cout << "Y :" << std::endl << Y << std::endl;
	}
	else if (ch == 'Z') {
		std::cout << "Z :" << std::endl << Z << std::endl;
	}
}

void Trajectory::resetData() {
	count = 0; time = 0;
	return;
}
