#pragma once
#include <Eigen/QR>
#include <iostream>


class Trajectory {
private:
	double Yd = 2.7, g = 9.8;
	int num = 6, stride_num = 3;

public:
	int Kdd = 17, Kcc = 93;
	double Kd = double(Kdd)/100.0, Kc = double(Kcc)/100.0, Ka = sqrt(Kd * g);

	Eigen::MatrixXd X = Eigen::MatrixXd(num - stride_num + 1,1);
	Eigen::MatrixXd Y = Eigen::MatrixXd(num - stride_num + 1, 1);
	Eigen::MatrixXd Z = Eigen::MatrixXd(num - stride_num + 1, 1);
	Eigen::MatrixXd T = Eigen::MatrixXd(num-stride_num + 1, 3);
	Eigen::MatrixXd TT = Eigen::MatrixXd(num - stride_num + 1, 2);
	Eigen::MatrixXd T_raw = Eigen::MatrixXd(num, 1);
	Eigen::MatrixXd X_raw = Eigen::MatrixXd(num, 1);
	Eigen::MatrixXd Y_raw = Eigen::MatrixXd(num, 1);
	Eigen::MatrixXd Z_raw = Eigen::MatrixXd(num, 1);
	Eigen::MatrixXd AX = Eigen::MatrixXd(2, 1);
	Eigen::MatrixXd AY = Eigen::MatrixXd(2, 1);
	Eigen::MatrixXd AZ = Eigen::MatrixXd(3, 1);
	double X0, Y0, Z0, vX0, vY0, vZ0;
	double Xe = 0, Ye = 0, Ze = 0, Te = 0;
	double vX = 0, vY = 0, vZ = 0 , vh = 0;
	double Th = 0, Zh = 0, Zh2 = 0, Kb = 0, Kb2 = 0, Tp = 0, vZp = 0, Th2 = 0;
	char ch;
	int count = 0;
	long time = 0;

	Trajectory();
	~Trajectory();
	void PutData(long t, double x, double y, double z);
	void Calculate();
	void PrintMat(char ch);
	void resetData();
	void Filter();
};