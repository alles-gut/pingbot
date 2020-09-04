/*
Two-Cam pingpong ball traker stand alone source code
2020 GIST IIT Creative Making Competition
Team. Korean-Azuker

DEPENDENCY OPENCV 3.4.10
		   Visual Studio 2017
*/

#define _AFXDLL

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <string>
#include <string.h>
#include <time.h>
#include <fstream>
#include <ctime>
#include <chrono>
#include <thread>
#include <vector>
#include <stdio.h>
#include <afxmt.h>
#include <windows.h>

#include "Trajectory.h"
#include "SerialPort.hpp"
#include "RobotArm.h"
#include "CLinear_actu.h"


using namespace cv;
using namespace std;

void processing(Mat1d, Mat1d, vector<unsigned __int64>, Mat, Mat, Mat);

const char* portName = "\\\\.\\COM3";
unsigned __int64 timestamp_ms;
Trajectory* Tra;
RobotArm* robo;
SerialPort* arduino;
double b1 = 0, b2 = 0, b3 = 0;
int count_start = 0, empty_ball = 0;
int dtime = 550;
float rotation_speed = 0.5725*2*3.14159;
std::thread arm_thread;
std::thread linear_thread;
std::thread cal;
CLinear_actu* LM_G;

void delay(clock_t n)

{
	clock_t start = clock();
	while (clock() - start < n);
	return;

}

void arm_move() {
	long rot_time = 0;
	rot_time = (long)(1000* atan(-Tra->vX / Tra->vY)/rotation_speed);
	timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	arduino = new SerialPort(portName);
	//std::cout << std::endl << "Connection established at port " << portName << std::endl;
	arduino->writeSerialPort(robo->command, strlen(robo->command));
	arduino->closeSerial();
	delay(100);
	cout << "delay" << Tra->time + (Tra->Te) * 1000 - (long)(timestamp_ms % 100000000) - dtime + rot_time << endl;;
	//delay(Tra->time+(Tra->Te)*1000 - (long)(timestamp_ms % 100000000) - dtime + rot_time);
	/*
	arduino = new SerialPort(portName);
	//std::cout << std::endl << "Connection established at port " << portName << std::endl;
	arduino->writeSerialPort(robo->rotate, strlen(robo->rotate));
	arduino->closeSerial();*/
	delay(3000);

	arduino = new SerialPort(portName);
	//std::cout << std::endl << "Connection established at port " << portName << std::endl;
	arduino->writeSerialPort(robo->base, strlen(robo->base));
	arduino->closeSerial();
	return;
}
void linear_move(int pose, int vel, int accel)
{
	LM_G->move_actu((pose), vel, accel);
	delay(5000);
	LM_G->move_actu(0, vel, accel);
	return;
}
void calculate() {
	Tra->Filter();
	Tra->Calculate();
	//Tra->PrintMat('T');
	//Tra->PrintMat('X');
	//Tra->PrintMat('Y');
	//Tra->PrintMat('Z');
	std::cout << "Xe"<<Tra->Xe << std::endl << "Ze" << Tra->Ze << std::endl << "Te" << Tra->Te << std::endl;
	
	if (_isnan(Tra->Ze)) {

	}
	else {
		robo->Putdata(Tra->Xe, Tra->Ye, Tra->Ze, Tra->vX, Tra->vY, Tra->vZ);
		robo->Calculate();
		robo->Makestring();
		
		linear_thread = std::thread(&linear_move, (-(robo->linearval))/10, 500, 500);
		linear_thread.detach();

		arm_thread = std::thread(&arm_move);
		arm_thread.detach();
		/*linear_thread = std::thread(&linear_move, -(Tra->Xe)*100, 500, 500);
		linear_thread.detach();*/
	}
	return;
	
}

using steady_clock = std::chrono::steady_clock;
using namespace std;
using namespace cv;
vector<Mat1d> positions; //shared memory to contain 2D position of balls
vector<unsigned __int64> times; //shared memory to contain the time image captured
int captured_order_flag = 0; // if cam 1 captured first, 1 cam 2 captured first, 2 else 0
char charCheckForEscKey = 0;
CCriticalSection printer_CS;
int capture_count = 0;

steady_clock::time_point next_frame;

void takeImages(cv::VideoCapture& cap, unsigned fps, int windowname, Mat R, Mat RSub, Mat K_)
{
	assert(fps > 0);
	assert(fps <= 1000);

	int lowH = 0;
	int highH = 81;

	int lowS = 0;
	int highS = 255;

	int lowV = 230;
	int highV = 255;
	/*
	//out setting
	string name_img;
	string name_thr;
	string name_trb;

	// 필요시 활성화
	
	name_img = to_string(windowname);
	name_thr = "th" + name_img;
	name_trb = "trackbar" + name_img;

	cv::namedWindow(name_img, WINDOW_FREERATIO);
	
	cv::namedWindow(name_thr, WINDOW_FREERATIO);
	cv::namedWindow(name_trb, WINDOW_FREERATIO);
	*/

	while (charCheckForEscKey != 27)
	{
		if (positions.size() == 2 && windowname == 1) {
			Mat1d temp1(3,1);
			Mat1d temp2(3,1);
			vector<unsigned __int64> time_temporal(2);
			if (positions.size() < 2 || positions[0].data == NULL || positions[1].data == NULL) {
				next_frame += std::chrono::milliseconds(1000 / fps);
				continue;
			}
			memcpy(temp1.data, positions[0].data, sizeof(unsigned char)*3*1);
			memcpy(temp2.data, positions[1].data, sizeof(unsigned char) * 3 * 1);
			//temporal = positions;
			memcpy(&time_temporal[0], &times[0], sizeof(unsigned __int64));
			memcpy(&time_temporal[1], &times[1], sizeof(unsigned __int64));
			//time_temporal = times;
			positions.clear();
			times.clear();
			captured_order_flag == 0;
			std::thread proc{ processing, temp1, temp2, time_temporal, R, RSub, K_ };
			proc.detach();
		}
		// Capture image from both camera at the same time (important).

		std::this_thread::sleep_until(next_frame);
		if (windowname == 1) {
			next_frame += std::chrono::milliseconds(1000 / fps);
		}

		cv::Mat imgOriginal;
		cv::Mat threshImg;
		cv::Mat hsvImg;

		//capture currently shown image frame
		if (charCheckForEscKey == 's') {
			capture_count++;
			char filename[20];
			sprintf_s(filename, sizeof(filename), "captured_cam%d_%d.png", windowname, capture_count);
			std::cout << "Image saved" << filename << endl;
			cv::imwrite(filename, imgOriginal);
		}

		if (!cap.grab())
			throw std::runtime_error("Can not grab image.");

		if (!cap.retrieve(imgOriginal, 3))
			throw std::runtime_error("Can not retrieve image.");

		// process image here
		vector<vector<Point> > contours;

		//(cv::Rect(cam_width_original_left, cam_height_original_top, cam_width_original_right, cam_height_original_bottom))
		cv::cvtColor(imgOriginal, hsvImg, CV_BGR2HSV);
		cv::GaussianBlur(hsvImg, hsvImg, cv::Size(3, 3), 0);
		cv::inRange(hsvImg, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), threshImg);
		//cv::GaussianBlur(threshImg, threshImg, cv::Size(3, 3), 0);
		//cv::erode(threshImg, threshImg, getStructuringElement(MORPH_RECT, Size(4, 4)));
		//cv::dilate(threshImg, threshImg, getStructuringElement(MORPH_RECT, Size(4, 4)));
		//cv::dilate(threshImg, threshImg, getStructuringElement(MORPH_RECT, Size(4, 4)));
		//cv::erode(threshImg, threshImg, getStructuringElement(MORPH_RECT, Size(4, 4)));
		cv::findContours(threshImg, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

		int nonflag = 0;

		Mat1d center(3, 1);
		double maxArea = 0;
		int maxIndContour = 0;
		if (contours.size() != 0) {
			nonflag = 0;
			for (int indContour = 0; indContour < contours.size(); indContour++) {
				double contourArea = cv::contourArea(contours.at(indContour));
				if (contourArea > maxArea) {
					maxArea = contourArea;
					maxIndContour = indContour;
				}
			}
			cv::Moments contourMoments = moments(contours[maxIndContour], false);
			if (contourMoments.m00 == 0) {
				center(0) = 0;
				center(1) = 0;
			}
			else {
				center(0) = contourMoments.m10 / contourMoments.m00;
				center(1) = contourMoments.m01 / contourMoments.m00;
			}
			center(2) = 1.0;
			circle(imgOriginal, Point2f(contourMoments.m10 / contourMoments.m00, contourMoments.m01 / contourMoments.m00), 2, Scalar(0, 255, 0), 1);
			//std::cout << windowname << " Center of the ball contour : " << center << "\n";

			timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
			//printer_CS.Lock();
			//cout << windowname << " / counted : " << countid << " Timestamp : " << timestamp_ms << " Center of the ball contour : ["
			//	<< center << "] / fps : " << cap.get(cv::CAP_PROP_FPS) << " size : "<< positions.size() << endl;
			//printer_CS.Unlock();

			if (positions.size() == 0) {
				positions.push_back(center);
				times.push_back(timestamp_ms);
				captured_order_flag = windowname;
			}
			else if (positions.size() == 1 && captured_order_flag == windowname) {
				positions.clear();
				times.clear();
				positions.push_back(center);
				times.push_back(timestamp_ms);
			}
			else if (positions.size() == 1 && captured_order_flag != windowname) {
				positions.push_back(center);
				times.push_back(timestamp_ms);
			}
			else {
				continue;
			}

		}
		else {
			nonflag = 1;
			std::cout << "Nothing!! \n";
			empty_ball++;
			if (empty_ball == 30) {
				Tra->resetData();
				robo->Resetdata();
				count_start = 0;
				b1 = 0; b2 = 0; b3 = 0;
				cout << "                                                                                                   초기화했어용" << endl;
			}
		}

		cv::imshow(to_string(windowname), threshImg);

		// 필요시 활성화
		/*
		cv::imshow(name_thr, threshImg);
		
		cv::createTrackbar("LowH", name_trb, &lowH, 255);
		cv::createTrackbar("HighH", name_trb, &highH, 255);

		cv::createTrackbar("LowS", name_trb, &lowS, 255);
		cv::createTrackbar("HighS", name_trb, &highS, 255);

		cv::createTrackbar("LowV", name_trb, &lowV, 255);
		cv::createTrackbar("HighV", name_trb, &highV, 255);
		*/

		charCheckForEscKey = cv::waitKey(1);
	}
}

void processing(Mat1d temp1, Mat1d temp2, vector<unsigned __int64> time_temporal, Mat R, Mat RSub, Mat K_) {
		/*
		vector<Mat> temporal;
		vector<unsigned __int64> time_temporal(2);
		if (positions.size() == 2) {
			temporal = positions;
			memcpy(&time_temporal[0], &times[0], sizeof(unsigned __int64));
			memcpy(&time_temporal[1], &times[1], sizeof(unsigned __int64));
			//time_temporal = times;
			positions.clear();
			times.clear();
			captured_order_flag == 0;
		}*/

		if (abs(time_temporal.back() - time_temporal.front()) <= 10) {
			Mat output;
			Mat centerRe;
			Mat centerSubRe;
			if (captured_order_flag == 1) {
				centerRe = temp1;
				centerSubRe = temp2;
			}
			else if (captured_order_flag == 2) {
				centerRe = temp2;
				centerSubRe = temp1;
			}
			//Mat centerRe = center.reshape(1);
			//Mat centerSubRe = centerSub.reshape(1);
			triangulatePoints(R, RSub, (K_ * centerRe.reshape(1))(cv::Rect(0, 0, 1, 2)), (K_ * centerSubRe.reshape(1))(cv::Rect(0, 0, 1, 2)), output);
			double* p_ = (double*)output.data;
			// timestamp_ms = 절대 시간
			// 단위 : 미터(m)
			// x = p_[0] / p_[3]
			// y = p_[1] / p_[3]
			// z = p_[2] / p_[3]
			//printer_CS.Lock();
			cout << "captured // Timestamp : " << time_temporal.at(0) << " and " << time_temporal.at(1) << " X : " << p_[0] / p_[3] << " Y : " << p_[1] / p_[3] << " Z : " << p_[2] / p_[3] << endl;
			//printer_CS.Unlock();
			
			//필요 코드 추가
			b3 = b2;
			b2 = b1;
			b1 = p_[1] / p_[3];
			empty_ball = 0;
			if (count_start == 0) {
				Tra->PutData((long)(timestamp_ms % 100000000), p_[0] / p_[3] * 1000, p_[1] / p_[3] * 1000, p_[2] / p_[3] * 1000);
				count_start++;
			}
			else if (count_start == 5) {
				Tra->PutData((long)(timestamp_ms % 100000000), p_[0] / p_[3] * 1000, p_[1] / p_[3] * 1000, p_[2] / p_[3] * 1000);
				//cal = std::thread(&calculate);
				//cal.detach();
				count_start++;
			}
			else if (count_start < 5 && count_start>0 && b2 < b1) {
				Tra->PutData((long)(timestamp_ms % 100000000), p_[0] / p_[3] * 1000, p_[1] / p_[3] * 1000, p_[2] / p_[3] * 1000);
				count_start++;
				cout << "                                                                                              데이터 넣었어용" << endl;
			}
			else if (b1 < b2 && b2 < b3) {
				Tra->resetData();
				robo->Resetdata();
				count_start = 0;
				cout << "                                                                                               초기화했어용" << endl;
			}
		}
		return;
}

void trajectorysetting() {
	while (charCheckForEscKey != 27) {
		//cv::namedWindow("timeset", WINDOW_FREERATIO);
		//cv::createTrackbar("dtime", "timeset", &dtime, 700);
		
		cv::namedWindow("trajset", WINDOW_FREERATIO);
		cv::createTrackbar("KD", "trajset", &Tra->Kdd, 30);
		cv::createTrackbar("KC", "trajset", &Tra->Kcc, 100);
		charCheckForEscKey = cv::waitKey(10);
	}
}

int main(int, char**)
{
	try
	{
		LM_G = new CLinear_actu();
		Tra = new Trajectory();
		robo = new RobotArm();

		//cam resolution
		int cam_width = 1280;
		int cam_height = 720;
		int cam_fps = 60;

		// Obtain position of the cameras using solvePnP 
		vector<Point3f> objectPoints;	// 3d world coordinates
		vector<Point2f> imagePoints;	// 2d image coordinates
		vector<Point2f> imagePointsSub;	// 2d image coordinates

		//camera intrinsic parameters
		double fx, fy, cx, cy, k1, k2, p1, p2;
		fx = 583.741 * (double(cam_width) / 1024);
		fy = 584.041 * (double(cam_height) / 576);
		cx = 512.785 * (double(cam_width) / 1024);
		cy = 290.748 * (double(cam_height) / 576);
		k1 = 0.186740;
		k2 = -0.398426;
		p1 = -0.000460;
		p2 = 0.001152;

		double m[] = { fx, 0, cx, 0, fy, cy, 0, 0, 1 };	// intrinsic camera matrix
		Mat K(3, 3, CV_64FC1, m);
		Mat K_ = K.inv();

		//double d[] = { k1, k2, p1, p2 };	// k1,k2: radial distortion, p1,p2: tangential distortion
		double d[] = { 0, 0, 0, 0 };
		Mat distCoeffs(4, 1, CV_64FC1, d);


		// 3D point data and observed 2D points in the image plane
		objectPoints = { Point3f(0,0,0)    ,	            Point3f(0,0.5,0)               , Point3f(0,1.0,0), \
						   Point3f(0.5,0,0)  ,              Point3f(0.5,0.5,0)             , Point3f(0.5,1.0,0) , \
						   Point3f(-0.5,0,0) ,              Point3f(-0.5,0.5,0)            , Point3f(-0.5, 1.0, 0) };

		imagePoints = { Point2f(955.278788 , 393.949091),Point2f(761.868878 , 392.845182),Point2f(578.668350 , 389.750842),
			Point2f(920.732348 , 247.412151),Point2f(753.104651 , 249.500000),Point2f(586.115456 , 245.029795),
			Point2f(1003.732489 , 590.796624),Point2f(783.015385 , 582.093590),Point2f(565.342857 , 579.825714) };

		imagePointsSub = { Point2f(333.779012 , 401.085185),Point2f(525.500000 , 402.091667),Point2f(704.587644 , 403.173851),
			Point2f(275.002404 , 585.935096),Point2f(493.955621 , 583.568047),Point2f(712.102236 , 587.165069),
			Point2f(379.695793 , 262.090615),Point2f(542.283761 , 267.343590),Point2f(699.891566 , 269.562249) };


		// estimate camera pose
		Mat rvec, tvec;	// rotation & translation vectors
		Mat rvecSub, tvecSub;
		solvePnP(objectPoints, imagePoints, K, distCoeffs, rvec, tvec, 0, SOLVEPNP_IPPE);
		solvePnP(objectPoints, imagePointsSub, K, distCoeffs, rvecSub, tvecSub, 0, SOLVEPNP_IPPE);


		// extract rotation & translation matrix
		Mat R;
		Mat RSub;
		Rodrigues(rvec, R);
		Rodrigues(rvecSub, RSub);
		Mat R_inv = R.inv();
		Mat RSub_inv = RSub.inv();

		Mat P = -R_inv * tvec;
		Mat PSub = -RSub_inv * tvecSub;
		double* p = (double*)P.data;
		double* pSub = (double*)PSub.data;


		// camera position
		printf("camera 1 position : x=%lf, y=%lf, z=%lf\n", p[0], p[1], p[2]);
		printf("camera 2 position : x=%lf, y=%lf, z=%lf\n", pSub[0], pSub[1], pSub[2]);
		std::cout << "K" << K << endl;
		std::cout << "K_" << K_ << endl;

		cv::hconcat(R, tvec, R);
		cv::hconcat(RSub, tvecSub, RSub);

		Mat output;

		// Declare video capture class variable, and usable variables 
		cv::VideoCapture capWebcam(0); // +cv::CAP_DSHOW);
		cv::VideoCapture subWebcam(1); // +cv::CAP_DSHOW);

		if (capWebcam.isOpened() == false || subWebcam.isOpened() == false)  //  To check if object was associated to webcam successfully
		{
			std::cout << "error: Webcam connect unsuccessful\n";
			return EXIT_FAILURE;
		}

		capWebcam.set(cv::CAP_PROP_FRAME_WIDTH, cam_width);
		capWebcam.set(cv::CAP_PROP_FRAME_HEIGHT, cam_height);
		capWebcam.set(cv::CAP_PROP_FPS, cam_fps);
		capWebcam.set(cv::CAP_PROP_EXPOSURE, -6);
		subWebcam.set(cv::CAP_PROP_FRAME_WIDTH, cam_width);
		subWebcam.set(cv::CAP_PROP_FRAME_HEIGHT, cam_height);
		subWebcam.set(cv::CAP_PROP_FPS, cam_fps);
		subWebcam.set(cv::CAP_PROP_EXPOSURE, -6);


		// pass the same time point to both threads to synchronize their
		// frame start times
		next_frame = steady_clock::now() + std::chrono::milliseconds(100);

		std::thread capture1{ takeImages, std::ref(capWebcam), 50, 1, R, RSub, K_ }; //capWebcam.get(cv::CAP_PROP_FPS)
		std::thread capture2{ takeImages, std::ref(subWebcam), 50, 2, R, RSub, K_ };
		//std::thread traj{ trajectorysetting };

		capture1.join();
		capture2.join();
		//traj.join();
	}
	catch (std::exception const& e)
	{
		std::cerr << e.what() << '\n';
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}