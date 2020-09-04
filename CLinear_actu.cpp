#include "CLinear_actu.h"

char mot_file[] = "C:\\Users\\admin\\Desktop\\ajin20190628.mot"; 
int max_position = 40;

CLinear_actu::CLinear_actu()
{
	DWORD Code = AxlOpen(7);
	if (Code == AXT_RT_SUCCESS)
	{
		//printf("라이브러리가 초기화되었습니다.\n");
		//모션 모듈이 있는지 검사
		DWORD uStatus;
		Code = AxmInfoIsMotionModule(&uStatus);
		if (Code == AXT_RT_SUCCESS)
		{
			//printf("라이브러리가 초기화되었습니다.\n");
			if (uStatus == STATUS_EXIST)
			{
				//printf("라이브러리가 초기화되었습니다.\n");

				AxmMotLoadParaAll(mot_file);

				AxmStatusSetActPos(0, 0.0);
				AxmStatusSetCmdPos(0, 0.0);

				AxmSignalServoOn(0, ENABLE);

				AxmMotSetAbsRelMode(0, 0); //0->abs, 1->Rel
				AxmMotSetProfileMode(0, 3);	//0->symetric trapezode, 1->unsymetric trapezode, 2->reserved, 3->symetric S Curve, 4->unsymetric S Cuve
			}
		}
	}
}


CLinear_actu::~CLinear_actu()
{
	AxmSignalServoOn(0, 0);
	AxlClose();
}

double CLinear_actu::get_act_pose()
{
	double dPos;
	AxmStatusGetActPos(0, &dPos);
	return dPos;
}


void CLinear_actu::move_actu(int pos, int vel, int accel)
{
	if(pos < max_position && pos > -max_position)
		AxmMovePos(0, pos, vel, accel, accel);
	/*
	DWORD uStatus;
	AxmStatusReadInMotion(0, &uStatus);
	while (uStatus)
	{
		AxmStatusReadInMotion(0, &uStatus);
	}*/
}