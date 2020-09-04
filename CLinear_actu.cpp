#include "CLinear_actu.h"

char mot_file[] = "C:\\Users\\admin\\Desktop\\ajin20190628.mot"; 
int max_position = 40;

CLinear_actu::CLinear_actu()
{
	DWORD Code = AxlOpen(7);
	if (Code == AXT_RT_SUCCESS)
	{
		//printf("���̺귯���� �ʱ�ȭ�Ǿ����ϴ�.\n");
		//��� ����� �ִ��� �˻�
		DWORD uStatus;
		Code = AxmInfoIsMotionModule(&uStatus);
		if (Code == AXT_RT_SUCCESS)
		{
			//printf("���̺귯���� �ʱ�ȭ�Ǿ����ϴ�.\n");
			if (uStatus == STATUS_EXIST)
			{
				//printf("���̺귯���� �ʱ�ȭ�Ǿ����ϴ�.\n");

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