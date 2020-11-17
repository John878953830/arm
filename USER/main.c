/**
  ******************************************************************************
  * @��	�� �� main.c
  * @��	�� �� INNFOS Software Team
  * @��	�� �� V1.5.1
  * @��	�� �� 2019.9.10
  * @ժ	Ҫ �� ���������
  ******************************************************************************/

/* Includes ----------------------------------------------------------------------*/
#include "bsp.h"
#include "SCA_API.h"
#include "SCA_APP.h"
#include "main.h"
#include <math.h>
#include "timer.h"

/* Variable defines --------------------------------------------------------------*/
uint8_t cmd = 0;						  //�ⲿ��������
unsigned char total_motor_number = 0;	  //save the total motor number
unsigned char if_error = 0;				  //if the system is alright
unsigned char if_need_lookup_monitor = 0; //if need to look up
unsigned char lookup_counter = 0;

/**
 * @brief timer counter for step
 */
unsigned int tim3_counter = 0;
unsigned int tim4_counter = 0;
unsigned int tim6_counter = 0;
unsigned int tim13_counter = 0;
unsigned int tim14_counter = 0;

//rotation matrix
float r1_0[3][3];
float r2_0[3][3];
float r3_0[3][3];
float r4_0[3][3];

//offset matrix
float porg_1_0[3][1];
float porg_2_1[3][1];
float porg_3_2[3][1];
float porg_4_3[3][1];
MOTOR_PARAMETER mp[5];
float mmk = 0;

float c39_x = 0;
float c39_y = 0;
float c39_z = 0;

float pre_angle[5];
/* Forward Declaration -----------------------------------------------------------*/
static void Log(void);
static void CMD_Handler(uint8_t cmd);

void calculate_r_1_0(float t1)
{
	r1_0[0][0] = cosf(A2R(t1));
	r1_0[0][1] = -sinf(A2R(t1));
	r1_0[0][2] = 0;

	r1_0[1][0] = sinf(A2R(t1));
	r1_0[1][1] = r1_0[0][0];
	r1_0[1][2] = 0;

	r1_0[2][0] = 0;
	r1_0[2][1] = 0;
	r1_0[2][2] = 1;
	return;
}

void calculate_r_2_0(float t1, float t2)
{
	r2_0[0][0] = cosf(A2R(t1)) * cosf(A2R(t2));
	r2_0[0][1] = -cosf(A2R(t1)) * sinf(A2R(t2));
	r2_0[0][2] = -sinf(A2R(t1));

	r2_0[1][0] = sinf(A2R(t1)) * cosf(A2R(t2));
	r2_0[1][1] = -sinf(A2R(t1)) * sinf(A2R(t2));
	r2_0[1][2] = cosf(A2R(t1));

	r2_0[2][0] = -sinf(A2R(t2));
	r2_0[2][1] = -cosf(A2R(t2));
	r2_0[2][2] = 0;
	return;
}

void calculate_r_3_0(float t1, float t2, float t3)
{
	r3_0[0][0] = cosf(A2R(t1)) * cosf(A2R(t2 + t3));
	r3_0[0][1] = -cosf(A2R(t1)) * sinf(A2R(t2 + t3));
	r3_0[0][2] = -sinf(A2R(t1));

	r3_0[1][0] = sinf(A2R(t1)) * cosf(A2R(t2 + t3));
	r3_0[1][1] = -sinf(A2R(t1)) * sinf(A2R(t2 + t3));
	r3_0[1][2] = cosf(A2R(t1));

	r3_0[2][0] = -sinf(A2R(t2 + t3));
	r3_0[2][1] = -cosf(A2R(t2 + t3));
	r3_0[2][2] = 0;
	return;
}

void calculate_r_4_0(float t1, float t2, float t3, float t4)
{
	r4_0[0][0] = cosf(A2R(t1)) * cosf(A2R(t2 + t3 + t4));
	r4_0[0][1] = -cosf(A2R(t1)) * sinf(A2R(t2 + t3 + t4));
	r4_0[0][2] = -sinf(A2R(t1));

	r4_0[1][0] = sinf(A2R(t1)) * cosf(A2R(t2 + t3 + t4));
	r4_0[1][1] = -sinf(A2R(t1)) * sinf(A2R(t2 + t3 + t4));
	r4_0[1][2] = cosf(A2R(t1));

	r4_0[2][0] = -sinf(A2R(t2 + t3 + t4));
	r4_0[2][1] = -cosf(A2R(t2 + t3 + t4));
	r4_0[2][2] = 0;
	return;
}

void motor_parameter_init(void)
{
	//init porg
	porg_1_0[0][0] = 0;
	porg_1_0[1][0] = 0;
	porg_1_0[2][0] = 0;

	porg_2_1[0][0] = 0;
	porg_2_1[1][0] = L1;
	porg_2_1[2][0] = 0;

	porg_3_2[0][0] = L2;
	porg_3_2[1][0] = 0;
	porg_3_2[2][0] = 0;

	porg_4_3[0][0] = L3;
	porg_4_3[1][0] = 0;
	porg_4_3[2][0] = D3;

	//init offset
	mp[1].zero_offset = 0.2937;
	mp[2].zero_offset = 16.0557;
	mp[3].zero_offset = 33.4606;
	mp[4].zero_offset = -15.09;

	//init angle dir
	mp[1].angle_dir = -1;
	mp[2].angle_dir = -1;
	mp[3].angle_dir = 1;
	mp[4].angle_dir = -1;
	return;
}

char calculate_position_xyz(float t1, float t2, float t3, float t4, float *x, float *y, float *z)
{
	float c1 = cosf(A2R(t1));
	float tc23 = t2 + t3;
	float c23 = cosf(A2R(tc23));
	float s1 = sinf(A2R(t1));
	float c2 = cosf(A2R(t2));
	float s23 = t2 + t3;
	*x = L3 * cosf(A2R(t1)) * c23 - D3 * sinf(A2R(t1)) + L2 * cosf(A2R(t1)) * cosf(A2R(t2)) - L1 * sinf(A2R(t1));
	*y = L3 * sinf(A2R(t1)) * c23 + D3 * cosf(A2R(t1)) + L2 * sinf(A2R(t1)) * cosf(A2R(t2)) + L1 * cosf(A2R(t1));
	*z = -L3 * sin(A2R(s23)) - L2 * sin(t2 * Pi / 180.0);
	return 0;
}

char motor_act(size_t id, float step, char dir)
{
	SCA_Handler_t *pSCA = NULL;
	mp[id].id = id;
	mp[id].step = step;
	//check system status
	if (if_error != 0)
	{
		return 1;
	}
	//check id
	if (id <= 0 || id > 4)
	{
		return 2;
	}
	//check step
	if (step > 1)
	{
		return 3;
	}
	//check dir
	if (dir != 0 && dir != 1)
	{
		return 4;
	}
	/* ��ȡ��ID����Ϣ��� */
	pSCA = getInstance(id);
	if (pSCA != NULL)
	{
		if (dir == 0)
		{
			setPosition(id, pSCA->Position_Real + step);
		}
		else
		{
			setPosition(id, pSCA->Position_Real - step);
		}
	}
	else
	{
		return 5;
	}

	return 0;
}

char update_status(void)
{
	char i = 0;
	if (timeout_flag)
	{
		timeout_flag = 0;

		//check id if online
		for (i = 1; i < 5; i++)
		{
			if (isOnline(i, Unblock) == SCA_NoError)
			{
				continue;
			}
			else
			{
				return 10 + i;
			}
		}
		//update position
		for (i = 1; i < 5; i++)
		{
			//if (getPosition(i, Unblock) == SCA_NoError)
			if (requestCVPValue(i, Unblock) == SCA_NoError) //new function, untested
			{
				continue;
			}
			else
			{
				return 20 + i;
			}
		}
	}
	return 0;
}

char motor_act_position(size_t id, float speed, float position)
{
	SCA_Handler_t *pSCA = NULL;
	pSCA = getInstance(id);
	if (pSCA != NULL)
	{
		mp[id].id = id;
		mp[id].speed = speed;
		mp[id].step = mp[id].speed / 1000; //0.1; //fabsf(pSCA->Position_Real - position)/1000;
		mp[id].target = position;
		switch (id)
		{
		case 1:
		{
			TIM_Cmd(TIM3, ENABLE);
			break;
		}
		case 2:
		{
			TIM_Cmd(TIM4, ENABLE);
			break;
		}
		case 3:
		{
			TIM_Cmd(TIM13, ENABLE);
			break;
		}
		case 4:
		{
			TIM_Cmd(TIM14, ENABLE);
			break;
		}
		}
		return 0;
	}
	return 100;
}

char angle_act_position(float t1, float t2, float t3, float t4, float v1, float v2, float v3, float v4)
{
	float tmp_t1 = t1 / 10 * mp[1].angle_dir + mp[1].zero_offset;
	float tmp_t2 = t2 / 10 * mp[2].angle_dir + mp[2].zero_offset;
	float tmp_t3 = t3 / 10 * mp[3].angle_dir + mp[3].zero_offset;
	float tmp_t4 = t4 / 10 * mp[4].angle_dir + mp[4].zero_offset;

	motor_act_position(1, v1, tmp_t1);
	motor_act_position(2, v2, tmp_t2);
	motor_act_position(3, v3, tmp_t3);
	motor_act_position(4, v4, tmp_t4);
	return 0;
}

char position_2_angle(float *t1, float *t2, float *t3, float *t4)
{
	char k = 0;
	SCA_Handler_t *pSCA = NULL;
	for (k = 1; k < 5; k++)
	{

		pSCA = getInstance(k);
		if (pSCA != NULL)
		{

			switch (k)
			{
			case 1:
			{
				*t1 = (pSCA->Position_Real - mp[1].zero_offset) * 10 / mp[1].angle_dir;
				break;
			}
			case 2:
			{
				*t2 = (pSCA->Position_Real - mp[2].zero_offset) * 10 / mp[2].angle_dir;
				break;
			}
			case 3:
			{
				*t3 = (pSCA->Position_Real - mp[3].zero_offset) * 10 / mp[3].angle_dir;
				break;
			}
			case 4:
			{
				*t4 = (pSCA->Position_Real - mp[4].zero_offset) * 10 / mp[4].angle_dir;
				break;
			}
			default:
				break;
			}
		}
		else
		{
			break;
		}
	}
	if (k != 5)
	{
		return 1;
	}
	return 0;
}

char calculate_inverse(float x4, float y4, float z4, float *t1, float *t2, float *t3, float *t4)
{
	float ct1 = 0, ct2 = 0, ct3 = 0, ct4 = 0;
	char res = position_2_angle(&ct1, &ct2, &ct3, &ct4);
	if (res != 0)
	{
		printf("position to angle calculate error\n");
		return 1;
	}
	else
	{
		float tmp_b = L1 + D3;
		int a_dir = 0;
		float tmp_a = sqrtf(x4 * x4 + y4 * y4 - tmp_b * tmp_b);

		float t1_array[5];
		//calculate the theta 1 when tmp_a > 0
		float t1_tmp = asinf((tmp_a * y4 - tmp_b * x4) / (x4 * x4 + y4 * y4));
		t1_array[1] = R2A(t1_tmp) > 180 ? t1_tmp - 360 : R2A(t1_tmp);
		t1_array[2] = (180 - R2A(t1_tmp)) > 180 ? 180 - R2A(t1_tmp) - 360 : 180 - R2A(t1_tmp);
		//printf("tmp t1 1 2 is       %f      %f\n", t1_array[1], t1_array[2]);

		//calculate the theta1 when a < 0
		tmp_a = -tmp_a;
		t1_tmp = asinf((tmp_a * y4 - tmp_b * x4) / (x4 * x4 + y4 * y4));
		t1_array[3] = R2A(t1_tmp) > 180 ? t1_tmp - 360 : R2A(t1_tmp);
		t1_array[4] = (180 - R2A(t1_tmp)) > 180 ? 180 - R2A(t1_tmp) - 360 : 180 - R2A(t1_tmp);
		//printf("tmp t1 3 4 is       %f      %f\n", t1_array[3], t1_array[4]);
		//find the nearest one to t1a
		char i = 1, index = 1;
		float tmpt1_f = 370;
		for (i = 1; i < 5; i++)
		{
			if (__fabs(ct1 - t1_array[i]) < tmpt1_f)
			{
				index = i;
				tmpt1_f = __fabs(ct1 - t1_array[i]);
			}
		}
		float _t1 = t1_array[index];
		if (index == 1 || index == 2)
		{
			a_dir = 1;
			tmp_a = sqrtf(x4 * x4 + y4 * y4 - tmp_b * tmp_b);
		}
		else
		{
			a_dir = -1;
			tmp_a = -sqrtf(x4 * x4 + y4 * y4 - tmp_b * tmp_b);
		}
		printf("theta 1       %f\n", _t1);
		*t1 = _t1;

		float t3_array[3];
		float _t3 = acosf((x4 * x4 + y4 * y4 + z4 * z4 - tmp_b * tmp_b - L2 * L2 - L3 * L3) / 2 / L2 / L3);
		t3_array[1] = R2A(_t3);
		t3_array[2] = 180 + R2A(_t3) > 180 ? -R2A(_t3) : 180 + R2A(_t3);
		//printf("theta 3 1  2  %f, %f\n", t3_array[1], t3_array[2]);

		//find the nearest
		_t3 = __fabs(ct3 - t3_array[1]) < __fabs(ct3 - t3_array[2]) ? t3_array[1] : t3_array[2];
		printf("theta 3       %f\n", _t3);
		*t3 = _t3;

		float mod = sqrtf(L3 * L3 + L2 * L2 + 2 * L2 * L3 * cosf(A2R(_t3)));

		float t2_array[5];
		float tmpt2 = acosf(tmp_a / mod);
		float tmpt2_0 = acosf((L3 * cosf(A2R(_t3)) + L2) / mod);

		t2_array[1] = tmpt2 - tmpt2_0;
		t2_array[2] = tmpt2 + tmpt2_0;
		t2_array[3] = -tmpt2 - tmpt2_0;
		t2_array[4] = -tmpt2 + tmpt2_0;

		printf("t2 %f,   %f   %f   %f\n", R2A(t2_array[1]), R2A(t2_array[2]), R2A(t2_array[3]), R2A(t2_array[4]));
		//find the nearest
		float tmp_t2f = 370;
		for (i = 1; i < 5; i++)
		{
			if (__fabs(ct2 - R2A(t2_array[i])) < tmp_t2f)
			{
				index = i;
				tmp_t2f = __fabs(ct2 - R2A(t2_array[i]));
			}
		}

		printf("t2 is   %f\n", R2A(t2_array[index]));
		*t2 = R2A(t2_array[index]);

		//calculate theta 4
		float t4_array[5];
		t4_array[1] = 0 - *t2 - *t3;
		t4_array[2] = 180 - *t2 - *t3;
		t4_array[3] = 360 - *t2 - *t3;
		//find the nearest
		float tmp_t4f = 370;
		for (i = 1; i < 4; i++)
		{
			if (__fabs(ct4 - t4_array[i]) < tmp_t4f)
			{
				index = i;
				tmp_t4f = __fabs(ct4 - t4_array[i]);
			}
		}
		printf("t4 is   %f\n", t4_array[index]);
		*t4 = t4_array[index];
	}

	return 0;
}
/**
  * @��	��	���������
  * @��	��	��
  * @��	��	��
  */
int main(void)
{
	/* �ײ�������ʼ�� */
	BSP_Init();

	/* �ȴ�ִ�����ȶ� */
	delay_ms(500);

	/* ����1��ӡLOG��Ϣ */
	Log();
	mmk = sinf(A2R(30));

	TIM2_init(100 - 1, 9000 - 1);

	TIM_init(50 - 1, 9000 - 1, 3);
	TIM_init(50 - 1, 9000 - 1, 4);
	TIM_init(50 - 1, 9000 - 1, 13);
	TIM_init(50 - 1, 9000 - 1, 14);
	TIM_init(10 - 1, 9000 - 1, 6);

	//TIM_Cmd(TIM6, ENABLE);

	motor_parameter_init();

	/* �ȴ������ */
	while (1)
	{
		if (cmd)
		{
			CMD_Handler(cmd);
			cmd = 0;
		}
		if_error = update_status();
		if (if_need_lookup_monitor)
		{
			if (lookup_counter > 10)
			{
				if_error = 100;
			}
			else
			{
				lookup_counter++;
				SCA_Lookup();
				SCA_Init();
				delay_ms(1000);
			}
		}
		else
		{
			lookup_counter = 0;
		}
	}
}

/**
  * @��	��	���ڴ�ӡ��ʾ��Ϣ
  * @��	��	��
  * @��	��	��
  */
static void Log()
{
	printf("\r\n��ӭʹ�� INNFOS SCA �������ԣ�\r\n");
	printf("��ϸͨ��Э��μ� INNFOS WIKI ��\r\n");
	printf("���� 1 ��ѯ�����ϵ�ִ����ID ��\r\n");
	printf("���� 2 ʹ��Ĭ��ID��ʼ��SCA������ ��\r\n");
	printf("���� 3 ����λ�ù�����Գ��� ��\r\n");
	printf("���� 4 ��������ת���Գ��� ��\r\n");
	printf("���� 5 ����ߵ��ٲ��Գ��� ��\r\n");
	printf("���� 6 ��ִ�����ػ� ��\r\n");
	printf("���� 7 ��ʾ������Ϣ ��\r\n");
}

/**
  * @��	��	�����������
  * @��	��	cmd�����յ���ָ��
  * @��	��	��
  */
static void CMD_Handler(uint8_t cmd)
{
	switch (cmd)
	{
	case 1:
	{
		printf("\r\nִ����ѯ����\r\n");

		/* ������ѯ���� */
		SCA_Lookup();

		printf("��ѯ������\r\n");
		break;
	}

	case 2:
	{
		printf("\r\nSCA��ʼ����\r\n");

		/* ���ó�ʼ������ */
		SCA_Init();

		/* �ȴ�ִ�����ȶ� */
		delay_ms(500);

		printf("SCA��ʼ��������\r\n");
		break;
	}

	case 3:
	{
		printf("\r\n����λ�ù�����ԣ�\r\n");

		/* ���ò��Գ��� λ�ù��� */
		//SCA_Homing();

		printf("λ�ù�����Խ�����\r\n");
		break;
	}

	case 4:
	{
		printf("\r\n��������ת�л����ԣ�\r\n");

		/* ���ò��Գ��� ����ת�л� */
		//SCA_Exp1();

		printf("����ת�л����Խ�����\r\n");
		break;
	}

	case 5:
	{
		printf("\r\n����ߵ����л����ԣ�\r\n");

		/* ���ò��Գ��� �ߵ����л� */
		//SCA_Exp2();

		printf("�ߵ����л����Խ�����\r\n");
		break;
	}

	case 6:
	{
		printf("\r\nִ�����ػ���\r\n");

		/* �ر�����ִ���� */
		disableAllActuators();

		printf("ִ�����ػ�������\r\n");
		break;
	}

	case 10:
	{
		printf("start all \n");
		SCA_Lookup();
		SCA_Init();
		delay_ms(1000);
		break;
	}
	case 11:
	{
		printf("close all \n");
		disableAllActuators();
		break;
	}
	case 12:
	{
		printf("write motor to position S mode \n");
		activateActuatorMode(1, SCA_Profile_Position_Mode, Block);
		activateActuatorMode(2, SCA_Profile_Position_Mode, Block);
		activateActuatorMode(3, SCA_Profile_Position_Mode, Block);
		activateActuatorMode(4, SCA_Profile_Position_Mode, Block);
		break;
	}
	case 13:
	{
		printf("write motor to position mode \n");
		activateActuatorMode(1, SCA_Position_Mode, Block);
		activateActuatorMode(2, SCA_Position_Mode, Block);
		activateActuatorMode(3, SCA_Position_Mode, Block);
		activateActuatorMode(4, SCA_Position_Mode, Block);
		break;
	}
	case 14:
	{
		break;
	}
	//motor 4 +
	case 15:
	{
		motor_act(4, 0.1, 0);
		break;
	}
	//motor 4 -
	case 16:
	{
		motor_act(4, 0.1, 1);
		break;
	}
	//motor 3 +
	case 17:
	{
		motor_act(3, 0.1, 0);
		break;
	}
	//motor 3 -
	case 18:
	{
		motor_act(3, 0.1, 1);
		break;
	}
	//motor 2 +
	case 19:
	{
		motor_act(2, 0.1, 0);
		break;
	}
	//motor 2 -
	case 20:
	{
		motor_act(2, 0.1, 1);
		break;
	}
	//motor 1 +
	case 21:
	{
		motor_act(1, 0.1, 0);
		break;
	}
	//motor 1 -
	case 22:
	{
		motor_act(1, 0.1, 1);
		break;
	}

	//motor 4 +, speed function
	case 23:
	{
		motor_act_position(4, 100, 36);
		break;
	}
	case 24:
	{
		motor_act_position(4, 100, 0);
		break;
	}

	case 25:
	{
		angle_act_position(10, 0, 0, 0, 10, 0, 0, 0);
		break;
	}
	case 26:
	{
		angle_act_position(0, 0, 0, 0, 10, 0, 0, 0);
		break;
	}
	case 27:
	{
		angle_act_position(-10, 0, 0, 0, 10, 0, 0, 0);
		break;
	}

	case 28:
	{
		angle_act_position(0, 10, 0, 0, 0, 10, 0, 0);
		break;
	}
	case 29:
	{
		angle_act_position(0, 0, 0, 0, 0, 10, 0, 0);
		break;
	}
	case 30:
	{
		angle_act_position(0, -10, 0, 0, 0, 10, 0, 0);
		break;
	}

	case 31:
	{
		angle_act_position(0, 0, 10, 0, 0, 0, 10, 0);
		break;
	}
	case 32:
	{
		angle_act_position(0, 0, 0, 0, 0, 0, 10, 0);
		break;
	}
	case 33:
	{
		angle_act_position(0, 0, -10, 0, 0, 0, 10, 0);
		break;
	}

	case 34:
	{
		angle_act_position(0, 0, 0, 10, 0, 0, 0, 10);
		break;
	}
	case 35:
	{
		angle_act_position(0, 0, 0, 0, 0, 0, 0, 10);
		break;
	}
	case 36:
	{
		angle_act_position(0, 0, 0, -10, 0, 0, 0, 10);
		break;
	}

	case 37:
	{
		float t1, t2, t3, t4;
		char res = position_2_angle(&t1, &t2, &t3, &t4);
		if (res != 0)
		{
			printf(" error while in processing position to angle\n");
		}
		else
		{
			t1 = __fabs(t1) < 0.01 ? 0 : t1;
			t2 = __fabs(t2) < 0.01 ? 0 : t2;
			t3 = __fabs(t3) < 0.01 ? 0 : t3;
			t4 = __fabs(t4) < 0.01 ? 0 : t4;
			float x = 0, y = 0, z = 0;
			printf("%f,%f,%f,%f\n", t1, t2, t3, t4);
			calculate_position_xyz(t1, t2, t3, t4, &x, &y, &z);
			printf("%f, %f, %f\n", x, y, z);
		}
		break;
	}
	case 38:
	{
		float t1, t2, t3, t4;
		char res = position_2_angle(&t1, &t2, &t3, &t4);
		if (res != 0)
		{
			printf(" error while in processing position to angle\n");
		}
		else
		{
			t1 = __fabs(t1) < 0.01 ? 0 : t1;
			t2 = __fabs(t2) < 0.01 ? 0 : t2;
			t3 = __fabs(t3) < 0.01 ? 0 : t3;
			t4 = __fabs(t4) < 0.01 ? 0 : t4;
			float x = 0, y = 0, z = 0;
			calculate_position_xyz(t1, t2, t3, t4, &x, &y, &z);
			printf("position   :  %f,   %f,    %f\n", x, y, z);
			printf("current angle      :  %f,   %f,    %f   %f\n", t1, t2, t3,t4);
			calculate_inverse(x, y, z, &t1, &t2, &t3, &t4);
			printf("target  angle      :  %f,   %f,    %f   %f\n", t1, t2, t3,t4);
		}
		break;
	}
	case 39:
	{
		float t1, t2, t3, t4;
		char res = position_2_angle(&t1, &t2, &t3, &t4);
		if (res != 0)
		{
			printf(" error while in processing position to angle in command 39\n");
		}
		else
		{
			t1 = __fabs(t1) < 0.01 ? 0 : t1;
			t2 = __fabs(t2) < 0.01 ? 0 : t2;
			t3 = __fabs(t3) < 0.01 ? 0 : t3;
			t4 = __fabs(t4) < 0.01 ? 0 : t4;
			calculate_position_xyz(t1, t2, t3, t4, &c39_x, &c39_y, &c39_z);
			printf("start point position   :  %f,   %f,    %f\n", c39_x, c39_y, c39_z);
		}
		break;
	}
	case 40:
	{
		float t1, t2, t3, t4;
		char res = position_2_angle(&t1, &t2, &t3, &t4);
		if (res != 0)
		{
			printf(" error while in processing position to angle\n");
		}
		else
		{
			t1 = __fabs(t1) < 0.01 ? 0 : t1;
			t2 = __fabs(t2) < 0.01 ? 0 : t2;
			t3 = __fabs(t3) < 0.01 ? 0 : t3;
			t4 = __fabs(t4) < 0.01 ? 0 : t4;
			float sv1,sv2,sv3,sv4;
			calculate_inverse(c39_x + 0.5, c39_y, c39_z, &sv1, &sv2, &sv3, &sv4);
			printf("angle      :  %f,   %f,    %f\n", sv1, sv2, sv3);
			//calculate the speed
			float sp1=__fabs(sv1-t1)/0.05;
			float sp2=__fabs(sv2-t2)/0.05;
			float sp3=__fabs(sv3-t3)/0.05;
			float sp4=__fabs(sv4-t4)/0.05;
			sp1=sp1>30?30:sp1;
			sp2=sp2>30?30:sp2;
			sp3=sp3>30?30:sp3;
			printf("speed    %f   %f   %f   %f\n",sp1,sp2,sp3,sp4);
			angle_act_position(sv1, sv2, sv3, sv4, sp1, sp2, sp3, sp4);
			c39_x += 0.5;
		}
		break;
	}
	case 41:
	{
		float t1, t2, t3, t4;
		char res = position_2_angle(&t1, &t2, &t3, &t4);
		if (res != 0)
		{
			printf(" error while in processing position to angle\n");
		}
		else
		{
			t1 = __fabs(t1) < 0.01 ? 0 : t1;
			t2 = __fabs(t2) < 0.01 ? 0 : t2;
			t3 = __fabs(t3) < 0.01 ? 0 : t3;
			t4 = __fabs(t4) < 0.01 ? 0 : t4;
			float sv1,sv2,sv3,sv4;
			calculate_inverse(c39_x - 0.5, c39_y, c39_z, &sv1, &sv2, &sv3, &sv4);
			printf("angle      :  %f,   %f,    %f\n", sv1, sv2, sv3);
			//calculate the speed
			float sp1=__fabs(sv1-t1)/0.05;
			float sp2=__fabs(sv2-t2)/0.05;
			float sp3=__fabs(sv3-t3)/0.05;
			float sp4=__fabs(sv4-t4)/0.05;
			sp1=sp1>30?30:sp1;
			sp2=sp2>30?30:sp2;
			sp3=sp3>30?30:sp3;
			printf("speed    %f   %f   %f   %f\n",sp1,sp2,sp3,sp4);
			angle_act_position(sv1, sv2, sv3, sv4, sp1, sp2, sp3, sp4);
			c39_x -= 0.5;
		}
		break;
	}
	default:
		Log();
		break;
	}
}
