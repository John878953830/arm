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

MOTOR_PARAMETER mp[5];

/* Forward Declaration -----------------------------------------------------------*/
static void Log(void);
static void CMD_Handler(uint8_t cmd);
char motor_act(size_t id, float step, char dir)
{
	SCA_Handler_t *pSCA = NULL;
	mp[id].id=id;
	mp[id].step=step;
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
		mp[id].step = mp[id].speed/100;//0.1; //fabsf(pSCA->Position_Real - position)/1000;
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

	TIM2_init(100 - 1, 9000 - 1);

	TIM_init(10 - 1, 9000 - 1, 3);
	TIM_init(10 - 1, 9000 - 1, 4);
	TIM_init(10 - 1, 9000 - 1, 13);
	TIM_init(10 - 1, 9000 - 1, 14);
	TIM_init(5 - 1, 9000 - 1, 6);
	
	TIM_Cmd(TIM6,ENABLE);

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
	default:
		Log();
		break;
	}
}
