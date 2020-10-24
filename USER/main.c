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
uint8_t cmd = 0; //�ⲿ��������
unsigned char total_motor_number = 0;

/* Forward Declaration -----------------------------------------------------------*/
static void Log(void);
static void CMD_Handler(uint8_t cmd);

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
	
	TIM2_init(50000-1,9000-1);

	/* �ȴ������ */
	while (1)
	{
		if (cmd)
		{
			CMD_Handler(cmd);
			cmd = 0;
		}
		else
			delay_ms(10);

		delay_ms(1000);
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
		SCA_Homing();

		printf("λ�ù�����Խ�����\r\n");
		break;
	}

	case 4:
	{
		printf("\r\n��������ת�л����ԣ�\r\n");

		/* ���ò��Գ��� ����ת�л� */
		SCA_Exp1();

		printf("����ת�л����Խ�����\r\n");
		break;
	}

	case 5:
	{
		printf("\r\n����ߵ����л����ԣ�\r\n");

		/* ���ò��Գ��� �ߵ����л� */
		SCA_Exp2();

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
		printf("read 1 motor mode \n");
		getActuatorMode(1, Block);
		break;
	}
	case 13:
	{
		printf("write 1 motor to position mode \n");
		activateActuatorMode(1, 3, Block);
		break;
	}
	case 14:
	{
		SCA_Handler_t *pSCA = NULL;
		printf(" motor 1 rotate 0.1 R \n");
		getPosition(1, Block);
		delay_ms(10);

		/* ��ȡ��ID����Ϣ��� */
		pSCA = getInstance(1);
		if (pSCA != NULL)
		{
			float target = pSCA->Position_Real + 0.1;
			setPosition(1, target);
		}
		break;
	}

	default:
		Log();
		break;
	}
}
