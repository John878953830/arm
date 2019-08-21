/**
  ******************************************************************************
  * @��	�� �� SCA_APP.c
  * @��	�� �� INNFOS Software Team
  * @��	�� �� V1.5.0
  * @��	�� �� 2019.8.20
  * @ժ	Ҫ �� SCA ���Գ���
  ******************************************************************************/ 
/* Update log --------------------------------------------------------------------*/
//V1.1.0 2019.8.5 ���Գ����޸�Ϊ����API�ӿ�
//V1.5.0 2019.8.20 ���Գ����޸�Ϊ����API�ӿڣ����ĳ�ʼ����ʽ

/* Includes ----------------------------------------------------------------------*/
#include "bsp.h"
#include "SCA_APP.h"
#include "SCA_API.h"

/* Variable defines --------------------------------------------------------------*/
SCA_Handler_t* pSCA_ID1 = NULL;		//��дָ�룬�����ڻ�ȡִ�������������FAST�ͺ���
SCA_Handler_t* pSCA_ID2 = NULL;		

/* CAN�˿���Ϣ���壬���ڰ�SCA�����ʵ�ֶ�˿ڿ��ơ���ֲʱ����ʵ���������壬Ĭ��ʹ��2�� */
CAN_Handler_t CAN_Port1,CAN_Port2;

/* Funcation defines -------------------------------------------------------------*/

/**
  * @��	��	��������ʼ��
  * @��	��	��
  * @��	��	��
  */
void SCA_Init()
{
	/* ��ʼ��CAN�˿ڲ��� */
	CAN_Port1.CanPort = 1;			//��Ƕ˿ں�
	CAN_Port1.Retry = 2;			//ʧ���ط�����
	CAN_Port1.Send = CAN1_Send_Msg;	//CAN1�˿ڷ��ͺ���
	
	CAN_Port2.CanPort = 2;			
	CAN_Port2.Retry = 2;			
	CAN_Port2.Send = CAN2_Send_Msg;	//CAN2�˿ڷ��ͺ���
	
	/* װ��ִ������ID����ʹ�õ�CAN�˿ں� */
	setupActuators( 1, &CAN_Port1);	//ID1 ��CAN1
	setupActuators( 2, &CAN_Port2);	//ID2 ��CAN2
	
	/* ��ȡID1��2�Ĳ������ */
	pSCA_ID1 = getInstance(1);
	pSCA_ID2 = getInstance(2);
	
	/* ��������ִ���� */
	enableAllActuators();
}

/**
  * @��	��	λ�ù���
  * @��	��	��
  * @��	��	��
  */
void SCA_Homing()
{
	/* δ����ֱ���˳� */
	if(isEnable(0x01) == Actr_Disable)	return;
	if(isEnable(0x02) == Actr_Disable)	return;
	
	/* �л�ִ��������ģʽ������λ��ģʽ */
	activateActuatorMode(0x01,SCA_Profile_Position_Mode,Block);
	activateActuatorMode(0x02,SCA_Profile_Position_Mode,Block);
	
	/* ���� 1��ִ���� */
	setPosition(0x01,0);
	
	/* �ȴ�����ɹ� */
	do
	{
		getPosition(0x01,Unblock);
		delay_ms(100);
	}
	while((pSCA_ID1->Position_Real > 0.1f)||(pSCA_ID1->Position_Real < -0.1f));
	
	/* ���� 2��ִ����*/
	setPosition(0x02,0);
	
	/* �ȴ�����ɹ� */
	do
	{
		getPosition(0x02,Unblock);
		delay_ms(100);
	}
	while((pSCA_ID2->Position_Real > 0.1f)||(pSCA_ID2->Position_Real < -0.1f));
}

/**
  * @��	��	����ת�л�����
  * @��	��	��
  * @��	��	��
  */
void SCA_Exp1()
{
	/* δ����ֱ���˳� */
	if(isEnable(0x01) == Actr_Disable)	return;
	if(isEnable(0x02) == Actr_Disable)	return;
	
	/* ���� */
	SCA_Homing();
	
	/* ��������ת */
	setPosition(0x01,2);			//��ͨ������ID���� 0x01��ִ����
	setPositionFast(pSCA_ID2,2);	//FAST�ͺ�����ָ����ʽ����0x02��ִ����
	delay_ms(1000);
	
	setPosition(0x01,0);
	setPositionFast(pSCA_ID2,0);
	delay_ms(1000);
	
	setPosition(0x01,2);
	setPositionFast(pSCA_ID2,2);
	delay_ms(1000);
	
	setPosition(0x01,0);
	setPositionFast(pSCA_ID2,0);
	delay_ms(1000);
} 

/**
  * @��	��	�ߵ����л�
  * @��	��	��
  * @��	��	��
  */
void SCA_Exp2()
{
	/* δ����ֱ���˳� */
	if(isEnable(0x01) == Actr_Disable)	return;
	if(isEnable(0x02) == Actr_Disable)	return;
	
	/* �л�ִ��������ģʽ�������ٶ�ģʽ */
	activateActuatorMode(0x01,SCA_Profile_Velocity_Mode,Block);
	activateActuatorMode(0x02,SCA_Profile_Velocity_Mode,Block);
	
	/* ��������ת */
	setVelocity(0x01,300);
	setVelocityFast(pSCA_ID2,300);
	delay_ms(1000);
	
	setVelocity(0x01,600);
	setVelocityFast(pSCA_ID2,600);
	delay_ms(1000);
	
	setVelocity(0x01,300);
	setVelocityFast(pSCA_ID2,300);
	delay_ms(1000);
	
	setVelocity(0x01,600);
	setVelocityFast(pSCA_ID2,600);
	delay_ms(1000);
	
	/* ֹͣ */
	setVelocity(0x01,0);
	setVelocityFast(pSCA_ID2,0);
}



