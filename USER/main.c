/**
  ******************************************************************************
  * @文	件 ： main.c
  * @作	者 ： INNFOS Software Team
  * @版	本 ： V1.5.1
  * @日	期 ： 2019.9.10
  * @摘	要 ： 主程序入口
  ******************************************************************************/

/* Includes ----------------------------------------------------------------------*/
#include "bsp.h"
#include "SCA_API.h"
#include "SCA_APP.h"
#include "main.h"
#include <math.h>
#include "timer.h"

/* Variable defines --------------------------------------------------------------*/
uint8_t cmd = 0;						  //外部控制命令
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
	mp[4].zero_offset = 0.8805;

	//init angle dir
	mp[1].angle_dir = -1;
	mp[2].angle_dir = -1;
	mp[3].angle_dir = 1;
	mp[4].angle_dir = -1;
	return;
}

char calculate_position_xyz(float t1, float t2, float t3, float t4, float *x, float *y, float *z)
{
	*x = L3 * cosf(A2R(t1)) * cosf(A2R(t2 + t3)) - D3 * sinf(A2R(t1)) + L2 * cosf(A2R(t1)) * cosf(A2R(t2)) - L1 * sinf(A2R(t1));
	*y = L3 * sinf(A2R(t1)) * cosf(A2R(t2 + t3)) + D3 * cosf(A2R(t1)) + L2 * sinf(A2R(t1)) * cosf(A2R(t2)) + L1 * cosf(A2R(t1));
	*z = -L3 * sinf(A2R(t2 + t3)) - L2 * sinf(A2R(t2));
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
	/* 获取该ID的信息句柄 */
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
/**
  * @功	能	主程序入口
  * @参	数	无
  * @返	回	无
  */
int main(void)
{
	/* 底层驱动初始化 */
	BSP_Init();

	/* 等待执行器稳定 */
	delay_ms(500);

	/* 串口1打印LOG信息 */
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

	/* 等待命令传入 */
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
  * @功	能	串口打印提示信息
  * @参	数	无
  * @返	回	无
  */
static void Log()
{
	printf("\r\n欢迎使用 INNFOS SCA 驱动测试！\r\n");
	printf("详细通信协议参见 INNFOS WIKI ！\r\n");
	printf("发送 1 轮询总线上的执行器ID ！\r\n");
	printf("发送 2 使用默认ID初始化SCA控制器 ！\r\n");
	printf("发送 3 进入位置归零测试程序 ！\r\n");
	printf("发送 4 进入正反转测试程序 ！\r\n");
	printf("发送 5 进入高低速测试程序 ！\r\n");
	printf("发送 6 将执行器关机 ！\r\n");
	printf("发送 7 显示帮助信息 ！\r\n");
}

/**
  * @功	能	串口命令处理函数
  * @参	数	cmd：接收到的指令
  * @返	回	无
  */
static void CMD_Handler(uint8_t cmd)
{
	switch (cmd)
	{
	case 1:
	{
		printf("\r\n执行轮询程序！\r\n");

		/* 调用轮询程序 */
		SCA_Lookup();

		printf("轮询结束！\r\n");
		break;
	}

	case 2:
	{
		printf("\r\nSCA初始化！\r\n");

		/* 调用初始化程序 */
		SCA_Init();

		/* 等待执行器稳定 */
		delay_ms(500);

		printf("SCA初始化结束！\r\n");
		break;
	}

	case 3:
	{
		printf("\r\n进入位置归零测试！\r\n");

		/* 调用测试程序 位置归零 */
		//SCA_Homing();

		printf("位置归零测试结束！\r\n");
		break;
	}

	case 4:
	{
		printf("\r\n进入正反转切换测试！\r\n");

		/* 调用测试程序 正反转切换 */
		//SCA_Exp1();

		printf("正反转切换测试结束！\r\n");
		break;
	}

	case 5:
	{
		printf("\r\n进入高低速切换测试！\r\n");

		/* 调用测试程序 高低速切换 */
		//SCA_Exp2();

		printf("高低速切换测试结束！\r\n");
		break;
	}

	case 6:
	{
		printf("\r\n执行器关机！\r\n");

		/* 关闭所有执行器 */
		disableAllActuators();

		printf("执行器关机结束！\r\n");
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
			t1=__fabs(t1)<0.01?0:t1;
			t2=__fabs(t2)<0.01?0:t2;
			t3=__fabs(t3)<0.01?0:t3;
			t4=__fabs(t4)<0.01?0:t4;
			float x = 0, y = 0, z = 0;
			calculate_position_xyz(t1, t2, t3, t4, &x, &y, &z);
			printf("%f, %f, %f\n", x, y, z);
		}
		break;
	}
	default:
		Log();
		break;
	}
}
