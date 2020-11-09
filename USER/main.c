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

	TIM2_init(100 - 1, 9000 - 1);

	TIM_init(10 - 1, 9000 - 1, 3);
	TIM_init(10 - 1, 9000 - 1, 4);
	TIM_init(10 - 1, 9000 - 1, 13);
	TIM_init(10 - 1, 9000 - 1, 14);
	TIM_init(5 - 1, 9000 - 1, 6);
	
	TIM_Cmd(TIM6,ENABLE);

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
	default:
		Log();
		break;
	}
}
