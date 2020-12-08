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

u8 cmd_buffer[100];
u8 cmd_counter = 0;
u8 cmd_flag = 0; //1: need to decode cmd  0: no need to decode
u8 cmd_shadow_buffer[100];
//volatile u16 cmd_len;
SYS_PARAMETER sys_p;
CMD_STRUCT cmd_s;
LOOP_BUFFER loop_buf[LOOP_BUF_LEN];
LOOP_BUFFER *loop_head;
LOOP_BUFFER *loop_tail;
u16 frame_index = 0;
float c39_x = 0;
float c39_y = 0;
float c39_z = 0;
PT current_position;
float pre_angle[5];
CMD_PARSER cmd_act[20] = {NULL};
u32 trace_point_counter = 0;
u32 trace_point_max = 0;
volatile STEP_STRUCT step_value;
volatile u8 update_next_pos_flag = 0;
volatile STEP_SPEED_STRUCT step_speed_value;
/* Forward Declaration -----------------------------------------------------------*/
static void Log(void);
static void CMD_Handler(uint8_t cmd);

u8 cmd1_p(CMD_STRUCT input)
{
	return 0;
}

u8 cmd2_p(CMD_STRUCT input)
{
	return 0;
}

u8 cmd3_p(CMD_STRUCT input)
{
	return 0;
}

u8 cmd4_p(CMD_STRUCT input)
{
	return 0;
}

u8 cmd5_p(CMD_STRUCT input)
{
	return 0;
}

u8 cmd6_p(CMD_STRUCT input)
{
	return 0;
}

u8 cmd7_p(CMD_STRUCT input)
{
	return 0;
}

u8 cmd8_p(CMD_STRUCT input)
{
	return 0;
}

u8 cmd9_p(CMD_STRUCT input)
{
	return 0;
}

u8 cmd10_p(CMD_STRUCT input)
{
	return 0;
}

u8 cmd11_p(CMD_STRUCT input)
{
	return 0;
}
void init_loop_buf(void)
{
	int k = 0;
	for (k = 0; k < LOOP_BUF_LEN; k++)
	{
		if (k != LOOP_BUF_LEN - 1)
		{
			loop_buf[k].data = 0;
			loop_buf[k].if_processed = 0;
			loop_buf[k].next = &loop_buf[k + 1];
		}
		else
		{
			loop_buf[k].data = 0;
			loop_buf[k].if_processed = 0;
			loop_buf[k].next = &loop_buf[0];
		}
	}
	loop_head = &loop_buf[0];
	loop_tail = loop_head;
}

u32 CRC_Cal(u8 *data_buffer, u16 size)
{
	if (size == 0 || size > 100)
	{
		return 0;
	}
	u32 data[25];
	u8 *p = data_buffer;
	u16 k = 0;
	u16 i = 0;
	for (k = 0; k < size;)
	{
		if (k + 4 > size - 1)
		{
			u8 tp0 = p[k];
			u8 tp1 = k + 1 >= size ? 0 : p[k + 1];
			u8 tp2 = k + 2 >= size ? 0 : p[k + 2];
			u8 tp3 = k + 3 >= size ? 0 : p[k + 3];
			;
			data[i++] = (((u32)tp0) << 24) | (((u32)tp1) << 16) | (((u32)tp2) << 8) | (((u32)tp3));
			break;
		}
		else
		{
			data[i++] = (((u32)p[k]) << 24) | (((u32)p[k + 1]) << 16) | (((u32)p[k + 2]) << 8) | (((u32)p[k + 3]));
			k += 4;
		}
	}
	u32 tmp = 0;
	CRC_ResetDR();
	for (k = 0; k < i; k++)
	{
		tmp = CRC_CalcCRC(data[k]);
	}
	return tmp;
}
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
	mp[1].zero_offset = -0.3167;
	mp[2].zero_offset = 15.8464;
	mp[3].zero_offset = 33.8108;
	mp[4].zero_offset = -2.2222;

	//init angle dir
	mp[1].angle_dir = -1;
	mp[2].angle_dir = -1;
	mp[3].angle_dir = 1;
	mp[4].angle_dir = -1;

	//init the cmd s
	cmd_s.cmd_index = 0;

	//init the function array
	cmd_act[1] = cmd1_p;
	cmd_act[2] = cmd2_p;
	cmd_act[3] = cmd3_p;
	cmd_act[4] = cmd4_p;
	cmd_act[5] = cmd5_p;
	cmd_act[6] = cmd6_p;
	cmd_act[7] = cmd7_p;
	cmd_act[8] = cmd8_p;
	cmd_act[9] = cmd9_p;
	cmd_act[10] = cmd10_p;
	cmd_act[11] = cmd11_p;

	//init step value
	step_value.sx = 0.1;
	step_value.sy = 0.1;
	step_value.sz = 0.1;

	//init step speed

	step_speed_value.spx = 3;
	step_speed_value.spy = 3;
	step_speed_value.spz = 3;

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
	*z = -L3 * sin(A2R(s23)) - L2 * sin(t2 * Pi / (double)(180.0));
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

		//calculate position
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
			calculate_position_xyz(t1, t2, t3, t4, &current_position.x, &current_position.y, &current_position.z);
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
		mp[id].step = mp[id].speed / 1000; //0.1; //__fabs(pSCA->Position_Real - position)/1000;
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
		//int a_dir = 0;
		float tmp_a = sqrtf(x4 * x4 + y4 * y4 - tmp_b * tmp_b);

		float t1_array[5];
		float t1_sin_value = (tmp_a * y4 - tmp_b * x4) / (x4 * x4 + y4 * y4);
		if (__fabs(t1_sin_value) > 1)
		{
			return 2;
		}
		//calculate the theta 1 when tmp_a > 0
		float t1_tmp = asinf(t1_sin_value);
		t1_array[1] = R2A(t1_tmp) > 180 ? t1_tmp - 360 : R2A(t1_tmp);
		t1_array[2] = (180 - R2A(t1_tmp)) > 180 ? 180 - R2A(t1_tmp) - 360 : 180 - R2A(t1_tmp);
		//printf("tmp t1 1 2 is       %f      %f\n", t1_array[1], t1_array[2]);

		//calculate the theta1 when a < 0
		tmp_a = -tmp_a;
		t1_sin_value = (tmp_a * y4 - tmp_b * x4) / (x4 * x4 + y4 * y4);
		t1_tmp = asinf(t1_sin_value);
		t1_array[3] = R2A(t1_tmp) > 180 ? t1_tmp - 360 : R2A(t1_tmp);
		t1_array[4] = (180 - R2A(t1_tmp)) > 180 ? 180 - R2A(t1_tmp) - 360 : 180 - R2A(t1_tmp);
		//printf("tmp t1    %f      %f    %f    %f\n", t1_array[1], t1_array[2], t1_array[3], t1_array[4]);
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
		if (x4 * x4 + y4 * y4 - tmp_b * tmp_b < 0)
		{
			return 3;
		}
		if (index == 1 || index == 2)
		{
			//a_dir = 1;
			tmp_a = sqrtf(x4 * x4 + y4 * y4 - tmp_b * tmp_b);
		}
		else
		{
			//a_dir = -1;
			tmp_a = -sqrtf(x4 * x4 + y4 * y4 - tmp_b * tmp_b);
		}
		//printf("theta 1       %f\n", _t1);
		*t1 = _t1;

		float t3_array[3];
		if (__fabs((x4 * x4 + y4 * y4 + z4 * z4 - tmp_b * tmp_b - L2 * L2 - L3 * L3) / 2 / L2 / L3) > 1)
		{
			return 4;
		}
		float _t3 = acosf((x4 * x4 + y4 * y4 + z4 * z4 - tmp_b * tmp_b - L2 * L2 - L3 * L3) / 2 / L2 / L3);
		t3_array[1] = R2A(_t3);
		t3_array[2] = 180 + R2A(_t3) > 180 ? -R2A(_t3) : 180 + R2A(_t3);
		//printf("theta 3   %f, %f\n", t3_array[1], t3_array[2]);

		//find the nearest
		_t3 = __fabs(ct3 - t3_array[1]) < __fabs(ct3 - t3_array[2]) ? t3_array[1] : t3_array[2];
		//printf("theta 3       %f\n", _t3);
		*t3 = _t3;

		float mod = sqrtf(L3 * L3 + L2 * L2 + 2 * L2 * L3 * cosf(A2R(_t3)));

		float t2_array[5];
		if (__fabs(tmp_a / mod) > 1 || __fabs((L3 * cosf(A2R(_t3)) + L2) / mod) > 1)
		{
			return 5;
		}
		float tmpt2 = acosf(tmp_a / mod);
		float tmpt2_0 = acosf((L3 * cosf(A2R(_t3)) + L2) / mod);

		t2_array[1] = (tmpt2 - tmpt2_0) > Pi ? (tmpt2 - tmpt2_0) - 2 * Pi : (tmpt2 - tmpt2_0);
		t2_array[2] = (tmpt2 + tmpt2_0) > Pi ? (tmpt2 + tmpt2_0) - 2 * Pi : (tmpt2 + tmpt2_0);
		t2_array[3] = (-tmpt2 - tmpt2_0) > Pi ? (-tmpt2 - tmpt2_0) - 2 * Pi : (-tmpt2 - tmpt2_0);
		t2_array[4] = (-tmpt2 + tmpt2_0) > Pi ? (-tmpt2 + tmpt2_0) - 2 * Pi : (-tmpt2 + tmpt2_0);

		//printf("t2 %f,   %f   %f   %f\n", R2A(t2_array[1]), R2A(t2_array[2]), R2A(t2_array[3]), R2A(t2_array[4]));
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

		//printf("t2 is   %f\n", R2A(t2_array[index]));
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
		//printf("t4 is   %f\n", t4_array[index]);
		*t4 = t4_array[index];
	}

	return 0;
}

u8 parse_cmd(CMD_STRUCT *command)
{
	while (1)
	{
		if (loop_head->data == 0xFF && (loop_head->next)->data == 0xFF)
			break;
		if (loop_head == loop_tail)
			return 1;
		loop_head->data = 0;
		loop_head->if_processed = 0;
		loop_head = loop_head->next;
	}

	if (loop_head->data == 0xFF && (loop_head->next)->data == 0xFF)
	{
		cmd_s.cmd_if_correct = 1;
		//copy out the data
		u8 tmp_buffer[100];
		u8 k = 0;
		loop_head->data = 0;
		loop_head->if_processed = 0;
		loop_head = loop_head->next;
		loop_head->data = 0;
		loop_head->if_processed = 0;
		loop_head = loop_head->next;
		while (loop_head->if_processed == 0)
		{
			u16 loop_counter0 = 0;
			delay_us(2);
			if (loop_counter0++ > 40)
			{
				loop_head->if_processed = 0;
				loop_head = loop_head->next;
				return 2;
			}
		}
		tmp_buffer[k++] = loop_head->data;
		loop_head->if_processed = 0;
		loop_head = loop_head->next;
		while (loop_head->if_processed == 0)
		{
			u16 loop_counter0 = 0;
			delay_us(2);
			if (loop_counter0++ > 40)
			{
				loop_head->if_processed = 0;
				loop_head = loop_head->next;
				return 2;
			}
		}
		tmp_buffer[k++] = loop_head->data;
		loop_head->if_processed = 0;
		loop_head = loop_head->next;
		u16 len = (((u16)tmp_buffer[0]) << 8) | ((u16)tmp_buffer[1]);
		if (len < 6)
		{
			loop_head->if_processed = 0;
			loop_head = loop_head->next;
			printf("data length error\n");
			return 3;
		}
		for (; k < len; k++)
		{
			u16 loop_counter = 0;
			while (loop_head->if_processed == 0)
			{
				delay_us(2);
				if (loop_counter++ > 60)
				{
					loop_head->if_processed = 0;
					loop_head = loop_head->next;
					return 2;
				}
			}
			tmp_buffer[k] = loop_head->data;
			loop_head->if_processed = 0;
			loop_head = loop_head->next;
		}
		//crc
		u32 crc_res = CRC_Cal(tmp_buffer, len - 4);
		printf("CRC result  %x\n", crc_res);
		u32 data_crc = (((u32)tmp_buffer[len - 4]) << 24) | (((u32)tmp_buffer[len - 3]) << 16) | (((u32)tmp_buffer[len - 2]) << 8) | (((u32)tmp_buffer[len - 1]));
		if (crc_res != data_crc)
		{
			printf("crc error\n");
			return 4;
		}
		//parse the data
		cmd_s.cmd_if_correct = 0;
		cmd_s.data_length = len - 4;
		cmd_s.cmd_index = ((u16)tmp_buffer[2] << 8) | ((u16)tmp_buffer[2]);
		cmd_s.function_code = tmp_buffer[4];
		if ((cmd_s.function_code >= 1 && cmd_s.function_code <= 4) || (cmd_s.function_code == 10) || (cmd_s.function_code == 11))
		{
			printf("control frame, function code  %d\n", cmd_s.function_code);
			cmd_s.cmd_type = 3;
			cmd_s.target_motor = 0;
			switch (cmd_s.function_code)
			{
			case 10:
			{
				u8 *tmpdata = &tmp_buffer[5];
				u8 tk = 0;
				for (tk = 0; tk < cmd_s.data_length; tk++)
				{
					if (*tmpdata == 1)
					{
						cmd_s.target_motor |= (1 << tk);
						if (tk + 1 <= 4)
						{
							enableActuator(tk + 1);
							activateActuatorMode(tk + 1, SCA_Profile_Position_Mode, Block);
						}
					}
					tmpdata++;
				}
				if (cmd_s.target_motor > 15)
				{
					printf("warning too much motor, enable 1 to 4 motor already\n");
				}
				break;
			}
			case 11:
			{
				u8 *tmpdata = &tmp_buffer[5];
				u8 tk = 0;
				cmd_s.target_motor = 0xFF;
				for (tk = 0; tk < cmd_s.data_length; tk++)
				{
					if (*tmpdata == 0)
					{
						cmd_s.target_motor &= (0 << tk);
						if (tk + 1 <= 4)
						{
							disableActuator(tk + 1);
						}
					}
					tmpdata++;
				}
				break;
			}
			case 1:
			{
				u8 tpid = tmp_buffer[5];
				if (tpid > 4)
				{
					printf("motor id error\n");
				}
				else
				{
					printf("abs position, cmd1\n");
					int32_t tpdata = 0;
					tpdata = (((int32_t)tmp_buffer[6]) << 24) | (((int32_t)tmp_buffer[7]) << 16) | (((int32_t)tmp_buffer[8]) << 8) | (((int32_t)tmp_buffer[9]));
					setPosition(tpid, (float)tpdata / (float)10000.0);
				}

				break;
			}
			case 2:
			{
				printf("cmd 2, speed cmd\n");
				break;
			}
			case 3:
			{
				printf("cmd 3, relative position cmd\n");
				int32_t tpdata = (((int32_t)tmp_buffer[6]) << 24) | (((int32_t)tmp_buffer[7]) << 16) | (((int32_t)tmp_buffer[8]) << 8) | (((int32_t)tmp_buffer[9]));
				float tppos = (float)tpdata / (float)100.0;
				u8 cmddir = tmp_buffer[5];
				switch (cmddir)
				{
				case 0:
				{
					TIM_Cmd(TIM6, DISABLE);
					cmd_s.px = current_position.x;
					cmd_s.py = current_position.y + __fabs(tppos);
					cmd_s.pz = current_position.z;
					printf("target position is %f, %f, %f\n", cmd_s.px, cmd_s.py, cmd_s.pz);
					TIM_Cmd(TIM6, ENABLE);
					break;
				}
				case 1:
				{
					TIM_Cmd(TIM6, DISABLE);
					cmd_s.px = current_position.x;
					cmd_s.py = current_position.y - __fabs(tppos);
					cmd_s.pz = current_position.z;
					printf("target position is %f, %f, %f\n", cmd_s.px, cmd_s.py, cmd_s.pz);
					TIM_Cmd(TIM6, ENABLE);
					break;
				}
				case 2:
				{
					TIM_Cmd(TIM6, DISABLE);
					cmd_s.px = current_position.x - __fabs(tppos);
					cmd_s.py = current_position.y;
					cmd_s.pz = current_position.z;
					printf("target position is %f, %f, %f\n", cmd_s.px, cmd_s.py, cmd_s.pz);
					TIM_Cmd(TIM6, ENABLE);
					break;
				}
				case 3:
				{
					TIM_Cmd(TIM6, DISABLE);
					cmd_s.px = current_position.x + __fabs(tppos);
					cmd_s.py = current_position.y;
					cmd_s.pz = current_position.z;
					printf("target position is %f, %f, %f\n", cmd_s.px, cmd_s.py, cmd_s.pz);
					TIM_Cmd(TIM6, ENABLE);
					break;
				}
				case 4:
				{
					TIM_Cmd(TIM6, DISABLE);
					cmd_s.px = current_position.x;
					cmd_s.py = current_position.y;
					cmd_s.pz = current_position.z + __fabs(tppos);
					printf("target position is %f, %f, %f\n", cmd_s.px, cmd_s.py, cmd_s.pz);
					TIM_Cmd(TIM6, ENABLE);
					break;
				}
				case 5:
				{
					TIM_Cmd(TIM6, DISABLE);
					cmd_s.px = current_position.x;
					cmd_s.py = current_position.y;
					cmd_s.pz = current_position.z - __fabs(tppos);
					printf("target position is %f, %f, %f\n", cmd_s.px, cmd_s.py, cmd_s.pz);
					TIM_Cmd(TIM6, ENABLE);
					break;
				}
				}
				break;
			}
			case 4:
			{
				printf("cmd4, abs position cmd\n");
				TIM_Cmd(TIM6, DISABLE);
				int32_t tprx = (((int32_t)tmp_buffer[5]) << 24) | (((int32_t)tmp_buffer[6]) << 16) | (((int32_t)tmp_buffer[7]) << 8) | (((int32_t)tmp_buffer[8]));
				int32_t tpry = (((int32_t)tmp_buffer[9]) << 24) | (((int32_t)tmp_buffer[10]) << 16) | (((int32_t)tmp_buffer[11]) << 8) | (((int32_t)tmp_buffer[12]));
				int32_t tprz = (((int32_t)tmp_buffer[13]) << 24) | (((int32_t)tmp_buffer[14]) << 16) | (((int32_t)tmp_buffer[15]) << 8) | (((int32_t)tmp_buffer[16]));
				int32_t tppx = (((int32_t)tmp_buffer[17]) << 24) | (((int32_t)tmp_buffer[18]) << 16) | (((int32_t)tmp_buffer[19]) << 8) | (((int32_t)tmp_buffer[20]));
				int32_t tppy = (((int32_t)tmp_buffer[21]) << 24) | (((int32_t)tmp_buffer[22]) << 16) | (((int32_t)tmp_buffer[23]) << 8) | (((int32_t)tmp_buffer[24]));
				int32_t tppz = (((int32_t)tmp_buffer[25]) << 24) | (((int32_t)tmp_buffer[26]) << 16) | (((int32_t)tmp_buffer[27]) << 8) | (((int32_t)tmp_buffer[28]));
				float frx = (float)tprx / 10000;
				float fry = (float)tpry / 10000;
				float frz = (float)tprz / 10000;
				float fpx = (float)tppx / 100;
				float fpy = (float)tppy / 100;
				float fpz = (float)tppz / 100;
				cmd_s.px = fpx;
				cmd_s.py = fpy;
				cmd_s.pz = fpz;
				cmd_s.rx = frx;
				cmd_s.ry = fry;
				cmd_s.rz = frz;
				printf("%f, %f, %f, %f, %f, %f\n", frx, fry, frz, fpx, fpy, fpz);
				TIM_Cmd(TIM6, ENABLE);
				break;
			}
			}
		}
		if (cmd_s.function_code >= 5 && cmd_s.function_code <= 8)
		{
			printf("read frame, function code  %d\n", cmd_s.function_code);
			cmd_s.cmd_type = 1;
		}
		if (cmd_s.function_code == 9)
		{
			printf("set frame, function code  %d\n", cmd_s.function_code);
			cmd_s.cmd_type = 2;
		}
		if (len == 6)
		{
			printf("heartbeat frame, function code  %d\n", cmd_s.function_code);
			cmd_s.cmd_type = 0;
		}

		return 0;
	}
	else
	{
		loop_head->data = 0;
		loop_head->if_processed = 0;
		loop_head = loop_head->next;
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

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);

	/* 串口1打印LOG信息 */
	Log();

	TIM2_init(100 - 1, 9000 - 1);

	TIM_init(50 - 1, 9000 - 1, 3);
	TIM_init(50 - 1, 9000 - 1, 4);
	TIM_init(50 - 1, 9000 - 1, 13);
	TIM_init(50 - 1, 9000 - 1, 14);
	TIM_init(600 - 1, 9000 - 1, 6);

	//TIM_Cmd(TIM6, ENABLE);

	motor_parameter_init();
	init_loop_buf();

	/* 等待命令传入 */
	while (1)
	{
		if (cmd)
		{
			CMD_Handler(cmd);
			cmd = 0;
		}
		if (__fabs(((u32)loop_tail) - ((u32)loop_head)) > 8)
		{
			if (parse_cmd(&cmd_s) != 0)
			{
				//printf("no frame\n");
			}
			else
			{
				frame_index++;
				printf("receive 1 frame, index   %u\n", frame_index);
			}
		}
		if_error = update_status();
		if (update_next_pos_flag == 1)
		{
			update_next_pos_flag = 0;
			float xn, yn, zn;
			if (__fabs(current_position.x - cmd_s.px) > 10 * step_value.sx)
			{
				xn = current_position.x + (cmd_s.px - current_position.x) / __fabs(current_position.x - cmd_s.px) * 5;//__fabs(current_position.x - cmd_s.px) / 5.0;
			}
			else
			{
				if (__fabs(current_position.x - cmd_s.px) < step_value.sx)
				{
					xn = current_position.x;
				}
				else
				{
					xn = current_position.x + (cmd_s.px - current_position.x) / __fabs(current_position.x - cmd_s.px) * step_value.sx / 2.0;
				}
			}

			if (__fabs(current_position.y - cmd_s.py) > 10 * step_value.sy)
			{
				yn = current_position.y + (cmd_s.py - current_position.y) / __fabs(current_position.y - cmd_s.py) * 5;//__fabs(current_position.y - cmd_s.py) / 5.0;
			}
			else
			{
				if (__fabs(current_position.y - cmd_s.py) < step_value.sy)
				{
					yn = current_position.y;
				}
				else
				{
					yn = current_position.y + (cmd_s.py - current_position.y) / __fabs(current_position.y - cmd_s.py) * step_value.sy / 2.0;
				}
			}

			if (__fabs(current_position.z - cmd_s.pz) > 10 * step_value.sz)
			{
				zn = current_position.z + (cmd_s.pz - current_position.z) / __fabs(current_position.z - cmd_s.pz) * 5;//__fabs(current_position.z - cmd_s.pz) / 5.0;
			}
			else
			{
				if (__fabs(current_position.z - cmd_s.pz) < step_value.sz)
				{
					zn = current_position.z;
				}
				else
				{
					zn = current_position.z + (cmd_s.pz - current_position.z) / __fabs(current_position.z - cmd_s.pz) * step_value.sz / 2.0;
				}
			}

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
				float sv1, sv2, sv3, sv4;
				char res = calculate_inverse(xn, yn, zn, &sv1, &sv2, &sv3, &sv4);
				if (res == 0)
				{
					//calculate the speed
					float sp1 = __fabs(sv1 - t1) / 0.1;
					float sp2 = __fabs(sv2 - t2) / 0.1;
					float sp3 = __fabs(sv3 - t3) / 0.1;
					float sp4 = __fabs(sv4 - t4) / 0.1;
					sp1 = sp1 > 100 ? 100 : sp1;
					sp2 = sp2 > 100 ? 100 : sp2;
					sp3 = sp3 > 100 ? 100 : sp3;
					sp4 = sp4 > 100 ? 100 : sp4;
					sv1 = __fabs(sv1) < 0.1 ? 0 : sv1;
					sv2 = __fabs(sv2) < 0.1 ? 0 : sv2;
					sv3 = __fabs(sv3) < 0.1 ? 0 : sv3;
					sv4 = __fabs(sv4) < 0.1 ? 0 : sv4;
					angle_act_position(sv1, sv2, sv3, sv4, sp1, sp2, sp3, sp4);
				}
				else
				{
					printf("no inverse out of range\n");
				}
			}

			update_next_pos_flag = 0;
		}
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
			printf("current angle      :  %f,   %f,    %f   %f\n", t1, t2, t3, t4);
			calculate_inverse(x, y, z, &t1, &t2, &t3, &t4);
			printf("target  angle      :  %f,   %f,    %f   %f\n", t1, t2, t3, t4);
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
			//calculate_position_xyz(t1, t2, t3, t4, &c39_x, &c39_y, &c39_z);
			float sv1, sv2, sv3, sv4;
			char res = calculate_inverse(c39_x + 5, c39_y, c39_z, &sv1, &sv2, &sv3, &sv4);
			if (res == 0)
			{

				printf("angle      :  %f,   %f,    %f   %f\n", sv1, sv2, sv3, sv4);
				//calculate the speed
				float sp1 = __fabs(sv1 - t1) / 0.1;
				float sp2 = __fabs(sv2 - t2) / 0.1;
				float sp3 = __fabs(sv3 - t3) / 0.1;
				float sp4 = __fabs(sv4 - t4) / 0.1;
				sp1 = sp1 > 100 ? 100 : sp1;
				sp2 = sp2 > 100 ? 100 : sp2;
				sp3 = sp3 > 100 ? 100 : sp3;
				sp4 = sp4 > 100 ? 100 : sp4;
				sv1 = __fabs(sv1) < 0.1 ? 0 : sv1;
				sv2 = __fabs(sv2) < 0.1 ? 0 : sv2;
				sv3 = __fabs(sv3) < 0.1 ? 0 : sv3;
				sv4 = __fabs(sv4) < 0.1 ? 0 : sv4;
				printf("speed    %f   %f   %f   %f\n", sp1, sp2, sp3, sp4);
				angle_act_position(sv1, sv2, sv3, sv4, sp1, sp2, sp3, sp4);
				c39_x += 1;
			}
			else
			{
				printf("no inverse out of range\n");
			}
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
			//calculate_position_xyz(t1, t2, t3, t4, &c39_x, &c39_y, &c39_z);
			float sv1, sv2, sv3, sv4;
			char res = calculate_inverse(c39_x - 5, c39_y, c39_z, &sv1, &sv2, &sv3, &sv4);
			if (res == 0)
			{

				printf("angle      :  %f,   %f,    %f    %f\n", sv1, sv2, sv3, sv4);
				//calculate the speed
				float sp1 = __fabs(sv1 - t1) / 0.1;
				float sp2 = __fabs(sv2 - t2) / 0.1;
				float sp3 = __fabs(sv3 - t3) / 0.1;
				float sp4 = __fabs(sv4 - t4) / 0.1;
				sp1 = sp1 > 100 ? 100 : sp1;
				sp2 = sp2 > 100 ? 100 : sp2;
				sp3 = sp3 > 100 ? 100 : sp3;
				sp4 = sp4 > 100 ? 100 : sp4;
				printf("speed    %f   %f   %f   %f\n", sp1, sp2, sp3, sp4);
				angle_act_position(sv1, sv2, sv3, sv4, sp1, sp2, sp3, sp4);
				c39_x -= 1;
			}
			else
			{
				printf("no inverse out of range\n");
			}
		}
		break;
	}
	case 42:
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
			float sv1, sv2, sv3, sv4;
			char res = calculate_inverse(c39_x + 30, c39_y, c39_z, &sv1, &sv2, &sv3, &sv4);
			if (res == 0)
			{

				printf("angle      :  %f,   %f,    %f    %f\n", sv1, sv2, sv3, sv4);
				//calculate the speed
				float sp1 = __fabs(sv1 - t1) / 0.1;
				float sp2 = __fabs(sv2 - t2) / 0.1;
				float sp3 = __fabs(sv3 - t3) / 0.1;
				float sp4 = __fabs(sv4 - t4) / 0.1;
				sp1 = sp1 > 30 ? 30 : sp1;
				sp2 = sp2 > 30 ? 30 : sp2;
				sp3 = sp3 > 30 ? 30 : sp3;
				sp4 = sp4 > 100 ? 100 : sp4;
				printf("speed    %f   %f   %f   %f\n", sp1, sp2, sp3, sp4);
				angle_act_position(sv1, sv2, sv3, sv4, sp1, sp2, sp3, sp4);
				c39_x += 30;
			}
			else
			{
				printf("no inverse out of range\n");
			}
		}
		break;
	}
	case 43:
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
			float sv1, sv2, sv3, sv4;
			char res = calculate_inverse(c39_x - 30, c39_y, c39_z, &sv1, &sv2, &sv3, &sv4);
			if (res == 0)
			{

				printf("angle      :  %f,   %f,    %f    %f\n", sv1, sv2, sv3, sv4);
				//calculate the speed
				float sp1 = __fabs(sv1 - t1) / 0.1;
				float sp2 = __fabs(sv2 - t2) / 0.1;
				float sp3 = __fabs(sv3 - t3) / 0.1;
				float sp4 = __fabs(sv4 - t4) / 0.1;
				sp1 = sp1 > 30 ? 30 : sp1;
				sp2 = sp2 > 30 ? 30 : sp2;
				sp3 = sp3 > 30 ? 30 : sp3;
				sp4 = sp4 > 100 ? 100 : sp4;
				printf("speed    %f   %f   %f   %f\n", sp1, sp2, sp3, sp4);
				angle_act_position(sv1, sv2, sv3, sv4, sp1, sp2, sp3, sp4);
				c39_x -= 30;
			}
			else
			{
				printf("no inverse out of range\n");
			}
		}
		break;
	}
	case 44:
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
			float sv1, sv2, sv3, sv4;
			char res = calculate_inverse(c39_x, c39_y + 30, c39_z, &sv1, &sv2, &sv3, &sv4);
			if (res == 0)
			{

				printf("angle      :  %f,   %f,    %f    %f\n", sv1, sv2, sv3, sv4);
				//calculate the speed
				float sp1 = __fabs(sv1 - t1) / 0.1;
				float sp2 = __fabs(sv2 - t2) / 0.1;
				float sp3 = __fabs(sv3 - t3) / 0.1;
				float sp4 = __fabs(sv4 - t4) / 0.1;
				sp1 = sp1 > 30 ? 30 : sp1;
				sp2 = sp2 > 30 ? 30 : sp2;
				sp3 = sp3 > 30 ? 30 : sp3;
				sp4 = sp4 > 100 ? 100 : sp4;
				printf("speed    %f   %f   %f   %f\n", sp1, sp2, sp3, sp4);
				angle_act_position(sv1, sv2, sv3, sv4, sp1, sp2, sp3, sp4);
				c39_y += 30;
			}
			else
			{
				printf("no inverse out of range\n");
			}
		}
		break;
	}
	case 45:
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
			float sv1, sv2, sv3, sv4;
			char res = calculate_inverse(c39_x, c39_y - 30, c39_z, &sv1, &sv2, &sv3, &sv4);
			if (res == 0)
			{

				printf("angle      :  %f,   %f,    %f    %f\n", sv1, sv2, sv3, sv4);
				//calculate the speed
				float sp1 = __fabs(sv1 - t1) / 0.1;
				float sp2 = __fabs(sv2 - t2) / 0.1;
				float sp3 = __fabs(sv3 - t3) / 0.1;
				float sp4 = __fabs(sv4 - t4) / 0.1;
				sp1 = sp1 > 30 ? 30 : sp1;
				sp2 = sp2 > 30 ? 30 : sp2;
				sp3 = sp3 > 30 ? 30 : sp3;
				sp4 = sp4 > 100 ? 100 : sp4;
				printf("speed    %f   %f   %f   %f\n", sp1, sp2, sp3, sp4);
				angle_act_position(sv1, sv2, sv3, sv4, sp1, sp2, sp3, sp4);
				c39_y -= 30;
			}
			else
			{
				printf("no inverse out of range \n");
			}
		}
		break;
	}
	case 46:
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
			float sv1, sv2, sv3, sv4;
			char res = calculate_inverse(c39_x, c39_y, c39_z + 30, &sv1, &sv2, &sv3, &sv4);
			if (res == 0)
			{
				printf("angle      :  %f,   %f,    %f    %f\n", sv1, sv2, sv3, sv4);
				//calculate the speed
				float sp1 = __fabs(sv1 - t1) / 0.1;
				float sp2 = __fabs(sv2 - t2) / 0.1;
				float sp3 = __fabs(sv3 - t3) / 0.1;
				float sp4 = __fabs(sv4 - t4) / 0.1;
				sp1 = sp1 > 30 ? 30 : sp1;
				sp2 = sp2 > 30 ? 30 : sp2;
				sp3 = sp3 > 30 ? 30 : sp3;
				sp4 = sp4 > 100 ? 100 : sp4;
				printf("speed    %f   %f   %f   %f\n", sp1, sp2, sp3, sp4);
				angle_act_position(sv1, sv2, sv3, sv4, sp1, sp2, sp3, sp4);
				c39_z += 30;
			}
			else
			{
				printf("no inverse out of range\n");
			}
		}
		break;
	}
	case 47:
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
			float sv1, sv2, sv3, sv4;
			char res = calculate_inverse(c39_x, c39_y, c39_z - 30, &sv1, &sv2, &sv3, &sv4);
			if (res == 0)
			{

				printf("angle      :  %f,   %f,    %f    %f\n", sv1, sv2, sv3, sv4);
				//calculate the speed
				float sp1 = __fabs(sv1 - t1) / 0.1;
				float sp2 = __fabs(sv2 - t2) / 0.1;
				float sp3 = __fabs(sv3 - t3) / 0.1;
				float sp4 = __fabs(sv4 - t4) / 0.1;
				sp1 = sp1 > 30 ? 30 : sp1;
				sp2 = sp2 > 30 ? 30 : sp2;
				sp3 = sp3 > 30 ? 30 : sp3;
				sp4 = sp4 > 100 ? 100 : sp4;
				printf("speed    %f   %f   %f   %f\n", sp1, sp2, sp3, sp4);
				angle_act_position(sv1, sv2, sv3, sv4, sp1, sp2, sp3, sp4);
				c39_z -= 30;
			}
			else
			{
				printf("no inverse , out of range\n");
			}
		}
		break;
	}
	case 48:
	{
		angle_act_position(0, 0, 0, 0, 50, 50, 50, 50);
		break;
	}
	case 49:
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
			float sv1, sv2, sv3, sv4;
			char res = calculate_inverse(c39_x - 40, c39_y - 20, c39_z - 30, &sv1, &sv2, &sv3, &sv4);
			if (res == 0)
			{

				printf("angle      :  %f,   %f,    %f    %f\n", sv1, sv2, sv3, sv4);
				//calculate the speed
				float sp1 = __fabs(sv1 - t1) / 0.1;
				float sp2 = __fabs(sv2 - t2) / 0.1;
				float sp3 = __fabs(sv3 - t3) / 0.1;
				float sp4 = __fabs(sv4 - t4) / 0.1;
				sp1 = sp1 > 30 ? 30 : sp1;
				sp2 = sp2 > 30 ? 30 : sp2;
				sp3 = sp3 > 30 ? 30 : sp3;
				sp4 = sp4 > 100 ? 100 : sp4;
				printf("speed    %f   %f   %f   %f\n", sp1, sp2, sp3, sp4);
				angle_act_position(sv1, sv2, sv3, sv4, sp1, sp2, sp3, sp4);
				c39_z -= 30;
				c39_x -= 40;
				c39_y -= 20;
			}
			else
			{
				printf("no inverse , out of range\n");
			}
		}
		break;
	}
	case 50:
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
			float sv1, sv2, sv3, sv4;
			char res = calculate_inverse(c39_x + 40, c39_y + 20, c39_z + 30, &sv1, &sv2, &sv3, &sv4);
			if (res == 0)
			{

				printf("angle      :  %f,   %f,    %f    %f\n", sv1, sv2, sv3, sv4);
				//calculate the speed
				float sp1 = __fabs(sv1 - t1) / 0.1;
				float sp2 = __fabs(sv2 - t2) / 0.1;
				float sp3 = __fabs(sv3 - t3) / 0.1;
				float sp4 = __fabs(sv4 - t4) / 0.1;
				sp1 = sp1 > 30 ? 30 : sp1;
				sp2 = sp2 > 30 ? 30 : sp2;
				sp3 = sp3 > 30 ? 30 : sp3;
				sp4 = sp4 > 100 ? 100 : sp4;
				printf("speed    %f   %f   %f   %f\n", sp1, sp2, sp3, sp4);
				angle_act_position(sv1, sv2, sv3, sv4, sp1, sp2, sp3, sp4);
				c39_z += 30;
				c39_x += 40;
				c39_y += 20;
			}
			else
			{
				printf("no inverse , out of range\n");
			}
		}
		break;
	}
	default:
		Log();
		break;
	}
}
