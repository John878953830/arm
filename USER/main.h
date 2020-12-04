#ifndef __MAIN_H
#define __MAIN_H
#include "bsp.h"
#include "SCA_API.h"
#include "SCA_APP.h"
#include "stm32f4xx.h"
#include "stm32f4xx_crc.h"
//#include "arm_math.h"

#define DEBUG_OUTPUT 1
#define LOOP_BUF_LEN 1000
#define Pi 3.141593f
#define A2R(angle) (angle * Pi / 180)
#define R2A(radian) (radian * 180 / Pi)

//matrix define
#define L0 0f
#define A0 0f
#define D0 0f
#define T0 0f
#define L1 119.4f
#define A1 -90.0f
#define D1 0f
#define T1 0f
#define L2 235.0f
#define A2 0f
#define D2 0f
#define T2 0f
#define L3 265.9f
#define A3 0f
#define D3 -9.5f
#define T3 0f

extern unsigned char total_motor_number;
extern unsigned char if_error;
extern unsigned char if_need_lookup_monitor;
extern unsigned char lookup_counter;

extern unsigned int tim3_counter;
extern unsigned int tim4_counter;
extern unsigned int tim6_counter;
extern unsigned int tim13_counter;
extern unsigned int tim14_counter;
extern u16 frame_index;

extern float pre_angle[5];

extern u8 cmd_buffer[100];
extern u8 cmd_counter;
extern u8 cmd_flag;
extern u8 cmd_shadow_buffer[100];
extern volatile u16 cmd_len;
extern volatile u8 update_next_pos_flag;
extern u32 trace_point_counter;
extern u32 trace_point_max;

typedef struct loop_buffer
{
    u8 data;
    u8 if_processed;
    struct loop_buffer *next;
} LOOP_BUFFER;

extern LOOP_BUFFER loop_buf[LOOP_BUF_LEN];
extern LOOP_BUFFER *loop_head;
extern LOOP_BUFFER *loop_tail;
typedef struct cmd_struct
{
    u8 cmd_if_correct; //0: correct 1: error
    u16 cmd_type;      //0: heart beat  1: read 2: set 3: control
    u16 data_length;
    u16 cmd_lenght;
    u16 cmd_index;
    u16 function_code;
    u16 target_motor; //bit 0: 1: motor 0 selected 0: motor 0 unselected
    u8 dir[5];        //0: x+ 1: x- 2: y+ 3: y- 4:z+ 5:z-
    float rx;
    float ry;
    float rz;
    float px;
    float py;
    float pz;
    float speed[5];
} CMD_STRUCT;

extern CMD_STRUCT cmd_s;
typedef u8 (*CMD_PARSER)(CMD_STRUCT input);
extern CMD_PARSER cmd_act[20];

//rotation matrix
extern float r1_0[3][3];
extern float r2_0[3][3];
extern float r3_0[3][3];
extern float r4_0[3][3];

//offset matrix
extern float porg_1_0[3][1];
extern float porg_2_1[3][1];
extern float porg_3_2[3][1];
extern float porg_4_3[3][1];

typedef struct motor_parameter
{
    float angle_dir;
    float zero_offset;
    float step;
    float speed;
    float target;
    float id;
} MOTOR_PARAMETER;

extern MOTOR_PARAMETER mp[5];

typedef struct position_struct
{
    float x;
    float y;
    float z;
} PT;

extern PT current_position;

typedef struct step_struct
{
    float sx;
    float sy;
    float sz;
} STEP_STRUCT;

extern volatile STEP_STRUCT step_value;

typedef struct step_speed
{
    float spx;
    float spy;
    float spz;
} STEP_SPEED_STRUCT;

extern volatile STEP_SPEED_STRUCT step_speed_value;

/**
 * @brief describe the system status, include location, speed, if_rotation
 * 
 */
typedef struct sys_parameter
{
    float x;
    float y;
    float z;
    float sp_x;
    float sp_y;
    float sp_z;
    char motor_status[5]; //0: motor stop  1: motor move positive -1: motor move negative
} SYS_PARAMETER;

extern SYS_PARAMETER sys_p;

/**
 * @brief calculate the rotation matrix
 * @param t1   angle1
 * @param t2   angle2
 * @param t3   angle3
 * @param t4   angle4
 */
void calculate_r_4_0(float t1, float t2, float t3, float t4);

void calculate_r_3_0(float t1, float t2, float t3);

void calculate_r_2_0(float t1, float t2);

void calculate_r_1_0(float t1);

void motor_parameter_init(void);

/**
 * @brief motor act function
 * @param id motor id
 * @param step step length
 * @param dir move direction, 0: positive, 1: negative
 * @return int 0: correct 1~255: error
 */
char motor_act(size_t id, float step, char dir);
/**
 * @brief update the motor status when timeout
 * 
 * @return char 0:success, 
 *              1~255: error  
 *              10+id: not online
 *              15+id: not enable
 *              20+id: position get error;
 */
char update_status(void);
/**
 * @brief move to position at the speed
 * @param id 
 * @param speed 
 * @param position 
 * @return char 0: no error, 1~255: error
 */
char motor_act_position(size_t id, float speed, float position);

/**
 * @brief   angle act command
 * 
 * @param t1 
 * @param t2 
 * @param t3 
 * @param t4 
 * @param v1 
 * @param v2 
 * @param v3 
 * @param v4 
 * @return char 
 */
char angle_act_position(float t1, float t2, float t3, float t4, float v1, float v2, float v3, float v4);
/**
 * @brief  give the position of x, y, z for the original point p4org
 * 
 * @param t1 
 * @param t2 
 * @param t3 
 * @param t4 
 * @param x 
 * @param y 
 * @param z 
 * @return char 
 */
char calculate_position_xyz(float t1, float t2, float t3, float t4, float *x, float *y, float *z);
/**
 * @brief      save angle in t1, t2, t3, t4
 * 
 * @param t1 
 * @param t2 
 * @param t3 
 * @param t4 
 * @return char 
 */
char position_2_angle(float *t1, float *t2, float *t3, float *t4);

/**
 * @brief calculate the inverse angle
 * 
 * @param x4 
 * @param y4 
 * @param z4 
 * @param t1 
 * @param t2 
 * @param t3 
 * @param t4 
 * @return char 
 */
char calculate_inverse(float x4, float y4, float z4, float *t1, float *t2, float *t3, float *t4);
/**
 * @brief calculate the crc32
 * 
 * @param data_buffer 
 * @param size 
 * @return u32 
 */
u32 CRC_Cal(u8 *data_buffer, u16 size);
/**
 * @brief init the loop buffer for cmd receive
 * 
 */
void init_loop_buf(void);
/**
 * @brief pase cmd function
 * 
 * @param command 
 * @return u8 
 */
u8 parse_cmd(CMD_STRUCT *command);

u8 cmd1_p(CMD_STRUCT input);

u8 cmd2_p(CMD_STRUCT input);

u8 cmd3_p(CMD_STRUCT input);

u8 cmd4_p(CMD_STRUCT input);

u8 cmd5_p(CMD_STRUCT input);

u8 cmd6_p(CMD_STRUCT input);

u8 cmd7_p(CMD_STRUCT input);

u8 cmd8_p(CMD_STRUCT input);

u8 cmd9_p(CMD_STRUCT input);

u8 cmd10_p(CMD_STRUCT input);

u8 cmd11_p(CMD_STRUCT input);

#endif
