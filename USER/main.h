#ifndef __MAIN_H
#define __MAIN_H
#include "bsp.h"
#include "SCA_API.h"
#include "SCA_APP.h"
#include "stm32f4xx.h"
//#include "arm_math.h"

#define DEBUG_OUTPUT 1
#define Pi 3.141593f
#define A2R(angle) (angle * Pi / 180)

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

char position_2_angle(float *t1, float *t2, float *t3, float *t4);
#endif
