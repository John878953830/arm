#ifndef __MAIN_H
#define __MAIN_H
#include "bsp.h"
#include "SCA_API.h"
#include "SCA_APP.h"
#include "stm32f4xx.h"
//#include "arm_math.h"

#define DEBUG_OUTPUT 1

//matrix define
#define L0 0
#define A0 0
#define D0 0
#define T0 0
#define L1 119.1
#define A1 -90
#define D1 0
#define T1 0
#define L2 235
#define A2 0
#define D2 0
#define T2 0
#define L3 265
#define A3 0
#define D3 9.5
#define T3 0
#define Pi 3.141593
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
    float zero_offset;
    float step;
    float speed;
    float target;
    float id;
} MOTOR_PARAMETER;

extern MOTOR_PARAMETER mp[5];

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
#endif
