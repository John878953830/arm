#ifndef __MAIN_H
#define __MAIN_H
#include "bsp.h"
#include "SCA_API.h"
#include "SCA_APP.h"
#include "stm32f4xx.h"
#include <math.h>
#define DEBUG_OUTPUT 1
extern unsigned char total_motor_number;
extern unsigned char if_error;
extern unsigned char if_need_lookup_monitor;
extern unsigned char lookup_counter;

extern unsigned int tim3_counter;
extern unsigned int tim4_counter;
extern unsigned int tim6_counter;
extern unsigned int tim13_counter;
extern unsigned int tim14_counter;

typedef struct motor_parameter
{
    float step;
    float speed;
    float target;
    float id;
} MOTOR_PARAMETER;

extern MOTOR_PARAMETER mp[5];

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
